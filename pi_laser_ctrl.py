#!/usr/bin/env python3
## @file Implementation for pi laser ctrl.
##
"""
Photonics Industries DX-355-5-WC Laser Controller - CLI
Protocol: HEXASC, 115200 8N1, CR-terminated, 2-byte checksum

Commands:
  status       Full status + temps + power
  poll         Complete diagnostic dump of all readable parameters
  go           Ready-check, configure, enable shutter+LDD
  startup      Guided startup sequence
  shutdown     Safe shutdown (current to zero)
  monitor      Live display (Ctrl-C to exit)
  shutter on|off
  ldd on|off
  current <A>  Set current (hard limit 7.2A)
  raw <hex>    Send raw HEXASC (2-byte checksum auto-appended)
  reset        System reset
  q / quit     Exit (laser state preserved - use 'shutdown' first if needed)
"""

import sys, time, threading
import serial, serial.tools.list_ports

# ---- Constants ----
BAUD             = 115200
MAX_CURRENT      = 7.2
TEMP_SETPOINTS   = {'SHG': 49.75, 'THG': 69.70}
TEMP_ALARM_DELTA = 2.0
POLL_INTERVAL    = 1.0
SYS_STATES = {0:'Initializing', 1:'Ramping', 2:'Running',
              3:'Standby', 4:'Soft Fault', 5:'Hard Fault'}

# ---- Protocol ----

## @brief Documents build_frame.
##
def build_frame(rw, nn, aa, cc, data):
    b = [rw, nn, aa, cc] + data
    return "".join(f"{x:02X}" for x in b) + f"{sum(b)&0xFFFF:04X}"

## @brief Documents send_cmd.
##
def send_cmd(ser, hexstr, timeout=1.0):
    ser.reset_input_buffer()
    ser.write((hexstr + "\r").encode("ascii"))
    deadline = time.time() + timeout
    buf = b""
    while time.time() < deadline:
        b = ser.read(1)
        if b in (b"\r", b"\n"):
            if buf: break
        elif b:
            buf += b
    return buf.decode("ascii", errors="replace").strip()

## @brief Documents parse_response.
##
def parse_response(resp):
    if len(resp) < 10 or len(resp) % 2 != 0:
        return False, 0, 0, []
    try:
        raw = bytes.fromhex(resp)
    except ValueError:
        return False, 0, 0, []
    rw, nn, cc = raw[0], raw[1], raw[3]
    if len(raw) != 4 + nn + 2: return False, rw, cc, []
    data = list(raw[4:4+nn])
    ok = ((raw[-2]<<8)|raw[-1]) == (sum(raw[:-2])&0xFFFF) and rw in (0x02,0x03)
    return ok, rw, cc, data

## @brief Documents _write.
##
def _write(ser, cc, data):
    r = send_cmd(ser, build_frame(0x00, len(data), 0x00, cc, data))
    ok, _, _, d = parse_response(r)
    return ok, d, r

## @brief Documents _read.
##
def _read(ser, cc, aa=0x00):
    r = send_cmd(ser, build_frame(0x01, 0x00, aa, cc, []))
    ok, _, _, d = parse_response(r)
    return ok, d, r

# ---- Read functions ----

## @brief Documents read_status.
##
def read_status(ser):
    ok, data, _ = _read(ser, 0x0F)
    if not ok or len(data) < 5: return None
    d1,d2,d3,d4,d5 = data[:5]
    return {
        'power_on':       bool(d1&0x01), 'shutter':        bool(d1&0x02),
        'key_switch':     bool(d1&0x04), 'ldd':            bool(d1&0x08),
        'qsw':            bool(d1&0x10), 'shutter_ilock':  bool(d1&0x20),
        'ldd_ilock':      bool(d1&0x40),
        'sys_faults': d2, 'gen_alarms': d3, 'temp_alarms': d4,
        'state': d5, 'state_str': SYS_STATES.get(d5, f'Unknown({d5})'),
        'alarm_flow':     bool(d3&0x08), 'alarm_humidity': bool(d3&0x20),
        'alarm_wet':      bool(d3&0x10), 'alarm_hfsync':   bool(d3&0x40),
        'alarm_pwrmon':   bool(d3&0x80),
    }

## @brief Documents read_2b.
##
def read_2b(ser, cc, scale=100.0):
    ok, data, _ = _read(ser, cc)
    if not ok or len(data) < 2: return None
    return ((data[0]<<8)|data[1]) / scale

## @brief Documents read_power_monitor.
##
def read_power_monitor(ser):
    """0x15 Power Monitor - laser returns 4 bytes, units 0.01W.
    Appears to be LDD electrical input power, not optical output."""
    ok, data, _ = _read(ser, 0x15)
    if not ok or len(data) < 2: return None
    if len(data) >= 4:
        val = (data[0]<<24)|(data[1]<<16)|(data[2]<<8)|data[3]
    else:
        val = (data[0]<<8)|data[1]
    return val / 100.0

## @brief Documents read_prf_ext.
##
def read_prf_ext(ser):
    """0x28 PRF Actual EXT - LMCV4 clock as measured at laser input.
    This is the LightBurn-requested fire rate arriving at the DX."""
    ok, data, _ = _read(ser, 0x28)
    if not ok or len(data) < 2: return None
    if len(data) >= 3:
        return (data[0]<<16)|(data[1]<<8)|data[2]
    return (data[0]<<8)|data[1]

## @brief Documents read_pulse_energy.
##
def read_pulse_energy(ser):
    """0x8C Pulse Energy - 0.01 µJ/count. Valid only when LDD on and firing."""
    ok, data, _ = _read(ser, 0x8C)
    if not ok or len(data) < 4: return None
    val = (data[0]<<24)|(data[1]<<16)|(data[2]<<8)|data[3]
    return val / 100.0  # µJ

## @brief Documents read_1b.
##
def read_1b(ser, cc):
    ok, data, _ = _read(ser, cc)
    return data[0] if ok and data else None

## @brief Documents read_3b.
##
def read_3b(ser, cc):
    ok, data, _ = _read(ser, cc)
    if not ok or len(data) < 3: return None
    return (data[0]<<16)|(data[1]<<8)|data[2]

## @brief Documents read_temp.
##
def read_temp(ser, aa):
    ok, data, _ = _read(ser, 0x32, aa)
    if not ok or len(data) < 2: return None
    return ((data[0]<<8)|data[1]) / 100.0

## @brief Documents read_soft_faults.
##
def read_soft_faults(ser):
    ok, data, _ = _read(ser, 0x0C)
    if not ok or len(data) < 2: return None
    d = data[1]
    return {'ldd_interlock': bool(d&0x02), 'shutter_interlock': bool(d&0x04),
            'low_ext_freq':  bool(d&0x08), 'high_ext_freq':     bool(d&0x10), 'raw': d}

## @brief Documents read_board_faults.
##
def read_board_faults(ser):
    ok, data, _ = _read(ser, 0x0D)
    if not ok or len(data) < 2: return None
    d = data[1]
    return {'main': bool(d&0x01), 'common': bool(d&0x02), 'temperature': bool(d&0x04),
            'sensor': bool(d&0x08), 'power_monitor': bool(d&0x10),
            'pulse': bool(d&0x20), 'ldd': bool(d&0x40), 'motor': bool(d&0x80), 'raw': d}

## @brief Documents poll_all.
##
def poll_all(ser):
    return {
        'status':        read_status(ser),
        'set_current':   read_2b(ser, 0x11),
        'act_current':   read_2b(ser, 0x12),
        'max_current':   read_2b(ser, 0x14),
        'diode_voltage': read_2b(ser, 0x13),
        'power_w':       read_power_monitor(ser),
        'prf_ext':       read_prf_ext(ser),
        'pulse_energy':  read_pulse_energy(ser),
        'temp_shg':      read_temp(ser, 0x00),
        'temp_thg':      read_temp(ser, 0x01),
        'temp_cb':       read_temp(ser, 0x03),  # 0x32 AA=0x03 (Temp4 alt) - confirmed CB on this unit
        'prf':           read_3b(ser, 0x21),
        'prf_source':    read_1b(ser, 0x23),
        'gate_source':   read_1b(ser, 0x24),
        'duty_mode':     read_1b(ser, 0x69),
        'duty_width':    read_3b(ser, 0x54),
        'pulsed_op':     read_1b(ser, 0x20),
        'burst':         read_1b(ser, 0x56),
        'flow':          read_2b(ser, 0x74, 10.0),
        'soft_faults':   read_soft_faults(ser),
        'board_faults':  read_board_faults(ser),
    }

# ---- Display ----

## @brief Documents fmt_poll.
##
def fmt_poll(d):
    st  = d['status']
    SEP = "="*56
    sep = "-"*56
    L   = ["", SEP, "  DX-355-5-WC  FULL DIAGNOSTIC POLL", SEP]

    if st:
        L += [
            f"  System State    : {st['state_str']}",
            f"  Key Switch      : {'ON' if st['key_switch'] else 'OFF'}",
            f"  Shutter         : {'OPEN' if st['shutter'] else 'closed'}",
            f"  LDD             : {'ON'   if st['ldd']        else 'off'}",
            f"  QSW             : {'ON'   if st['qsw']        else 'off'}",
            f"  Shutter Ilock   : {'OK'   if st['shutter_ilock'] else 'FAULT'}",
            f"  LDD Interlock   : {'OK'   if st['ldd_ilock']     else 'FAULT'}",
        ]
        faults = []
        if st['sys_faults']:  faults.append(f"SYS 0x{st['sys_faults']:02X}")
        if st['gen_alarms']:  faults.append(f"GEN 0x{st['gen_alarms']:02X}")
        if st['temp_alarms']: faults.append(f"TEMP 0x{st['temp_alarms']:02X}")
        active_alarms = [n for k,n in [
            ('alarm_flow','FLOW'),('alarm_humidity','HUMIDITY'),
            ('alarm_wet','WET'),('alarm_hfsync','HF SYNC'),
            ('alarm_pwrmon','PWR MON')] if st.get(k)]
        if faults or active_alarms:
            L.append(f"  ⚠ Alarms        : {', '.join(faults+active_alarms)}")
    else:
        L.append("  Status read FAILED")

    L.append(sep)

    ## @brief Documents fv.
    ##
    def fv(val, fmt, unit): return f"{val:{fmt}} {unit}" if val is not None else "N/A"
    L += [
        f"  Set Current     : {fv(d['set_current'],   '.2f', 'A')}",
        f"  Actual Current  : {fv(d['act_current'],   '.2f', 'A')}",
        f"  Max Current     : {fv(d['max_current'],   '.2f', 'A')}",
        f"  Diode Voltage   : {fv(d['diode_voltage'], '.2f', 'V')}",
        f"  Power Output    : {fv(d['power_w'],       '.2f', 'W')}",
    ]
    L.append(sep)

    for key, name, sp_key in [('temp_shg','SHG','SHG'),('temp_thg','THG','THG')]:
        val = d[key]; sp = TEMP_SETPOINTS.get(sp_key, 0)
        if val is not None:
            flag = "  ⚠ ALARM" if abs(val-sp) > TEMP_ALARM_DELTA else ""
            L.append(f"  {name} Temp        : {val:.2f}°C  (sp {sp:.2f}°C){flag}")
        else:
            L.append(f"  {name} Temp        : N/A")
    cb = d.get('temp_cb')
    L.append(f"  CB Temp         : {cb:.2f}°C" if cb is not None else "  CB Temp         : N/A")

    L.append(sep)
    prf_src  = {0:'INT', 1:'EXT'}.get(d['prf_source'],  '?')
    gate_src = {0:'INT', 1:'EXT'}.get(d['gate_source'], '?')
    duty_m   = {0:'Fixed', 1:'Track'}.get(d['duty_mode'], '?')
    pulsed   = {0:'Disabled', 1:'Enabled'}.get(d['pulsed_op'], '?')
    burst    = {0:'Off', 1:'On'}.get(d['burst'], '?')
    prf_v    = f"{d['prf']:,} Hz" if d['prf'] else "0  (EXT - owned by LMCV4)"
    prf_ext  = d.get('prf_ext')
    prf_act  = d.get('prf')   # 0x22 confirmed fires
    pe       = d.get('pulse_energy')
    dw       = d['duty_width']

    # Q-pulse chain: LMCV4 in → confirmed fires → missed pulses
    if prf_ext and prf_act is not None:
        missed = prf_ext - prf_act
        pct    = (prf_act / prf_ext * 100) if prf_ext else 0
        qchain = (f"{prf_ext:,} Hz in  |  {prf_act:,} Hz confirmed  "
                  f"|  {missed:+,} Hz delta  ({pct:.0f}% accepted)")
    elif prf_ext:
        qchain = f"{prf_ext:,} Hz  (LMCV4 clock at laser input)"
    else:
        qchain = "N/A"

    L += [
        f"  PRF Set         : {prf_v}",
        f"  PRF Source      : {prf_src}",
        f"  Gate Source     : {gate_src}",
        f"  Q-Pulse Chain   : {qchain}",
        f"  Pulse Energy    : {pe:.2f} µJ" if pe is not None else "  Pulse Energy    : N/A",
        f"  Duty Mode       : {duty_m}",
        f"  Duty Width      : {dw} ns" if dw is not None else "  Duty Width      : N/A",
        f"  Pulsed Op       : {pulsed}",
        f"  Burst           : {burst}",
    ]
    L.append(sep)
    flow = d['flow']
    L.append(f"  Flow Rate       : {flow:.1f} L/min" if flow else "  Flow Rate       : N/A  (no meter)")

    sf = d['soft_faults']
    bf = d['board_faults']
    if sf and sf['raw']:
        L.append(f"  Soft Faults     : {', '.join(k for k,v in sf.items() if k!='raw' and v)}")
    if bf and bf['raw']:
        L.append(f"  Board Faults    : {', '.join(k for k,v in bf.items() if k!='raw' and v)}")

    L += [SEP, ""]
    return "\n".join(L)

## @brief Documents fmt_status.
##
def fmt_status(ser):
    st   = read_status(ser)
    pw   = read_power_monitor(ser)
    amps = read_2b(ser, 0x12)
    shg  = read_temp(ser, 0x00)
    thg  = read_temp(ser, 0x01)
    cb   = read_temp(ser, 0x03)
    if not st: return "ERR: status read failed"
    L = [
        f"  System State  : {st['state_str']}",
        f"  Power On      : {'YES' if st['power_on'] else 'no'}",
        f"  Shutter       : {'OPEN' if st['shutter'] else 'closed'}",
        f"  Key Switch    : {'ON'   if st['key_switch'] else 'OFF'}",
        f"  LDD           : {'ON'   if st['ldd'] else 'off'}",
        f"  QSW           : {'ON'   if st['qsw'] else 'off'}",
        f"  Shutter Ilock : {'OK'   if st['shutter_ilock'] else 'FAULT'}",
        f"  LDD Interlock : {'OK'   if st['ldd_ilock'] else 'FAULT'}",
        f"  Actual Current: {amps:.2f} A" if amps is not None else "  Actual Current: N/A",
        f"  Power Output  : {pw:.2f} W"   if pw   is not None else "  Power Output  : N/A",
    ]
    for name, val, sp_key in [('SHG',shg,'SHG'),('THG',thg,'THG')]:
        sp = TEMP_SETPOINTS.get(sp_key, 0)
        if val is not None:
            flag = "  ⚠ ALARM" if abs(val-sp) > TEMP_ALARM_DELTA else ""
            L.append(f"  {name}           : {val:.2f}°C  (sp {sp:.2f}°C){flag}")
        else:
            L.append(f"  {name}           : N/A")
    if st['sys_faults']: L.append(f"  Sys Faults    : 0x{st['sys_faults']:02X}")
    if st['gen_alarms']: L.append(f"  Gen Alarms    : 0x{st['gen_alarms']:02X}")
    L.append(f"  CB            : {cb:.2f}°C" if cb is not None else "  CB            : N/A")
    return "\n".join(L)

# ---- Safety ----

## @brief Documents check_temps.
##
def check_temps(ser):
    alarms = []
    for name, aa, sp_key in [('SHG',0x00,'SHG'),('THG',0x01,'THG')]:
        t  = read_temp(ser, aa)
        sp = TEMP_SETPOINTS.get(sp_key, 0)
        if t is not None and abs(t-sp) > TEMP_ALARM_DELTA:
            alarms.append(f"TEMP ALARM: {name} = {t:.2f}°C (sp {sp:.2f}°C)")
    return alarms

# ---- Shutter open helper (LDD cycle) ----

## @brief Documents open_shutter.
##
def open_shutter(ser):
    st = read_status(ser)
    if st and st['shutter']: return True, "already open"
    ldd_was_on = st and st['ldd']
    if ldd_was_on:
        _write(ser, 0x10, [0x00]); time.sleep(0.2)
    ok, _, r = _write(ser, 0x04, [0x01])
    if ldd_was_on and ok:
        time.sleep(0.1); _write(ser, 0x10, [0x01])
    return ok, r

# ---- Sequences ----

## @brief Documents cmd_go.
##
def cmd_go(ser):
    print("\n---- Go Sequence ----")
    st = read_status(ser)
    if not st: print("ERR: cannot read status."); return
    if st['state'] not in (2,3): print(f"ERR: {st['state_str']}"); return
    if not st['key_switch']:     print("ERR: key switch OFF"); return
    if not st['shutter_ilock']:  print("ERR: shutter interlock FAULT"); return
    if not st['ldd_ilock']:      print("ERR: LDD interlock FAULT"); return
    alarms = check_temps(ser)
    if alarms:
        for a in alarms: print(f"  ⚠ {a}")
        print("ERR: temp alarms - aborting."); return

    val = int(round(MAX_CURRENT*100))
    ok,_,_ = _write(ser, 0x14, [(val>>8)&0xFF, val&0xFF])
    print(f"  {'✓' if ok else '✗'} Max Current → {MAX_CURRENT}A")
    ok,_,_ = _write(ser, 0x23, [0x01]); print(f"  {'✓' if ok else '✗'} PRF Source → EXT")
    ok,_,_ = _write(ser, 0x24, [0x00]); print(f"  {'✓' if ok else '✗'} Gate Source → INT")

    ok, r = open_shutter(ser)[:2]
    print(f"  {'✓' if ok else '✗'} Shutter → OPEN  ({r})")
    if not ok: print("ERR: shutter failed."); return

    st2 = read_status(ser)
    if st2 and not st2['ldd']:
        ok,_,r = _write(ser, 0x10, [0x01])
        print(f"  {'✓' if ok else '✗'} LDD → ON  ({r})")

    ok,_,r = _write(ser, 0x11, [(val>>8)&0xFF, val&0xFF])
    print(f"  {'✓' if ok else '✗'} Current → {MAX_CURRENT}A  ({r})")
    print("\n---- Ready. LightBurn may fire. ----\n")

## @brief Documents cmd_startup.
##
def cmd_startup(ser):
    print("\n---- Startup Sequence ----")
    print("Reading current state...")
    d = poll_all(ser)
    st = d['status']
    if not st: print("ERR: cannot read status."); return

    print(fmt_poll(d))

    if st['state'] == 1:
        print("  ⚠  Temperatures still ramping.")
        if input("  Continue anyway? (yes/no): ").strip().lower() != 'yes':
            print("Aborted."); return

    reasons = []
    if not st['key_switch']:    reasons.append("Key switch OFF")
    if not st['shutter_ilock']: reasons.append("Shutter interlock FAULT")
    if not st['ldd_ilock']:     reasons.append("LDD interlock FAULT")
    if st['state'] not in (2,3): reasons.append(f"State: {st['state_str']}")
    alarms = check_temps(ser)
    reasons.extend(alarms)
    if reasons:
        for r in reasons: print(f"    ⚠ {r}")
        if input("  Proceed despite warnings? (yes/no): ").strip().lower() != 'yes':
            print("Aborted."); return

    val = int(round(MAX_CURRENT*100))
    _write(ser, 0x14, [(val>>8)&0xFF, val&0xFF]); print(f"  ✓ Max Current → {MAX_CURRENT}A")
    _write(ser, 0x23, [0x01]); print("  ✓ PRF Source → EXT")
    _write(ser, 0x24, [0x00]); print("  ✓ Gate Source → INT")

    if input("\n  Open shutter and enable LDD? (yes/no): ").strip().lower() != 'yes':
        print("Stopped before enabling output."); return

    ok, r = open_shutter(ser)
    if isinstance(ok, bool) and not ok:
        print(f"  ✗ Shutter failed ({r}). Aborting."); return
    print(f"  ✓ Shutter → OPEN")

    ok,_,r = _write(ser, 0x10, [0x01]); print(f"  {'✓' if ok else '✗'} LDD → ON  ({r})")
    ok,_,r = _write(ser, 0x11, [(val>>8)&0xFF, val&0xFF])
    print(f"  {'✓' if ok else '✗'} Current → {MAX_CURRENT}A  ({r})")
    print("\n---- Laser is active. ----\n")

## @brief Documents cmd_shutdown.
##
def cmd_shutdown(ser):
    print("\n---- Shutdown Sequence ----")
    _write(ser, 0x04, [0x00]); print("  ✓ Shutter → Closed")
    _write(ser, 0x10, [0x00]); print("  ✓ LDD → Off")
    print("  Waiting for current to reach zero...")
    for _ in range(30):
        time.sleep(0.5)
        a = read_2b(ser, 0x12)
        if a is not None:
            print(f"    {a:.2f} A", end="\r")
            if a < 0.1: break
    print()
    d = poll_all(ser)
    print(fmt_poll(d))
    print("\n---- Ready to exit. ----\n")

# ---- Monitor thread ----

## @brief Documents Monitor.
##
class Monitor:
    ## @brief Documents __init__.
    ##
    def __init__(self, ser):
        self.ser = ser
        self._stop  = threading.Event()
        self._pause = threading.Event()
        self._lock  = threading.Lock()
        self.last   = {}
        self._active_alarms = set()
        self._fault_alerted = False
        threading.Thread(target=self._run, daemon=True).start()

    ## @brief Documents pause.
    ##
    def pause(self):  self._pause.set();   time.sleep(0.12)
    ## @brief Documents resume.
    ##
    def resume(self): self._pause.clear()
    ## @brief Documents stop.
    ##
    def stop(self):   self._stop.set()

    ## @brief Documents _run.
    ##
    def _run(self):
        while not self._stop.is_set():
            if self._pause.is_set():
                self._stop.wait(0.05); continue
            try:
                with self._lock:
                    st   = read_status(self.ser)
                    pw   = read_2b(self.ser, 0x15)
                    amps = read_2b(self.ser, 0x12)
                    shg  = read_temp(self.ser, 0x00)
                    thg  = read_temp(self.ser, 0x01)
                    prf  = read_3b(self.ser, 0x21)
                    prf_src = read_1b(self.ser, 0x23)
                    dw   = read_3b(self.ser, 0x54)
                    self.last = dict(st=st,pw=pw,amps=amps,shg=shg,thg=thg,
                                     prf=prf,prf_src=prf_src,dw=dw)
                if st:
                    alarms = []
                    for name, val, sp in [('SHG',shg,TEMP_SETPOINTS['SHG']),
                                          ('THG',thg,TEMP_SETPOINTS['THG'])]:
                        if val and abs(val-sp) > TEMP_ALARM_DELTA:
                            alarms.append(f"TEMP {name}={val:.1f}°C")
                    alarm_set = set(alarms)
                    if alarm_set - self._active_alarms:
                        print(f"\n{'='*50}")
                        for a in alarm_set - self._active_alarms:
                            print(f"  ⚠  {a}")
                        print(f"{'='*50}")
                        if st['ldd']:
                            print("  AUTO SHUTDOWN: temp limit exceeded")
                            with self._lock:
                                _write(self.ser, 0x04, [0x00])
                                _write(self.ser, 0x10, [0x00])
                    self._active_alarms = alarm_set
                    if st['state'] in (4,5) and not self._fault_alerted:
                        print(f"\n  ⚠  LASER FAULT: {st['state_str']}")
                        self._fault_alerted = True
                    elif st['state'] not in (4,5):
                        self._fault_alerted = False
            except Exception:
                pass
            self._stop.wait(POLL_INTERVAL)

    ## @brief Documents display.
    ##
    def display(self):
        d = self.last
        st = d.get('st'); amps = d.get('amps'); pw = d.get('pw')
        shg = d.get('shg'); thg = d.get('thg')
        prf = d.get('prf'); prf_src = d.get('prf_src'); dw = d.get('dw')
        if not st: return
        prf_str = f"{prf:,}Hz({'EXT' if prf_src==1 else 'INT'})" if prf else "-(EXT)"
        dw_str  = f"{dw}ns" if dw else "-"
        pw_str  = f"{pw:.2f}W" if pw else "-"
        amp_str = f"{amps:.2f}A" if amps else "-"
        t_str   = f"SHG:{shg:.1f}°C " if shg else ""
        t_str  += f"THG:{thg:.1f}°C"  if thg else ""
        print(f"\r  {st['state_str']:<10}  "
              f"Sht:{'ON' if st['shutter'] else 'off':<4}  "
              f"LDD:{'ON' if st['ldd'] else 'off':<4}  "
              f"I:{amp_str:<8}  P:{pw_str:<8}  "
              f"PRF:{prf_str:<18}  DW:{dw_str:<10}  "
              f"{t_str}    ", end="", flush=True)

# ---- REPL ----

HELP = """
Commands:
  status       Full status + power output
  poll         Complete diagnostic dump
  go           Quick enable sequence
  startup      Guided startup
  shutdown     Safe shutdown
  monitor      Live display (Ctrl-C to exit)
  shutter on|off
  ldd on|off
  current <A>  (max 7.2A)
  raw <hex>    Send raw HEXASC
  reset
  q / quit     Exit (laser state preserved - shutdown first if needed)
"""

## @brief Documents main.
##
def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbserial-3130"
    print(f"\nPhotonics Industries DX-355-5-WC  |  {port} @ {BAUD} 8N1\n")
    try:
        ser = serial.Serial(port=port, baudrate=BAUD,
                            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                            stopbits=serial.STOPBITS_ONE, timeout=1.0,
                            dsrdtr=False, rtscts=False)
        ser.dtr = True; ser.rts = True; time.sleep(0.1)
    except serial.SerialException as e:
        print(f"ERROR: {e}")
        for p in serial.tools.list_ports.comports():
            print(f"  {p.device}  {p.description}")
        sys.exit(1)

    print("Connected. Reading state...\n")
    time.sleep(0.2)
    print(fmt_status(ser))
    print(HELP)

    mon = Monitor(ser)

    try:
        while True:
            try:
                line = input("laser> ").strip()
            except EOFError:
                break
            if not line: continue
            parts = line.split()
            cmd   = parts[0].lower()

            mon.pause()
            try:
                if cmd in ("q","quit","exit"):
                    st = read_status(ser)
                    if st and (st['shutter'] or st['ldd']):
                        print("WARNING: Laser is active (shutter/LDD on). Exiting without shutdown.")
                        print("  Run 'shutdown' first if you want to bring it down safely.")
                    break
                elif cmd == "help":       print(HELP)
                elif cmd == "status":     print(fmt_status(ser))
                elif cmd == "poll":       print(fmt_poll(poll_all(ser)))
                elif cmd == "go":         cmd_go(ser)
                elif cmd == "startup":    cmd_startup(ser)
                elif cmd == "shutdown":   cmd_shutdown(ser)
                elif cmd == "monitor":
                    print("Live monitor - Ctrl-C to exit\n")
                    try:
                        while True:
                            mon.display(); time.sleep(1)
                    except KeyboardInterrupt:
                        print("\n")
                elif cmd == "shutter" and len(parts) == 2:
                    en = parts[1].lower() in ("on","open","1")
                    if en:
                        ok, r = open_shutter(ser)
                        print(f"{'OK' if ok else 'ERR'}  shutter → ON  ({r})")
                    else:
                        ok,_,r = _write(ser, 0x04, [0x00])
                        print(f"{'OK' if ok else 'ERR'}  shutter → OFF  ({r})")
                elif cmd == "ldd" and len(parts) == 2:
                    en = parts[1].lower() in ("on","1")
                    ok,_,r = _write(ser, 0x10, [0x01 if en else 0x00])
                    print(f"{'OK' if ok else 'ERR'}  LDD → {'ON' if en else 'off'}  ({r})")
                elif cmd == "current" and len(parts) == 2:
                    a = float(parts[1])
                    if a > MAX_CURRENT:
                        print(f"ERR: max is {MAX_CURRENT}A")
                    else:
                        v = int(round(a*100))
                        ok,_,r = _write(ser, 0x11, [(v>>8)&0xFF, v&0xFF])
                        print(f"{'OK' if ok else 'ERR'}  current → {a:.2f}A  ({r})")
                elif cmd == "reset":
                    _write(ser, 0x07, []); print("Reset sent.")
                elif cmd == "raw" and len(parts) >= 2:
                    h = parts[1].replace(" ","").upper()
                    if len(h)%2: print("ERR: odd hex chars"); continue
                    rb = list(bytes.fromhex(h))
                    full = h + f"{sum(rb)&0xFFFF:04X}"
                    r = send_cmd(ser, full)
                    ok,_,_,data = parse_response(r)
                    print(f"TX: {full}  RX: {r}  ({'OK' if ok else 'ERR'})  data={[hex(x) for x in data]}")
                else:
                    print(f"Unknown: '{line}'. Type 'help'.")
            except Exception as e:
                print(f"ERR: {e}")
            finally:
                mon.resume()

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        mon.stop()
        ser.close()
        print("Port closed.")

if __name__ == "__main__":
    main()

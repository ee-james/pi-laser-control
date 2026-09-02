#!/usr/bin/env python3
"""
Photonics Industries DX-355-5-WC Laser Control GUI
Replaces PI Control Software v5
Dark theme: cyan / green / maroon / blue / light gray
"""

import sys
import time
import threading
import tkinter as tk
from tkinter import ttk, font as tkfont
import random
import serial
import serial.tools.list_ports

# ---- Protocol layer ----
# copied in from pi_laser_ctrl.py so the GUI runs standalone

BAUD        = 115200
MAX_CURRENT = 7.2

TEMP_SETPOINTS = {
    'SHG': 49.75,
    'THG': 69.70,
}
TEMP_ALARM_DELTA = 2.0
POLL_INTERVAL    = 1.0

SYS_STATES = {
    0: 'Initializing',
    1: 'Ramping',
    2: 'Running',
    3: 'Standby',
    4: 'Soft Fault',
    5: 'Hard Fault',
}

# ---- Colors ----
C = {
    'bg':           '#0D0D0D',
    'bg2':          '#141414',
    'bg3':          '#1C1C1C',
    'panel':        '#111111',
    'border':       '#2A2A2A',
    'cyan':         '#00E5FF',
    'cyan_dim':     '#007A8C',
    'green':        '#00CC55',
    'green_dim':    '#006628',
    'red':          '#8B0000',
    'red_bright':   '#CC2200',
    'blue':         '#3399FF',
    'blue_dim':     '#1A4A7A',
    'gray':         '#AAAAAA',
    'gray_dim':     '#555555',
    'white':        '#E8E8E8',
    'yellow':       '#FFCC00',
    'off':          '#333333',
}

# ---- Protocol ----

def build_frame(rw, nn, aa, cc, data):
    frame_bytes = [rw, nn, aa, cc] + data
    cs = sum(frame_bytes) & 0xFFFF
    return "".join(f"{b:02X}" for b in frame_bytes) + f"{cs:04X}"

def send_cmd(ser, hexstr, timeout=1.0):
    wire = (hexstr + "\r").encode("ascii")
    ser.reset_input_buffer()
    ser.write(wire)
    deadline = time.time() + timeout
    buf = b""
    while time.time() < deadline:
        b = ser.read(1)
        if b in (b"\r", b"\n"):
            if buf:
                break
        elif b:
            buf += b
    return buf.decode("ascii", errors="replace").strip()

def parse_response(resp):
    if len(resp) < 10 or len(resp) % 2 != 0:
        return False, 0, 0, []
    try:
        raw = bytes.fromhex(resp)
    except ValueError:
        return False, 0, 0, []
    rw = raw[0]; nn = raw[1]; cc = raw[3]
    if len(raw) != 4 + nn + 2:
        return False, rw, cc, []
    data = list(raw[4:4+nn])
    cs_recv = (raw[-2] << 8) | raw[-1]
    cs_calc  = sum(raw[:-2]) & 0xFFFF
    ok = (cs_recv == cs_calc) and rw in (0x02, 0x03)
    return ok, rw, cc, data

def _write(ser, cc, data):
    f = build_frame(0x00, len(data), 0x00, cc, data)
    r = send_cmd(ser, f)
    ok, _, _, d = parse_response(r)
    return ok, d, r

def _read(ser, cc, aa=0x00):
    f = build_frame(0x01, 0x00, aa, cc, [])
    r = send_cmd(ser, f)
    ok, _, _, d = parse_response(r)
    return ok, d, r

def read_status(ser):
    ok, data, _ = _read(ser, 0x0F)
    if not ok or len(data) < 5:
        return None
    d1, d2, d3, d4, d5 = data[:5]
    return {
        'power_on':      bool(d1 & 0x01),
        'shutter':       bool(d1 & 0x02),
        'key_switch':    bool(d1 & 0x04),
        'ldd':           bool(d1 & 0x08),
        'qsw':           bool(d1 & 0x10),
        'shutter_ilock': bool(d1 & 0x20),
        'ldd_ilock':     bool(d1 & 0x40),
        'sys_faults':    d2,
        'gen_alarms':    d3,
        'temp_alarms':   d4,
        'state':         d5,
        'state_str':     SYS_STATES.get(d5, f'Unknown({d5})'),
        # Gen alarm bits decoded
        'alarm_ldd1':    bool(d3 & 0x01),
        'alarm_ldd2':    bool(d3 & 0x02),
        'alarm_qsw':     bool(d3 & 0x04),
        'alarm_flow':    bool(d3 & 0x08),
        'alarm_wet':     bool(d3 & 0x10),
        'alarm_humidity':bool(d3 & 0x20),
        'alarm_hfsync':  bool(d3 & 0x40),
        'alarm_pwrmon':  bool(d3 & 0x80),
    }

def read_2byte(ser, cc, scale=100.0):
    """Generic 2-byte scaled read. Returns float or None."""
    ok, data, _ = _read(ser, cc)
    if not ok or len(data) < 2:
        return None
    return ((data[0] << 8) | data[1]) / scale

def read_actual_current(ser):
    return read_2byte(ser, 0x12, 100.0)

def read_set_current(ser):
    return read_2byte(ser, 0x11, 100.0)

def read_max_current(ser):
    return read_2byte(ser, 0x14, 100.0)

def read_diode_voltage(ser):
    return read_2byte(ser, 0x13, 100.0)

def read_temp(ser, aa):
    ok, data, _ = _read(ser, 0x32, aa)
    if not ok or len(data) < 2:
        return None
    return ((data[0] << 8) | data[1]) / 100.0

def read_prf(ser):
    """Returns PRF in Hz (measured or set depending on source mode)."""
    ok, data, _ = _read(ser, 0x21)
    if not ok or len(data) < 3:
        return None
    return (data[0] << 16) | (data[1] << 8) | data[2]

def read_prf_source(ser):
    """Returns 0=INT, 1=EXT or None."""
    ok, data, _ = _read(ser, 0x23)
    if not ok or len(data) < 1:
        return None
    return data[0]

def read_gate_source(ser):
    """Returns 0=INT, 1=EXT or None."""
    ok, data, _ = _read(ser, 0x24)
    if not ok or len(data) < 1:
        return None
    return data[0]

def read_duty_mode(ser):
    """Returns 0=Fixed, 1=Track or None."""
    ok, data, _ = _read(ser, 0x69)
    if not ok or len(data) < 1:
        return None
    return data[0]

def read_duty_width(ser):
    ok, data, _ = _read(ser, 0x54)
    if not ok or len(data) < 3:
        return None
    return (data[0] << 16) | (data[1] << 8) | data[2]

def read_flow(ser):
    """Returns flow rate in L/min or None."""
    ok, data, _ = _read(ser, 0x74)
    if not ok or len(data) < 2:
        return None
    return ((data[0] << 8) | data[1]) / 10.0

def read_power_meter(ser):
    """0x15 Power Monitor - laser returns 4 bytes, units 0.01W.
    Likely LDD electrical input power, not optical output."""
    ok, data, _ = _read(ser, 0x15)
    if not ok or len(data) < 2:
        return None
    if len(data) >= 4:
        val = (data[0]<<24)|(data[1]<<16)|(data[2]<<8)|data[3]
    else:
        val = (data[0]<<8)|data[1]
    return val / 100.0

def read_prf_ext(ser):
    """0x28 PRF Actual EXT - LMCV4 clock as measured at laser input.
    Represents the LightBurn-requested fire rate entering the DX."""
    ok, data, _ = _read(ser, 0x28)
    if not ok or len(data) < 2:
        return None
    if len(data) >= 3:
        return (data[0]<<16)|(data[1]<<8)|data[2]
    return (data[0]<<8)|data[1]

def read_prf_actual(ser):
    """0x22 PRF Actual - gate-qualified confirmed pulse count.
    Delta vs prf_ext reveals missed/rejected Q-switch events."""
    ok, data, _ = _read(ser, 0x22)
    if not ok or len(data) < 2:
        return None
    if len(data) >= 3:
        return (data[0]<<16)|(data[1]<<8)|data[2]
    return (data[0]<<8)|data[1]

def read_pulse_energy(ser):
    """0x8C Pulse Energy - 0.01 µJ/count. Valid only when LDD on and firing."""
    ok, data, _ = _read(ser, 0x8C)
    if not ok or len(data) < 4:
        return None
    val = (data[0]<<24)|(data[1]<<16)|(data[2]<<8)|data[3]
    return val / 100.0  # µJ

def read_soft_faults(ser):
    """Returns dict of soft fault bits or None."""
    ok, data, _ = _read(ser, 0x0C)
    if not ok or len(data) < 2:
        return None
    d2 = data[1]
    return {
        'ldd_interlock':    bool(d2 & 0x02),
        'shutter_interlock':bool(d2 & 0x04),
        'low_ext_freq':     bool(d2 & 0x08),
        'high_ext_freq':    bool(d2 & 0x10),
        'raw':              d2,
    }

def read_board_faults(ser):
    """Returns dict of board fault bits or None."""
    ok, data, _ = _read(ser, 0x0D)
    if not ok or len(data) < 2:
        return None
    d2 = data[1]
    return {
        'main':          bool(d2 & 0x01),
        'common':        bool(d2 & 0x02),
        'temperature':   bool(d2 & 0x04),
        'sensor':        bool(d2 & 0x08),
        'power_monitor': bool(d2 & 0x10),
        'pulse':         bool(d2 & 0x20),
        'ldd':           bool(d2 & 0x40),
        'motor':         bool(d2 & 0x80),
        'raw':           d2,
    }

def _read_single_byte(ser, cc):
    """Read a single-byte value, return int or None."""
    ok, data, _ = _read(ser, cc)
    return data[0] if ok and data else None

def poll_all(ser):
    # read everything and stuff it in a dict. never raises - a failed read
    # just lands as None for that key.
    d = {}
    d['status']        = read_status(ser)
    d['set_current']   = read_set_current(ser)
    d['act_current']   = read_actual_current(ser)
    d['max_current']   = read_max_current(ser)
    d['diode_voltage'] = read_diode_voltage(ser)
    d['temp_shg']      = read_temp(ser, 0x00)
    d['temp_thg']      = read_temp(ser, 0x01)
    d['temp_cb']       = read_temp(ser, 0x03)  # 0x32 AA=0x03 - confirmed CB on this unit
    d['prf']           = read_prf(ser)
    d['prf_source']    = read_prf_source(ser)
    d['gate_source']   = read_gate_source(ser)
    d['duty_mode']     = read_duty_mode(ser)
    d['duty_width']    = read_duty_width(ser)
    d['flow']          = read_flow(ser)
    d['power_w']       = read_power_meter(ser)
    d['prf_ext']       = read_prf_ext(ser)       # 0x28 LMCV4 clock at laser input
    d['prf_actual']    = read_prf_actual(ser)     # 0x22 confirmed fires
    d['pulse_energy']  = read_pulse_energy(ser)   # 0x8C µJ per pulse
    d['pulsed_op']     = _read_single_byte(ser, 0x20)
    d['burst']         = _read_single_byte(ser, 0x56)
    d['soft_faults']   = read_soft_faults(ser)
    d['board_faults']  = read_board_faults(ser)
    return d

def print_poll_all(ser):
    # full diagnostic dump, formatted for a terminal
    d = poll_all(ser)
    st = d['status']
    lines = ["", "=" * 56, "  DX-355-5-WC  FULL DIAGNOSTIC POLL", "=" * 56]

    # System state
    if st:
        lines += [
            f"  System State    : {st['state_str']}",
            f"  Power On        : {'YES' if st['power_on'] else 'no'}",
            f"  Key Switch      : {'ON' if st['key_switch'] else 'OFF'}",
            f"  Shutter         : {'OPEN' if st['shutter'] else 'closed'}",
            f"  LDD             : {'ON' if st['ldd'] else 'off'}",
            f"  QSW             : {'ON' if st['qsw'] else 'off'}",
            f"  Shutter Ilock   : {'OK' if st['shutter_ilock'] else 'FAULT'}",
            f"  LDD Interlock   : {'OK' if st['ldd_ilock'] else 'FAULT'}",
        ]
        # Fault bytes
        if st['sys_faults']:
            lines.append(f"  ⚠ Sys Faults    : 0x{st['sys_faults']:02X}")
        # Gen alarms decoded
        alarm_names = [
            ('alarm_ldd1',    'LDD1'),
            ('alarm_ldd2',    'LDD2'),
            ('alarm_qsw',     'QSW'),
            ('alarm_flow',    'Flow'),
            ('alarm_wet',     'Wet'),
            ('alarm_humidity','Humidity'),
            ('alarm_hfsync',  'HF Sync'),
            ('alarm_pwrmon',  'Power Monitor'),
        ]
        active = [name for key, name in alarm_names if st.get(key)]
        if active:
            lines.append(f"  ⚠ Gen Alarms    : {', '.join(active)}")
        if st['temp_alarms']:
            lines.append(f"  ⚠ Temp Alarms   : 0x{st['temp_alarms']:02X}")
    else:
        lines.append("  Status read FAILED")

    lines.append("-" * 56)

    # Current / voltage
    lines += [
        f"  Set Current     : {d['set_current']:.2f} A"    if d['set_current']   is not None else "  Set Current     : N/A",
        f"  Actual Current  : {d['act_current']:.2f} A"    if d['act_current']   is not None else "  Actual Current  : N/A",
        f"  Max Current     : {d['max_current']:.2f} A"    if d['max_current']   is not None else "  Max Current     : N/A",
        f"  Diode Voltage   : {d['diode_voltage']:.2f} V"  if d['diode_voltage'] is not None else "  Diode Voltage   : N/A",
    ]

    lines.append("-" * 56)

    # Temperatures
    temp_names = [('temp_shg', 'SHG', TEMP_SETPOINTS.get('SHG')),
                  ('temp_thg', 'THG', TEMP_SETPOINTS.get('THG')),]
    for key, name, sp in temp_names:
        val = d[key]
        if val is not None:
            delta = abs(val - sp) if sp else 0
            flag  = "  ⚠ ALARM" if delta > TEMP_ALARM_DELTA else ""
            lines.append(f"  {name} Temp        : {val:.2f}°C  (setpoint {sp:.2f}°C){flag}")
        else:
            lines.append(f"  {name} Temp        : N/A")
    lines.append(f"  CB Temp         : {d['temp_cb']:.2f}°C" if d.get('temp_cb') is not None else "  CB Temp         : N/A")

    lines.append("-" * 56)

    # Pulse
    prf_src = {0: 'INT', 1: 'EXT'}.get(d['prf_source'], '?')
    gate_src = {0: 'INT', 1: 'EXT'}.get(d['gate_source'], '?')
    duty_mode = {0: 'Fixed', 1: 'Track'}.get(d['duty_mode'], '?')
    prf_val = f"{d['prf']:,} Hz" if d['prf'] else "0 (EXT mode - value from LMCV4)"
    lines += [
        f"  PRF             : {prf_val}",
        f"  PRF Source      : {prf_src}",
        f"  Gate Source     : {gate_src}",
        f"  Duty Mode       : {duty_mode}",
        f"  Duty Width      : {d['duty_width']} ns" if d['duty_width'] is not None else "  Duty Width      : N/A",
    ]

    lines.append("-" * 56)

    # Monitors
    flow = d['flow']
    lines.append(f"  Flow Rate       : {flow:.1f} L/min" if flow is not None else "  Flow Rate       : N/A (no meter)")
    pw = d.get('power_w')
    lines.append(f"  Power Output    : {pw:.2f} W" if pw is not None else "  Power Output    : N/A")
    lines.append(f"  Humidity Alarm  : {'ACTIVE' if st and st.get('alarm_humidity') else 'clear'}")

    # Soft/board faults
    sf = d['soft_faults']
    bf = d['board_faults']
    if sf and sf['raw']:
        active = [k for k, v in sf.items() if k != 'raw' and v]
        lines.append(f"  Soft Faults     : {', '.join(active)}")
    else:
        lines.append(f"  Soft Faults     : none")
    if bf and bf['raw']:
        active = [k for k, v in bf.items() if k != 'raw' and v]
        lines.append(f"  Board Faults    : {', '.join(active)}")
    else:
        lines.append(f"  Board Faults    : none")

    lines += ["=" * 56, ""]
    print("\n".join(lines))


# ---- Simulator ----

SIM_MODE = False

class SimState:
    """Mutable laser state for simulation."""
    def __init__(self):
        self.shutter       = False
        self.ldd           = False
        self.key_switch    = True
        self.shutter_ilock = True
        self.ldd_ilock     = True
        self.current_set   = 0.0
        self.current_act   = 0.0
        self.max_current   = MAX_CURRENT
        self.state         = 3   # Standby
        self.prf           = 0
        self.duty_width    = 19000
        self.duty_mode     = 0
        self.shg           = 49.75
        self.thg           = 69.70
        self.sys_faults    = 0
        self.gen_alarms    = 0
        self.temp_alarms   = 0

_sim = SimState()

def _sim_drift(val, target, rate=0.05):
    """Drift val toward target with small noise."""
    val += (target - val) * rate
    val += random.uniform(-0.02, 0.02)
    return round(val, 2)

def sim_read_status():
    _sim.shg = _sim_drift(_sim.shg, TEMP_SETPOINTS['SHG'])
    _sim.thg = _sim_drift(_sim.thg, TEMP_SETPOINTS['THG'])
    if _sim.ldd:
        _sim.current_act = _sim_drift(_sim.current_act, _sim.current_set)
        _sim.state = 2
    else:
        _sim.current_act = _sim_drift(_sim.current_act, 0.0, rate=0.2)
        _sim.state = 3 if not _sim.shutter else 2
    return {
        'power_on':      True,
        'shutter':       _sim.shutter,
        'key_switch':    _sim.key_switch,
        'ldd':           _sim.ldd,
        'qsw':           _sim.ldd,
        'shutter_ilock': _sim.shutter_ilock,
        'ldd_ilock':     _sim.ldd_ilock,
        'sys_faults':    _sim.sys_faults,
        'gen_alarms':    _sim.gen_alarms,
        'temp_alarms':   _sim.temp_alarms,
        'state':         _sim.state,
        'state_str':     SYS_STATES.get(_sim.state, 'Unknown'),
    }

def sim_write(cc, data):
    if cc == 0x04:   # shutter
        if data[0] == 0x01:
            _sim.shutter = True
        else:
            _sim.shutter = False
        return True
    elif cc == 0x10:  # LDD
        _sim.ldd = bool(data[0])
        return True
    elif cc == 0x11:  # set current
        val = ((data[0] << 8) | data[1]) / 100.0
        if val <= _sim.max_current:
            _sim.current_set = val
            return True
        return False
    elif cc == 0x14:  # max current
        _sim.max_current = ((data[0] << 8) | data[1]) / 100.0
        return True
    elif cc in (0x07, 0x20, 0x23, 0x24, 0x56, 0x69):
        return True   # accept silently
    elif cc == 0x54:  # duty width
        _sim.duty_width = (data[0] << 16) | (data[1] << 8) | data[2]
        return True
    return False

# ---- GUI ----

class LaserGUI:
    def __init__(self, root):
        self.root   = root
        self.ser    = None
        self._lock  = threading.Lock()
        self._poll_running = False
        self._last_status  = None
        self._alarms_silenced = False
        self._active_alarms   = set()
        self._ctrl_lockout    = {}   # {var_name: expiry_time} - suppress poll overwrite
        self._poll_interval   = 1.0   # seconds, controlled by UI

        root.title("DX-355-5-WC Laser Control")
        root.configure(bg=C['bg'])
        root.resizable(False, False)

        self._build_fonts()
        self._build_ui()
        self._scan_ports()

    def _build_fonts(self):
        self.f_title   = tkfont.Font(family="Helvetica Neue", size=13, weight="bold")
        self.f_label   = tkfont.Font(family="Helvetica Neue", size=10, weight="bold")
        self.f_value   = tkfont.Font(family="Courier New",    size=11, weight="bold")
        self.f_big     = tkfont.Font(family="Helvetica Neue", size=14, weight="bold")
        self.f_btn     = tkfont.Font(family="Helvetica Neue", size=11, weight="bold")
        self.f_status  = tkfont.Font(family="Helvetica Neue", size=9,  weight="bold")
        self.f_small   = tkfont.Font(family="Courier New",    size=9)

    def _panel(self, parent, title, col, row, colspan=1, rowspan=1, padx=4, pady=4):
        outer = tk.Frame(parent, bg=C['border'], padx=1, pady=1)
        outer.grid(column=col, row=row, columnspan=colspan, rowspan=rowspan,
                   padx=padx, pady=pady, sticky='nsew')
        inner = tk.Frame(outer, bg=C['panel'], padx=8, pady=6)
        inner.pack(fill='both', expand=True)
        if title:
            tk.Label(inner, text=title, font=self.f_label,
                     bg=C['panel'], fg=C['cyan']).pack(anchor='w', pady=(0,4))
        # Return a sub-frame so callers can freely mix grid without
        # conflicting with the pack'd title label above
        content = tk.Frame(inner, bg=C['panel'])
        content.pack(fill='both', expand=True)
        return content

    def _led(self, parent, label, row):
        fr = tk.Frame(parent, bg=C['panel'])
        fr.grid(row=row, column=0, sticky='w', pady=1)
        canvas = tk.Canvas(fr, width=12, height=12, bg=C['panel'],
                           highlightthickness=0)
        canvas.pack(side='left', padx=(0,5))
        lbl = tk.Label(fr, text=label, font=self.f_status,
                       bg=C['panel'], fg=C['gray'])
        lbl.pack(side='left')
        return canvas

    def _draw_led(self, canvas, color):
        canvas.delete('all')
        canvas.create_oval(2, 2, 10, 10, fill=color, outline=color)

    def _make_button(self, parent, text, color, command, width=None):
        """
        macOS-safe button using Label + bindings.
        tk.Button ignores bg/fg on macOS native theme.
        """
        cfg = dict(
            font=self.f_btn,
            bg=color, fg=C['white'],
            relief='flat', cursor='hand2',
            padx=10, pady=6,
        )
        if width:
            cfg['width'] = width
        btn = tk.Label(parent, text=text, **cfg)
        btn._base_color = color
        btn._command    = command

        def on_enter(e):
            btn.config(bg=self._brighten(color))
        def on_leave(e):
            btn.config(bg=btn._base_color)
        def on_click(e):
            btn._command()

        btn.bind('<Enter>',          on_enter)
        btn.bind('<Leave>',          on_leave)
        btn.bind('<ButtonRelease-1>', on_click)
        return btn

    def _brighten(self, hex_color):
        """Lighten a hex color by ~30% for hover effect."""
        h = hex_color.lstrip('#')
        r, g, b = int(h[0:2],16), int(h[2:4],16), int(h[4:6],16)
        r = min(255, int(r * 1.35))
        g = min(255, int(g * 1.35))
        b = min(255, int(b * 1.35))
        return f'#{r:02X}{g:02X}{b:02X}'

    def _build_ui(self):
        root = self.root

        # ---- Title bar ----
        top = tk.Frame(root, bg=C['bg2'], pady=6)
        top.pack(fill='x')
        tk.Label(top, text="PHOTONICS INDUSTRIES", font=self.f_title,
                 bg=C['bg2'], fg=C['cyan']).pack(side='left', padx=12)
        tk.Label(top, text="DX-355-5-WC  Laser Control Interface",
                 font=self.f_title, bg=C['bg2'], fg=C['gray']).pack(side='left')

        self.lbl_conn = tk.Label(top, text="● NOT CONNECTED",
                                 font=self.f_status, bg=C['bg2'], fg=C['red_bright'])
        self.lbl_conn.pack(side='right', padx=12)

        # ---- Connection bar ----
        conn = tk.Frame(root, bg=C['bg3'], pady=4)
        conn.pack(fill='x', padx=4)

        tk.Label(conn, text="PORT", font=self.f_label,
                 bg=C['bg3'], fg=C['gray']).pack(side='left', padx=6)
        self.port_var = tk.StringVar()
        self.port_cb  = ttk.Combobox(conn, textvariable=self.port_var, width=28,
                                     font=self.f_small)
        self.port_cb.pack(side='left', padx=4)

        self.btn_connect = self._make_button(
            conn, "CONNECT", C['blue_dim'], self._on_connect)
        self.btn_connect.pack(side='left', padx=6)

        self.lbl_state = tk.Label(conn, text="-", font=self.f_label,
                                  bg=C['bg3'], fg=C['cyan'])
        self.lbl_state.pack(side='left', padx=10)

        tk.Label(conn, text="POLL", font=self.f_label,
                 bg=C['bg3'], fg=C['gray']).pack(side='left', padx=(20,4))
        self.var_poll_rate = tk.StringVar(value="1s")
        poll_rates = ["1s", "10s", "30s", "1min", "OFF"]
        self.cb_poll_rate = ttk.Combobox(conn, textvariable=self.var_poll_rate,
                                          values=poll_rates, width=6,
                                          font=self.f_small, state='readonly')
        self.cb_poll_rate.pack(side='left')
        self.cb_poll_rate.bind('<<ComboboxSelected>>', self._on_poll_rate_change)

        self.btn_poll_now = self._make_button(
            conn, "POLL NOW", C['blue_dim'], self._on_poll_now)
        self.btn_poll_now.pack(side='left', padx=6)

        # ---- Main grid ----
        main = tk.Frame(root, bg=C['bg'])
        main.pack(fill='both', expand=True, padx=4, pady=4)

        # col 0: status sidebar
        # col 1: power control
        # col 2: temperature
        # col 3: pulse control

        # ---- Status sidebar ----
        sp = self._panel(main, "SYSTEM STATUS", 0, 0, rowspan=2)
        sp.grid_propagate(False)

        self.led_power   = self._led(sp, "Power",            0)
        self.led_shutter = self._led(sp, "Shutter",          1)
        self.led_key     = self._led(sp, "Key Switch",       2)
        self.led_ldd     = self._led(sp, "LDD",              3)
        self.led_qsw     = self._led(sp, "QSW",              4)
        self.led_pulse   = self._led(sp, "Pulsed Op",        5)
        self.led_silock  = self._led(sp, "Shutter Interlock",6)
        self.led_lilock  = self._led(sp, "LDD Interlock",    7)

        tk.Frame(sp, bg=C['border'], height=1).grid(
            row=8, column=0, sticky='ew', pady=6)

        self.lbl_alarms = tk.Label(sp, text="", font=self.f_status,
                                   bg=C['panel'], fg=C['red_bright'],
                                   wraplength=130, justify='left')
        self.lbl_alarms.grid(row=9, column=0, sticky='w')

        self.btn_silence = self._make_button(
            sp, "SILENCE", C['bg3'], self._on_silence_alarms)
        self.btn_silence.grid(row=10, column=0, sticky='ew', pady=(6,0))

        # ---- Power control ----
        pp = self._panel(main, "POWER CONTROL  -  LDD1", 1, 0)

        def val_row(parent, label, row, color=C['cyan']):
            tk.Label(parent, text=label, font=self.f_label,
                     bg=C['panel'], fg=C['gray']).grid(
                         row=row, column=0, sticky='w', pady=2)
            lbl = tk.Label(parent, text="-", font=self.f_value,
                           bg=C['panel'], fg=color, width=10, anchor='e')
            lbl.grid(row=row, column=1, sticky='e', padx=(10,0))
            return lbl

        self.lbl_set_i    = val_row(pp, "Set Current (A)",    0)
        self.lbl_act_i    = val_row(pp, "Actual Current (A)", 1, C['green'])
        self.lbl_max_i    = val_row(pp, "Max Current (A)",    2, C['gray'])

        tk.Frame(pp, bg=C['border'], height=1).grid(
            row=3, column=0, columnspan=2, sticky='ew', pady=4)

        tk.Label(pp, text="LDD Status", font=self.f_label,
                 bg=C['panel'], fg=C['gray']).grid(row=4, column=0, sticky='w')
        self.lbl_ldd_status = tk.Label(pp, text="LDD OFF", font=self.f_big,
                                       bg=C['red'], fg=C['white'],
                                       width=9, anchor='center', relief='flat')
        self.lbl_ldd_status.grid(row=4, column=1, sticky='e', padx=(10,0))

        tk.Label(pp, text="Shutter Status", font=self.f_label,
                 bg=C['panel'], fg=C['gray']).grid(row=5, column=0, sticky='w', pady=(4,0))
        self.lbl_shutter_status = tk.Label(pp, text="CLOSED", font=self.f_big,
                                           bg=C['off'], fg=C['gray'],
                                           width=9, anchor='center', relief='flat')
        self.lbl_shutter_status.grid(row=5, column=1, sticky='e', padx=(10,0), pady=(4,0))

        tk.Frame(pp, bg=C['border'], height=1).grid(
            row=6, column=0, columnspan=2, sticky='ew', pady=4)

        self.lbl_power_out = tk.Label(pp, text="Power:  - mW",
                                      font=self.f_big, bg=C['panel'], fg=C['cyan'])
        self.lbl_power_out.grid(row=7, column=0, columnspan=2, sticky='w')

        # Current set entry
        tk.Frame(pp, bg=C['border'], height=1).grid(
            row=8, column=0, columnspan=2, sticky='ew', pady=4)
        tk.Label(pp, text="Set Current →", font=self.f_label,
                 bg=C['panel'], fg=C['gray']).grid(row=9, column=0, sticky='w')
        self.entry_current = tk.Entry(pp, font=self.f_value, width=7,
                                      bg=C['bg2'], fg=C['cyan'],
                                      insertbackground=C['cyan'],
                                      relief='flat', justify='right')
        self.entry_current.insert(0, str(MAX_CURRENT))
        self.entry_current.grid(row=9, column=1, sticky='e', padx=(10,0))
        self.entry_current.bind('<Return>', self._on_set_current)

        # ---- Temperature ----
        tp = self._panel(main, "TEMPERATURE CONTROLS", 2, 0)

        tk.Label(tp, text="", bg=C['panel']).grid(row=0, column=0)
        tk.Label(tp, text="SET", font=self.f_label,
                 bg=C['panel'], fg=C['gray']).grid(row=0, column=1, padx=8)
        tk.Label(tp, text="ACTUAL", font=self.f_label,
                 bg=C['panel'], fg=C['gray']).grid(row=0, column=2, padx=8)

        self.temp_labels = {}
        for i, name in enumerate(['SHG', 'THG', 'CB']):
            sp_val = TEMP_SETPOINTS.get(name, '-')
            tk.Label(tp, text=name, font=self.f_label,
                     bg=C['panel'], fg=C['gray']).grid(
                         row=i+1, column=0, sticky='w', pady=3)
            tk.Label(tp, text=f"{sp_val:.2f}" if isinstance(sp_val, float) else "-",
                     font=self.f_value, bg=C['panel'],
                     fg=C['gray']).grid(row=i+1, column=1, padx=8)
            lbl = tk.Label(tp, text="-", font=self.f_value,
                           bg=C['panel'], fg=C['cyan'], width=8, anchor='e')
            lbl.grid(row=i+1, column=2, padx=8)
            self.temp_labels[name] = lbl

        # ---- Pulse control ----
        pulp = self._panel(main, "PULSE CONTROL", 1, 1, colspan=2)

        def pulse_row(parent, label, row, val="-", color=C['cyan']):
            tk.Label(parent, text=label, font=self.f_label,
                     bg=C['panel'], fg=C['gray']).grid(
                         row=row, column=0, sticky='w', pady=2, padx=(0,12))
            lbl = tk.Label(parent, text=val, font=self.f_value,
                           bg=C['panel'], fg=color, width=14, anchor='w')
            lbl.grid(row=row, column=1, sticky='w')
            return lbl

        # PRF - Q-pulse chain: LMCV4 in (0x28) → confirmed fires (0x22) → delta
        self.lbl_prf = pulse_row(pulp, "Q-Pulse Chain", 0, color=C['cyan'])

        # Pulse Energy - 0x8C, valid when LDD on and firing
        self.lbl_pulse_energy = pulse_row(pulp, "Pulse Energy", 1, color=C['cyan'])

        # PRF Source selector
        tk.Label(pulp, text="PRF Source", font=self.f_label,
                 bg=C['panel'], fg=C['gray']).grid(row=2, column=0, sticky='w', pady=2)
        self.var_prf_src = tk.StringVar(value="EXT")
        cb_prf = ttk.Combobox(pulp, textvariable=self.var_prf_src,
                               values=["INT", "EXT"], width=8,
                               font=self.f_value, state='readonly')
        cb_prf.grid(row=2, column=1, sticky='w', padx=(10,0))
        cb_prf.bind('<<ComboboxSelected>>',
                    lambda e: self._on_set_prf_source(self.var_prf_src.get()))
        self.lbl_prf_src_act = tk.Label(pulp, text='', font=self.f_value,
                                         bg=C['panel'], fg=C['gray_dim'], width=6)
        self.lbl_prf_src_act.grid(row=2, column=2, sticky='w', padx=(4,0))

        # Gate Source selector
        tk.Label(pulp, text="Gate Source", font=self.f_label,
                 bg=C['panel'], fg=C['gray']).grid(row=3, column=0, sticky='w', pady=2)
        self.var_gate_src = tk.StringVar(value="INT")
        cb_gate = ttk.Combobox(pulp, textvariable=self.var_gate_src,
                                values=["INT", "EXT"], width=8,
                                font=self.f_value, state='readonly')
        cb_gate.grid(row=3, column=1, sticky='w', padx=(10,0))
        cb_gate.bind('<<ComboboxSelected>>',
                     lambda e: self._on_set_gate_source(self.var_gate_src.get()))
        self.lbl_gate_src_act = tk.Label(pulp, text='', font=self.f_value,
                                          bg=C['panel'], fg=C['gray_dim'], width=6)
        self.lbl_gate_src_act.grid(row=3, column=2, sticky='w', padx=(4,0))

        # Duty Mode selector
        tk.Label(pulp, text="Duty Mode", font=self.f_label,
                 bg=C['panel'], fg=C['gray']).grid(row=4, column=0, sticky='w', pady=2)
        self.var_duty_mode = tk.StringVar(value="Fixed")
        cb_duty = ttk.Combobox(pulp, textvariable=self.var_duty_mode,
                                values=["Fixed", "Track"], width=8,
                                font=self.f_value, state='readonly')
        cb_duty.grid(row=4, column=1, sticky='w', padx=(10,0))
        cb_duty.bind('<<ComboboxSelected>>',
                     lambda e: self._on_set_duty_mode(self.var_duty_mode.get()))
        self.lbl_duty_mode_act = tk.Label(pulp, text='', font=self.f_value,
                                           bg=C['panel'], fg=C['gray_dim'], width=7)
        self.lbl_duty_mode_act.grid(row=4, column=2, sticky='w', padx=(4,0))

        # Duty Width entry - units match LightBurn "UV Min/Max Pulse (ns)"
        tk.Label(pulp, text="Duty Width (ns)", font=self.f_label,
                 bg=C['panel'], fg=C['gray']).grid(row=5, column=0, sticky='w', pady=2)
        self.entry_duty_width = tk.Entry(pulp, font=self.f_value, width=9,
                                          bg=C['bg2'], fg=C['cyan'],
                                          insertbackground=C['cyan'],
                                          relief='flat', justify='right')
        self.entry_duty_width.grid(row=5, column=1, sticky='w', padx=(10,0))
        self.entry_duty_width.bind('<Return>', self._on_set_duty_width)
        self.lbl_duty_width = tk.Label(pulp, text="-", font=self.f_value,
                                        bg=C['panel'], fg=C['gray_dim'])
        self.lbl_duty_width.grid(row=5, column=2, sticky='w', padx=(6,0))

        # PRF Set entry (used in INT mode)
        tk.Label(pulp, text="PRF Set (Hz)", font=self.f_label,
                 bg=C['panel'], fg=C['gray']).grid(row=6, column=0, sticky='w', pady=2)
        self.entry_prf_set = tk.Entry(pulp, font=self.f_value, width=9,
                                       bg=C['bg2'], fg=C['cyan'],
                                       insertbackground=C['cyan'],
                                       relief='flat', justify='right')
        self.entry_prf_set.grid(row=6, column=1, sticky='w', padx=(10,0))
        self.entry_prf_set.bind('<Return>', self._on_set_prf)

        # Pulsed Operation toggle
        tk.Label(pulp, text="Pulsed Op", font=self.f_label,
                 bg=C['panel'], fg=C['gray']).grid(row=7, column=0, sticky='w', pady=2)
        self.var_pulsed = tk.StringVar(value="Enabled")
        cb_pulsed = ttk.Combobox(pulp, textvariable=self.var_pulsed,
                                   values=["Enabled", "Disabled"], width=8,
                                   font=self.f_value, state='readonly')
        cb_pulsed.grid(row=7, column=1, sticky='w', padx=(10,0))
        cb_pulsed.bind('<<ComboboxSelected>>',
                       lambda e: self._on_set_pulsed(self.var_pulsed.get()))
        self.lbl_pulsed_act = tk.Label(pulp, text='', font=self.f_value,
                                        bg=C['panel'], fg=C['gray_dim'], width=9)
        self.lbl_pulsed_act.grid(row=7, column=2, sticky='w', padx=(4,0))

        # Burst toggle
        tk.Label(pulp, text="Burst", font=self.f_label,
                 bg=C['panel'], fg=C['gray']).grid(row=8, column=0, sticky='w', pady=2)
        self.var_burst = tk.StringVar(value="Off")
        cb_burst = ttk.Combobox(pulp, textvariable=self.var_burst,
                                  values=["Off", "On"], width=8,
                                  font=self.f_value, state='readonly')
        cb_burst.grid(row=8, column=1, sticky='w', padx=(10,0))
        cb_burst.bind('<<ComboboxSelected>>',
                      lambda e: self._on_set_burst(self.var_burst.get()))
        self.lbl_burst_act = tk.Label(pulp, text='', font=self.f_value,
                                       bg=C['panel'], fg=C['gray_dim'], width=5)
        self.lbl_burst_act.grid(row=8, column=2, sticky='w', padx=(4,0))

        # Flow (read-only)
        self.lbl_flow = pulse_row(pulp, "Flow Rate", 9)

        # ---- Action buttons ----
        btnp = self._panel(main, None, 3, 0, rowspan=2)

        def big_btn(parent, text, color, command, row):
            b = self._make_button(parent, text, color, command, width=12)
            b.grid(row=row, column=0, pady=4, sticky='ew')
            return b

        big_btn(btnp, "▶  GO",        C['green'],      self._on_go,           0)
        big_btn(btnp, "⬛  SHUTDOWN",  C['red'],        self._on_shutdown,     1)

        tk.Frame(btnp, bg=C['border'], height=1).grid(
            row=2, column=0, sticky='ew', pady=6)

        self.btn_shutter = self._make_button(
            btnp, "SHUTTER  ON", C['blue_dim'], self._on_shutter_toggle, width=12)
        self.btn_shutter.grid(row=3, column=0, pady=3, sticky='ew')

        self.btn_ldd = self._make_button(
            btnp, "LDD  ON", C['blue_dim'], self._on_ldd_toggle, width=12)
        self.btn_ldd.grid(row=4, column=0, pady=3, sticky='ew')

        tk.Frame(btnp, bg=C['border'], height=1).grid(
            row=5, column=0, sticky='ew', pady=6)

        big_btn(btnp, "⟳  RESET",     C['gray_dim'],   self._on_reset,        6)

        # ---- Status bar ----
        sbar = tk.Frame(root, bg=C['bg2'], pady=3)
        sbar.pack(fill='x', side='bottom')
        self.lbl_statusbar = tk.Label(sbar, text="Not connected.",
                                      font=self.f_small, bg=C['bg2'],
                                      fg=C['gray'], anchor='w')
        self.lbl_statusbar.pack(side='left', padx=8)

        # Initialize LED states
        self._reset_leds()

    # ---- Port scanning ----

    def _scan_ports(self):
        ports = ['SIMULATOR'] + [p.device for p in serial.tools.list_ports.comports()]
        self.port_cb['values'] = ports
        # Auto-select usbserial if present, else default to SIMULATOR
        for p in ports:
            if 'usbserial' in p.lower():
                self.port_var.set(p)
                return
        self.port_var.set('SIMULATOR')

    # ---- Connection ----

    def _on_connect(self):
        global SIM_MODE
        if SIM_MODE or (self.ser and self.ser.is_open):
            self._disconnect()
            return
        port = self.port_var.get()
        if port == "SIMULATOR":
            SIM_MODE = True
            self.btn_connect.config(text='DISCONNECT', bg=C['red'])
            self.btn_connect._base_color = C['red']
            self.lbl_conn.config(text="● SIM MODE", fg=C['yellow'])
            self._status("Simulator active - no hardware connected.")
            self._poll_running = True
            threading.Thread(target=self._poll_loop, daemon=True).start()
            return
        try:
            ser = serial.Serial(
                port=port, baudrate=BAUD,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1.0, dsrdtr=False, rtscts=False,
            )
            ser.dtr = True
            ser.rts = True
            time.sleep(0.1)
            self.ser = ser
        except serial.SerialException as e:
            self._status(f"ERR: {e}", error=True)
            return

        self.btn_connect.config(text='DISCONNECT', bg=C['red'])
        self.btn_connect._base_color = C['red']
        self.lbl_conn.config(text="● CONNECTED", fg=C['green'])
        self._status(f"Connected to {port}")
        self._poll_running = True
        threading.Thread(target=self._poll_loop, daemon=True).start()

    def _disconnect(self):
        global SIM_MODE
        self._poll_running = False
        time.sleep(0.2)
        if self.ser:
            self.ser.close()
            self.ser = None
        SIM_MODE = False
        self.btn_connect.config(text='CONNECT', bg=C['blue_dim'])
        self.btn_connect._base_color = C['blue_dim']
        self.lbl_conn.config(text="● NOT CONNECTED", fg=C['red_bright'])
        self._reset_leds()
        self._status("Disconnected.")

    # ---- Poll loop ----

    def _poll_loop(self):
        while self._poll_running:
            if SIM_MODE:
                try:
                    d = {
                        'status':      sim_read_status(),
                        'temp_shg':    _sim.shg,
                        'temp_thg':    _sim.thg,
                        'temp_cb':     None,
                        'act_current': round(_sim.current_act + random.uniform(-0.01,0.01), 2),
                        'set_current': _sim.current_set,
                        'max_current': MAX_CURRENT,
                        'diode_voltage': None,
                        'prf':         _sim.prf if _sim.prf else 0,
                        'prf_source':  1,
                        'gate_source': 0,
                        'duty_mode':   0,
                        'duty_width':  _sim.duty_width,
                        'flow':        None,
                        'power_w':     None,
                        'soft_faults': None,
                        'board_faults':None,
                    }
                    self.root.after(0, lambda d=d: self._update_ui(d))
                except Exception:
                    pass
            elif self.ser and self.ser.is_open:
                try:
                    with self._lock:
                        d = poll_all(self.ser)
                    self.root.after(0, lambda d=d: self._update_ui(d))
                except Exception:
                    pass
            interval = self._poll_interval
            if interval == 0:
                # Paused - check every 0.2s for unpause without burning CPU
                while self._poll_interval == 0 and self._poll_running:
                    time.sleep(0.2)
            else:
                time.sleep(interval)

    def _update_ui(self, d):
        st   = d.get('status')
        if not st:
            return

        self._last_status = st
        shg  = d.get('temp_shg')
        thg  = d.get('temp_thg')
        amps = d.get('act_current')
        prf  = d.get('prf')
        prf_src  = d.get('prf_source')
        prf_ext  = d.get('prf_ext')      # 0x28 LMCV4 clock at laser input
        prf_act  = d.get('prf_actual')   # 0x22 confirmed fires
        pe       = d.get('pulse_energy') # 0x8C µJ
        dw   = d.get('duty_width')
        pw   = d.get('power_w')
        flow = d.get('flow')
        duty_mode = d.get('duty_mode')
        gate_src  = d.get('gate_source')

        # State label
        state_color = {
            2: C['green'], 3: C['cyan'], 4: C['yellow'],
            5: C['red_bright']
        }.get(st['state'], C['gray'])
        self.lbl_state.config(text=st['state_str'], fg=state_color)

        # LEDs
        self._draw_led(self.led_power,   C['green']      if st['power_on']      else C['off'])
        self._draw_led(self.led_shutter, C['cyan']       if st['shutter']       else C['off'])
        self._draw_led(self.led_key,     C['green']      if st['key_switch']    else C['red_bright'])
        self._draw_led(self.led_ldd,     C['green']      if st['ldd']           else C['off'])
        self._draw_led(self.led_qsw,     C['green']      if st['qsw']           else C['off'])
        self._draw_led(self.led_silock,  C['green']      if st['shutter_ilock'] else C['red_bright'])
        self._draw_led(self.led_lilock,  C['green']      if st['ldd_ilock']     else C['red_bright'])

        # LDD badge
        self.lbl_ldd_status.config(
            text="LDD ON"  if st['ldd'] else "LDD OFF",
            bg=C['green']  if st['ldd'] else C['red'],
            fg=C['bg']     if st['ldd'] else C['white'])

        # Shutter badge
        self.lbl_shutter_status.config(
            text="OPEN"    if st['shutter'] else "CLOSED",
            bg=C['cyan']   if st['shutter'] else C['off'],
            fg=C['bg']     if st['shutter'] else C['gray'])

        # Button labels
        self.btn_shutter.config(
            text="SHUTTER  OFF" if st['shutter'] else "SHUTTER  ON",
            bg=C['cyan_dim']    if st['shutter'] else C['blue_dim'])
        self.btn_shutter._base_color = C['cyan_dim'] if st['shutter'] else C['blue_dim']
        self.btn_ldd.config(
            text="LDD  OFF" if st['ldd'] else "LDD  ON",
            bg=C['green_dim'] if st['ldd'] else C['blue_dim'])
        self.btn_ldd._base_color = C['green_dim'] if st['ldd'] else C['blue_dim']

        # Current
        if amps is not None:
            color = C['green'] if abs(amps - MAX_CURRENT) < 0.5 else C['yellow']
            self.lbl_act_i.config(text=f"{amps:.2f}", fg=color)
        set_i = d.get('set_current')
        if set_i is not None:
            self.lbl_set_i.config(text=f"{set_i:.2f}")
        self.lbl_max_i.config(text=f"{MAX_CURRENT:.2f}")

        # Power output in watts
        if pw is not None and pw > 0.0:
            self.lbl_power_out.config(text=f"Power:  {pw:.2f} W", fg=C['cyan'])
        elif pw == 0.0:
            self.lbl_power_out.config(text=f"Power:  0.00 W", fg=C['gray_dim'])
        else:
            self.lbl_power_out.config(text=f"Power:  - W", fg=C['gray_dim'])

        # Temperatures
        for name, val, key in [
            ('SHG', shg, 'SHG'),
            ('THG', thg, 'THG'),
        ]:
            lbl = self.temp_labels[name]
            if val is not None:
                sp    = TEMP_SETPOINTS.get(key, 0)
                delta = abs(val - sp)
                color = C['red_bright'] if delta > TEMP_ALARM_DELTA else                         C['yellow']     if delta > TEMP_ALARM_DELTA * 0.5 else                         C['green']
                lbl.config(text=f"{val:.2f}°C", fg=color)
        self.temp_labels['CB'].config(text=f"{d.get('temp_cb'):.2f}°C" if d.get('temp_cb') is not None else "N/A",
                                      fg=C['cyan'] if d.get('temp_cb') is not None else C['gray_dim'])

        # PRF display - 0x21 is laser-internal set register (0 in EXT mode).
        # 0x28 = LMCV4 clock arriving at laser (LightBurn's requested rate).
        # 0x22 = gate-qualified confirmed fires. Delta = missed/rejected pulses.
        if prf_ext is not None and prf_ext > 0:
            if prf_act is not None and prf_act > 0:
                delta = prf_ext - prf_act
                pct   = prf_act / prf_ext * 100
                chain_text = f"{prf_ext:,} → {prf_act:,} Hz  ({pct:.0f}%  Δ{delta:+,})"
                chain_color = C['green'] if pct >= 95 else C['yellow'] if pct >= 80 else C['red_bright']
            else:
                chain_text  = f"{prf_ext:,} Hz  (no confirmed fires)"
                chain_color = C['cyan']
            self.lbl_prf.config(text=chain_text, fg=chain_color)
        elif prf_src == 1:
            self.lbl_prf.config(text="- waiting for LMCV4", fg=C['gray_dim'])
        else:
            # INT mode - show set PRF
            self.lbl_prf.config(
                text=f"{prf:,} Hz" if prf else "0 Hz",
                fg=C['cyan'] if prf else C['gray_dim'])

        # Pulse energy (valid only when LDD on and firing)
        if pe is not None and pe > 0.0:
            self.lbl_pulse_energy.config(text=f"{pe:.2f} µJ", fg=C['cyan'])
        else:
            self.lbl_pulse_energy.config(text="- µJ", fg=C['gray_dim'])

        # Sync selectors to actual laser state.
        # Lockout for 3s after a user write so poll doesn't immediately overwrite.
        now = time.time()
        def sync(var, val, key):
            if val is not None and now > self._ctrl_lockout.get(key, 0):
                var.set(val)
        # Also show actual value in brackets next to each control
        prf_src_str  = {0:'INT', 1:'EXT'}.get(prf_src,  '?')
        gate_src_str = {0:'INT', 1:'EXT'}.get(gate_src, '?')
        duty_str     = {0:'Fixed', 1:'Track'}.get(duty_mode, '?')
        sync(self.var_prf_src,  prf_src_str,  'prf_src')
        sync(self.var_gate_src, gate_src_str, 'gate_src')
        sync(self.var_duty_mode, duty_str,    'duty_mode')

        # Actual value labels (color: green=matches control, red=mismatch)
        def mismatch_color(var, actual):
            return C['green'] if var.get() == actual else C['red_bright']
        self.lbl_prf_src_act.config(
            text=f"[{prf_src_str}]",
            fg=mismatch_color(self.var_prf_src, prf_src_str))
        self.lbl_gate_src_act.config(
            text=f"[{gate_src_str}]",
            fg=mismatch_color(self.var_gate_src, gate_src_str))
        self.lbl_duty_mode_act.config(
            text=f"[{duty_str}]",
            fg=mismatch_color(self.var_duty_mode, duty_str))

        # Duty width
        if dw is not None:
            self.lbl_duty_width.config(text=f"{dw} ns actual")
            if not self.entry_duty_width.get():
                self.entry_duty_width.delete(0, 'end')
                self.entry_duty_width.insert(0, str(dw))

        # Pulsed op and burst
        pulsed_op = d.get('pulsed_op')
        burst     = d.get('burst')
        pulsed_str = "Enabled" if pulsed_op else "Disabled"
        burst_str  = "On" if burst else "Off"
        sync(self.var_pulsed, pulsed_str, 'pulsed')
        sync(self.var_burst,  burst_str,  'burst')
        self.lbl_pulsed_act.config(
            text=f"[{pulsed_str}]",
            fg=mismatch_color(self.var_pulsed, pulsed_str))
        self.lbl_burst_act.config(
            text=f"[{burst_str}]",
            fg=mismatch_color(self.var_burst, burst_str))

        # Flow
        if flow is not None:
            self.lbl_flow.config(
                text=f"{flow:.1f} L/min",
                fg=C['green'] if flow > 5.0 else C['yellow'])
        else:
            self.lbl_flow.config(text="no meter", fg=C['gray_dim'])

        # ---- Alarm system ----
        alarm_names = [
            ('sys_faults',    f"SYS FAULT 0x{st['sys_faults']:02X}"),
            ('gen_alarms',    f"GEN ALARM 0x{st['gen_alarms']:02X}"),
            ('temp_alarms',   f"TEMP ALARM 0x{st['temp_alarms']:02X}"),
        ]
        current_alarms = set(
            label for key, label in alarm_names if st.get(key)
        )
        # Also decode gen alarm bits
        alarm_bits = [
            ('alarm_flow',     'FLOW ALARM'),
            ('alarm_humidity', 'HUMIDITY ALARM'),
            ('alarm_wet',      'WET SENSOR'),
            ('alarm_hfsync',   'HF SYNC FAULT'),
            ('alarm_pwrmon',   'POWER MON FAULT'),
        ]
        for key, label in alarm_bits:
            if st.get(key):
                current_alarms.add(label)

        new_alarms = current_alarms - self._active_alarms
        self._active_alarms = current_alarms

        if new_alarms:
            # New alarm - un-silence
            self._alarms_silenced = False

        if current_alarms and not self._alarms_silenced:
            self.lbl_alarms.config(
                text="⚠  " + "\n⚠  ".join(sorted(current_alarms)),
                fg=C['red_bright'])
            self.btn_silence.config(bg=C['red'], fg=C['white'])
            self.root.bell()
        elif current_alarms and self._alarms_silenced:
            self.lbl_alarms.config(
                text="(silenced)\n" + "\n".join(sorted(current_alarms)),
                fg=C['gray_dim'])
            self.btn_silence.config(bg=C['gray_dim'], fg=C['white'])
        else:
            self.lbl_alarms.config(text="", fg=C['gray'])
            self.btn_silence.config(bg=C['bg3'], fg=C['gray_dim'])
    def _reset_leds(self):
        for led in [self.led_power, self.led_shutter, self.led_key,
                    self.led_ldd, self.led_qsw, self.led_silock,
                    self.led_lilock, self.led_pulse]:
            self._draw_led(led, C['off'])

    # ---- Command handlers ----

    def _on_set_prf_source(self, val):
        if not self._require_connected(): return
        self._ctrl_lockout['prf_src'] = time.time() + 3.0
        code = 1 if val == "EXT" else 0
        def do():
            ok, _, r = self._w(0x23, [code])
            self._status(f"PRF Source → {val}" if ok else f"ERR PRF Source: {r}", error=not ok)
        self._cmd(do)

    def _on_set_gate_source(self, val):
        if not self._require_connected(): return
        self._ctrl_lockout['gate_src'] = time.time() + 3.0
        code = 1 if val == "EXT" else 0
        def do():
            ok, _, r = self._w(0x24, [code])
            self._status(f"Gate Source → {val}" if ok else f"ERR Gate Source: {r}", error=not ok)
        self._cmd(do)

    def _on_set_duty_mode(self, val):
        if not self._require_connected(): return
        self._ctrl_lockout['duty_mode'] = time.time() + 3.0
        code = 1 if val == "Track" else 0
        def do():
            ok, _, r = self._w(0x69, [code])
            self._status(f"Duty Mode → {val}" if ok else f"ERR Duty Mode: {r}", error=not ok)
        self._cmd(do)

    def _on_set_duty_width(self, event=None):
        if not self._require_connected(): return
        try:
            ns = int(self.entry_duty_width.get())
        except ValueError:
            self._status("Invalid duty width - enter integer ns.", error=True); return
        if ns < 600:
            self._status("ERR: minimum duty width is 600 ns", error=True); return
        def do():
            ok, _, r = self._w(0x54, [(ns >> 16) & 0xFF, (ns >> 8) & 0xFF, ns & 0xFF])
            self._status(f"Duty Width → {ns} ns" if ok else f"ERR: {r}", error=not ok)
        self._cmd(do)

    def _on_set_prf(self, event=None):
        if not self._require_connected(): return
        try:
            hz = int(self.entry_prf_set.get())
        except ValueError:
            self._status("Invalid PRF value.", error=True); return
        def do():
            ok, _, r = self._w(0x21, [(hz >> 16) & 0xFF, (hz >> 8) & 0xFF, hz & 0xFF])
            self._status(f"PRF → {hz:,} Hz" if ok else f"ERR: {r}", error=not ok)
        self._cmd(do)

    def _on_set_pulsed(self, val):
        if not self._require_connected(): return
        self._ctrl_lockout['pulsed'] = time.time() + 3.0
        code = 1 if val == "Enabled" else 0
        def do():
            ok, _, r = self._w(0x20, [code])
            self._status(f"Pulsed Op → {val}" if ok else f"ERR Pulsed Op: {r}", error=not ok)
        self._cmd(do)

    def _on_set_burst(self, val):
        if not self._require_connected(): return
        self._ctrl_lockout['burst'] = time.time() + 3.0
        code = 1 if val == "On" else 0
        def do():
            ok, _, r = self._w(0x56, [code])
            self._status(f"Burst → {val}" if ok else f"ERR Burst: {r}", error=not ok)
        self._cmd(do)

    def _on_poll_rate_change(self, event=None):
        rates = {"1s": 1.0, "10s": 10.0, "30s": 30.0, "1min": 60.0, "OFF": 0}
        val = self.var_poll_rate.get()
        self._poll_interval = rates.get(val, 1.0)
        self._status(f"Poll rate → {val}")

    def _on_poll_now(self):
        """Force an immediate single poll regardless of rate/pause state."""
        if not self._require_connected(): return
        def do():
            try:
                if SIM_MODE:
                    # Reuse the same sim data path as the poll loop
                    d = {
                        'status':        sim_read_status(),
                        'temp_shg':      _sim.shg,
                        'temp_thg':      _sim.thg,
                        'temp_cb':       None,
                        'act_current':   round(_sim.current_act, 2),
                        'set_current':   _sim.current_set,
                        'max_current':   MAX_CURRENT,
                        'diode_voltage': None,
                        'prf':           _sim.prf if _sim.prf else 0,
                        'prf_ext':       None,
                        'prf_actual':    None,
                        'pulse_energy':  None,
                        'prf_source':    1,
                        'gate_source':   0,
                        'duty_mode':     0,
                        'duty_width':    _sim.duty_width,
                        'flow':          None,
                        'power_w':       None,
                        'soft_faults':   None,
                        'board_faults':  None,
                    }
                else:
                    with self._lock:
                        d = poll_all(self.ser)
                self.root.after(0, lambda d=d: self._update_ui(d))
                self._status("Manual poll complete.")
            except Exception as e:
                self._status(f"Poll ERR: {e}", error=True)
        threading.Thread(target=do, daemon=True).start()

    def _on_silence_alarms(self):
        self._alarms_silenced = True

    def _require_connected(self):
        if not SIM_MODE and (not self.ser or not self.ser.is_open):
            self._status("Not connected.", error=True)
            return False
        return True

    def _w(self, cc, data):
        """Write to hardware - acquires serial lock."""
        if SIM_MODE:
            ok = sim_write(cc, data)
            return ok, [], "SIM"
        with self._lock:
            return _write(self.ser, cc, data)

    def _r(self, cc, aa=0x00):
        """Read from hardware - acquires serial lock."""
        if SIM_MODE:
            return True, [], "SIM"
        with self._lock:
            return _read(self.ser, cc, aa)

    def _cmd(self, fn):
        """Run a command in a background thread.
        Serial lock in _w/_r ensures no collision with poll loop."""
        def run():
            try:
                fn()
            except Exception as e:
                self._status(f"CMD ERR: {e}", error=True)
        threading.Thread(target=run, daemon=True).start()

    def _sim_write(self, cc, data):
        """Sim mode write - updates SimState directly."""
        return sim_write(cc, data)

    def _on_set_current(self, event=None):
        if not self._require_connected():
            return
        try:
            a = float(self.entry_current.get())
        except ValueError:
            self._status("Invalid current value.", error=True)
            return
        if a > MAX_CURRENT:
            self._status(f"ERR: Max allowed is {MAX_CURRENT}A", error=True)
            return
        def do():
            val = int(round(a * 100))
            ok, _, r = self._w( 0x11, [(val >> 8) & 0xFF, val & 0xFF])
            if ok:
                self.lbl_set_i.config(text=f"{a:.2f}")
                self._status(f"Current set to {a:.2f}A")
            else:
                self._status(f"ERR setting current: {r}", error=True)
        self._cmd(do)

    def _on_shutter_toggle(self):
        if not self._require_connected():
            return
        st = self._last_status
        if st is None:
            return
        enable = not st['shutter']
        def do():
            if enable:
                ldd_was_on = st['ldd']
                if ldd_was_on:
                    self._w( 0x10, [0x00])
                    time.sleep(0.2)
                ok, _, r = self._w( 0x04, [0x01])
                if ldd_was_on and ok:
                    time.sleep(0.1)
                    self._w( 0x10, [0x01])
                self._status("Shutter OPEN" if ok else f"Shutter ERR: {r}",
                             error=not ok)
            else:
                ok, _, r = self._w( 0x04, [0x00])
                self._status("Shutter CLOSED" if ok else f"Shutter ERR: {r}",
                             error=not ok)
        self._cmd(do)

    def _on_ldd_toggle(self):
        if not self._require_connected():
            return
        st = self._last_status
        if st is None:
            return
        enable = not st['ldd']
        def do():
            ok, _, r = self._w( 0x10, [0x01 if enable else 0x00])
            self._status(f"LDD {'ON' if enable else 'OFF'}" if ok
                         else f"LDD ERR: {r}", error=not ok)
        self._cmd(do)

    def _on_go(self):
        if not self._require_connected():
            return
        def do():
            st = read_status(self.ser) if not SIM_MODE else sim_read_status()
            if not st:
                self._status("ERR: Cannot read status.", error=True)
                return
            if st['state'] not in (2, 3):
                self._status(f"ERR: System not ready: {st['state_str']}", error=True)
                return
            if not st['key_switch']:
                self._status("ERR: Key switch OFF.", error=True)
                return
            # Config
            val = int(round(MAX_CURRENT * 100))
            self._w( 0x14, [(val >> 8) & 0xFF, val & 0xFF])
            self._w( 0x23, [0x01])   # PRF EXT
            self._w( 0x24, [0x00])   # Gate INT
            self._w( 0x69, [0x00])   # Duty Fixed
            self._w( 0x56, [0x00])   # Burst off
            self._w( 0x20, [0x01])   # Pulsed enable
            # Shutter
            if not st['shutter']:
                ldd_was_on = st['ldd']
                if ldd_was_on:
                    self._w( 0x10, [0x00])
                    time.sleep(0.2)
                ok, _, _ = self._w( 0x04, [0x01])
                if not ok:
                    self._status("ERR: Shutter failed.", error=True)
                    return
                if ldd_was_on:
                    time.sleep(0.1)
                    self._w( 0x10, [0x01])
            # LDD
            st2 = read_status(self.ser) if not SIM_MODE else sim_read_status()
            if st2 and not st2['ldd']:
                self._w( 0x10, [0x01])
            # Current
            self._w( 0x11, [(val >> 8) & 0xFF, val & 0xFF])
            self._status("GO complete - LightBurn may fire.")
        self._cmd(do)

    def _on_shutdown(self):
        if not self._require_connected():
            return
        def do():
            self._w( 0x04, [0x00])   # shutter close
            self._w( 0x10, [0x00])   # LDD off
            for _ in range(20):
                time.sleep(0.5)
                amps = read_actual_current(self.ser) if not SIM_MODE else round(_sim.current_act, 2)
                if amps is not None and amps < 0.1:
                    break
            self._status("Shutdown complete. Current at zero.")
        self._cmd(do)

    def _on_reset(self):
        if not self._require_connected():
            return
        def do():
            ok, _, r = self._w( 0x07, [])
            self._status(f"Reset sent.")
        self._cmd(do)

    def _status(self, msg, error=False):
        color = C['red_bright'] if error else C['gray']
        prefix = "ERR" if error else "OK "
        print(f"[{prefix}] {msg}", flush=True)
        self.root.after(0, lambda: self.lbl_statusbar.config(
            text=msg, fg=color))


# ---- Main ----

def main():
    root = tk.Tk()

    # Dark title bar on macOS via tkinter hack
    try:
        root.tk.call('tk::unsupported::MacWindowStyle', 'style',
                     root._w, 'document', 'none')
    except Exception:
        pass

    app = LaserGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()

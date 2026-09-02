# pi-laser-control

Serial control for a Photonics Industries DX-355-5-WC (water-cooled 355nm DPSS
laser). A CLI and a Tk GUI that talk the DX serial command set directly, so you
don't need the vendor's Windows control software running to bring the laser up,
watch temps and power, and shut it down safely.

Written for macOS/Linux. Needs `pyserial`.

> Class 4 laser. This drives real hardware with real hazards - use it only if you
> know the machine and its interlocks. No warranty, see LICENSE.

## Protocol

HEXASC framing over serial, 115200 8N1, CR-terminated.

```
frame  = RW NN AA CC [data...]   hex-encoded
       + 2-byte checksum  (sum of the above bytes, & 0xFFFF)
RW     = 0x00 write, 0x01 read     (0x02 / 0x03 on responses)
```

System states: 0 init, 1 ramping, 2 running, 3 standby, 4 soft fault, 5 hard fault.
Diode current is clamped to 7.2 A in software. SHG/THG temp setpoints are 49.75 /
69.70 °C with a 2 °C alarm band.

## Files

| file | what |
|------|------|
| `pi_laser_ctrl.py` | CLI - `status`, `poll`, `monitor`, `startup`, `shutdown`, `go`, `shutter on/off`, `ldd on/off`, `current <A>`, `raw <hex>`, `reset` |
| `pi_laser_gui.py`  | Tk control panel, same functionality, dark theme. Runs against a real port or a built-in simulator when nothing's connected. |

## Use

```
python3 pi_laser_ctrl.py /dev/cu.usbserial-XXXX status
python3 pi_laser_ctrl.py /dev/cu.usbserial-XXXX monitor
python3 pi_laser_gui.py
```

## Safety

`shutdown` ramps current to zero before dropping the LDD and closing the shutter.
Quitting the CLI leaves the laser exactly as it is - run `shutdown` first if the
diode is on. The 7.2 A clamp is a backstop, not a substitute for the hardware
interlocks.

## License

MIT - see [LICENSE](LICENSE).

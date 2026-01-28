# Rapid 87-1752 PSU Control

Python GUI controller for the Rapid Electronics 87-1752 (and compatible) 30V/3A programmable power supply via RS-485.

![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)
![License](https://img.shields.io/badge/License-Public%20Domain-green.svg)

## Supported Hardware

This software works with several rebadged versions of the same Chinese OEM power supply:

| Brand | Model |
|-------|-------|
| Rapid Electronics | 87-1752 |
| PeakTech | 1860 |
| Manson | NDP-4303, NDP-4185, NDP-4601 |
| Voltcraft | PPS series |

All use identical RS-485 protocol.

## Features

- Real-time voltage/current monitoring
- Voltage and current control with sliders
- Quick presets (3.3V, 5V, 9V, 12V, 24V)
- LiPo battery charger (CC/CV with termination detection)
- Auto-reconnect on USB disconnect/reconnect
- Multi-unit support (addresses 00-30)

## Requirements

- Python 3.8+
- USB-to-RS485 adapter (CH340 or FTDI based)
- 120Ω termination resistor across A+/B-

### Python Dependencies

```bash
pip install pyserial
```

Tkinter is included with most Python installations.

## Hardware Setup

1. Connect USB-to-RS485 adapter to your computer
2. Wire adapter to PSU rear panel:
   - A+ to A+
   - B- to B-
   - GND to GND
3. Add 120Ω termination resistor across A+ and B-
4. Set PSU address via front panel (default: 01)

## Usage

```bash
python psu_control.py
```

The GUI will auto-detect the serial port and attempt connection on startup.

### LiPo Charging

1. Select cell count (1S-7S)
2. Set charge current (typ. 1C)
3. Set termination current (typ. C/20 = 0.05A for 1Ah)
4. Click "Start Charge"
5. Charging stops automatically when current drops below termination threshold

## Linux: Prevent USB Suspend

If using a CH340-based adapter, USB autosuspend can cause disconnects. Install the udev rule:

```bash
sudo cp 99-ch340-no-suspend.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
```

Then reconnect the adapter.

## Protocol Documentation

See [PROTOCOL.md](PROTOCOL.md) for the complete RS-485 command reference.

## Screenshots

```
┌─────────────────────────────────────────────────┐
│ PSU Control - Rapid 87-1752                     │
├─────────────────────────────────────────────────┤
│ Connection: /dev/ttyUSB0  Addr: 01  [Connected] │
├─────────────────────────────────────────────────┤
│        ┌────────┐      ┌────────┐               │
│        │ 12.00  │      │ 0.500  │               │
│        │ Volts  │      │ Amps   │               │
│        └────────┘      └────────┘               │
│                    CV                           │
├─────────────────────────────────────────────────┤
│ Voltage: [====|=========] 12.0V  [Set V]        │
│ Current: [==|===========] 1.00A  [Set I]        │
├─────────────────────────────────────────────────┤
│     [OUTPUT ON]    [Exit Remote]  [Refresh]     │
├─────────────────────────────────────────────────┤
│ [3.3V] [5V] [9V] [12V] [24V]                    │
├─────────────────────────────────────────────────┤
│ LiPo: 3S (12.3V) @ 1.00A  [Start Charge]        │
│ Phase: Idle   Time: 00:00:00   Charged: 0 mAh   │
└─────────────────────────────────────────────────┘
```

## Known Issues

- Output enable logic is inverted in the protocol (`0` = ON, `1` = OFF)
- Front panel is locked during remote mode - use "Exit Remote" to regain manual control
- 4-digit readings but only 3-digit setpoint precision

## Contributing

Issues and pull requests welcome. This project originated from reverse-engineering a 20-year-old PSU with lost documentation.

## Attribution

- Protocol reverse-engineering: Tom (EEVblog community)
- PeakTech 1860 manual preservation: pewa.de
- OEM variant identification: EEVblog forum user **smk**
- Documentation assistance: Claude (Anthropic)

## License

Public Domain. Use freely, share freely, modify freely.
# rapid-87-1752-psu

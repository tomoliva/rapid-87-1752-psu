# Rapid Electronics 87-1752 Power Supply Protocol Reference

> **Document Attribution:** This protocol reference was compiled with assistance from Claude (Anthropic AI) based on reverse-engineering research, manual analysis, and community forum discussions. Original research by Tom (EEVblog community). Shared freely for the benefit of the maker/engineering community.

## Hardware Identification

**Primary Model:** Rapid Electronics Stock/Model Number 87-1752

**Common Specifications (all variants):**
- 30V / 3A programmable bench power supply
- 90W total output power
- USB 1.0/1.1 interface (obsolete - incompatible with modern OS)
- **RS-485 interface** (fully functional, documented here)
- Manufactured/badged circa 2005-2006
- Linear regulation topology

### Known OEM Variants (Identical Protocol)

This is a Chinese OEM design from the mid-2000s that was rebadged by multiple European distributors. All variants share **identical hardware, firmware, and RS-485 protocol**:

| Brand | Model | Notes |
|-------|-------|-------|
| **Rapid Electronics** | 87-1752 | UK distributor, now obsolete |
| **PeakTech** | 1860 | German brand, manual still available |
| **Manson** | NDP-4303 | Also sold as NDP-4185 (18V/5A), NDP-4601 (60V/1.5A) |
| **Voltcraft** | PPS series | Likely similar OEM base |

**All models use the exact same command set and protocol documented below.**

**Current Address Setting:** 01 (configurable 00-30 via front panel keypad)

## Protocol Specification

### Physical Layer - RS-485

- **Baud Rate:** 9600 bps
- **Data Format:** 8 data bits, No parity, 1 stop bit (8N1)
- **Interface:** RS-485 half-duplex
- **Transmission Distance:** Up to 1000m
- **Multi-drop Support:** Up to 31 units (addresses 00-30)
- **Termination:** 120Ω resistor across A+/B- at first and last unit in chain

### Command Structure

All commands follow ASCII format:

```
Command: <COMMAND> <ADDRESS> [PARAMETERS] <CR>
Response: [DATA] [OK] <CR>
```

Where:
- `<CR>` = Carriage Return (0x0D)
- `<ADDRESS>` = 2-byte ASCII decimal address (e.g., "01" for address 1)
- Parameters and responses are ASCII text
- All numeric values are zero-padded fixed-width ASCII

### Data Encoding

**Voltage Values:**
- Format: 3-digit ASCII (000-300)
- Represents: 00.0V to 30.0V
- Conversion: Value ÷ 10 = Volts
- Example: "150" = 15.0V, "052" = 5.2V

**Current Values:**
- Format: 3-digit ASCII (000-300)
- Represents: 0.00A to 3.00A
- Conversion: Value ÷ 100 = Amps
- Example: "100" = 1.00A, "025" = 0.25A

**Time Values (Timed Programs):**
- Minutes: 2-digit ASCII (00-99)
- Seconds: 2-digit ASCII (00-59)

**Memory Locations:**
- Preset Memory: 1-9 (single digit)
- Timed Program Steps: 00-19 (2 digits)

## Command Reference

### Session Control

**Enter Remote Mode (Disable Front Panel)**
```
Command:  SESS 01<CR>
Response: OK<CR>
```
Required before sending control commands. Locks front panel keypad.

**Exit Remote Mode (Enable Front Panel)**
```
Command:  ENDS 01<CR>
Response: OK<CR>
```
Returns control to front panel keypad.

### Query Commands

**Get Maximum Voltage/Current Ratings**
```
Command:  GMAX 01<CR>
Response: Voltage 300 Current 300<CR>
          OK<CR>
```
Returns: Max voltage (30.0V), Max current (3.00A)

**Get Upper Voltage Limit (OVP Setting)**
```
Command:  GOVP 01<CR>
Response: Voltage <VVV><CR>
          OK<CR>
```
Returns: Upper voltage limit setting

**Get Current Output Readings**
```
Command:  GETD 01<CR>
Response: Voltage <VVVV> Current <CCCC> <M><CR>
          OK<CR>
```
- `<VVVV>`: 4-digit voltage reading (e.g., "1234" = 12.34V)
- `<CCCC>`: 4-digit current reading (e.g., "0987" = 0.987A)
- `<M>`: Mode flag - `0` = CV (Constant Voltage), `1` = CC (Constant Current)

**Get Voltage/Current Set Values**
```
Command:  GETS 01<CR>
Response: Voltage <VVV> Current <CCC><CR>
          OK<CR>
```
Returns current setpoint values (not actual output).

**Get Communication Interface Setting**
```
Command:  GCOM 01<CR>
Response: <RS> RS485 Address <AA><CR>
          OK<CR>
```
- `<RS>`: 0 = USB, 1 = RS-485
- `<AA>`: RS-485 address

### Control Commands

**Set Voltage**
```
Command:  VOLT 01 <VVV><CR>
Response: OK<CR>
```
Example: `VOLT 01 150<CR>` sets 15.0V

**Set Current Limit**
```
Command:  CURR 01 <CCC><CR>
Response: OK<CR>
```
Example: `CURR 01 100<CR>` sets 1.00A

**Set Upper Voltage Limit (OVP)**
```
Command:  SOVP 01 <VVV><CR>
Response: OK<CR>
```
Example: `SOVP 01 300<CR>` sets 30.0V limit

**Enable Output**
```
Command:  SOUT 01 0<CR>
Response: OK<CR>
```
Note: 0 = Enable (not a typo in the protocol!)

**Disable Output**
```
Command:  SOUT 01 1<CR>
Response: OK<CR>
```
Note: 1 = Disable

### Preset Memory Commands

**Get All Preset Memories (1-9)**
```
Command:  GETM 01<CR>
Response: Memory 1 Voltage <VVV> Current <CCC><CR>
          Memory 2 Voltage <VVV> Current <CCC><CR>
          ...
          Memory 9 Voltage <VVV> Current <CCC><CR>
          OK<CR>
```

**Get Specific Preset Memory**
```
Command:  GETM 01 <L><CR>
Response: Voltage <VVV> Current <CCC><CR>
          OK<CR>
```
Where `<L>` = location 1-9

**Program Preset Memory**
```
Command:  PROM 01 <L> <VVV> <CCC><CR>
Response: OK<CR>
```
Example: `PROM 01 5 120 050<CR>` stores 12.0V/0.50A in preset 5

**Recall Preset Memory**
```
Command:  RUNM 01 <L><CR>
Response: OK<CR>
```
Immediately applies the preset voltage/current from location 1-9.

### Timed Program Commands

**Get All Timed Program Steps (00-19)**
```
Command:  GETP 01<CR>
Response: Program 00 Voltage <VVV> Current <CCC> Minute <MM> Second <SS><CR>
          Program 01 Voltage <VVV> Current <CCC> Minute <MM> Second <SS><CR>
          ...
          Program 19 Voltage <VVV> Current <CCC> Minute <MM> Second <SS><CR>
          OK<CR>
```

**Get Specific Timed Program Step**
```
Command:  GETP 01 <PP><CR>
Response: Voltage <VVV> Current <CCC> Minute <MM> Second <SS><CR>
          OK<CR>
```
Where `<PP>` = program 00-19

**Program Timed Step**
```
Command:  PROP 01 <PP> <VVV> <CCC> <MM> <SS><CR>
Response: OK<CR>
```
Example: `PROP 01 00 050 100 01 30<CR>` sets step 0 to 5.0V/1.00A for 1min 30sec

**Run Timed Program**
```
Command:  RUNP 01 <CCC><CR>
Response: OK<CR>
```
- `<CCC>`: Number of cycles (000 = infinite, 001-999 = specific count)
- Executes programmed sequence starting from step 00
- Continues until step with time = 00:00 or after specified cycles

**Stop Timed Program**
```
Command:  STOP 01<CR>
Response: OK<CR>
```

### Interface Configuration

**Change Communication Interface**
```
Command:  CCOM 01 <RS> <AAA><CR>
Response: OK<CR>
```
- `<RS>`: 0 = USB, 1 = RS-485
- `<AAA>`: RS-485 address (000-255, but only 00-30 supported for multi-drop)

## Implementation Notes

### Protocol Quirks ("ASCII Weirdness")

1. **No decimal points in numeric values** - all values are fixed-width integers
2. **Output control inverted logic** - `SOUT 01 0` enables, `SOUT 01 1` disables
3. **4-digit readings vs 3-digit setpoints** - GETD returns 4 digits, SET commands use 3
4. **Response format varies** - some commands return multi-line, others single line
5. **All responses end with `OK<CR>`** - use this to detect command completion

### Error Handling

- If invalid parameter (exceeds max rating), device shows error and displays max value
- Commands timeout after ~10 seconds of inactivity
- Front panel locked during SESS (remote mode)
- No explicit error codes - watch for non-`OK` responses or timeouts

### Timing Considerations

- Allow ~100ms between commands for response processing
- GETD command may take longer when PSU is changing states
- Timed programs update display at 4Hz (250ms intervals)
- Relay life: 100k operations (relevant for frequent output switching)

### Safety Features

- Over-Voltage Protection (OVP) - tracks Upper Voltage Limit setting
- Over-Current Protection (OCP) - current limiting mode
- Over-Temperature Protection (OTP) - automatic shutdown
- Short circuit protection

## Testing Strategy

### Initial Connection Test Sequence

1. **Verify communication:**
   ```
   GMAX 01<CR>
   Expected: Voltage 300 Current 300<CR>OK<CR>
   ```

2. **Read current settings (safe - doesn't change anything):**
   ```
   GETS 01<CR>
   GETD 01<CR>
   ```

3. **Read status:**
   ```
   GCOM 01<CR>
   GOVP 01<CR>
   ```

### Safe Test Procedure

Before connecting any load:

1. Enter remote mode: `SESS 01<CR>`
2. Set low voltage: `VOLT 01 050<CR>` (5.0V)
3. Set low current: `CURR 01 010<CR>` (0.10A)
4. Enable output: `SOUT 01 0<CR>`
5. Read output: `GETD 01<CR>`
6. Disable output: `SOUT 01 1<CR>`
7. Exit remote mode: `ENDS 01<CR>`

### Example Python Test Snippet

```python
import serial
import time

# Configure serial port (adjust port name for your system)
ser = serial.Serial(
    port='/dev/ttyUSB0',  # Linux
    # port='COM3',        # Windows
    baudrate=9600,
    bytesize=8,
    parity='N',
    stopbits=1,
    timeout=1.0
)

def send_command(cmd):
    """Send command and read response"""
    ser.write((cmd + '\r').encode('ascii'))
    time.sleep(0.1)
    response = ser.read(ser.in_waiting).decode('ascii')
    return response

# Test sequence
print("Testing PSU communication...")

# Get max ratings
resp = send_command("GMAX 01")
print(f"Max ratings: {resp}")

# Get current settings
resp = send_command("GETS 01")
print(f"Current settings: {resp}")

# Get output readings
resp = send_command("GETD 01")
print(f"Output: {resp}")
```

## RS-485 Hardware Connection

**3-Wire Connection (RS-485 Connector on PSU rear panel):**
- A+ (positive differential signal)
- B- (negative differential signal)
- GND (common ground)

**For single PSU:**
- Connect 120Ω termination resistor across A+ and B-
- One resistor is included with the PSU

**For multiple PSUs (daisy chain):**
- Connect A+ to A+, B- to B- across all units
- 120Ω resistor at FIRST and LAST unit only
- Maximum 31 units (addresses 00-30)
- Each unit needs unique address via front panel

**USB to RS-485 Adapter Notes:**
- Standard USB-to-RS485 converters work (e.g., FTDI, CH340-based)
- No special drivers beyond standard USB-serial
- Original Windows software not required for RS-485 communication

## Historical Context & OEM Archaeology

This PSU represents the mid-2000s generation of Chinese programmable power supplies that used custom ASCII protocols over RS-485 for industrial multi-drop applications, predating the widespread adoption of SCPI and Modbus RTU in this price range.

**Manufacturing Timeline:**
- Design origins: ~2004-2005 (Chinese OEM manufacturer unknown)
- Peak distribution: 2005-2007 across Europe
- Current status: Obsolete, but many units still in service

**Protocol Characteristics:**
The "ASCII weirdness" refers to the fixed-width integer encoding without decimal points (e.g., "150" = 15.0V), which was common in industrial displays and control systems of that era. This differs from later SCPI standards that use decimal notation.

**Why RS-485 Survived While USB Didn't:**
- USB 1.1 drivers were Windows XP/98-specific, now incompatible
- RS-485 uses standard serial protocol, works on any modern system
- Industrial multi-drop capability (31 units) was the intended use case
- USB was likely added as a marketing feature for home/lab users

**Similar Designs from the Era:**
Other Chinese manufacturers from this period (Atten, Korad, Longwei) used different protocols documented in community forums and the sigrok project. Each OEM had their own command set, making this documentation valuable for owners of these specific rebadges.

## Community Research & Attribution

This protocol documentation was compiled through:

1. **Manual Analysis:** PeakTech 1860 manual (PDF available at pewa.de)
2. **Forum Research:** EEVblog community discussion thread on "Programmable PSU Archaeology"
3. **Cross-Reference:** Manson NDP-4303 datasheet confirmation
4. **AI-Assisted Documentation:** Claude (Anthropic) helped organize and format the protocol reference

**Special Thanks:**
- EEVblog forum user **smk** for identifying the PeakTech 1860 and Manson NDP-4303 connection
- Original manual preservation by pewa.de
- Community members who've kept these old PSUs running

**Forum Discussion:**
This research originated from troubleshooting a 20-year-old PSU with lost documentation. If you have additional variants or protocol information, please share on the EEVblog forums!

## Software Calibration

The PSU includes a software calibration mode accessible via the front panel keypad. This allows fine-tuning of voltage and current readings without hardware modification.

### Entering Calibration Mode

**Keypad Sequence:** `SHIFT` → `3` → `9`

The display will show `d-A` indicating calibration mode is active.

### Calibration Modes

**d-A Mode (Actual Output Calibration):**
- Adjusts the actual DAC output to match the display value
- Use when a multimeter shows different voltage/current than the PSU display
- Changes the actual power output

**A-d Mode (Display Calibration):**
- Adjusts the ADC display reading to match actual output
- Use when the display doesn't match actual measured output
- Only changes the displayed value, not actual output

Toggle between modes using the designated key (see manual).

### Voltage Calibration Procedure

1. Enter calibration mode (`SHIFT` → `3` → `9`)
2. Connect a calibrated multimeter to the output terminals
3. Set a reference voltage (e.g., 15.00V on display)
4. Enable output
5. Read actual voltage on multimeter
6. Look up the difference in Appendix A offset table
7. Enter the offset code using the keypad
8. Verify calibration across multiple voltage points

### Current Calibration Procedure

1. Enter calibration mode
2. Connect a calibrated ammeter or shunt in series with a load
3. Set a reference current (e.g., 1.500A on display)
4. Enable output with appropriate load
5. Read actual current on ammeter
6. Look up the difference in Appendix B offset table
7. Enter the offset code using the keypad
8. Verify calibration across multiple current points

### Offset Lookup Tables

**Appendix A - Voltage Offset (from PeakTech 1860 Manual)**

| Offset Range | Code | Notes |
|--------------|------|-------|
| -1.27V to +1.27V | 00-FF | 256 steps, ~10mV per step |
| Center (0V offset) | 80 | Default factory setting |

**Appendix B - Current Offset (from PeakTech 1860 Manual)**

| Offset Range | Code | Notes |
|--------------|------|-------|
| -0.127A to +0.127A | 00-FF | 256 steps, ~1mA per step |
| Center (0A offset) | 80 | Default factory setting |

### Calibration Notes

- **Factory reset:** If calibration is badly off, the ~10mA current offset mentioned in community discussions may be a firmware issue rather than calibration error
- **Reference equipment:** Use a 4.5+ digit multimeter for best results
- **Multiple points:** Calibrate at multiple V/I points to verify linearity
- **Documentation:** Record your offset codes in case of accidental reset
- **Exit calibration:** Power cycle the unit to exit calibration mode

### Known Calibration Issues

- Some units have a persistent ~10mA offset on current readings (likely ADC offset in STM32-derived firmware)
- Calibration does not persist through firmware issues - note your offset values
- The 12-bit DAC/ADC provides ~7.3mV voltage resolution (30V / 4096)

---

## References & Resources

### Primary Documentation Sources

1. **PeakTech 1860 Operation Manual** (English)
   - URL: https://www.pewa.de/DATENBLATT/DBL_HGL_P1860_MANUAL_DEUTSCH.PDF
   - Contains complete command set (Appendix C, pages 25-27)
   - Interface specifications (Section 4.1 and 7.2)
   - RS-485 adapter pinout (Appendix D)

2. **Manson NDP-4303 Datasheet**
   - URL: http://www.weclonline.com/downloads/pdf/64-05-5185.pdf
   - Confirms identical specifications across OEM variants
   - Also documents NDP-4185 (18V/5A) and NDP-4601 (60V/1.5A) models

### Community Resources

3. **EEVblog Forum Discussion**
   - Thread: "Programmable PSU Archaeology (China)"
   - Community identification of OEM variants
   - Credit to forum user **smk** for cross-referencing

4. **Related Protocol Documentation**
   - Sigrok project (sigrok.org) - for other Chinese PSU protocols (Atten, Korad)
   - Community reverse-engineering efforts for similar 2005-era equipment

### Hardware Components

- **RS-232 to RS-485 Adapter:** ATR-2485 (or any standard USB-to-RS485 adapter)
- **Termination Resistor:** 120Ω across A+/B- (one typically included with PSU)

---

## Quick Command Summary Table

| Function | Command | Example |
|----------|---------|---------|
| Enter remote | `SESS 01<CR>` | - |
| Exit remote | `ENDS 01<CR>` | - |
| Get max ratings | `GMAX 01<CR>` | Returns: 300V, 300A |
| Get settings | `GETS 01<CR>` | Returns: V, A setpoints |
| Get output | `GETD 01<CR>` | Returns: V, A, mode |
| Set voltage | `VOLT 01 <VVV><CR>` | `VOLT 01 120<CR>` = 12.0V |
| Set current | `CURR 01 <CCC><CR>` | `CURR 01 150<CR>` = 1.50A |
| Enable output | `SOUT 01 0<CR>` | Note: 0 = ON |
| Disable output | `SOUT 01 1<CR>` | Note: 1 = OFF |
| Recall preset | `RUNM 01 <L><CR>` | `RUNM 01 3<CR>` = preset 3 |

---

## Practical Usage Tips

### Modern System Integration

**Works perfectly with:**
- Linux (any USB-to-RS485 adapter via `/dev/ttyUSB*`)
- Windows (COM port via USB-serial drivers)
- macOS (via `/dev/tty.usbserial*`)
- Raspberry Pi / embedded systems

**Does NOT work:**
- Original Windows XP software USB interface (driver incompatibility)
- Modern USB direct connection (legacy driver issues)

**Solution:** Use RS-485 interface with any standard USB-to-RS485 converter (~$10-20). The FTDI and CH340-based adapters both work reliably.

### Known Gotchas

1. **Output enable logic is inverted:** `SOUT 01 0` = ON, `SOUT 01 1` = OFF
2. **Front panel locks during remote mode:** Must send `ENDS` to regain manual control
3. **Relay click on every output state change:** 100k operation lifetime
4. **No acknowledgment of bad commands:** Watch for timeout or missing `OK<CR>`
5. **4-digit readings vs 3-digit setpoints:** `GETD` returns more precision than `SET` accepts

### Preservation Note

If you still have the original Rapid Electronics packaging or documentation, consider scanning/photographing it for archival purposes. Many of these PSUs are still in daily use 20 years later, but documentation has become scarce.

---

**Document Version:** 1.1
**Last Updated:** 2026-02-02
**Hardware Tested:** Rapid 87-1752 at address 01
**License:** Public Domain - share freely!

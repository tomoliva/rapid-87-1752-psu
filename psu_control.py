#!/usr/bin/env python3
"""
Rapid 87-1752 / PeakTech 1860 / Manson NDP-4303 PSU Control GUI
RS-485 interface controller for 30V/3A programmable power supply
"""

import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import time
import sys
from dataclasses import dataclass
from typing import Optional, Callable


def check_port_available(port: str) -> tuple[bool, str]:
    """Check if a serial port is available. Returns (available, reason)."""
    try:
        ser = serial.Serial(port, 9600, timeout=0.1)
        ser.close()
        return True, "available"
    except serial.SerialException as e:
        if "busy" in str(e).lower() or "in use" in str(e).lower() or "permission" in str(e).lower():
            return False, "in use by another application"
        return False, str(e)
    except Exception as e:
        return False, str(e)


def get_port_status() -> tuple[list, list]:
    """Get lists of available and busy ports. Returns (available, busy)."""
    available = []
    busy = []
    for p in serial.tools.list_ports.comports():
        is_available, reason = check_port_available(p.device)
        if is_available:
            available.append(p.device)
        else:
            busy.append((p.device, reason))
    return available, busy


@dataclass
class PSUState:
    """Current PSU state"""
    voltage_set: float = 0.0
    current_set: float = 0.0
    voltage_out: float = 0.0
    current_out: float = 0.0
    mode: str = "CV"  # CV or CC
    output_on: bool = False
    remote_mode: bool = False
    ovp_limit: float = 31.0
    max_voltage: float = 31.0
    max_current: float = 3.10
    connected: bool = False


@dataclass
class LiPoChargeState:
    """LiPo charging state"""
    active: bool = False
    cell_count: int = 1
    target_voltage: float = 4.1
    charge_current: float = 1.0
    termination_current: float = 0.05  # C/20 default
    phase: str = "Idle"  # Idle, CC, CV, Done, Error
    start_time: float = 0.0
    charge_time: float = 0.0
    mah_charged: float = 0.0
    last_update: float = 0.0

    CELL_VOLTAGE = 4.1  # Termination voltage per cell


class PSUSerial:
    """Serial communication handler for the PSU"""

    def __init__(self, port: str = '/dev/ttyUSB0', address: str = '01', baudrate: int = 9600):
        self.port = port
        self.address = address
        self.baudrate = baudrate
        self.serial: Optional[serial.Serial] = None
        self.lock = threading.Lock()
        # Dynamic limits from GMAX - initialized to common defaults
        self.max_voltage: float = 31.0
        self.max_current: float = 3.10

    def connect(self) -> bool:
        """Connect to the PSU"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=8,
                parity='N',
                stopbits=1,
                timeout=1.0
            )
            time.sleep(0.1)
            self.serial.reset_input_buffer()
            return True
        except Exception as e:
            print(f"Connection error: {e}")
            return False

    def disconnect(self):
        """Disconnect from the PSU"""
        if self.serial and self.serial.is_open:
            self.serial.close()
        self.serial = None

    def send_command(self, cmd: str) -> str:
        """Send command and return response"""
        if not self.serial or not self.serial.is_open:
            return ""

        with self.lock:
            try:
                self.serial.reset_input_buffer()
                full_cmd = f"{cmd} {self.address}\r"
                self.serial.write(full_cmd.encode('ascii'))
                self.serial.flush()
                time.sleep(0.15)

                response = self.serial.read(self.serial.in_waiting or 500)
                return response.decode('ascii', errors='replace').strip()
            except Exception as e:
                print(f"Command error: {e}")
                return ""

    def send_command_with_params(self, cmd: str, params: str) -> str:
        """Send command with additional parameters"""
        if not self.serial or not self.serial.is_open:
            return ""

        with self.lock:
            try:
                self.serial.reset_input_buffer()
                full_cmd = f"{cmd} {self.address} {params}\r"
                self.serial.write(full_cmd.encode('ascii'))
                self.serial.flush()
                time.sleep(0.15)

                response = self.serial.read(self.serial.in_waiting or 500)
                return response.decode('ascii', errors='replace').strip()
            except Exception as e:
                print(f"Command error: {e}")
                return ""

    # Query commands
    def get_max(self) -> tuple[float, float]:
        """Get max voltage and current ratings from PSU via GMAX command"""
        resp = self.send_command("GMAX")
        if resp and "OK" in resp:
            data = resp.replace("OK", "").replace("\r", "").strip()
            if len(data) >= 6:
                try:
                    v = int(data[:3]) / 10.0
                    c = int(data[3:6]) / 100.0
                    # Store for use by set_voltage/set_current clamping
                    self.max_voltage = v
                    self.max_current = c
                    return (v, c)
                except ValueError:
                    pass
        return (self.max_voltage, self.max_current)

    def get_setpoints(self) -> tuple[float, float]:
        """Get voltage and current setpoints"""
        resp = self.send_command("GETS")
        if resp and "OK" in resp:
            data = resp.replace("OK", "").replace("\r", "").strip()
            if len(data) >= 6:
                try:
                    v = int(data[:3]) / 10.0
                    c = int(data[3:6]) / 100.0
                    return (v, c)
                except ValueError:
                    pass
        return (0.0, 0.0)

    def get_output(self) -> tuple[float, float, str]:
        """Get actual output voltage, current, and mode"""
        resp = self.send_command("GETD")
        if resp and "OK" in resp:
            data = resp.replace("OK", "").replace("\r", "").strip()
            if len(data) >= 9:
                try:
                    v = int(data[:4]) / 100.0
                    c = int(data[4:8]) / 1000.0
                    mode = "CC" if data[8] == '1' else "CV"
                    return (v, c, mode)
                except ValueError:
                    pass
        return (0.0, 0.0, "CV")

    def get_ovp(self) -> float:
        """Get OVP limit"""
        resp = self.send_command("GOVP")
        if resp and "OK" in resp:
            data = resp.replace("OK", "").replace("\r", "").strip()
            try:
                return int(data[:3]) / 10.0
            except ValueError:
                pass
        return self.max_voltage

    # Control commands
    def enter_remote(self) -> bool:
        """Enter remote mode (lock front panel)"""
        resp = self.send_command("SESS")
        return "OK" in resp

    def exit_remote(self) -> bool:
        """Exit remote mode (unlock front panel)"""
        resp = self.send_command("ENDS")
        return "OK" in resp

    def set_voltage(self, voltage: float) -> bool:
        """Set output voltage (clamped to PSU max from GMAX)"""
        v = int(voltage * 10)
        max_v = int(self.max_voltage * 10)
        v = max(0, min(max_v, v))
        resp = self.send_command_with_params("VOLT", f"{v:03d}")
        return "OK" in resp

    def set_current(self, current: float) -> bool:
        """Set current limit (clamped to PSU max from GMAX)"""
        c = int(current * 100)
        max_c = int(self.max_current * 100)
        c = max(0, min(max_c, c))
        resp = self.send_command_with_params("CURR", f"{c:03d}")
        return "OK" in resp

    def set_ovp(self, voltage: float) -> bool:
        """Set OVP limit (clamped to PSU max from GMAX)"""
        v = int(voltage * 10)
        max_v = int(self.max_voltage * 10)
        v = max(0, min(max_v, v))
        resp = self.send_command_with_params("SOVP", f"{v:03d}")
        return "OK" in resp

    def output_on(self) -> bool:
        """Enable output (note: 0 = ON in this protocol)"""
        resp = self.send_command_with_params("SOUT", "0")
        return "OK" in resp

    def output_off(self) -> bool:
        """Disable output (note: 1 = OFF in this protocol)"""
        resp = self.send_command_with_params("SOUT", "1")
        return "OK" in resp


def guess_psu_model(max_voltage: float, max_current: float) -> str:
    """Guess PSU model from GMAX values"""
    # Known OEM variants based on max ratings
    models = {
        (30.0, 3.00): "Rapid 87-1752 / PeakTech 1860 / Manson NDP-4303",
        (30.0, 3.10): "Rapid 87-1752 / PeakTech 1860 / Manson NDP-4303",
        (18.0, 5.00): "Manson NDP-4185",
        (60.0, 1.50): "Manson NDP-4601",
    }

    # Try exact match first
    key = (max_voltage, max_current)
    if key in models:
        return models[key]

    # Fuzzy match based on voltage class
    if 29.0 <= max_voltage <= 31.0 and 2.5 <= max_current <= 3.5:
        return "30V/3A Class PSU"
    elif 17.0 <= max_voltage <= 19.0 and 4.5 <= max_current <= 5.5:
        return "18V/5A Class PSU"
    elif 59.0 <= max_voltage <= 61.0 and 1.0 <= max_current <= 2.0:
        return "60V/1.5A Class PSU"

    # Unknown - show actual specs
    return f"{max_voltage:.0f}V/{max_current:.1f}A PSU"


class PSUControlGUI:
    """Main GUI application"""

    def __init__(self):
        self.root = tk.Tk()
        self.root.title("PSU Control - Connecting...")
        self.root.resizable(True, True)

        self.psu = PSUSerial()
        self.state = PSUState()
        self.lipo = LiPoChargeState()
        self.polling = False
        self.poll_thread: Optional[threading.Thread] = None
        self.port_monitor_active = True
        self.last_selected_port = '/dev/ttyUSB0'
        self.reconnect_attempts = 0
        self.max_reconnect_attempts = 3

        self._build_gui()
        self._bind_events()

        # Auto-connect on startup
        self.root.after(100, self.auto_connect)

        # Start port monitoring
        self.root.after(1000, self._monitor_port)

    def _build_gui(self):
        """Build the GUI layout"""
        # Main container with padding
        main = ttk.Frame(self.root, padding=10)
        main.grid(row=0, column=0, sticky="nsew")

        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main.columnconfigure(0, weight=1)

        # === Connection Frame ===
        conn_frame = ttk.LabelFrame(main, text="Connection", padding=5)
        conn_frame.grid(row=0, column=0, sticky="ew", pady=(0, 10))
        conn_frame.columnconfigure(1, weight=1)

        ttk.Label(conn_frame, text="Port:").grid(row=0, column=0, padx=5)
        self.port_var = tk.StringVar(value="/dev/ttyUSB0")
        self.port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, width=20)
        self.port_combo.grid(row=0, column=1, sticky="ew", padx=5)
        self._refresh_ports()

        ttk.Label(conn_frame, text="Addr:").grid(row=0, column=2, padx=5)
        self.addr_var = tk.StringVar(value="01")
        ttk.Entry(conn_frame, textvariable=self.addr_var, width=4).grid(row=0, column=3, padx=5)

        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=4, padx=5)

        self.status_label = ttk.Label(conn_frame, text="Disconnected", foreground="red")
        self.status_label.grid(row=0, column=5, padx=10)

        # === Output Display Frame ===
        display_frame = ttk.LabelFrame(main, text="Output", padding=10)
        display_frame.grid(row=1, column=0, sticky="ew", pady=(0, 10))
        display_frame.columnconfigure(0, weight=1)
        display_frame.columnconfigure(1, weight=1)

        # Voltage display
        v_frame = ttk.Frame(display_frame)
        v_frame.grid(row=0, column=0, padx=20, pady=5)

        self.voltage_display = tk.Label(
            v_frame, text="00.00", font=("Courier", 48, "bold"),
            fg="#00aa00", bg="black", width=6, anchor="e"
        )
        self.voltage_display.pack()
        ttk.Label(v_frame, text="Volts", font=("Arial", 12)).pack()

        # Current display
        c_frame = ttk.Frame(display_frame)
        c_frame.grid(row=0, column=1, padx=20, pady=5)

        self.current_display = tk.Label(
            c_frame, text="0.000", font=("Courier", 48, "bold"),
            fg="#00aa00", bg="black", width=6, anchor="e"
        )
        self.current_display.pack()
        ttk.Label(c_frame, text="Amps", font=("Arial", 12)).pack()

        # Mode indicator
        self.mode_label = ttk.Label(display_frame, text="CV", font=("Arial", 16, "bold"))
        self.mode_label.grid(row=1, column=0, columnspan=2, pady=5)

        # === Control Frame ===
        control_frame = ttk.LabelFrame(main, text="Controls", padding=10)
        control_frame.grid(row=2, column=0, sticky="ew", pady=(0, 10))
        control_frame.columnconfigure(1, weight=1)
        control_frame.columnconfigure(3, weight=1)

        # Voltage control
        ttk.Label(control_frame, text="Voltage (V):").grid(row=0, column=0, padx=5, pady=5, sticky="e")
        self.voltage_var = tk.StringVar(value="5.0")
        self.voltage_entry = ttk.Entry(control_frame, textvariable=self.voltage_var, width=10)
        self.voltage_entry.grid(row=0, column=1, padx=5, pady=5, sticky="w")

        self.voltage_scale = ttk.Scale(
            control_frame, from_=0, to=31.0, orient="horizontal",
            command=self._voltage_scale_changed
        )
        self.voltage_scale.grid(row=0, column=2, columnspan=2, sticky="ew", padx=5, pady=5)
        self.voltage_scale.set(5.0)

        ttk.Button(control_frame, text="Set V", command=self.set_voltage).grid(row=0, column=4, padx=5, pady=5)

        # Current control
        ttk.Label(control_frame, text="Current (A):").grid(row=1, column=0, padx=5, pady=5, sticky="e")
        self.current_var = tk.StringVar(value="1.00")
        self.current_entry = ttk.Entry(control_frame, textvariable=self.current_var, width=10)
        self.current_entry.grid(row=1, column=1, padx=5, pady=5, sticky="w")

        self.current_scale = ttk.Scale(
            control_frame, from_=0, to=3.10, orient="horizontal",
            command=self._current_scale_changed
        )
        self.current_scale.grid(row=1, column=2, columnspan=2, sticky="ew", padx=5, pady=5)
        self.current_scale.set(1.0)

        ttk.Button(control_frame, text="Set I", command=self.set_current).grid(row=1, column=4, padx=5, pady=5)

        # === Output Control Frame ===
        output_frame = ttk.Frame(main)
        output_frame.grid(row=3, column=0, sticky="ew", pady=(0, 10))
        output_frame.columnconfigure(0, weight=1)
        output_frame.columnconfigure(1, weight=1)
        output_frame.columnconfigure(2, weight=1)
        output_frame.columnconfigure(3, weight=1)

        self.output_btn = tk.Button(
            output_frame, text="OUTPUT OFF", font=("Arial", 14, "bold"),
            bg="#cc0000", fg="white", width=15, height=2,
            command=self.toggle_output
        )
        self.output_btn.grid(row=0, column=0, columnspan=2, padx=10, pady=5)

        self.remote_btn = ttk.Button(
            output_frame, text="Enter Remote", command=self.toggle_remote
        )
        self.remote_btn.grid(row=0, column=2, padx=10, pady=5)

        ttk.Button(output_frame, text="Refresh", command=self.refresh_state).grid(
            row=0, column=3, padx=10, pady=5
        )

        # === Setpoints Display ===
        setpoint_frame = ttk.LabelFrame(main, text="Setpoints", padding=5)
        setpoint_frame.grid(row=4, column=0, sticky="ew", pady=(0, 10))

        self.setpoint_label = ttk.Label(setpoint_frame, text="V: --.- V  |  I: -.-- A  |  OVP: --.- V")
        self.setpoint_label.pack()

        # === Presets Frame ===
        preset_frame = ttk.LabelFrame(main, text="Quick Presets", padding=5)
        preset_frame.grid(row=5, column=0, sticky="ew", pady=(0, 10))

        presets = [
            ("3.3V", 3.3, 0.5),
            ("5V", 5.0, 1.0),
            ("9V", 9.0, 1.0),
            ("12V", 12.0, 1.0),
            ("24V", 24.0, 1.0),
        ]

        for i, (label, v, c) in enumerate(presets):
            btn = ttk.Button(
                preset_frame, text=label,
                command=lambda v=v, c=c: self.apply_preset(v, c)
            )
            btn.pack(side="left", padx=5, pady=5)

        # === LiPo Charging Frame ===
        lipo_frame = ttk.LabelFrame(main, text="LiPo Charger (4.1V/cell)", padding=5)
        lipo_frame.grid(row=6, column=0, sticky="ew", pady=(0, 10))

        # Row 0: Cell count and current
        lipo_row0 = ttk.Frame(lipo_frame)
        lipo_row0.pack(fill="x", pady=2)

        ttk.Label(lipo_row0, text="Cells:").pack(side="left", padx=5)
        self.lipo_cells_var = tk.StringVar(value="1")
        self.lipo_cells_combo = ttk.Combobox(
            lipo_row0, textvariable=self.lipo_cells_var, width=4,
            values=["1", "2", "3", "4", "5", "6", "7"], state="readonly"
        )
        self.lipo_cells_combo.pack(side="left", padx=5)
        self.lipo_cells_combo.bind("<<ComboboxSelected>>", self._lipo_cells_changed)

        self.lipo_voltage_label = ttk.Label(lipo_row0, text="(4.1V)", width=10)
        self.lipo_voltage_label.pack(side="left", padx=5)

        ttk.Label(lipo_row0, text="Current (A):").pack(side="left", padx=(20, 5))
        self.lipo_current_var = tk.StringVar(value="1.00")
        ttk.Entry(lipo_row0, textvariable=self.lipo_current_var, width=6).pack(side="left", padx=5)

        ttk.Label(lipo_row0, text="Term (A):").pack(side="left", padx=(20, 5))
        self.lipo_term_var = tk.StringVar(value="0.05")
        ttk.Entry(lipo_row0, textvariable=self.lipo_term_var, width=6).pack(side="left", padx=5)

        # Row 1: Start/Stop button and status
        lipo_row1 = ttk.Frame(lipo_frame)
        lipo_row1.pack(fill="x", pady=5)

        self.lipo_start_btn = tk.Button(
            lipo_row1, text="Start Charge", font=("Arial", 11, "bold"),
            bg="#0066cc", fg="white", width=12,
            command=self.toggle_lipo_charge
        )
        self.lipo_start_btn.pack(side="left", padx=10)

        self.lipo_status_label = ttk.Label(
            lipo_row1, text="Phase: Idle", font=("Arial", 10)
        )
        self.lipo_status_label.pack(side="left", padx=10)

        self.lipo_time_label = ttk.Label(lipo_row1, text="Time: 00:00:00")
        self.lipo_time_label.pack(side="left", padx=10)

        self.lipo_mah_label = ttk.Label(lipo_row1, text="Charged: 0 mAh")
        self.lipo_mah_label.pack(side="left", padx=10)

        # === Log Frame ===
        log_frame = ttk.LabelFrame(main, text="Log", padding=5)
        log_frame.grid(row=7, column=0, sticky="nsew", pady=(0, 5))
        main.rowconfigure(7, weight=1)

        self.log_text = tk.Text(log_frame, height=6, width=60, font=("Courier", 9))
        self.log_text.pack(fill="both", expand=True)

        scrollbar = ttk.Scrollbar(self.log_text, command=self.log_text.yview)
        scrollbar.pack(side="right", fill="y")
        self.log_text.config(yscrollcommand=scrollbar.set)

    def _bind_events(self):
        """Bind keyboard events"""
        self.voltage_entry.bind("<Return>", lambda e: self.set_voltage())
        self.current_entry.bind("<Return>", lambda e: self.set_current())
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def _refresh_ports(self):
        """Refresh available serial ports, preferring available ones"""
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports

        # If no port selected yet, prefer an available port
        if ports and not self.port_var.get():
            available, busy = get_port_status()
            if available:
                self.port_var.set(available[0])
            else:
                self.port_var.set(ports[0])

    def _voltage_scale_changed(self, value):
        """Handle voltage scale change"""
        self.voltage_var.set(f"{float(value):.1f}")

    def _current_scale_changed(self, value):
        """Handle current scale change"""
        self.current_var.set(f"{float(value):.2f}")

    def log(self, msg: str):
        """Add message to log"""
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.insert("end", f"[{timestamp}] {msg}\n")
        self.log_text.see("end")

    def auto_connect(self):
        """Attempt auto-connection on startup"""
        self.last_selected_port = self.port_var.get()
        if self.psu.connect():
            self.psu.address = self.addr_var.get()
            # Verify PSU responds
            test = self.psu.send_command("GMAX")
            if "OK" in test:
                self.state.connected = True
                self.connect_btn.config(text="Disconnect")
                self.status_label.config(text="Connected", foreground="green")
                self.log("Connected to PSU")
                self._update_port_indicator(True)
                self.start_polling()
                self.refresh_state()
            else:
                self.psu.disconnect()
                self.log("Port open but PSU not responding")
                self.status_label.config(text="No Response", foreground="orange")
                self._update_port_indicator(False)

    def toggle_connection(self):
        """Connect or disconnect from PSU"""
        if self.state.connected:
            self.stop_polling()
            if self.state.remote_mode:
                self.psu.exit_remote()
                self.state.remote_mode = False
                self.remote_btn.config(text="Enter Remote")
            self.psu.disconnect()
            self.state.connected = False
            self.connect_btn.config(text="Connect")
            self.status_label.config(text="Disconnected", foreground="red")
            self._update_port_indicator(False)
            self.log("Disconnected")
        else:
            self.psu.port = self.port_var.get()
            self.psu.address = self.addr_var.get()
            self.last_selected_port = self.psu.port
            self.reconnect_attempts = 0
            if self.psu.connect():
                # Verify PSU responds
                test = self.psu.send_command("GMAX")
                if "OK" in test:
                    self.state.connected = True
                    self.connect_btn.config(text="Disconnect")
                    self.status_label.config(text="Connected", foreground="green")
                    self._update_port_indicator(True)
                    self.log(f"Connected to {self.psu.port}")
                    self.start_polling()
                    self.refresh_state()
                else:
                    self.psu.disconnect()
                    self.log("Port open but PSU not responding")
                    self.status_label.config(text="No Response", foreground="orange")
            else:
                # Check why connection failed
                port = self.psu.port
                is_available, reason = check_port_available(port)
                available, busy = get_port_status()

                if not is_available:
                    self.log(f"Port {port} is {reason}")
                    self.status_label.config(text="Port Busy", foreground="orange")

                    msg = f"Cannot open {port}: {reason}\n\n"
                    if available:
                        msg += f"Available ports: {', '.join(available)}"
                    else:
                        msg += "No available ports found."
                    messagebox.showerror("Connection Failed", msg)
                else:
                    self.log("Connection failed")
                    messagebox.showerror("Error", f"Failed to connect to {port}")

    def start_polling(self):
        """Start background polling thread"""
        self.polling = True
        self.poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
        self.poll_thread.start()

    def stop_polling(self):
        """Stop background polling"""
        self.polling = False
        if self.poll_thread:
            self.poll_thread.join(timeout=1.0)

    def _poll_loop(self):
        """Background polling loop"""
        while self.polling and self.state.connected:
            try:
                v, c, mode = self.psu.get_output()
                self.state.voltage_out = v
                self.state.current_out = c
                self.state.mode = mode

                # Detect if output is on based on current flow or voltage presence
                self.state.output_on = c > 0.001 or (v > 0.5 and self.state.voltage_set > 0)

                # Update display in main thread
                self.root.after(0, self._update_display)

                # Update LiPo charging if active
                if self.lipo.active:
                    self._update_lipo_charge()

            except Exception as e:
                self.root.after(0, lambda: self.log(f"Poll error: {e}"))

            time.sleep(0.5)

    def _update_display(self):
        """Update display with current state"""
        self.voltage_display.config(text=f"{self.state.voltage_out:05.2f}")
        self.current_display.config(text=f"{self.state.current_out:5.3f}")

        # Mode indicator color
        if self.state.mode == "CC":
            self.mode_label.config(text="CC", foreground="red")
        else:
            self.mode_label.config(text="CV", foreground="green")

        # Output button state
        if self.state.output_on:
            self.output_btn.config(text="OUTPUT ON", bg="#00aa00")
        else:
            self.output_btn.config(text="OUTPUT OFF", bg="#cc0000")

    def refresh_state(self):
        """Refresh all state from PSU"""
        if not self.state.connected:
            return

        # Get setpoints
        v, c = self.psu.get_setpoints()
        self.state.voltage_set = v
        self.state.current_set = c

        # Get OVP
        self.state.ovp_limit = self.psu.get_ovp()

        # Get max ratings and update window title
        self.state.max_voltage, self.state.max_current = self.psu.get_max()
        model = guess_psu_model(self.state.max_voltage, self.state.max_current)
        self.root.title(f"PSU Control - {model}")

        # Update setpoint label
        self.setpoint_label.config(
            text=f"V: {self.state.voltage_set:.1f} V  |  I: {self.state.current_set:.2f} A  |  OVP: {self.state.ovp_limit:.1f} V"
        )

        # Update sliders to match PSU limits
        self.voltage_scale.config(to=self.state.max_voltage)
        self.current_scale.config(to=self.state.max_current)

        self.log(f"PSU max: {self.state.max_voltage:.1f}V / {self.state.max_current:.2f}A")
        self.log(f"Setpoints: {self.state.voltage_set:.1f}V, {self.state.current_set:.2f}A")

    def toggle_remote(self):
        """Toggle remote mode"""
        if not self.state.connected:
            return

        if self.state.remote_mode:
            if self.psu.exit_remote():
                self.state.remote_mode = False
                self.remote_btn.config(text="Enter Remote")
                self.log("Exited remote mode")
        else:
            if self.psu.enter_remote():
                self.state.remote_mode = True
                self.remote_btn.config(text="Exit Remote")
                self.log("Entered remote mode (front panel locked)")

    def toggle_output(self):
        """Toggle output on/off"""
        if not self.state.connected:
            return

        # Auto-enter remote mode if needed
        if not self.state.remote_mode:
            if self.psu.enter_remote():
                self.state.remote_mode = True
                self.remote_btn.config(text="Exit Remote")
                self.log("Auto-entered remote mode")

        if self.state.output_on:
            if self.psu.output_off():
                self.state.output_on = False
                self.output_btn.config(text="OUTPUT OFF", bg="#cc0000")
                self.log("Output OFF")
        else:
            if self.psu.output_on():
                self.state.output_on = True
                self.output_btn.config(text="OUTPUT ON", bg="#00aa00")
                self.log("Output ON")

    def set_voltage(self):
        """Set voltage from entry"""
        if not self.state.connected:
            return

        # Auto-enter remote mode if needed
        if not self.state.remote_mode:
            if self.psu.enter_remote():
                self.state.remote_mode = True
                self.remote_btn.config(text="Exit Remote")

        try:
            v = float(self.voltage_var.get())
            if self.psu.set_voltage(v):
                self.state.voltage_set = v
                self.log(f"Voltage set to {v:.1f} V")
                self.refresh_state()
            else:
                self.log("Failed to set voltage")
        except ValueError:
            self.log("Invalid voltage value")

    def set_current(self):
        """Set current limit from entry"""
        if not self.state.connected:
            return

        # Auto-enter remote mode if needed
        if not self.state.remote_mode:
            if self.psu.enter_remote():
                self.state.remote_mode = True
                self.remote_btn.config(text="Exit Remote")

        try:
            c = float(self.current_var.get())
            if self.psu.set_current(c):
                self.state.current_set = c
                self.log(f"Current limit set to {c:.2f} A")
                self.refresh_state()
            else:
                self.log("Failed to set current")
        except ValueError:
            self.log("Invalid current value")

    def apply_preset(self, voltage: float, current: float):
        """Apply a preset voltage and current"""
        if not self.state.connected:
            return

        # Auto-enter remote mode if needed
        if not self.state.remote_mode:
            if self.psu.enter_remote():
                self.state.remote_mode = True
                self.remote_btn.config(text="Exit Remote")

        self.voltage_var.set(f"{voltage:.1f}")
        self.current_var.set(f"{current:.2f}")
        self.voltage_scale.set(voltage)
        self.current_scale.set(current)

        if self.psu.set_voltage(voltage) and self.psu.set_current(current):
            self.log(f"Preset applied: {voltage:.1f}V, {current:.2f}A")
            self.refresh_state()

    def _monitor_port(self):
        """Monitor for port presence/absence and handle reconnection"""
        if not self.port_monitor_active:
            return

        available_ports = [p.device for p in serial.tools.list_ports.comports()]
        port = self.last_selected_port

        if self.state.connected:
            # Check if our port disappeared
            if port not in available_ports:
                self.log(f"Port {port} disconnected!")
                if self.lipo.active:
                    self.lipo.active = False
                    self.lipo.phase = "Error"
                    self.log("LiPo charge aborted: port lost!")
                    self.lipo_start_btn.config(text="Start Charge", bg="#0066cc")
                    self.lipo_status_label.config(text="Phase: ERROR", foreground="red")
                self.state.connected = False
                self.stop_polling()
                self.psu.disconnect()
                self.connect_btn.config(text="Connect")
                self.status_label.config(text="Port Lost", foreground="orange")
                self.reconnect_attempts = 0
                self._update_port_indicator(False)
        else:
            # Check if our port reappeared
            if port in available_ports and self.reconnect_attempts < self.max_reconnect_attempts:
                self.log(f"Port {port} detected, reconnecting...")
                self.reconnect_attempts += 1
                self.psu.port = port
                if self.psu.connect():
                    # Verify PSU responds
                    test = self.psu.send_command("GMAX")
                    if "OK" in test:
                        self.state.connected = True
                        self.connect_btn.config(text="Disconnect")
                        self.status_label.config(text="Connected", foreground="green")
                        self.log("Reconnected successfully")
                        self.reconnect_attempts = 0
                        self.start_polling()
                        self.refresh_state()
                        self._update_port_indicator(True)
                    else:
                        self.psu.disconnect()
                        self.log(f"Port open but PSU not responding (attempt {self.reconnect_attempts})")
                        self.status_label.config(text="No Response", foreground="orange")
                else:
                    self.log(f"Reconnect failed (attempt {self.reconnect_attempts})")

        # Update port combo highlighting
        self._refresh_ports()

        # Schedule next check
        self.root.after(2000, self._monitor_port)

    def _update_port_indicator(self, connected: bool):
        """Update visual indicators for port status"""
        if connected:
            self.voltage_display.config(fg="#00aa00")
            self.current_display.config(fg="#00aa00")
        else:
            self.voltage_display.config(fg="#666666")
            self.current_display.config(fg="#666666")
            self.voltage_display.config(text="--.--")
            self.current_display.config(text="-.---")

    # === LiPo Charging Methods ===

    def _lipo_cells_changed(self, event=None):
        """Update voltage label when cell count changes"""
        try:
            cells = int(self.lipo_cells_var.get())
            voltage = cells * LiPoChargeState.CELL_VOLTAGE
            self.lipo_voltage_label.config(text=f"({voltage:.1f}V)")
        except ValueError:
            pass

    def toggle_lipo_charge(self):
        """Start or stop LiPo charging"""
        if self.lipo.active:
            self.stop_lipo_charge()
        else:
            self.start_lipo_charge()

    def start_lipo_charge(self):
        """Start LiPo charging process"""
        if not self.state.connected:
            self.log("Cannot start charge: not connected")
            return

        try:
            cells = int(self.lipo_cells_var.get())
            current = float(self.lipo_current_var.get())
            term_current = float(self.lipo_term_var.get())
        except ValueError:
            self.log("Invalid charge parameters")
            return

        target_voltage = cells * LiPoChargeState.CELL_VOLTAGE

        # Safety check: PSU max voltage
        if target_voltage > self.state.max_voltage:
            self.log(f"Error: {cells}S requires {target_voltage:.1f}V, PSU max is {self.state.max_voltage:.1f}V")
            return

        # Safety check: current limit
        if current > self.state.max_current:
            self.log(f"Error: {current:.2f}A exceeds PSU max {self.state.max_current:.2f}A")
            return

        # Enter remote mode if needed
        if not self.state.remote_mode:
            if self.psu.enter_remote():
                self.state.remote_mode = True
                self.remote_btn.config(text="Exit Remote")
                self.log("Entered remote mode")
            else:
                self.log("Failed to enter remote mode")
                return

        # Set up charging parameters
        self.lipo.cell_count = cells
        self.lipo.target_voltage = target_voltage
        self.lipo.charge_current = current
        self.lipo.termination_current = term_current
        self.lipo.phase = "CC"
        self.lipo.start_time = time.time()
        self.lipo.last_update = time.time()
        self.lipo.mah_charged = 0.0
        self.lipo.active = True

        # Configure PSU: set voltage and current
        if not self.psu.set_voltage(target_voltage):
            self.log("Failed to set voltage")
            self.lipo.active = False
            return

        if not self.psu.set_current(current):
            self.log("Failed to set current")
            self.lipo.active = False
            return

        # Enable output
        if not self.psu.output_on():
            self.log("Failed to enable output")
            self.lipo.active = False
            return

        self.state.output_on = True
        self.output_btn.config(text="OUTPUT ON", bg="#00aa00")

        # Update UI
        self.lipo_start_btn.config(text="Stop Charge", bg="#cc0000")
        self.lipo_status_label.config(text="Phase: CC", foreground="blue")
        self.log(f"LiPo charge started: {cells}S @ {current:.2f}A to {target_voltage:.1f}V")

    def stop_lipo_charge(self):
        """Stop LiPo charging"""
        self.lipo.active = False
        self.lipo.phase = "Idle"

        # Turn off output
        if self.state.connected:
            self.psu.output_off()
            self.state.output_on = False
            self.output_btn.config(text="OUTPUT OFF", bg="#cc0000")

        # Update UI
        self.lipo_start_btn.config(text="Start Charge", bg="#0066cc")
        self.lipo_status_label.config(text="Phase: Stopped", foreground="black")
        self.log(f"LiPo charge stopped. Total: {self.lipo.mah_charged:.0f} mAh")

    def _update_lipo_charge(self):
        """Update LiPo charging state - called from poll loop"""
        if not self.lipo.active:
            return

        now = time.time()
        dt = now - self.lipo.last_update
        self.lipo.last_update = now

        # Calculate mAh charged (current in A * time in hours * 1000)
        self.lipo.mah_charged += self.state.current_out * (dt / 3600.0) * 1000.0

        # Update charge time
        self.lipo.charge_time = now - self.lipo.start_time

        # Check charging phase
        if self.lipo.phase == "CC":
            # In CC mode, waiting for voltage to reach target
            if self.state.voltage_out >= (self.lipo.target_voltage - 0.05):
                self.lipo.phase = "CV"
                self.log(f"LiPo entering CV phase at {self.state.voltage_out:.2f}V")

        elif self.lipo.phase == "CV":
            # In CV mode, waiting for current to drop below termination
            if self.state.current_out <= self.lipo.termination_current:
                self.lipo.phase = "Done"
                self.log(f"LiPo charge complete! {self.lipo.mah_charged:.0f} mAh")
                # Turn off output
                self.psu.output_off()
                self.state.output_on = False
                self.output_btn.config(text="OUTPUT OFF", bg="#cc0000")
                self.lipo.active = False

        # Safety timeout: 8 hours max
        if self.lipo.charge_time > 8 * 3600:
            self.lipo.phase = "Error"
            self.log("LiPo charge timeout (8 hours)")
            self.stop_lipo_charge()

        # Update UI in main thread
        self.root.after(0, self._update_lipo_display)

    def _update_lipo_display(self):
        """Update LiPo charging display"""
        # Time display
        hours = int(self.lipo.charge_time // 3600)
        mins = int((self.lipo.charge_time % 3600) // 60)
        secs = int(self.lipo.charge_time % 60)
        self.lipo_time_label.config(text=f"Time: {hours:02d}:{mins:02d}:{secs:02d}")

        # mAh display
        self.lipo_mah_label.config(text=f"Charged: {self.lipo.mah_charged:.0f} mAh")

        # Phase display with colors
        phase = self.lipo.phase
        if phase == "CC":
            self.lipo_status_label.config(text="Phase: CC (Const Current)", foreground="blue")
        elif phase == "CV":
            self.lipo_status_label.config(text="Phase: CV (Const Voltage)", foreground="orange")
        elif phase == "Done":
            self.lipo_status_label.config(text="Phase: DONE", foreground="green")
            self.lipo_start_btn.config(text="Start Charge", bg="#0066cc")
        elif phase == "Error":
            self.lipo_status_label.config(text="Phase: ERROR", foreground="red")
            self.lipo_start_btn.config(text="Start Charge", bg="#0066cc")

    def on_close(self):
        """Handle window close"""
        self.port_monitor_active = False
        if self.lipo.active:
            self.stop_lipo_charge()
        self.stop_polling()
        if self.state.connected:
            if self.state.remote_mode:
                self.psu.exit_remote()
            self.psu.disconnect()
        self.root.destroy()

    def run(self):
        """Start the GUI main loop"""
        self.root.mainloop()


if __name__ == "__main__":
    # Check for available serial ports before starting
    available, busy = get_port_status()

    if not available and not busy:
        root = tk.Tk()
        root.withdraw()
        messagebox.showerror(
            "No Serial Ports",
            "No serial ports detected.\n\n"
            "Please connect a USB-to-RS485 adapter."
        )
        root.destroy()
        sys.exit(1)

    if not available and busy:
        busy_list = "\n".join([f"  {port}: {reason}" for port, reason in busy])
        root = tk.Tk()
        root.withdraw()
        messagebox.showerror(
            "No Available Ports",
            f"All serial ports are in use:\n\n{busy_list}\n\n"
            "Please close other applications using these ports,\n"
            "or connect another USB-to-RS485 adapter."
        )
        root.destroy()
        sys.exit(1)

    # Show port status in console
    if available:
        print(f"Available ports: {', '.join(available)}")
    if busy:
        print(f"Busy ports: {', '.join([p for p, r in busy])}")

    app = PSUControlGUI()
    app.run()

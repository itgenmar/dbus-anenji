#!/usr/bin/env python3
"""
Final Anenji -> Victron VE.Bus emulator (Victron-compatible outputs)

- Full register map implemented (based on your table)
- Fault & warning bitfield decoding
- Read/write controls for writable registers
- Mode-aware DC-overridden energy model (no double counting)
- PV daily accumulator persistence
- Safe Modbus write wrapper and validation
- Designed for Venus OS / Linux
"""

import os
import sys
import json
import logging
import signal
from time import time, sleep
from threading import Event

import dbus
import dbus.mainloop.glib
from gi.repository import GLib

# allow velib_python in ext/
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(BASE_DIR, "ext", "velib_python"))
from vedbus import VeDbusService  # vendored velib

import minimalmodbus
import serial

# ---- Create true independent dbus connections (same pattern as Victron dbusmonitor.py) ----
class SystemBus(dbus.bus.BusConnection):
    def __new__(cls):
        return dbus.bus.BusConnection.__new__(cls, dbus.bus.BusConnection.TYPE_SYSTEM)


# ---------------- CONFIG ----------------
ANENJI_PORT    = os.environ.get("ANENJI_PORT", "/dev/ttyUSB3")
ANENJI_ID      = int(os.environ.get("ANENJI_ID", "1"))
ANENJI_BAUD    = int(os.environ.get("ANENJI_BAUD", "9600"))
ANENJI_TIMEOUT = float(os.environ.get("ANENJI_TIMEOUT", "1.5"))
POLL_SEC       = float(os.environ.get("ANENJI_POLL", "2.0"))

SERVICE_NAME   = os.environ.get("SERVICE_NAME", "com.victronenergy.vebus.ttyS3")
PRODUCT_ID     = int(os.environ.get("PRODUCT_ID", "41727"))  # 0xA31F
PRODUCT_NAME   = os.environ.get("PRODUCT_NAME", "MultiPlus-II (Anenji bridge)")
FIRMWARE_VER   = os.environ.get("FIRMWARE_VER", "v4.0-anenji-final")
DEVICE_INSTANCE= int(os.environ.get("ANENJI_DEVICE_INSTANCE", "28"))

PHASE_COUNT        = int(os.environ.get("PHASE_COUNT", "1"))
INVERTER_MAX_POWER = int(os.environ.get("ANENJI_INV_MAX_POWER", "3000"))

# SolarCharger (MPPT) service (optional: makes an MPPT device appear)
PV_SERVICE_NAME    = os.environ.get("PV_SERVICE_NAME", "com.victronenergy.solarcharger.anenji")
PV_DEVICE_INSTANCE = int(os.environ.get("PV_DEVICE_INSTANCE", "4500"))
PV_PRODUCT_ID      = int(os.environ.get("PV_PRODUCT_ID", "0xA067"), 0)
PV_PRODUCT_NAME    = os.environ.get("PV_PRODUCT_NAME", "Anenji MPPT")


ENERGY_FILE     = os.environ.get("ENERGY_FILE", os.path.join(BASE_DIR, "energy.json"))
PV_PERSIST_FILE = os.environ.get("PV_PERSIST_FILE", os.path.join(BASE_DIR, "pv_today.json"))
ENERGY_SAVE_SEC = int(os.environ.get("ENERGY_SAVE_SEC", "60"))

GRID_VOLTAGE_THRESHOLD = float(os.environ.get("GRID_VOLTAGE_THRESHOLD", "180.0"))
GRID_DEBOUNCE_SEC      = float(os.environ.get("GRID_DEBOUNCE_SEC", "4.0"))

# register mappings for write (optional): set env var to a register number to map a control
REG_MAP = {
    "CHARGE_ENABLE": os.environ.get("ANENJI_REG_CHARGE_ENABLE"),    # optional
    "CHARGE_CURRENT": os.environ.get("ANENJI_REG_CHARGE_CURRENT"),
    "INVERTER_ENABLE": os.environ.get("ANENJI_REG_INVERTER_ENABLE"),
    "GRID_LIMIT": os.environ.get("ANENJI_REG_GRID_LIMIT"),
}

# safety limits
SAFETY_MAX_CHARGE_CURRENT_A = float(os.environ.get("SAFETY_MAX_CHARGE_CURRENT_A", "200.0"))
SAFETY_MIN_OUTPUT_V = float(os.environ.get("SAFETY_MIN_OUTPUT_V", "180.0"))
SAFETY_MAX_OUTPUT_V = float(os.environ.get("SAFETY_MAX_OUTPUT_V", "260.0"))
SAFETY_MIN_FREQ = float(os.environ.get("SAFETY_MIN_FREQ", "45.0"))
SAFETY_MAX_FREQ = float(os.environ.get("SAFETY_MAX_FREQ", "65.0"))

# logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s:%(name)s:%(message)s")
log = logging.getLogger("anenji-vebus")

# globals
time_driver_started = int(time())
stop_event = Event()

# energy structure (kWh)
_energy = {
    "AcIn1ToInverter_kWh": 0.0,
    "AcIn1ToAcOut_kWh": 0.0,
    "InverterToAcOut_kWh": 0.0,
    "ToGrid_kWh": 0.0,
    "FromGrid_kWh": 0.0,
}
_last_energy_save = 0.0

# pv today (Wh)
_pv_today_wh = 0.0
_pv_today_ts = int(time())
_pv_max_power_today = 0.0

# debounce
_last_grid_seen = 0.0
_grid_confirmed = False

# ---------- Fault & Warning bit maps (from your table) ----------
FAULT_BITS = {
    0: "Reserve",
    1: "Over temperature of DCDC module",
    2: "Battery over voltage",
    3: "Reserve",
    4: "Output short circuited",
    5: "Over Inverter voltage",
    6: "Output over load",
    7: "Bus over voltage",
    8: "Bus soft start times out",
    9: "PV over current",
    10: "PV over voltage",
    11: "Battery over current",
    12: "Inverter over current",
    13: "Bus low voltage",
    14: "Reserve",
    15: "Inverter DC component is too high",
    16: "Reserve",
    17: "The zero bias of Output current is too large",
    18: "The zero bias of inverter current is too large",
    19: "The zero bias of battery current is too large",
    20: "The zero bias of PV current is too large",
    21: "Inverter low voltage",
    22: "Inverter negative power protection",
    23: "The host in the parallel system is lost",
    24: "Synchronization signal abnormal in the parallel system",
    25: "Reserve",
    26: "Parallel versions are incompatible",
    # bits beyond 26 reserved/unknown
}

WARNING_BITS = {
    0: "Zero crossing loss of mains power",
    1: "Mains waveform abnormal",
    2: "Mains over voltage",
    3: "Mains low voltage",
    4: "Mains over frequency",
    5: "Mains low frequency",
    6: "PV low voltage",
    7: "Over temperature",
    8: "Battery low voltage",
    9: "Battery is not connected",
    10: "Overload",
    11: "Battery Eq charging",
    12: "Battery is discharged at a low voltage and it has not been charged back to the recovery point",
    13: "Output power derating",
    14: "Fan blocked",
    15: "PV energy is too low to be used",
    16: "Parallel communication interrupted",
    17: "Output mode of Single and Parallel systems is inconsistent",
    18: "Battery voltage difference of parallel system is too large",
    # beyond 18 unknown/reserved
}

# ---------- Helpers ----------
def mkdir_p(path):
    d = os.path.dirname(path)
    if d and not os.path.exists(d):
        os.makedirs(d, exist_ok=True)

def safe_float(x, default=0.0):
    try:
        return float(x)
    except Exception:
        return default

# ---------- Persistence ----------
def _load_energy():
    global _energy
    if os.path.exists(ENERGY_FILE):
        try:
            with open(ENERGY_FILE, "r") as f:
                data = json.load(f)
            for k in _energy:
                if k in data:
                    _energy[k] = float(data[k])
            log.info("Loaded energy file: %s", ENERGY_FILE)
        except Exception as e:
            log.error("Failed load energy: %s", e)
    else:
        log.info("No energy file found; starting fresh.")

def _save_energy():
    global _last_energy_save
    now = time()
    if now - _last_energy_save < ENERGY_SAVE_SEC:
        return
    try:
        mkdir_p(ENERGY_FILE)
        with open(ENERGY_FILE, "w") as f:
            json.dump(_energy, f)
        _last_energy_save = now
        log.debug("Saved energy")
    except Exception as e:
        log.error("Failed save energy: %s", e)

def _load_pv_today():
    global _pv_today_wh, _pv_today_ts
    if os.path.exists(PV_PERSIST_FILE):
        try:
            with open(PV_PERSIST_FILE, "r") as f:
                data = json.load(f)
            _pv_today_wh = float(data.get("wh", 0.0))
            _pv_today_ts = int(data.get("ts", int(time())))
            log.info("Loaded PV today: %.3f kWh", _pv_today_wh / 1000.0)
        except Exception as e:
            log.debug("Failed load pv today: %s", e)

def _save_pv_today():
    try:
        mkdir_p(PV_PERSIST_FILE)
        with open(PV_PERSIST_FILE, "w") as f:
            json.dump({"wh": _pv_today_wh, "ts": _pv_today_ts}, f)
    except Exception as e:
        log.debug("Failed save pv today: %s", e)

# ---------- Modbus helpers ----------
def mk_instrument():
    inv = minimalmodbus.Instrument(ANENJI_PORT, ANENJI_ID)
    inv.serial.baudrate = ANENJI_BAUD
    inv.serial.timeout  = ANENJI_TIMEOUT
    inv.serial.bytesize = 8
    inv.serial.stopbits = 1
    inv.serial.parity   = serial.PARITY_NONE
    inv.mode = minimalmodbus.MODE_RTU
    inv.clear_buffers_before_each_transaction = True
    inv.close_port_after_each_call = True
    return inv

def r16(inv, reg, signed=False):
    sleep(0.01)
    return inv.read_register(reg, 0, functioncode=3, signed=signed)

def r16_block_as_ascii(inv, start_reg, count):
    """Read ASCII from consecutive registers (16-bit each)."""
    out = []
    for i in range(count):
        try:
            v = inv.read_register(start_reg + i, 0, functioncode=3, signed=False)
            # each register contains two ASCII chars in high+low bytes
            hi = (v >> 8) & 0xFF
            lo = v & 0xFF
            if hi != 0:
                out.append(chr(hi))
            if lo != 0:
                out.append(chr(lo))
        except Exception:
            break
    return "".join(out)

def w16(inv, reg, value, decimals=0):
    sleep(0.02)
    return inv.write_register(reg, int(round(value * (10**decimals))), decimals, functioncode=16)

def try_write_register(reg_env, value, decimals=0):
    if not reg_env:
        return False
    try:
        reg = int(reg_env)
    except Exception:
        log.error("Bad reg mapping env: %s", reg_env)
        return False
    try:
        inv = mk_instrument()
        w16(inv, reg, value, decimals)
        log.info("Wrote %s to reg %d (dec=%d)", value, reg, decimals)
        return True
    except Exception as e:
        log.error("Modbus write failed reg %d: %s", reg, e)
        return False

# ---------- Fault & warning decode ----------
def decode_bitfield_32(low_reg, high_reg):
    """Combine two 16-bit registers into a 32-bit int; low_reg, high_reg are integers."""
    return (high_reg << 16) | (low_reg & 0xFFFF)

def decode_faults_from_32bit(val32):
    out = []
    for b, text in FAULT_BITS.items():
        if (val32 >> b) & 1:
            out.append(text)
    return out

def decode_warnings_from_32bit(val32):
    out = []
    for b, text in WARNING_BITS.items():
        if (val32 >> b) & 1:
            out.append(text)
    return out

# ---------- Validation helpers ----------
def _validate_charge_current(amps):
    if amps < 0:
        return 0.0
    return min(amps, SAFETY_MAX_CHARGE_CURRENT_A)

def _validate_output_voltage(v):
    return max(SAFETY_MIN_OUTPUT_V, min(SAFETY_MAX_OUTPUT_V, v))

def _validate_frequency(hz):
    return max(SAFETY_MIN_FREQ, min(SAFETY_MAX_FREQ, hz))

# ---------- Control high-level functions (map to registers listed) ----------
def inverter_remote_on(inv):
    try:
        w16(inv, 420, 1)
        return True
    except Exception as e:
        log.error("remote_on failed: %s", e)
        return False

def inverter_remote_off(inv):
    try:
        w16(inv, 420, 0)
        return True
    except Exception as e:
        log.error("remote_off failed: %s", e)
        return False

def inverter_exit_fault(inv):
    try:
        w16(inv, 426, 1)
        return True
    except Exception as e:
        log.error("exit_fault failed: %s", e)
        return False

def inverter_set_output_mode(inv, mode):
    try:
        w16(inv, 300, int(mode))
        return True
    except Exception as e:
        log.error("set_output_mode failed: %s", e)
        return False

def inverter_set_priority(inv, mode):
    try:
        w16(inv, 301, int(mode))
        return True
    except Exception as e:
        log.error("set_priority failed: %s", e)
        return False

def inverter_set_input_range(inv, narrow):
    try:
        w16(inv, 302, 1 if narrow else 0)
        return True
    except Exception as e:
        log.error("set_input_range failed: %s", e)
        return False

def inverter_set_output_voltage(inv, volts):
    volts = _validate_output_voltage(volts)
    try:
        w16(inv, 320, int(round(volts * 10)))
        return True
    except Exception as e:
        log.error("set_output_voltage failed: %s", e)
        return False

def inverter_set_output_frequency(inv, hz):
    hz = _validate_frequency(hz)
    try:
        w16(inv, 321, int(round(hz * 100)))
        return True
    except Exception as e:
        log.error("set_output_frequency failed: %s", e)
        return False

def inverter_set_max_charge_current(inv, amps):
    amps = _validate_charge_current(amps)
    try:
        w16(inv, 332, int(round(amps * 10)))
        return True
    except Exception as e:
        log.error("set_max_charge_current failed: %s", e)
        return False

def inverter_set_utility_charge_current(inv, amps):
    amps = _validate_charge_current(amps)
    try:
        w16(inv, 333, int(round(amps * 10)))
        return True
    except Exception as e:
        log.error("set_utility_charge_current failed: %s", e)
        return False

def inverter_set_charge_voltages(inv, absorb, floatv, eq):
    try:
        w16(inv, 324, int(round(absorb * 10)))
        w16(inv, 325, int(round(floatv * 10)))
        w16(inv, 334, int(round(eq * 10)))
        return True
    except Exception as e:
        log.error("set_charge_voltages failed: %s", e)
        return False

# ---------- VE.Bus service class ----------
class MultiPlusEmuService:
    def __init__(self):
        self.bus_vebus = SystemBus()
        self.bus_pv = SystemBus()
        self.svc = VeDbusService(SERVICE_NAME, bus=self.bus_vebus)
        # SolarCharger service on separate bus connection
        self.pv = VeDbusService(PV_SERVICE_NAME, bus=self.bus_pv)
        self._idx = 0
        self._add_paths()
        self._add_pv_paths()
        self._last_modbus_ok = True
        self._consecutive_modbus_failures = 0
        log.info("Started Anenji emulator: %s and %s", SERVICE_NAME, PV_SERVICE_NAME)

    def _add_paths(self):
        s = self.svc
        # Management & identity
        s.add_path("/Mgmt/ProcessName", __file__)
        s.add_path("/Mgmt/ProcessVersion", FIRMWARE_VER)
        s.add_path("/Mgmt/Connection", ANENJI_PORT)
        s.add_path("/DeviceInstance", DEVICE_INSTANCE)
        s.add_path("/ProductId", PRODUCT_ID)
        s.add_path("/ProductName", PRODUCT_NAME)
        s.add_path("/FirmwareVersion", FIRMWARE_VER)
        s.add_path("/Serial", "")
        s.add_path("/Connected", 0)
        s.add_path("/Ac/NumberOfPhases", PHASE_COUNT)

        # Mode/state (Mode is writeable)
        s.add_path("/Mode", 4, writeable=True, onchangecallback=self._on_set_mode)
        s.add_path("/State", 1)
        s.add_path("/VebusError", 0)
        s.add_path("/VebusChargeState", 0)
        s.add_path("/UpdateIndex", 0)

        # AC Input
        s.add_path("/Ac/ActiveIn/Connected", 0)
        s.add_path("/Ac/ActiveIn/ActiveInput", 240)
        s.add_path("/Ac/ActiveIn/Available", 1)
        s.add_path("/Ac/ActiveIn/Source", 1)

        s.add_path("/Ac/ActiveIn/L1/V", None, gettextcallback=lambda p,v: f"{v:.1f} V" if v is not None else "")
        s.add_path("/Ac/ActiveIn/L1/I", None, gettextcallback=lambda p,v: f"{v:.2f} A" if v is not None else "")
        s.add_path("/Ac/ActiveIn/L1/F", None, gettextcallback=lambda p,v: f"{v:.2f} Hz" if v is not None else "")
        s.add_path("/Ac/ActiveIn/L1/P", None, gettextcallback=lambda p,v: f"{v} W" if v is not None else "")
        s.add_path("/Ac/ActiveIn/L1/S", None, gettextcallback=lambda p,v: f"{v} VA" if v is not None else "")
        s.add_path("/Ac/ActiveIn/L1/PF", None, gettextcallback=lambda p,v: f"{v:.2f}" if v is not None else "")

        s.add_path("/Ac/ActiveIn/V", None)
        s.add_path("/Ac/ActiveIn/Current", None)
        s.add_path("/Ac/ActiveIn/Power", None)
        s.add_path("/Ac/ActiveIn/S", None)
        s.add_path("/Ac/ActiveIn/Frequency", None)

        # AC Out
        s.add_path("/Ac/Out/L1/V", None, gettextcallback=lambda p,v: f"{v:.1f} V" if v is not None else "")
        s.add_path("/Ac/Out/L1/I", None, gettextcallback=lambda p,v: f"{v:.2f} A" if v is not None else "")
        s.add_path("/Ac/Out/L1/F", None, gettextcallback=lambda p,v: f"{v:.2f} Hz" if v is not None else "")
        s.add_path("/Ac/Out/L1/P", None, gettextcallback=lambda p,v: f"{v} W" if v is not None else "")
        s.add_path("/Ac/Out/L1/S", None, gettextcallback=lambda p,v: f"{v} VA" if v is not None else "")
        s.add_path("/Ac/Out/L1/PF", None, gettextcallback=lambda p,v: f"{v:.2f}" if v is not None else "")

        s.add_path("/Ac/Out/P", None)
        s.add_path("/Ac/Out/S", None)
        s.add_path("/Ac/Out/NominalInverterPower", INVERTER_MAX_POWER)

        # DC / Battery
        s.add_path("/Dc/0/Voltage", None, gettextcallback=lambda p,v: f"{v:.1f} V" if v is not None else "")
        s.add_path("/Dc/0/Current", None, gettextcallback=lambda p,v: f"{v:.1f} A" if v is not None else "")
        s.add_path("/Dc/0/Power", None, gettextcallback=lambda p,v: f"{v} W" if v is not None else "")
        s.add_path("/Soc", None, gettextcallback=lambda p,v: f"{v} %")
        s.add_path("/Dc/0/Temperature", None)

        # PV
        s.add_path("/Pv/0/Power", 0.0, gettextcallback=lambda p,v: f"{v} W")
        s.add_path("/Pv/0/Voltage", None)
        s.add_path("/Pv/0/Current", None)
        s.add_path("/Pv/0/Today", 0.0, gettextcallback=lambda p,v: f"{v:.3f} kWh")

        # Energy
        s.add_path("/Energy/AcIn1ToInverter", _energy["AcIn1ToInverter_kWh"])
        s.add_path("/Energy/AcIn1ToAcOut", _energy["AcIn1ToAcOut_kWh"])
        s.add_path("/Energy/InverterToAcOut", _energy["InverterToAcOut_kWh"])
        s.add_path("/Energy/ToGrid", _energy["ToGrid_kWh"])
        s.add_path("/Energy/FromGrid", _energy["FromGrid_kWh"])

        # Faults/warnings (expose decoded strings)
        s.add_path("/Faults/Code32", 0)
        s.add_path("/Faults/Decoded", "")
        s.add_path("/Warnings/Code32", 0)
        s.add_path("/Warnings/Decoded", "")

        # Controls (writeable)
        s.add_path("/Settings/ChargeEnabled", 1, writeable=True, onchangecallback=self._on_change_charge_enabled)
        s.add_path("/Settings/MaxChargeCurrent", 0, writeable=True, onchangecallback=self._on_change_max_charge_current)

    # ---- SolarCharger (PV) service paths ----
    def _add_pv_paths(self):
        """Create the D-Bus paths required for Venus to show a SolarCharger/MPPT device."""
        p = self.pv
        p.add_path("/Mgmt/ProcessName", __file__)
        p.add_path("/Mgmt/ProcessVersion", FIRMWARE_VER)
        p.add_path("/Mgmt/Connection", ANENJI_PORT)
        p.add_path("/DeviceInstance", PV_DEVICE_INSTANCE)
        p.add_path("/ProductId", PV_PRODUCT_ID)
        p.add_path("/ProductName", PV_PRODUCT_NAME)
        p.add_path("/FirmwareVersion", FIRMWARE_VER)
        p.add_path("/Connected", 0)
        p.add_path("/UpdateIndex", 0)

        # Minimal set that makes Venus show an MPPT
        p.add_path("/Pv/V", None, gettextcallback=lambda path, v: f"{v:.2f} V" if v is not None else "")
        p.add_path("/Pv/I", None, gettextcallback=lambda path, v: f"{v:.2f} A" if v is not None else "")
        p.add_path("/Pv/P", None, gettextcallback=lambda path, v: f"{v} W" if v is not None else "")
        p.add_path("/Yield/Power", None, gettextcallback=lambda path, v: f"{v} W" if v is not None else "")
        p.add_path("/Yield/Today", 0.0, gettextcallback=lambda path, v: f"{v:.3f} kWh")

        # DC output (charger to battery)
        p.add_path("/Dc/0/Voltage", None, gettextcallback=lambda path, v: f"{v:.2f} V" if v is not None else "")
        p.add_path("/Dc/0/Current", None, gettextcallback=lambda path, v: f"{v:.2f} A" if v is not None else "")
        p.add_path("/Dc/0/Power", None, gettextcallback=lambda path, v: f"{v} W" if v is not None else "")
        # Battery temperature (VE.Can MPPTs only but harmless to expose)
        p.add_path("/Dc/0/Temperature", None, gettextcallback=lambda path, v: f"{v:.1f} °C" if v is not None else "")

        # Charger controls & status (common paths used by Venus)
        p.add_path("/Mode", 1, writeable=True, onchangecallback=self._on_change_pv_mode)  # 1=On;4=Off
        p.add_path("/ErrorCode", 0)
        p.add_path("/MppOperationMode", 255)
        p.add_path("/Relay/0/State", 0)

        # Equalization (mostly unused here)
        p.add_path("/Equalization/Pending", 0)
        p.add_path("/Equalization/TimeRemaining", 0)

        # Daily history (used in some UIs)
        p.add_path("/History/Daily/0/Yield", 0.0)      # kWh
        p.add_path("/History/Daily/0/MaxPower", 0)     # W
        p.add_path("/History/Daily/1/Yield", 0.0)      # kWh
        p.add_path("/History/Daily/1/MaxPower", 0)     # W

        p.add_path("/State", 0)
        p.add_path("/Settings/InverterEnabled", 1, writeable=True, onchangecallback=self._on_change_inverter_enabled)
        p.add_path("/Settings/GridCurrentLimit", 0, writeable=True, onchangecallback=self._on_change_grid_limit)

        # Uptime
        p.add_path("/Devices/0/UpTime", 0)

    # ---------- D-Bus write handlers ----------
    def _on_set_mode(self, path, value):
        log.info("Mode write request via D-Bus: %s = %s", path, value)
        try:
            inv = mk_instrument()
            v = int(value)
            if v == 4:
                inverter_remote_off(inv)
            else:
                inverter_remote_on(inv)
                if v == 1:
                    inverter_set_priority(inv, 0)
                elif v == 2:
                    inverter_set_priority(inv, 2)
                elif v == 3:
                    inverter_set_priority(inv, 0)
            return True
        except Exception as e:
            log.error("Failed set mode: %s", e)
            return False

    def _on_change_charge_enabled(self, path, value):
        log.info("ChargeEnabled write: %s=%s", path, value)
        # try env mapped register first
        if REG_MAP.get("CHARGE_ENABLE"):
            return try_write_register(REG_MAP["CHARGE_ENABLE"], int(bool(value)))
        try:
            inv = mk_instrument()
            if bool(value):
                inverter_remote_on(inv)
            else:
                inverter_remote_off(inv)
            return True
        except Exception as e:
            log.error("Failed change charge enabled: %s", e)
            return False

    def _on_change_max_charge_current(self, path, value):
        log.info("MaxChargeCurrent write: %s=%s", path, value)
        try:
            amps = float(value)
            amps = _validate_charge_current(amps)
            if REG_MAP.get("CHARGE_CURRENT"):
                return try_write_register(REG_MAP["CHARGE_CURRENT"], amps, decimals=0)
            inv = mk_instrument()
            return inverter_set_max_charge_current(inv, amps)
        except Exception as e:
            log.error("Failed change max charge current: %s", e)
            return False

    def _on_change_inverter_enabled(self, path, value):
        log.info("InverterEnabled write: %s=%s", path, value)
        try:
            inv = mk_instrument()
            if bool(value):
                inverter_remote_on(inv)
            else:
                inverter_remote_off(inv)
            return True
        except Exception as e:
            log.error("Failed inverter enabled: %s", e)
            return False

    
    def _on_change_pv_mode(self, path, value):
        """Write handler for PV charger /Mode (1=On, 4=Off). We don't control a real charger here,
        so we just store the value and reflect it in /Connected and /State."""
        try:
            v = int(value)
        except Exception:
            v = 1
        # 1=On, 4=Off
        if v == 4:
            try:
                self.pv["/Connected"] = 0
            except Exception:
                pass
        return True

    def _on_change_grid_limit(self, path, value):
        log.info("GridCurrentLimit write: %s=%s", path, value)
        try:
            amps = float(value)
            if REG_MAP.get("GRID_LIMIT"):
                return try_write_register(REG_MAP["GRID_LIMIT"], amps, decimals=0)
            inv = mk_instrument()
            return inverter_set_utility_charge_current(inv, amps)
        except Exception as e:
            log.error("Failed set grid limit: %s", e)
            return False

    # ---------- main update ----------
    def _safe_update(self):
        try:
            return self.update()
        except Exception as e:
            log.exception('Unhandled exception in update: %s', e)
            return True

    def update(self):
        global _energy, _pv_today_wh, _pv_today_ts, _last_grid_seen, _grid_confirmed

        s = self.svc
        # tick update index early (used as heartbeat)
        self._idx = (getattr(self, '_idx', 0) + 1) % 256
        s['/UpdateIndex'] = self._idx
        try:
            self.pv['/UpdateIndex'] = self._idx
        except Exception:
            pass
        try:
            inv = mk_instrument()

            # READ registers per provided table
            # fault bits 100~101 (32-bit)
            fault_lo = r16(inv, 100)
            fault_hi = r16(inv, 101)
            fault32 = decode_bitfield_32(fault_lo, fault_hi)
            fault_strs = decode_faults_from_32bit(fault32)

            # warnings 108~109 (32-bit)
            warn_lo = r16(inv, 108)
            warn_hi = r16(inv, 109)
            warn32 = decode_bitfield_32(warn_lo, warn_hi)
            warn_strs = decode_warnings_from_32bit(warn32)

            # serial number ASCII 186~197 (12 registers)
            serial = r16_block_as_ascii(inv, 186, 12)

            # core registers
            wm = r16(inv, 201)                       # Working Mode (USHORT)
            in_v = r16(inv, 202, True) * 0.1         # mains voltage
            in_f = r16(inv, 203, True) * 0.01        # mains frequency
            in_p = r16(inv, 204, True)               # average mains power (W)
            inv_v = r16(inv, 205, True) * 0.1        # inverter voltage
            inv_i_raw = r16(inv, 206, True) * 0.1    # inverter current (could be apparent)
            inv_freq = r16(inv, 207, True) * 0.01
            inv_power = r16(inv, 208, True)
            inv_charge_power = r16(inv, 209, True)

            out_v = r16(inv, 210, True) * 0.1        # output voltage
            out_i_reg = r16(inv, 211, True) * 0.1 if True else None  # documented as effective current (but often apparent)
            out_f = r16(inv, 212, True) * 0.01
            out_p = r16(inv, 213, True)
            out_s = r16(inv, 214, True) if True else None

            bat_v = r16(inv, 215, True) * 0.1
            bat_i = r16(inv, 216, True) * 0.1
            bat_p = r16(inv, 217, True)
            # PV registers
            pv_v = None
            pv_i = None
            pv_p = 0.0
            try:
                pv_v = r16(inv, 219, True) * 0.1
                pv_i = r16(inv, 220, True) * 0.1
                pv_p = r16(inv, 223, True)
            except Exception:
                pv_v = None; pv_i = None; pv_p = 0.0

            load_pct = r16(inv, 225, True)
            dcdc_temp = r16(inv, 226, True)
            inv_temp = r16(inv, 227, True)
            soc = r16(inv, 229)
            bat_avg_i = r16(inv, 232, True) * 0.1
            inv_chg_i = r16(inv, 233, True) * 0.1
            pv_chg_i = r16(inv, 234, True) * 0.1

            # optional rated power
            try:
                rated_power = r16(inv, 643)
            except Exception:
                rated_power = INVERTER_MAX_POWER

            # control/readable parameters (R/W) we will read to reflect state
            try:
                output_mode = r16(inv, 300)
            except Exception:
                output_mode = 0
            try:
                output_priority = r16(inv, 301)
            except Exception:
                output_priority = 0

            # succeeded modbus
            self._consecutive_modbus_failures = 0
            if not self._last_modbus_ok:
                log.info("Modbus restored")
            self._last_modbus_ok = True

        except Exception as e:
            # modbus read fail - soft fail and leave connected=0
            self._consecutive_modbus_failures += 1
            self._last_modbus_ok = False
            log.error("Modbus read error: %s (count=%d)", e, self._consecutive_modbus_failures)
            s["/Connected"] = 0
            try:
                self.pv["/Connected"] = 0
            except Exception:
                pass
            return True

        # Connected
        s["/Connected"] = 1
        # ---- PV / SolarCharger publishing ----
        try:
            p = self.pv
            p["/Connected"] = 1
            p["/UpdateIndex"] = self._idx
            # PV input
            p["/Pv/V"] = float(pv_v) if pv_v is not None else 0.0
            p["/Pv/I"] = float(pv_i) if pv_i is not None else 0.0
            p["/Pv/P"] = float(pv_p) if pv_p is not None else 0.0

            # Charger output (to battery). Use battery voltage and PV charge current if available.
            p["/Dc/0/Voltage"] = float(bat_v) if bat_v is not None else 0.0
            # pv_chg_i is the PV charge current into battery (from register 234). Fallback to 0.
            p["/Dc/0/Current"] = float(pv_chg_i) if pv_chg_i is not None else 0.0
            # Power: prefer pv_p; if missing, compute V*I
            _pv_power = float(pv_p) if pv_p is not None else (float(pv_v or 0.0) * float(pv_i or 0.0))
            p["/Dc/0/Power"] = _pv_power

            # Yield power is what Venus systemcalc uses for totals
            p["/Yield/Power"] = _pv_power
            # Today's yield in kWh
            p["/Yield/Today"] = float(_pv_today_wh) / 1000.0
            # Track max power today
            global _pv_max_power_today
            if _pv_power > _pv_max_power_today:
                _pv_max_power_today = _pv_power

            # Daily history
            p["/History/Daily/0/Yield"] = float(_pv_today_wh) / 1000.0
            p["/History/Daily/0/MaxPower"] = int(_pv_max_power_today)
            p["/History/Daily/1/Yield"] = 0.0
            p["/History/Daily/1/MaxPower"] = 0

            # Basic charger state/mpp mode
            p["/MppOperationMode"] = 2 if _pv_power > 5 else 0
            p["/State"] = 3 if _pv_power > 5 else 0
            p["/ErrorCode"] = 0

        except Exception:
            pass

        # Mode / State mapping
        s["/Mode"] = (3 if wm == 3 else 1) if wm in (2,3,4,5) else 4
        s["/State"] = (3 if wm == 5 else (9 if wm == 3 else 1))
        s["/VebusError"] = 0
        s["/VebusChargeState"] = 3 if wm == 5 else 1

        # Faults / Warnings publish
        s["/Faults/Code32"] = int(fault32)
        s["/Faults/Decoded"] = "; ".join(fault_strs) if fault_strs else ""
        s["/Warnings/Code32"] = int(warn32)
        s["/Warnings/Decoded"] = "; ".join(warn_strs) if warn_strs else ""

        # Grid detection debounce
        raw_grid = (wm in (2,4,5)) or (in_v and in_v > GRID_VOLTAGE_THRESHOLD)
        nowt = time()
        if raw_grid:
            _last_grid_seen = nowt
        if (nowt - _last_grid_seen) <= GRID_DEBOUNCE_SEC:
            _grid_confirmed = True
        else:
            _grid_confirmed = False
        mains_present = _grid_confirmed

        s["/Ac/ActiveIn/Connected"] = 1 if mains_present else 0
        s["/Ac/ActiveIn/ActiveInput"] = 0 if mains_present else 240
        s["/Ac/ActiveIn/Source"] = 1 if mains_present else 0

        # Derive currents (Victron expects real current). Use P/V.
        if out_v and abs(out_v) > 1e-6:
            out_i = out_p / out_v
        else:
            out_i = 0.0
        if in_v and abs(in_v) > 1e-6:
            in_i = in_p / in_v
        else:
            in_i = 0.0

        # apparent / S and PF
        in_s_calc = in_v * in_i if in_v and in_i else 0.0
        in_pf = (in_p / in_s_calc) if in_s_calc > 0 else 0.0
        out_s_calc = out_v * out_i if out_v and out_i else 0.0
        out_pf = (out_p / out_s_calc) if out_s_calc > 0 else 0.0

        # Publish AC-in
        s["/Ac/ActiveIn/L1/V"] = in_v
        s["/Ac/ActiveIn/L1/I"] = in_i
        s["/Ac/ActiveIn/L1/F"] = in_f
        s["/Ac/ActiveIn/L1/P"] = in_p
        s["/Ac/ActiveIn/L1/S"] = in_s_calc
        s["/Ac/ActiveIn/L1/PF"] = round(in_pf, 3) if in_pf else None

        s["/Ac/ActiveIn/V"] = in_v
        s["/Ac/ActiveIn/Current"] = in_i
        s["/Ac/ActiveIn/Power"] = in_p
        s["/Ac/ActiveIn/S"] = in_s_calc
        s["/Ac/ActiveIn/Frequency"] = in_f

        # Publish AC-out
        s["/Ac/Out/L1/V"] = out_v
        s["/Ac/Out/L1/I"] = out_i
        s["/Ac/Out/L1/F"] = out_f
        s["/Ac/Out/L1/P"] = out_p
        s["/Ac/Out/L1/S"] = out_s if out_s is not None else out_s_calc
        s["/Ac/Out/L1/PF"] = round(out_pf, 3) if out_pf else None
        s["/Ac/Out/P"] = out_p
        s["/Ac/Out/S"] = out_s if out_s is not None else out_s_calc

        # Publish DC/Battery and PV
        s["/Dc/0/Voltage"] = bat_v
        s["/Dc/0/Current"] = bat_i
        s["/Dc/0/Power"] = bat_p
        if soc is not None:
            s["/Soc"] = soc
        if dcdc_temp is not None:
            s["/Dc/0/Temperature"] = dcdc_temp

        s["/Pv/0/Power"] = pv_p
        s["/Pv/0/Voltage"] = pv_v
        s["/Pv/0/Current"] = pv_i

        # Energy model
        dt = POLL_SEC
        Wh = dt / 3600.0

        is_bypass = wm in (2,4)
        is_invert = wm == 3
        is_charging = wm == 5

        grid_to_load_w = 0.0
        grid_to_batt_w = 0.0
        batt_to_load_w = 0.0
        export_to_grid_w = 0.0

        if is_bypass:
            grid_to_load_w = max(out_p, 0)
            export_to_grid_w = max(-out_p, 0)
        elif is_invert:
            batt_to_load_w = max(out_p, 0)
        elif is_charging:
            grid_to_load_w = min(in_p, out_p)
            grid_to_batt_w = max(in_p - out_p, 0)
            if out_p > in_p:
                batt_to_load_w = out_p - in_p
        else:
            if mains_present:
                grid_to_load_w = min(in_p, out_p)
                grid_to_batt_w = max(in_p - out_p, 0)
                batt_to_load_w = max(out_p - in_p, 0)
                export_to_grid_w = max(-in_p, 0)
            else:
                batt_to_load_w = max(out_p, 0)

        # DC override: bat_p (>0 discharging, <0 charging)
        if bat_p is not None:
            if bat_p > 50:
                batt_to_load_w = max(batt_to_load_w, bat_p)
                # ensure we don't count that as grid import
                if grid_to_load_w > 0:
                    reduce_by = min(grid_to_load_w, max(0, bat_p - max(0, out_p - grid_to_load_w)))
                    grid_to_load_w = max(grid_to_load_w - reduce_by, 0)
                grid_to_batt_w = 0
                export_to_grid_w = 0
            elif bat_p < -50:
                if mains_present:
                    grid_to_batt_w = max(grid_to_batt_w, -bat_p)

        # PV handling
        if pv_p and pv_p > 0:
            pv_to_load = min(pv_p, out_p)
            pv_to_batt = max(pv_p - pv_to_load, 0)
            if grid_to_load_w > 0:
                grid_to_load_w = max(grid_to_load_w - pv_to_load, 0)
            else:
                grid_to_batt_w = max(grid_to_batt_w - pv_to_batt, 0)
            _pv_today_wh += pv_p * Wh
            _pv_today_ts = int(time())
            s["/Pv/0/Today"] = round(_pv_today_wh / 1000.0, 3)

        # export detection if in_p negative
        if in_p < -50:
            export_to_grid_w = max(export_to_grid_w, -in_p)

        # integrate
        _energy["AcIn1ToAcOut_kWh"] += max(grid_to_load_w, 0) / 1000.0 * Wh
        _energy["AcIn1ToInverter_kWh"] += max(grid_to_batt_w, 0) / 1000.0 * Wh
        _energy["InverterToAcOut_kWh"] += max(batt_to_load_w, 0) / 1000.0 * Wh
        _energy["ToGrid_kWh"] += max(export_to_grid_w, 0) / 1000.0 * Wh
        _energy["FromGrid_kWh"] += max(grid_to_load_w + grid_to_batt_w, 0) / 1000.0 * Wh

        _save_energy()
        _save_pv_today()

        # publish SolarCharger (MPPT)
        try:
            self.pv["/Connected"] = 1
            self.pv["/Pv/V"] = pv_v
            self.pv["/Pv/I"] = pv_i
            self.pv["/Pv/P"] = int(pv_p) if pv_p is not None else None
            self.pv["/Yield/Power"] = int(pv_p) if pv_p is not None else None
            self.pv["/Yield/Today"] = round(_pv_today_wh / 1000.0, 3)
            self.pv["/Dc/0/Voltage"] = bat_v
            # positive current = charging into battery
            self.pv["/Dc/0/Current"] = max(pv_chg_i or 0.0, 0.0)
            self.pv["/Dc/0/Power"] = int(pv_p) if pv_p is not None else 0
            self.pv["/State"] = 3 if pv_p is not None and pv_p > 0 else 0
        except Exception as e:
            log.debug("PV service update skipped: %s", e)

        # publish energy rounded
        s["/Energy/AcIn1ToInverter"] = round(_energy["AcIn1ToInverter_kWh"], 6)
        s["/Energy/AcIn1ToAcOut"] = round(_energy["AcIn1ToAcOut_kWh"], 6)
        s["/Energy/InverterToAcOut"] = round(_energy["InverterToAcOut_kWh"], 6)
        s["/Energy/ToGrid"] = round(_energy["ToGrid_kWh"], 6)
        s["/Energy/FromGrid"] = round(_energy["FromGrid_kWh"], 6)

        # publish other housekeeping
        s["/Devices/0/UpTime"] = int(time()) - time_driver_started

        return True

# ---------- signal handling ----------
def handle_sigterm(signum, frame):
    log.info("Received signal %s, shutting down...", signum)
    stop_event.set()

# ---------- main ----------
def main():
    _load_energy()
    _load_pv_today()
    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    svc = MultiPlusEmuService()
    GLib.timeout_add(int(POLL_SEC * 1000), svc._safe_update)
    signal.signal(signal.SIGTERM, handle_sigterm)
    signal.signal(signal.SIGINT, handle_sigterm)
    try:
        loop = GLib.MainLoop()
        loop.run()
    except KeyboardInterrupt:
        pass
    finally:
        log.info("Shutting down, saving state")
        _save_energy()
        _save_pv_today()

if __name__ == "__main__":
    main()
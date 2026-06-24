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

NOTE (VRM Grid voltage fix):
- VRM/Grid tile reads /Ac/In/1/*, not only /Ac/ActiveIn/*
- This file now publishes /Ac/In/1/* and drives Connected/ActiveInput from AC-in voltage presence.
"""

import os
import sys
import json
import logging
import signal
from time import time, sleep
from threading import Event
from datetime import datetime

import dbus
import dbus.mainloop.glib
from gi.repository import GLib

# ---- Create independent D-Bus connections (needed when exporting two services in one process) ----
class SystemBus(dbus.bus.BusConnection):
    def __new__(cls):
        return dbus.bus.BusConnection.__new__(cls, dbus.bus.BusConnection.TYPE_SYSTEM)

# allow velib_python in ext/
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(BASE_DIR, "ext", "velib_python"))
from vedbus import VeDbusService  # vendored velib

import minimalmodbus
import serial

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

# MPPT (SolarCharger) service - for VRM/SystemCalc solar accounting
PV_SERVICE_NAME    = os.environ.get("PV_SERVICE_NAME", "com.victronenergy.solarcharger.anenji")
PV_DEVICE_INSTANCE = int(os.environ.get("PV_DEVICE_INSTANCE", "4500"))
PV_PRODUCT_ID      = int(os.environ.get("PV_PRODUCT_ID", "4500"))
PV_PRODUCT_NAME    = os.environ.get("PV_PRODUCT_NAME", "Solar Charger PV Yield")
PV_HISTORY_FILE    = os.environ.get("PV_HISTORY_FILE", os.path.join(BASE_DIR, "pv_history.json"))

PHASE_COUNT        = int(os.environ.get("PHASE_COUNT", "1"))
INVERTER_MAX_POWER = int(os.environ.get("ANENJI_INV_MAX_POWER", "3000"))

ENERGY_FILE     = os.environ.get("ENERGY_FILE", os.path.join(BASE_DIR, "energy.json"))
PV_PERSIST_FILE = os.environ.get("PV_PERSIST_FILE", os.path.join(BASE_DIR, "pv_today.json"))
AC_LIMIT_FILE = os.environ.get("AC_LIMIT_FILE", os.path.join(BASE_DIR, "ac_limit.json"))
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

# PV history (Wh) with midnight rollover + persistence (30 days)
_pv_hist = {
    "day": datetime.now().strftime("%Y-%m-%d"),
    "today_wh": 0.0,
    "yesterday_wh": 0.0,
    "history_wh": [0.0] * 30
}

# debounce
_last_grid_seen = 0.0

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
}

# ---------- Helpers ----------
def mkdir_p(path):
    d = os.path.dirname(path)
    if d and not os.path.exists(d):
        os.makedirs(d, exist_ok=True)


def load_ac_limit():
    """Load persisted AC input current limit (amps)."""
    try:
        if os.path.exists(AC_LIMIT_FILE):
            with open(AC_LIMIT_FILE, "r") as f:
                data = json.load(f)
            v = float(data.get("ac_current_limit", 0.0))
            if v < 0:
                v = 0.0
            return v
    except Exception as e:
        log.warning("Could not load AC limit file %s: %s", AC_LIMIT_FILE, e)
    return float(os.environ.get('AC_CURRENT_LIMIT_DEFAULT', '16'))

def save_ac_limit(amps: float):
    try:
        mkdir_p(AC_LIMIT_FILE)
        with open(AC_LIMIT_FILE, "w") as f:
            json.dump({"ac_current_limit": float(amps)}, f)
    except Exception as e:
        log.warning("Could not save AC limit file %s: %s", AC_LIMIT_FILE, e)


def _validate_ac_current_limit(amps) -> float:
    """Reasonable clamp for AC input current limit in amps."""
    try:
        a = float(amps)
    except Exception:
        return 0.0
    if a < 0:
        a = 0.0
    # clamp to something sane; change if needed
    if a > 200:
        a = 200.0
    return a


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

def _load_pv_history():
    """Load PV history from PV_HISTORY_FILE. If old pv_today.json exists, migrate its value into today."""
    global _pv_hist
    try:
        if os.path.exists(PV_HISTORY_FILE):
            with open(PV_HISTORY_FILE, "r") as f:
                d = json.load(f)
            if isinstance(d, dict):
                _pv_hist["day"] = str(d.get("day", _pv_hist["day"]))
                _pv_hist["today_wh"] = float(d.get("today_wh", 0.0))
                _pv_hist["yesterday_wh"] = float(d.get("yesterday_wh", 0.0))
                h = d.get("history_wh", _pv_hist["history_wh"])
                if isinstance(h, list) and len(h) > 0:
                    _pv_hist["history_wh"] = [float(x) for x in h[:30]] + [0.0] * max(0, 30 - len(h[:30]))
        elif os.path.exists(PV_PERSIST_FILE):
            # migrate legacy pv_today.json
            with open(PV_PERSIST_FILE, "r") as f:
                d = json.load(f)
            _pv_hist["day"] = datetime.now().strftime("%Y-%m-%d")
            _pv_hist["today_wh"] = float(d.get("wh", 0.0))
        log.info("Loaded PV history: today=%.3f kWh yesterday=%.3f kWh",
                 _pv_hist["today_wh"]/1000.0, _pv_hist["yesterday_wh"]/1000.0)
    except Exception as e:
        log.warning("Failed load PV history: %s", e)

def _save_pv_history():
    try:
        mkdir_p(PV_HISTORY_FILE)
        with open(PV_HISTORY_FILE, "w") as f:
            json.dump(_pv_hist, f)
    except Exception as e:
        log.debug("Failed save PV history: %s", e)

def _pv_midnight_rollover_if_needed():
    """Rollover today->yesterday at local midnight (calendar-based)."""
    global _pv_hist
    today = datetime.now().strftime("%Y-%m-%d")
    if _pv_hist.get("day") != today:
        _pv_hist["yesterday_wh"] = float(_pv_hist.get("today_wh", 0.0))
        hist = _pv_hist.get("history_wh", [0.0]*1)
        if not isinstance(hist, list):
            hist = [0.0]*1
        hist = [float(_pv_hist["yesterday_wh"])] + [float(x) for x in hist[:29]]
        _pv_hist["history_wh"] = hist[:1]
        _pv_hist["today_wh"] = 0.0
        _pv_hist["max_power_today"] = 0
        _pv_hist["day"] = today
        _save_pv_history()

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

# ---------- Control high-level functions ----------
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
        self.pv = VeDbusService(PV_SERVICE_NAME, bus=self.bus_pv)
        self._idx = 0
        self._ac_current_limit = load_ac_limit()
        self._add_paths()
        self._add_pv_paths()
        self._last_modbus_ok = True
        self._consecutive_modbus_failures = 0
        log.info("Started Anenji VE.Bus emulator: %s", SERVICE_NAME)

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

        # AC ActiveIn (still publish)
        s.add_path("/Ac/ActiveIn/Connected", 0)
        s.add_path("/Ac/ActiveIn/ActiveInput", 240)
        s.add_path("/Ac/ActiveIn/Available", 1)
        s.add_path("/Ac/ActiveIn/Source", 1)

        # AC input current limit (editable from Venus UI)
        s.add_path("/Ac/ActiveIn/CurrentLimit", self._ac_current_limit, writeable=True, onchangecallback=self._on_change_ac_current_limit)

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

        # ---- AC In 1 (VRM Grid tile reads these) ----
        s.add_path("/Ac/In/1/Connected", 0)
        s.add_path("/Ac/In/1/CurrentLimit", self._ac_current_limit, writeable=True, onchangecallback=self._on_change_ac_current_limit)
        s.add_path("/Ac/In/1/CurrentLimitIsAdjustable", 1)
        s.add_path("/Ac/ActiveIn/CurrentLimitIsAdjustable", 1)
        s.add_path("/Settings/GridCurrentLimitIsAdjustable", 1)

        s.add_path("/Ac/In/1/L1/V", None)
        s.add_path("/Ac/In/1/L1/I", None)
        s.add_path("/Ac/In/1/L1/F", None)
        s.add_path("/Ac/In/1/L1/P", None)
        s.add_path("/Ac/In/1/L1/S", None)
        s.add_path("/Ac/In/1/L1/PF", None)

        s.add_path("/Ac/In/1/V", None)
        s.add_path("/Ac/In/1/Current", None)
        s.add_path("/Ac/In/1/Power", None)
        s.add_path("/Ac/In/1/S", None)
        s.add_path("/Ac/In/1/Frequency", None)

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

        # PV (VE.Bus display only)
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

        # Faults/warnings
        s.add_path("/Faults/Code32", 0)
        s.add_path("/Faults/Decoded", "")
        s.add_path("/Warnings/Code32", 0)
        s.add_path("/Warnings/Decoded", "")

        # Controls (writeable)
        s.add_path("/Settings/ChargeEnabled", 1, writeable=True, onchangecallback=self._on_change_charge_enabled)
        s.add_path("/Settings/MaxChargeCurrent", 0, writeable=True, onchangecallback=self._on_change_max_charge_current)
        s.add_path("/Settings/InverterEnabled", 1, writeable=True, onchangecallback=self._on_change_inverter_enabled)
        s.add_path("/Settings/GridCurrentLimit", self._ac_current_limit, writeable=True, onchangecallback=self._on_change_ac_current_limit)

        # Uptime
        s.add_path("/Devices/0/UpTime", 0)

    def _add_pv_paths(self):
        """Expose a Victron-like SolarCharger service so VRM/SystemCalc can count solar production."""
        p = self.pv
        p.add_path("/Mgmt/ProcessName", __file__)
        p.add_path("/Mgmt/ProcessVersion", FIRMWARE_VER)
        p.add_path("/Mgmt/Connection", ANENJI_PORT)

        p.add_path("/DeviceInstance", PV_DEVICE_INSTANCE)
        p.add_path("/ProductId", PV_PRODUCT_ID)
        p.add_path("/ProductName", PV_PRODUCT_NAME)
        p.add_path("/FirmwareVersion", FIRMWARE_VER)
        p.add_path("/Serial", "")
        p.add_path("/Connected", 0)

        p.add_path("/Mode", 1)           # 1=On
        p.add_path("/State", 1)          # 1=Idle
        p.add_path("/ErrorCode", 0)

        p.add_path("/Pv/V", 0.0)
        p.add_path("/Pv/I", 0.0)

        p.add_path("/Dc/0/Voltage", 0.0)
        p.add_path("/Dc/0/Current", 0.0)
        p.add_path("/Dc/0/Power", 0.0)

        p.add_path("/Yield/Power", 0.0)
        p.add_path("/Yield/Today", 0.0)
        p.add_path("/Yield/User", 0.0)
        p.add_path("/Yield/System", 0.0)


# Full 30-day MPPT history (Victron style)
       
        for i in range(1):
           base = f"/History/Daily/{i}"

        p.add_path(f"{base}/Yield", 0.0)
        p.add_path(f"{base}/MaxPower", 0.0)
        p.add_path(f"{base}/MaxPvVoltage", 0.0)

        p.add_path(f"{base}/MaxBatteryVoltage", 0.0)
        p.add_path(f"{base}/MinBatteryVoltage", 0.0)
        p.add_path(f"{base}/MaxBatteryCurrent", 0.0)

        p.add_path(f"{base}/TimeInBulk", 0)
        p.add_path(f"{base}/TimeInAbsorption", 0)
        p.add_path(f"{base}/TimeInFloat", 0)

        p.add_path(f"{base}/Consumption", 0.0)
        p.add_path(f"{base}/Nr", 0)

        p.add_path(f"{base}/LastError1", 0)
        p.add_path(f"{base}/LastError2", 0)
        p.add_path(f"{base}/LastError3", 0)
        p.add_path(f"{base}/LastError4", 0)


#        p.add_path("/History/Daily/0/Yield", 0.0)  # today
 #       p.add_path("/History/Daily/1/Yield", 0.0)  # yesterday

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
            amps = _validate_charge_current(float(value))
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

    def _on_change_ac_current_limit(self, path, value):
        log.info("AC input current limit write: %s=%s", path, value)
        try:
            amps = _validate_ac_current_limit(value)

            # Always accept the new limit so Venus UI remains editable.
            # We still attempt to push it to the real device (if present),
            # but a Modbus write failure should not make the setting read-only.
            self._ac_current_limit = float(amps)
            save_ac_limit(self._ac_current_limit)

            # Keep related paths in sync for Venus UI
            self.svc["/Settings/GridCurrentLimit"] = self._ac_current_limit
            self.svc["/Ac/In/1/CurrentLimit"] = self._ac_current_limit
            self.svc["/Ac/ActiveIn/CurrentLimit"] = self._ac_current_limit

            pushed = False
            try:
                if REG_MAP.get("GRID_LIMIT"):
                    pushed = try_write_register(REG_MAP["GRID_LIMIT"], amps, decimals=0)
                else:
                    inv = mk_instrument()
                    pushed = inverter_set_utility_charge_current(inv, amps)
            except Exception as e:
                log.warning("Could not push AC current limit to inverter (keeping local value): %s", e)

            if not pushed:
                log.info("AC current limit stored locally (not pushed to inverter)")
            return True
        except Exception as e:
            log.error("Failed set AC current limit: %s", e)
            return False

    def _on_change_grid_limit(self, path, value):
        # Backwards compatible alias
        return self._on_change_ac_current_limit(path, value)

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
    def update(self):
        global _energy, _pv_hist, _last_grid_seen

        s = self.svc
        try:
            inv = mk_instrument()

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

            inv_v = r16(inv, 205, True) * 0.1
            inv_i_raw = r16(inv, 206, True) * 0.1
            inv_freq = r16(inv, 207, True) * 0.01
            inv_power = r16(inv, 208, True)
            inv_charge_power = r16(inv, 209, True)

            out_v = r16(inv, 210, True) * 0.1
            out_i_reg = r16(inv, 211, True) * 0.1
            out_f = r16(inv, 212, True) * 0.01
            out_p = r16(inv, 213, True)
            out_s = r16(inv, 214, True)

            bat_v = r16(inv, 215, True) * 0.1
            bat_i = r16(inv, 216, True) * 0.1
            bat_p = r16(inv, 217, True)

            pv_v = pv_i = pv_p = None
            try:
                pv_v = r16(inv, 219, True) * 0.1
            except Exception:
                pv_v = None
            try:
                pv_i = r16(inv, 220, True) * 0.1
            except Exception:
                pv_i = None
            try:
                pv_p = float(r16(inv, 223, True))
            except Exception:
                pv_p = None

            load_pct = r16(inv, 225, True)
            dcdc_temp = r16(inv, 226, True)
            inv_temp = r16(inv, 227, True)
            soc = r16(inv, 229)
            bat_avg_i = r16(inv, 232, True) * 0.1
            inv_chg_i = r16(inv, 233, True) * 0.1
            pv_chg_i = r16(inv, 234, True) * 0.1

            try:
                rated_power = r16(inv, 643)
            except Exception:
                rated_power = INVERTER_MAX_POWER

            try:
                output_mode = r16(inv, 300)
            except Exception:
                output_mode = 0
            try:
                output_priority = r16(inv, 301)
            except Exception:
                output_priority = 0

            self._consecutive_modbus_failures = 0
            if not self._last_modbus_ok:
                log.info("Modbus restored")
            self._last_modbus_ok = True

        except Exception as e:
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

        # ---- Grid detection based on AC-in voltage (AC IN 1) ----
        nowt = time()
        raw_grid = (in_v is not None) and (in_v > GRID_VOLTAGE_THRESHOLD)
        if raw_grid:
            _last_grid_seen = nowt
        mains_present = (nowt - _last_grid_seen) <= GRID_DEBOUNCE_SEC

        # ActiveIn flags
        # NOTE: These are Victron conventions used by SystemCalc/VRM.
        # - /Ac/ActiveIn/Connected is a boolean (0/1)
        # - /Ac/ActiveIn/ActiveInput is 0 for AC-in-1, 1 for AC-in-2, and 240 when not connected
        s["/Ac/ActiveIn/Connected"] = 1 if mains_present else 0
        s["/Ac/ActiveIn/ActiveInput"] = 0 if mains_present else 240  # AC IN 1 only
        s["/Ac/ActiveIn/Source"] = 1 if mains_present else 0


        # Derive currents (Victron expects real current). Use P/V.
        out_i = (out_p / out_v) if (out_v and abs(out_v) > 1e-6) else 0.0
        in_i = (in_p / in_v) if (in_v and abs(in_v) > 1e-6) else 0.0

        in_s_calc = in_v * in_i if (in_v and in_i) else 0.0
        in_pf = (in_p / in_s_calc) if in_s_calc > 0 else 0.0

        out_s_calc = out_v * out_i if (out_v and out_i) else 0.0
        out_pf = (out_p / out_s_calc) if out_s_calc > 0 else 0.0

        # Publish AC ActiveIn
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

        # ---- Publish AC In 1 (VRM Grid tile uses this) ----
        s["/Ac/In/1/Connected"] = 1 if mains_present else 0

        s["/Ac/In/1/L1/V"] = in_v
        s["/Ac/In/1/L1/I"] = in_i
        s["/Ac/In/1/L1/F"] = in_f
        s["/Ac/In/1/L1/P"] = in_p
        s["/Ac/In/1/L1/S"] = in_s_calc
        s["/Ac/In/1/L1/PF"] = round(in_pf, 3) if in_pf else None

        s["/Ac/In/1/V"] = in_v
        s["/Ac/In/1/Current"] = in_i
        s["/Ac/In/1/Power"] = in_p
        s["/Ac/In/1/S"] = in_s_calc
        s["/Ac/In/1/Frequency"] = in_f

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

        s["/Pv/0/Power"] = float(pv_p) if pv_p is not None else 0.0
        s["/Pv/0/Voltage"] = pv_v
        s["/Pv/0/Current"] = pv_i

        # ---- SolarCharger (MPPT) service export ----
        _pv_midnight_rollover_if_needed()

        if pv_p is not None and pv_p > 0:
            _pv_hist["today_wh"] += float(pv_p) * (POLL_SEC / 3600.0)
            if int(time()) % max(int(ENERGY_SAVE_SEC), 30) == 0:
                _save_pv_history()

        today_kwh = _pv_hist["today_wh"] / 1000.0
        yday_kwh = _pv_hist["yesterday_wh"] / 1000.0

        s["/Pv/0/Today"] = float(today_kwh)

        pv_power = float(pv_p) if pv_p is not None else 0.0
        bat_v_float = float(bat_v) if bat_v is not None else 0.0
        dc_current = (pv_power / bat_v_float) if (pv_power > 0 and bat_v_float > 0.1) else 0.0

        p = self.pv
        p["/Connected"] = 1
        p["/Mode"] = 1

        p["/Pv/V"] = float(pv_v) if pv_v is not None else 0.0
        p["/Pv/I"] = float(pv_i) if pv_i is not None else 0.0

        p["/Dc/0/Voltage"] = bat_v_float
        p["/Dc/0/Current"] = float(dc_current)
        p["/Dc/0/Power"] = pv_power

        p["/Yield/Power"] = pv_power
        #p["/Yield/Today"] = float(round(today_kwh, 3))
        #p["/History/Daily/0/Yield"] = float(round(today_kwh, 3))
        #p["/History/Daily/1/Yield"] = float(round(yday_kwh, 3))
        
        p["/Yield/Today"] = float(round(today_kwh, 3))
        p["/Yield/User"] = float(round(today_kwh, 3))
        p["/Yield/System"] = float(round(today_kwh, 3))
# ---- FULL HISTORY (Victron MPPT style) ----
        for i in range(1):
            try:
                val = _pv_hist["history_wh"][i] / 1000.0
            except:
                val = 0.0

        p[f"/History/Daily/{i}/Yield"] = float(round(val, 3))

# Always override today/yesterday with live values
        #["/History/Daily/0/Yield"] = float(round(today_kwh, 3))
        #["/History/Daily/1/Yield"] = float(round(yday_kwh, 3))
        
        #["/History/Overall/Yield"] = sum(_pv_hist["history_wh"]) / 1000.0
        #["/History/Overall/MaxPower"] = float(_pv_hist.get("max_power_today", 0))

# ADD THIS BLOCK HERE 👇 
        
        hist = _pv_hist.get("history_wh", [])

        max_p = float(_pv_hist.get("max_power_today", 0.0))
        max_v = float(pv_v) if pv_v else 0.0

        bat_v_float = float(bat_v) if bat_v else 0.0
        bat_i_float = float(bat_i) if bat_i else 0.0

        for i in range(1):
            base = f"/History/Daily/{i}"

            if i == 0:
                yield_val = _pv_hist["today_wh"]
                pwr = max_p
                volt = max_v

                max_bat_v = bat_v_float
                min_bat_v = bat_v_float
                max_bat_i = abs(bat_i_float)

                bulk = 0
                absorb = 0
                flt = 0

            elif i == 1:
                yield_val = _pv_hist["yesterday_wh"]
                pwr = 0.0
                volt = 0.0

                max_bat_v = 0.0
                min_bat_v = 0.0
                max_bat_i = 0.0

                bulk = 0
                absorb = 0
                flt = 0

            elif (i - 1) < len(hist):
                yield_val = hist[i - 1]
                pwr = 0.0
                volt = 0.0

                max_bat_v = 0.0
                min_bat_v = 0.0
                max_bat_i = 0.0

                bulk = 0
                absorb = 0
                flt = 0

            else:
                yield_val = 0.0
                pwr = 0.0
                volt = 0.0

                max_bat_v = 0.0
                min_bat_v = 0.0
                max_bat_i = 0.0

                bulk = 0
                absorb = 0
                flt = 0

        p[f"{base}/Yield"] = round(yield_val / 1000.0, 3)
        p[f"{base}/MaxPower"] = pwr
        p[f"{base}/MaxPvVoltage"] = volt

        p[f"{base}/MaxBatteryVoltage"] = max_bat_v
        p[f"{base}/MinBatteryVoltage"] = min_bat_v
        p[f"{base}/MaxBatteryCurrent"] = max_bat_i

        p[f"{base}/TimeInBulk"] = bulk
        p[f"{base}/TimeInAbsorption"] = absorb
        p[f"{base}/TimeInFloat"] = flt

        p[f"{base}/Consumption"] = 0.0
        p[f"{base}/Nr"] = i

        p[f"{base}/LastError1"] = 0
        p[f"{base}/LastError2"] = 0
        p[f"{base}/LastError3"] = 0
        p[f"{base}/LastError4"] = 0


        if pv_power > 20:
            p["/State"] = 3
        elif pv_power > 5:
            p["/State"] = 4
        else:
            p["/State"] = 1

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

        # publish energy rounded
        s["/Energy/AcIn1ToInverter"] = round(_energy["AcIn1ToInverter_kWh"], 6)
        s["/Energy/AcIn1ToAcOut"] = round(_energy["AcIn1ToAcOut_kWh"], 6)
        s["/Energy/InverterToAcOut"] = round(_energy["InverterToAcOut_kWh"], 6)
        s["/Energy/ToGrid"] = round(_energy["ToGrid_kWh"], 6)
        s["/Energy/FromGrid"] = round(_energy["FromGrid_kWh"], 6)

        # publish housekeeping
        s["/Devices/0/UpTime"] = int(time()) - time_driver_started
        self._idx = (self._idx + 1) % 256
        s["/UpdateIndex"] = self._idx

        return True

# ---------- signal handling ----------
def handle_sigterm(signum, frame):
    log.info("Received signal %s, shutting down...", signum)
    stop_event.set()

# ---------- main ----------
def main():
    _load_energy()
    _load_pv_history()
    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    svc = MultiPlusEmuService()
    GLib.timeout_add(int(POLL_SEC * 1000), svc.update)
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
        _save_pv_history()

if __name__ == "__main__":
    main()
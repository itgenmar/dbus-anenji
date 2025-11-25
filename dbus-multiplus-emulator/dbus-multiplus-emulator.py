#!/usr/bin/env python3
# Modbus-backed MultiPlus emulator for Sumry / Anenji inverter
#
# Service: com.victronenergy.vebus.ttyS3
# VRM compatible for Grid + AC-Out energy (kWh) with persistence.

import os
import sys
import json
import logging
import traceback
from time import time, sleep

import dbus
import dbus.mainloop.glib
from gi.repository import GLib

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(1, os.path.join(BASE_DIR, "ext", "velib_python"))
from vedbus import VeDbusService  # noqa: E402

import minimalmodbus  # noqa: E402
import serial         # noqa: E402

# ------------------- CONFIG -------------------
ANENJI_PORT    = os.environ.get("ANENJI_PORT", "/dev/ttyUSB1")
ANENJI_ID      = int(os.environ.get("ANENJI_ID", "1"))
ANENJI_BAUD    = int(os.environ.get("ANENJI_BAUD", "9600"))
ANENJI_TIMEOUT = float(os.environ.get("ANENJI_TIMEOUT", "1.5"))
POLL_SEC       = float(os.environ.get("ANENJI_POLL", "2.0"))

SERVICE_NAME   = "com.victronenergy.vebus.ttyS3"

PRODUCT_ID      = 0xA31F
PRODUCT_NAME    = "MultiPlus-II (Anenji bridge)"
FIRMWARE_VER    = "v1.5-anenji"
DEVICE_INSTANCE = int(os.environ.get("ANENJI_DEVICE_INSTANCE", "28"))

PHASE_COUNT        = 1
INVERTER_MAX_POWER = int(os.environ.get("ANENJI_INV_MAX_POWER", "3000"))

ENERGY_FILE        = os.path.join(BASE_DIR, "energy.json")
ENERGY_SAVE_SEC    = 60  # write to disk at most once per 60s

time_driver_started = int(time())

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s:%(name)s:%(message)s"
)
log = logging.getLogger("anenji-vebus")

# --------- ENERGY STORAGE (kWh) ---------
_energy = {
    "AcIn1ToInverter_kWh": 0.0,
    "AcIn1ToAcOut_kWh": 0.0,
    "InverterToAcOut_kWh": 0.0,
}
_last_energy_save = 0.0


def _load_energy():
    global _energy
    try:
        with open(ENERGY_FILE, "r") as f:
            data = json.load(f)
        for k in _energy:
            if k in data and isinstance(data[k], (int, float)):
                _energy[k] = float(data[k])
        log.info("Loaded energy counters from %s: %s", ENERGY_FILE, _energy)
    except Exception:
        log.info("No previous energy file, starting from zero.")


def _save_energy():
    global _last_energy_save
    now = time()
    if now - _last_energy_save < ENERGY_SAVE_SEC:
        return
    try:
        with open(ENERGY_FILE, "w") as f:
            json.dump(_energy, f)
        _last_energy_save = now
        log.debug("Saved energy counters: %s", _energy)
    except Exception as e:
        log.error("Failed to save energy file: %s", e)


# ------------------- HELPERS -------------------
def _fmt(fmt):
    def cb(path, value):
        if value is None:
            return ""
        try:
            return fmt.format(value)
        except Exception:
            return str(value)
    return cb


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
    sleep(0.02)
    return inv.read_register(reg, 0, functioncode=3, signed=signed)


# ---- Working mode → VE.Bus mapping ----
def vebus_mode_from_wm(wm: int) -> int:
    # /Mode: 1=Charger only, 2=Inverter only, 3=On, 4=Off
    if wm == 3:              # Off-grid / inverting
        return 3
    if wm in (2, 4, 5):      # Mains / Bypass / Charging
        return 1
    return 4                 # Off / Standby / Fault


def vebus_state_from_wm(wm: int) -> int:
    # /State: 0 Off, 1 Low power, 3 Bulk, 9 Inverting (simplified)
    if wm == 5:
        return 3  # Charging → Bulk
    if wm == 3:
        return 9  # Off-grid → Inverting
    if wm in (2, 4):
        return 3  # On mains/bypass, treat as Bulk/active
    return 1      # Power-on / Standby / Fault → Low power


# ------------------- EMULATOR SERVICE -------------------
class MultiPlusEmuService:
    def __init__(self):
        self.svc = VeDbusService(SERVICE_NAME)
        self._idx = 0
        self._add_paths()
        log.info("Started Anenji-backed MultiPlus emulator as %s", SERVICE_NAME)

    def _add_paths(self):
        s = self.svc

        # Management
        s.add_path("/Mgmt/ProcessName", __file__)
        s.add_path("/Mgmt/ProcessVersion", FIRMWARE_VER)
        s.add_path("/Mgmt/Connection", ANENJI_PORT)

        # Identity
        s.add_path("/DeviceInstance", DEVICE_INSTANCE)
        s.add_path("/ProductId", PRODUCT_ID)
        s.add_path("/ProductName", PRODUCT_NAME)
        s.add_path("/FirmwareVersion", FIRMWARE_VER)
        s.add_path("/Serial", "")
        s.add_path("/Connected", 0)
        s.add_path("/Ac/NumberOfPhases", PHASE_COUNT)

        # VE.Bus state
        s.add_path("/Mode", 4)
        s.add_path("/State", 1)
        s.add_path("/VebusError", 0)
        s.add_path("/VebusChargeState", 0)
        s.add_path("/UpdateIndex", 0)

        # AC Input (Grid – AC In 1)
        s.add_path("/Ac/ActiveIn/Connected", 0)
        s.add_path("/Ac/ActiveIn/ActiveInput", 240)  # 0=AC in 1, 240=disconnected
        s.add_path("/Ac/ActiveIn/Available", 1)
        s.add_path("/Ac/ActiveIn/Source", 1)         # 1=Grid

        s.add_path("/Ac/ActiveIn/V", None, gettextcallback=_fmt("{:.1f} V"))
        s.add_path("/Ac/ActiveIn/Current", None, gettextcallback=_fmt("{:.1f} A"))
        s.add_path("/Ac/ActiveIn/Power", None, gettextcallback=_fmt("{} W"))
        s.add_path("/Ac/ActiveIn/Frequency", None, gettextcallback=_fmt("{:.2f} Hz"))

        s.add_path("/Ac/ActiveIn/L1/V", None, gettextcallback=_fmt("{:.1f} V"))
        s.add_path("/Ac/ActiveIn/L1/I", None, gettextcallback=_fmt("{:.1f} A"))
        s.add_path("/Ac/ActiveIn/L1/F", None, gettextcallback=_fmt("{:.2f} Hz"))
        s.add_path("/Ac/ActiveIn/L1/P", None, gettextcallback=_fmt("{} W"))
        s.add_path("/Ac/ActiveIn/L1/S", None, gettextcallback=_fmt("{} VA"))

        s.add_path("/Ac/ActiveIn/P", None, gettextcallback=_fmt("{} W"))
        s.add_path("/Ac/ActiveIn/S", None, gettextcallback=_fmt("{} VA"))

        # AC Out
        s.add_path("/Ac/Out/L1/V", None, gettextcallback=_fmt("{:.1f} V"))
        s.add_path("/Ac/Out/L1/I", None, gettextcallback=_fmt("{:.1f} A"))
        s.add_path("/Ac/Out/L1/F", None, gettextcallback=_fmt("{:.2f} Hz"))
        s.add_path("/Ac/Out/L1/P", None, gettextcallback=_fmt("{} W"))
        s.add_path("/Ac/Out/L1/S", None, gettextcallback=_fmt("{} VA"))

        s.add_path("/Ac/Out/P", None, gettextcallback=_fmt("{} W"))
        s.add_path("/Ac/Out/S", None, gettextcallback=_fmt("{} VA"))
        s.add_path("/Ac/Out/NominalInverterPower", INVERTER_MAX_POWER)

        # DC / Battery
        s.add_path("/Dc/0/Voltage", None, gettextcallback=_fmt("{:.1f} V"))
        s.add_path("/Dc/0/Current", None, gettextcallback=_fmt("{:.1f} A"))
        s.add_path("/Dc/0/Power",   None, gettextcallback=_fmt("{} W"))
        s.add_path("/Soc", None, gettextcallback=_fmt("{} %"))

        # Energy counters (kWh)
        s.add_path("/Energy/AcIn1ToInverter", _energy["AcIn1ToInverter_kWh"])
        s.add_path("/Energy/AcIn1ToAcOut",    _energy["AcIn1ToAcOut_kWh"])
        s.add_path("/Energy/InverterToAcOut", _energy["InverterToAcOut_kWh"])
        s.add_path("/Energy/OutToInverter",   0.0)  # unused but present

        # Uptime
        s.add_path("/Devices/0/UpTime", 0)

    # ---------------------------------------------------
    def update(self):
        global _energy

        s = self.svc
        try:
            inv = mk_instrument()

            # Working mode
            wm = r16(inv, 201, signed=False)

            # AC-in
            in_v = r16(inv, 202, True) * 0.1
            in_f = r16(inv, 203, True) * 0.01
            in_p = r16(inv, 204, True)  # W
            in_i = (in_p / in_v) if in_v > 5 else 0.0

            # AC-out
            out_v = r16(inv, 210, True) * 0.1
            out_i = r16(inv, 211, True) * 0.1
            out_f = r16(inv, 212, True) * 0.01
            out_p = r16(inv, 213, True)  # W
            out_s = r16(inv, 214, True)  # VA

            # DC
            dc_v = r16(inv, 215, True) * 0.1
            dc_i = r16(inv, 216, True) * 0.1
            dc_p = r16(inv, 217, True)  # W
            try:
                soc = r16(inv, 229, signed=False)
            except Exception:
                soc = None

        except Exception as e:
            log.error("Modbus read error: %s", e)
            traceback.print_exc()
            s["/Connected"] = 0
            return True

        # Connected
        s["/Connected"] = 1

        # Mode / State
        s["/Mode"]  = vebus_mode_from_wm(wm)
        s["/State"] = vebus_state_from_wm(wm)
        s["/VebusError"] = 0
        s["/VebusChargeState"] = 3 if wm == 5 else 1

        mains_present = wm in (2, 4, 5) or in_v > 150

        # --- AC Input (Grid) ---
        s["/Ac/ActiveIn/Connected"]   = 1 if mains_present else 0
        s["/Ac/ActiveIn/ActiveInput"] = 0 if mains_present else 240
        s["/Ac/ActiveIn/Available"]   = 1
        s["/Ac/ActiveIn/Source"]      = 1 if mains_present else 0

        s["/Ac/ActiveIn/L1/V"] = in_v
        s["/Ac/ActiveIn/L1/I"] = in_i
        s["/Ac/ActiveIn/L1/F"] = in_f
        s["/Ac/ActiveIn/L1/P"] = in_p
        s["/Ac/ActiveIn/L1/S"] = abs(in_p)

        s["/Ac/ActiveIn/V"]         = in_v
        s["/Ac/ActiveIn/Current"]   = in_i
        s["/Ac/ActiveIn/Power"]     = in_p
        s["/Ac/ActiveIn/Frequency"] = in_f
        s["/Ac/ActiveIn/P"]         = in_p
        s["/Ac/ActiveIn/S"]         = abs(in_p)

        # --- AC Output ---
        s["/Ac/Out/L1/V"] = out_v
        s["/Ac/Out/L1/I"] = out_i
        s["/Ac/Out/L1/F"] = out_f
        s["/Ac/Out/L1/P"] = out_p
        s["/Ac/Out/L1/S"] = out_s

        s["/Ac/Out/P"] = out_p
        s["/Ac/Out/S"] = out_s

        # --- DC / Battery ---
        s["/Dc/0/Voltage"] = dc_v
        s["/Dc/0/Current"] = dc_i
        s["/Dc/0/Power"]   = dc_p
        if soc is not None:
            s["/Soc"] = soc

        # --------- ENERGY ACCUMULATION (kWh) ---------
        # dt seconds per cycle
        dt = POLL_SEC

        # Grid import energy (AC Input 1)
        if mains_present and in_p > 0:
            delta_kwh_in = (in_p / 1000.0) * (dt / 3600.0)
            _energy["AcIn1ToInverter_kWh"] += delta_kwh_in
            _energy["AcIn1ToAcOut_kWh"]    += delta_kwh_in

        # AC-Out energy (consumption)
        if out_p > 0:
            delta_kwh_out = (out_p / 1000.0) * (dt / 3600.0)
            _energy["InverterToAcOut_kWh"] += delta_kwh_out

        # Push to DBus (kWh)
        s["/Energy/AcIn1ToInverter"] = _energy["AcIn1ToInverter_kWh"]
        s["/Energy/AcIn1ToAcOut"]    = _energy["AcIn1ToAcOut_kWh"]
        s["/Energy/InverterToAcOut"] = _energy["InverterToAcOut_kWh"]
        s["/Energy/OutToInverter"]   = 0.0

        # Save to disk throttled
        _save_energy()
        # ---------------------------------------------

        # Uptime & UpdateIndex
        s["/Devices/0/UpTime"] = int(time()) - time_driver_started
        self._idx = (self._idx + 1) % 256
        s["/UpdateIndex"] = self._idx

        return True


# ------------------- MAIN -------------------
def main():
    _load_energy()
    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    svc = MultiPlusEmuService()
    GLib.timeout_add(int(POLL_SEC * 1000), svc.update)
    GLib.MainLoop().run()


if __name__ == "__main__":
    main()

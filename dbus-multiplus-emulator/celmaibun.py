#!/usr/bin/env python3
# MultiPlus-style VE.Bus service backed by Anenji / Sumry Modbus inverter
#
# D-Bus service:
#   com.victronenergy.vebus.ttyS3
#
# Designed to work with mr-manuel's install.sh & Venus OS GUI/VRM.

import os
import sys
import logging
import traceback
from time import time, sleep

import dbus
import dbus.mainloop.glib
from gi.repository import GLib

# ---- velib_python from Venus OS ------------------------------------------------
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(1, os.path.join(BASE_DIR, "ext", "velib_python"))
from vedbus import VeDbusService  # noqa: E402

# ---- Modbus / serial ----------------------------------------------------------
import minimalmodbus  # noqa: E402
import serial         # noqa: E402

# ---- Config -------------------------------------------------------------------
PORT    = os.environ.get("ANENJI_PORT", "/dev/ttyUSB1")
SLAVEID = int(os.environ.get("ANENJI_ID", "1"))
BAUD    = int(os.environ.get("ANENJI_BAUD", "9600"))
TIMEOUT = float(os.environ.get("ANENJI_TIMEOUT", "1.5"))
POLL    = float(os.environ.get("ANENJI_POLL", "2.0"))

SERVICE_NAME = "com.victronenergy.vebus.ttyS3"

PRODUCT_ID      = 0xA31F
PRODUCT_NAME    = "MultiPlus-II (Anenji bridge)"
FIRMWARE_VER    = "v1.4-anenji"
DEVICE_INSTANCE = int(os.environ.get("ANENJI_DEVICE_INSTANCE", "28"))

PHASE_COUNT     = 1
MAX_INV_POWER   = int(os.environ.get("ANENJI_INV_MAX_POWER", "3000"))

start_time = int(time())

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s:%(name)s:%(message)s"
)
log = logging.getLogger("dbus-multiplus-anenji")

# ---- Helpers ------------------------------------------------------------------

def _fmt(fmt):
    def cb(path, value):
        if value is None:
            return ""
        try:
            return fmt.format(value)
        except Exception:
            return str(value)
    return cb


def mk_instrument() -> minimalmodbus.Instrument:
    inv = minimalmodbus.Instrument(PORT, SLAVEID)
    inv.serial.baudrate = BAUD
    inv.serial.timeout  = TIMEOUT
    inv.serial.bytesize = 8
    inv.serial.stopbits = 1
    inv.serial.parity   = serial.PARITY_NONE
    inv.mode = minimalmodbus.MODE_RTU
    inv.clear_buffers_before_each_transaction = True
    inv.close_port_after_each_call = True
    try:
        inv.serial.exclusive = True
    except Exception:
        pass
    return inv


def r16(inv, reg, signed=False):
    # small delay between Modbus frames to keep RS485 stable
    sleep(0.02)
    return inv.read_register(reg, 0, functioncode=3, signed=signed)

# ---- VE.Bus Mapping -----------------------------------------------------------
#
# Working mode (reg 201):
#  0 PowerOn
#  1 Standby
#  2 Mains
#  3 Off-grid (Inverting)
#  4 Bypass
#  5 Charging
#  6 Fault
#
# /Mode:  1 Charger only, 2 Inverter only, 3 On, 4 Off
# /State: 0 Off, 1 Low Power, 3 Bulk, 9 Inverting

def vebus_mode_from_wm(wm: int) -> int:
    if wm == 3:           # off-grid / inverting
        return 3          # On
    if wm in (2, 4, 5):   # mains / bypass / charging
        return 1          # Charger only
    return 4              # Off / standby / fault


def vebus_state_from_wm(wm: int) -> int:
    if wm == 5:           # charging
        return 3          # Bulk
    if wm == 3:           # off-grid
        return 9          # Inverting
    if wm in (2, 4):      # mains / bypass
        return 3          # treat as Bulk / on-grid
    if wm in (0, 1):      # power on / standby
        return 1          # Low power / AES-ish
    return 1

# ---- Service ------------------------------------------------------------------

class MultiPlusEmuService:
    def __init__(self):
        self._dbusservice = VeDbusService(SERVICE_NAME)
        self._idx = 0
        self._add_paths()
        log.info("Started Anenji-backed MultiPlus emulator as %s", SERVICE_NAME)

    def _add_paths(self):
        s = self._dbusservice

        # Management
        s.add_path("/Mgmt/ProcessName", __file__)
        s.add_path("/Mgmt/ProcessVersion", FIRMWARE_VER)
        s.add_path("/Mgmt/Connection", PORT)

        # Identity
        s.add_path("/DeviceInstance", DEVICE_INSTANCE)
        s.add_path("/ProductId", PRODUCT_ID)
        s.add_path("/ProductName", PRODUCT_NAME)
        s.add_path("/FirmwareVersion", FIRMWARE_VER)
        s.add_path("/Serial", "")
        s.add_path("/Connected", 0)
        s.add_path("/Ac/NumberOfPhases", PHASE_COUNT)

        # Mode/state
        s.add_path("/Mode", 4)
        s.add_path("/State", 1)
        s.add_path("/VebusError", 0)
        s.add_path("/VebusChargeState", 0)
        s.add_path("/UpdateIndex", 0)

        # LEDs
        s.add_path("/Leds/Mains", 0)
        s.add_path("/Leds/Inverter", 0)
        s.add_path("/Leds/LowBattery", 0)
        s.add_path("/Leds/Overload", 0)
        s.add_path("/Leds/Temperature", 0)

        # ---- AC INPUT (ActiveIn) ----
        s.add_path("/Ac/ActiveIn/Connected", 0)
        s.add_path("/Ac/ActiveIn/ActiveInput", 240)  # 0 = AC in 1, 240 = disconnected
        s.add_path("/Ac/ActiveIn/Available", 1)      # hardware exists
        s.add_path("/Ac/ActiveIn/Source", 240)       # 1 = Grid, 240 = not available

        # GUI summary nodes
        s.add_path("/Ac/ActiveIn/V", None, gettextcallback=_fmt("{:.1f} V"))
        s.add_path("/Ac/ActiveIn/Current", None, gettextcallback=_fmt("{:.1f} A"))
        s.add_path("/Ac/ActiveIn/Power", None, gettextcallback=_fmt("{} W"))
        s.add_path("/Ac/ActiveIn/Frequency", None, gettextcallback=_fmt("{:.2f} Hz"))

        # Current limit UI
        s.add_path("/Ac/ActiveIn/CurrentLimit", 32.0, gettextcallback=_fmt("{:.1f} A"))
        s.add_path("/Ac/ActiveIn/CurrentLimitIsAdjustable", 0)
        s.add_path("/Ac/ActiveIn/CurrentLimitOffered", 32.0, gettextcallback=_fmt("{:.1f} A"))

        # Phase L1 (VRM + detail page)
        s.add_path("/Ac/ActiveIn/L1/V", None, gettextcallback=_fmt("{:.1f} V"))
        s.add_path("/Ac/ActiveIn/L1/I", None, gettextcallback=_fmt("{:.1f} A"))
        s.add_path("/Ac/ActiveIn/L1/F", None, gettextcallback=_fmt("{:.2f} Hz"))
        s.add_path("/Ac/ActiveIn/L1/P", None, gettextcallback=_fmt("{} W"))
        s.add_path("/Ac/ActiveIn/L1/S", None, gettextcallback=_fmt("{} VA"))

        s.add_path("/Ac/ActiveIn/P", None, gettextcallback=_fmt("{} W"))
        s.add_path("/Ac/ActiveIn/S", None, gettextcallback=_fmt("{} VA"))

        # ---- AC OUTPUT ----
        s.add_path("/Ac/Out/L1/V", None, gettextcallback=_fmt("{:.1f} V"))
        s.add_path("/Ac/Out/L1/I", None, gettextcallback=_fmt("{:.1f} A"))
        s.add_path("/Ac/Out/L1/F", None, gettextcallback=_fmt("{:.2f} Hz"))
        s.add_path("/Ac/Out/L1/P", None, gettextcallback=_fmt("{} W"))
        s.add_path("/Ac/Out/L1/S", None, gettextcallback=_fmt("{} VA"))
        s.add_path("/Ac/Out/L1/NominalInverterPower", MAX_INV_POWER)

        s.add_path("/Ac/Out/P", None, gettextcallback=_fmt("{} W"))
        s.add_path("/Ac/Out/S", None, gettextcallback=_fmt("{} VA"))
        s.add_path("/Ac/Out/NominalInverterPower", MAX_INV_POWER)

        # ---- DC / Battery ----
        s.add_path("/Dc/0/Voltage", None, gettextcallback=_fmt("{:.1f} V"))
        s.add_path("/Dc/0/Current", None, gettextcallback=_fmt("{:.1f} A"))
        s.add_path("/Dc/0/Power", None, gettextcallback=_fmt("{} W"))
        s.add_path("/Soc", None, gettextcallback=_fmt("{} %"))

        # ---- Misc ----
        s.add_path("/Devices/0/UpTime", 0)
        s.add_path("/Energy/InverterToAcOut", 0)
        s.add_path("/Energy/OutToInverter", 0)

    # ----------------------------------------------------------------------
    # Periodic update
    # ----------------------------------------------------------------------
    def update(self):
        s = self._dbusservice

        try:
            inv = mk_instrument()

            # Working mode
            wm = r16(inv, 201, signed=False)

            # AC-IN (grid)
            in_v_raw = r16(inv, 202, signed=True)  # 0.1V
            in_f_raw = r16(inv, 203, signed=True)  # 0.01Hz
            in_p_raw = r16(inv, 204, signed=True)  # W

            in_v = in_v_raw * 0.1
            in_f = in_f_raw * 0.01
            # GUI & VE.Bus expect positive power for AC input
            in_p = abs(in_p_raw)
            in_i = (in_p / in_v) if in_v > 20 else 0.0

            # AC OUT
            out_v = r16(inv, 210, True) * 0.1
            out_i = r16(inv, 211, True) * 0.1
            out_f = r16(inv, 212, True) * 0.01
            out_p = r16(inv, 213, True)
            out_s = r16(inv, 214, True)

            # DC / battery
            dc_v = r16(inv, 215, True) * 0.1
            dc_i = r16(inv, 216, True) * 0.1
            dc_p = r16(inv, 217, True)
            try:
                soc = r16(inv, 229, signed=False)
            except Exception:
                soc = None

        except Exception as e:
            log.error("Modbus read error: %s", e)
            traceback.print_exc()
            s["/Connected"] = 0
            return True

        # Device connected
        s["/Connected"] = 1

        # Mode & state
        s["/Mode"]  = vebus_mode_from_wm(wm)
        s["/State"] = vebus_state_from_wm(wm)

        # Grid presence: based on working mode OR voltage
        mains_present = (wm in (2, 4, 5)) or (in_v > 150.0)

        s["/Ac/ActiveIn/Connected"]   = 1 if mains_present else 0
        s["/Ac/ActiveIn/ActiveInput"] = 0 if mains_present else 240
        s["/Ac/ActiveIn/Available"]   = 1
        s["/Ac/ActiveIn/Source"]      = 1 if mains_present else 240  # 1=Grid, 240=disconnected

        # AC-IN L1
        s["/Ac/ActiveIn/L1/V"] = in_v
        s["/Ac/ActiveIn/L1/I"] = in_i
        s["/Ac/ActiveIn/L1/F"] = in_f
        s["/Ac/ActiveIn/L1/P"] = in_p
        s["/Ac/ActiveIn/L1/S"] = in_p

        s["/Ac/ActiveIn/V"]         = in_v
        s["/Ac/ActiveIn/Current"]   = in_i
        s["/Ac/ActiveIn/Power"]     = in_p
        s["/Ac/ActiveIn/Frequency"] = in_f

        s["/Ac/ActiveIn/P"] = in_p
        s["/Ac/ActiveIn/S"] = in_p

        # Keep current limit static
        s["/Ac/ActiveIn/CurrentLimit"]           = 32.0
        s["/Ac/ActiveIn/CurrentLimitOffered"]    = 32.0
        s["/Ac/ActiveIn/CurrentLimitIsAdjustable"] = 0

        # AC OUT
        s["/Ac/Out/L1/V"] = out_v
        s["/Ac/Out/L1/I"] = out_i
        s["/Ac/Out/L1/F"] = out_f
        s["/Ac/Out/L1/P"] = out_p
        s["/Ac/Out/L1/S"] = out_s
        s["/Ac/Out/P"]    = out_p
        s["/Ac/Out/S"]    = out_s

        # DC / battery
        s["/Dc/0/Voltage"] = dc_v
        s["/Dc/0/Current"] = dc_i
        s["/Dc/0/Power"]   = dc_p
        if soc is not None:
            s["/Soc"] = soc

        # LEDs
        s["/Leds/Mains"]       = 1 if mains_present else 0
        s["/Leds/Inverter"]    = 1 if wm == 3 else 0
        s["/Leds/LowBattery"]  = 1 if dc_v < 46 else 0
        s["/Leds/Overload"]    = 0
        s["/Leds/Temperature"] = 0

        # Uptime
        now = int(time())
        s["/Devices/0/UpTime"] = now - start_time

        # UpdateIndex bump
        self._idx = (self._idx + 1) % 256
        s["/UpdateIndex"] = self._idx

        return True

# ---- Main ---------------------------------------------------------------------

def main():
    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    svc = MultiPlusEmuService()
    GLib.timeout_add(int(POLL * 1000), svc.update)
    GLib.MainLoop().run()

if __name__ == "__main__":
    main()

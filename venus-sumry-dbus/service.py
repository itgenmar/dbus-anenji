#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Venus OS D-Bus services for Sumry inverter:
#   - PV inverter service:  com.victronenergy.pvinverter.sumry_ttyUSB2  (objectPath=/ttyUSB2_pv)
#   - Battery service:      com.victronenergy.battery.sumry_ttyUSB2     (objectPath=/ttyUSB2_bat)
#
# Serial: /dev/ttyUSB2 @ 9600 8N1, slave id 1 (env-overridable)
# Uses vendored velib_python from ./ext/velib_python
#
# Env overrides (optional):
#   SUMRY_PORT=/dev/ttyUSB2 SUMRY_ID=1 SUMRY_BAUD=9600 SUMRY_POLL=2.0 python3 service.py
#

import os, sys, time, threading, traceback, logging
from typing import Dict, Any

# -------- logging --------
logging.basicConfig(level=logging.INFO, format="%(levelname)s:%(name)s:%(message)s")
log = logging.getLogger("sumry-dbus")

# --- Vendored Victron velib_python ---
sys.path.insert(1, os.path.join(os.path.dirname(__file__), "ext", "velib_python"))
from vedbus import VeDbusService  # noqa: E402

# --- DBus + GLib main loop (must be set before creating VeDbusService) ---
import gi  # noqa: E402
gi.require_version('GLib', '2.0')
from gi.repository import GLib  # noqa: E402

import dbus.mainloop.glib  # noqa: E402
dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)

# --- Modbus / serial ---
import minimalmodbus  # noqa: E402
import serial  # noqa: E402

# --------- Configuration ----------
PORT     = os.environ.get("SUMRY_PORT", "/dev/ttyUSB2")
SLAVE_ID = int(os.environ.get("SUMRY_ID", "1"))
BAUD     = int(os.environ.get("SUMRY_BAUD", "9600"))
PARITY   = serial.PARITY_NONE       # your unit: 8N1
TIMEOUT  = float(os.environ.get("SUMRY_TIMEOUT", "1.5"))
POLL_SEC = float(os.environ.get("SUMRY_POLL", "2.0"))

PV_SERVICE_NAME   = os.environ.get("SUMRY_PV_NAME",   "com.victronenergy.pvinverter.sumry_ttyUSB2")
BAT_SERVICE_NAME  = os.environ.get("SUMRY_BAT_NAME",  "com.victronenergy.battery.sumry_ttyUSB2")

PRODUCT_NAME_PV   = "Sumry PV (from inverter DC)"
PRODUCT_NAME_BAT  = "Sumry Battery (from inverter)"
FIRMWARE_VER      = "1.0"
CONNECTION        = f"serial:{PORT}@{BAUD},8N1,id={SLAVE_ID}"

MODE_MAP = {
    0:"PowerOn", 1:"Standby", 2:"Mains", 3:"OffGrid",
    4:"Bypass", 5:"Charging", 6:"Fault"
}

# ------------- Modbus helpers -------------
def mk_instrument() -> minimalmodbus.Instrument:
    inv = minimalmodbus.Instrument(PORT, SLAVE_ID)
    inv.serial.baudrate = BAUD
    inv.serial.bytesize = 8
    inv.serial.parity   = PARITY
    inv.serial.stopbits = 1
    inv.serial.timeout  = TIMEOUT
    inv.mode = minimalmodbus.MODE_RTU
    inv.clear_buffers_before_each_transaction = True
    inv.close_port_after_each_call = True
    try: inv.serial.exclusive = True
    except Exception: pass
    return inv

def r16(inv: minimalmodbus.Instrument, addr: int, signed=False, fc=3) -> int:
    time.sleep(0.02)
    return inv.read_register(addr, 0, functioncode=fc, signed=signed)

def r32(inv: minimalmodbus.Instrument, addr: int, fc=3) -> int:
    hi = r16(inv, addr,   signed=False, fc=fc)
    lo = r16(inv, addr+1, signed=False, fc=fc)
    return (hi << 16) | lo

def read_ascii(inv: minimalmodbus.Instrument, addr: int, count: int) -> str:
    if count > 6:
        words = inv.read_registers(addr, 6, functioncode=3) + \
                inv.read_registers(addr+6, count-6, functioncode=3)
    else:
        words = inv.read_registers(addr, count, functioncode=3)
    b = bytearray()
    for w in words:
        b += bytes([(w >> 8) & 0xFF, w & 0xFF])
    return b.rstrip(b"\x00").decode(errors="ignore")

def collect_snapshot() -> Dict[str, Any]:
    inv = mk_instrument()

    wm = r16(inv, 201, signed=False)
    mode = MODE_MAP.get(wm, str(wm))

    # Mains (grid) and internal AC
    mains_v   = r16(inv, 202, True) * 0.1
    mains_hz  = r16(inv, 203, True) * 0.01
    mains_w   = r16(inv, 204, True)

    inv_v     = r16(inv, 205, True) * 0.1
    inv_a     = r16(inv, 206, True) * 0.1
    inv_hz    = r16(inv, 207, True) * 0.01
    inv_w     = r16(inv, 208, True)
    inv_chg_w = r16(inv, 209, True)

    out_v     = r16(inv, 210, True) * 0.1
    out_a     = r16(inv, 211, True) * 0.1
    out_hz    = r16(inv, 212, True) * 0.01
    out_w     = r16(inv, 213, True)
    out_va    = r16(inv, 214, True)

    # Battery DC
    batt_v    = r16(inv, 215, True) * 0.1
    batt_a    = r16(inv, 216, True) * 0.1
    batt_w    = r16(inv, 217, True)
    soc       = r16(inv, 229, False)

    # PV DC (from inverter’s charger section)
    pv_v      = r16(inv, 219, True) * 0.1
    pv_a      = r16(inv, 220, True) * 0.1
    pv_w      = r16(inv, 223, True)
    pv_chg_w  = r16(inv, 224, True)

    dcdc_t    = r16(inv, 226, True)
    inv_t     = r16(inv, 227, True)

    fault     = r32(inv, 100)
    warn      = r32(inv, 108)
    serial_no = read_ascii(inv, 186, 12)

    return {
        "Mode": mode, "Fault": fault, "Warn": warn, "Serial": serial_no,

        "Grid_V": mains_v, "Grid_F": mains_hz, "Grid_P": mains_w,
        "Inverter_V": inv_v, "Inverter_A": inv_a, "Inverter_F": inv_hz,
        "Inverter_P": inv_w, "Inverter_Pch": inv_chg_w,

        "Output_V": out_v, "Output_A": out_a, "Output_F": out_hz,
        "Output_P": out_w, "Output_S": out_va,

        "Batt_V": batt_v, "Batt_A": batt_a, "Batt_P": batt_w, "SoC": soc,
        "Pv_V": pv_v, "Pv_A": pv_a, "Pv_P": pv_w, "Pv_Pch": pv_chg_w,

        "T_Dcdc": dcdc_t, "T_Inv": inv_t,
    }

# ------------- utils -------------
def _fmt(fmt):
    def _cb(path, val):
        if val is None: return ""
        try: return fmt.format(val)
        except Exception: return str(val)
    return _cb

# ------------- PV inverter service -------------
class PvInverterDbusService:
    """
    Minimal PV inverter schema:
      /Mgmt/*, /DeviceInstance, /ProductName, /FirmwareVersion, /Connected, /Serial, /Mgmt/Connection
      /Ac/Power, /Ac/L1/Power, (optionally /Ac/L1/Voltage, /Ac/L1/Current, /Ac/L1/Frequency)
    We publish /Ac/Power and /Ac/L1/Power from PV charging power.
    """
    def __init__(self, name=PV_SERVICE_NAME, device_instance=51):
        # Unique objectPath avoids root '/' collision with other services in same process
        self._service = VeDbusService(name, objectPath="/ttyUSB2_pv", register=False)
        self._update_index = 0
        self._add_mandatory_paths(PRODUCT_NAME_PV, device_instance)
        self._service.register()
        self._add_value_paths()
        log.info("registered PV service on D-Bus as %s", name)

    def _add_mandatory_paths(self, product_name, device_instance):
        s = self._service
        s.add_path('/Mgmt/ProcessName', __file__)
        s.add_path('/Mgmt/ProcessVersion', FIRMWARE_VER)
        s.add_path('/DeviceInstance', device_instance)
        s.add_path('/ProductName', product_name)
        s.add_path('/FirmwareVersion', FIRMWARE_VER)
        s.add_path('/Serial', "")
        s.add_path('/Connected', 0)
        s.add_path('/ProductId', 0)
        s.add_path('/Mgmt/Connection', CONNECTION)
        s.add_path('/UpdateIndex', 0)

    def _add_value_paths(self):
        s = self._service
        s.add_path('/Ac/Power', None, gettextcallback=_fmt("{} W"))
        s.add_path('/Ac/L1/Power', None, gettextcallback=_fmt("{} W"))
        # Optional: reflect PV DC V/I on AC path for visibility; comment out if undesired
        s.add_path('/Ac/L1/Voltage', None, gettextcallback=_fmt("{:.1f} V"))
        s.add_path('/Ac/L1/Current', None, gettextcallback=_fmt("{:.1f} A"))
        s.add_path('/Ac/L1/Frequency', None, gettextcallback=_fmt("{:.2f} Hz"))
        # Status passthroughs
        s.add_path('/Status/Mode', "")
        s.add_path('/Status/FaultCode', 0)
        s.add_path('/Status/WarningCode', 0)

    def publish(self, snap: Dict[str, Any]):
        s = self._service
        s['/Connected'] = 1
        s['/Serial'] = snap.get("Serial","")

        pv_p = snap["Pv_Pch"] if snap["Pv_Pch"] else snap["Pv_P"]
        s['/Ac/Power']    = pv_p
        s['/Ac/L1/Power'] = pv_p

        # optional visibility of DC PV values
        s['/Ac/L1/Voltage']   = snap["Pv_V"]
        s['/Ac/L1/Current']   = snap["Pv_A"]
        s['/Ac/L1/Frequency'] = None

        s['/Status/Mode']        = snap["Mode"]
        s['/Status/FaultCode']   = snap["Fault"]
        s['/Status/WarningCode'] = snap["Warn"]

        self._bump()

    def _bump(self):
        self._update_index = (self._update_index + 1) % 256
        self._service['/UpdateIndex'] = self._update_index

# ------------- Battery service -------------
class BatteryDbusService:
    """
    Minimal battery schema:
      /Mgmt/*, /DeviceInstance, /ProductName, /FirmwareVersion, /Connected, /Serial, /Mgmt/Connection
      /Dc/0/Voltage, /Dc/0/Current, /Soc  (+ /Dc/0/Power as convenience)
    """
    def __init__(self, name=BAT_SERVICE_NAME, device_instance=20):
        self._service = VeDbusService(name, objectPath="/ttyUSB2_bat", register=False)
        self._update_index = 0
        self._add_mandatory_paths(PRODUCT_NAME_BAT, device_instance)
        self._service.register()
        self._add_value_paths()
        log.info("registered Battery service on D-Bus as %s", name)

    def _add_mandatory_paths(self, product_name, device_instance):
        s = self._service
        s.add_path('/Mgmt/ProcessName', __file__)
        s.add_path('/Mgmt/ProcessVersion', FIRMWARE_VER)
        s.add_path('/DeviceInstance', device_instance)
        s.add_path('/ProductName', product_name)
        s.add_path('/FirmwareVersion', FIRMWARE_VER)
        s.add_path('/Serial', "")
        s.add_path('/Connected', 0)
        s.add_path('/ProductId', 0)
        s.add_path('/Mgmt/Connection', CONNECTION)
        s.add_path('/UpdateIndex', 0)

    def _add_value_paths(self):
        s = self._service
        s.add_path('/Dc/0/Voltage', None, gettextcallback=_fmt("{:.1f} V"))
        s.add_path('/Dc/0/Current', None, gettextcallback=_fmt("{:.1f} A"))
        s.add_path('/Dc/0/Power',   None, gettextcallback=_fmt("{} W"))
        s.add_path('/Soc', None, gettextcallback=_fmt("{} %"))
        s.add_path('/Temperature', None, gettextcallback=_fmt("{} C"))  # optional
        s.add_path('/Status/Mode', "")
        s.add_path('/Status/FaultCode', 0)
        s.add_path('/Status/WarningCode', 0)

    def publish(self, snap: Dict[str, Any]):
        s = self._service
        s['/Connected'] = 1
        s['/Serial']    = snap.get("Serial","")

        s['/Dc/0/Voltage'] = snap["Batt_V"]
        s['/Dc/0/Current'] = snap["Batt_A"]
        s['/Dc/0/Power']   = snap["Batt_P"]
        s['/Soc']          = snap["SoC"]
        s['/Temperature']  = snap["T_Inv"]  # or T_Dcdc if you prefer

        s['/Status/Mode']        = snap["Mode"]
        s['/Status/FaultCode']   = snap["Fault"]
        s['/Status/WarningCode'] = snap["Warn"]

        self._bump()

    def _bump(self):
        self._update_index = (self._update_index + 1) % 256
        self._service['/UpdateIndex'] = self._update_index

# ------------- Orchestrator -------------
class SumryPublisher:
    def __init__(self):
        self._stop = False
        self._pv   = PvInverterDbusService(device_instance=51)   # choose stable, non-conflicting instances
        self._bat  = BatteryDbusService(device_instance=20)
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        while not self._stop:
            try:
                snap = collect_snapshot()
                self._pv.publish(snap)
                self._bat.publish(snap)
            except Exception as e:
                log.error("poll error: %s", e)
                traceback.print_exc()
                # mark disconnected so GUI reflects it
                try:
                    self._pv._service['/Connected'] = 0
                    self._bat._service['/Connected'] = 0
                except Exception:
                    pass
                time.sleep(2.0)
            time.sleep(POLL_SEC)

# ------------- main -------------
def main():
    # tip: free the serial port from serial-starter first:
    # /opt/victronenergy/serial-starter/stop-tty.sh ttyUSB2
    SumryPublisher()
    GLib.MainLoop().run()

if __name__ == "__main__":
    main()

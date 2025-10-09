#!/usr/bin/env python3
# Venus OS Battery D-Bus service for Sumry inverter (RS-232 Modbus RTU)
# Bus name: com.victronenergy.battery.sumry_ttyUSB2

import os, sys, time, logging, traceback
from typing import Dict, Any

# Logging
logging.basicConfig(level=logging.INFO, format="%(levelname)s:%(name)s:%(message)s")
log = logging.getLogger("sumry-battery")

# Vendored Victron velib
sys.path.insert(1, os.path.join(os.path.dirname(__file__), "ext", "velib_python"))
from vedbus import VeDbusService  # noqa: E402

# DBus + GLib main loop
import gi  # noqa: E402
gi.require_version('GLib', '2.0')
from gi.repository import GLib  # noqa: E402
import dbus.mainloop.glib  # noqa: E402
dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)

# Modbus / serial
import minimalmodbus  # noqa: E402
import serial  # noqa: E402

# --- Config (env-overridable) ---
PORT     = os.environ.get("SUMRY_PORT", "/dev/ttyUSB0")
SLAVE_ID = int(os.environ.get("SUMRY_ID", "1"))
BAUD     = int(os.environ.get("SUMRY_BAUD", "9600"))
PARITY   = serial.PARITY_NONE   # your unit: 8N1
TIMEOUT  = float(os.environ.get("SUMRY_TIMEOUT", "1.5"))
POLL_SEC = float(os.environ.get("SUMRY_POLL", "2.0"))

SERVICE_NAME = os.environ.get("SUMRY_BAT_NAME", "com.victronenergy.battery.sumry_ttyUSB2")
PRODUCT_NAME = "Sumry Battery (from inverter)"
FIRMWARE_VER = "1.0"
CONNECTION   = f"serial:{PORT}@{BAUD},8N1,id={SLAVE_ID}"

MODE_MAP = {0:"PowerOn",1:"Standby",2:"Mains",3:"OffGrid",4:"Bypass",5:"Charging",6:"Fault"}

# --- Helpers ---
def _fmt(fmt):
    def _cb(path, val):
        if val is None: return ""
        try: return fmt.format(val)
        except Exception: return str(val)
    return _cb

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

def r16(inv, addr, signed=False, fc=3):
    time.sleep(0.02); return inv.read_register(addr, 0, functioncode=fc, signed=signed)

def r32(inv, addr, fc=3):
    hi = r16(inv, addr, False, fc); lo = r16(inv, addr+1, False, fc)
    return (hi<<16)|lo

def read_ascii(inv, addr, count):
    if count > 6:
        words = inv.read_registers(addr, 6, functioncode=3) + \
                inv.read_registers(addr+6, count-6, functioncode=3)
    else:
        words = inv.read_registers(addr, count, functioncode=3)
    b = bytearray()
    for w in words: b += bytes([(w>>8)&0xFF, w&0xFF])
    return b.rstrip(b"\x00").decode(errors="ignore")

def collect_snapshot() -> Dict[str, Any]:
    inv = mk_instrument()
    mode   = MODE_MAP.get(r16(inv, 201, False), "")
    batt_v = r16(inv, 215, True)*0.1
    batt_a = r16(inv, 216, True)*0.1
    batt_w = r16(inv, 217, True)
    soc    = r16(inv, 229, False)
    t_inv  = r16(inv, 227, True)
    fault  = r32(inv, 100)
    warn   = r32(inv, 108)
    serial_no = read_ascii(inv, 186, 12)
    return {"Mode":mode,"Batt_V":batt_v,"Batt_A":batt_a,"Batt_P":batt_w,
            "SoC":soc,"Temp":t_inv,"Fault":fault,"Warn":warn,"Serial":serial_no}

class BatteryService:
    def __init__(self, device_instance=20):
        self.svc = VeDbusService(SERVICE_NAME, register=False)
        self._idx = 0
        self._add_mandatory(device_instance)
        self.svc.register()
        self._add_values()
        log.info("registered ourselves on D-Bus as %s", SERVICE_NAME)

    def _add_mandatory(self, device_instance):
        s=self.svc
        s.add_path('/Mgmt/ProcessName', __file__)
        s.add_path('/Mgmt/ProcessVersion', FIRMWARE_VER)
        s.add_path('/DeviceInstance', device_instance)
        s.add_path('/ProductName', PRODUCT_NAME)
        s.add_path('/FirmwareVersion', FIRMWARE_VER)
        s.add_path('/Serial', "")
        s.add_path('/Connected', 0)
        s.add_path('/ProductId', 0)
        s.add_path('/Mgmt/Connection', CONNECTION)
        s.add_path('/UpdateIndex', 0)

    def _add_values(self):
        s=self.svc
        s.add_path('/Dc/0/Voltage', None, gettextcallback=_fmt("{:.1f} V"))
        s.add_path('/Dc/0/Current', None, gettextcallback=_fmt("{:.1f} A"))
        s.add_path('/Dc/0/Power',   None, gettextcallback=_fmt("{} W"))
        s.add_path('/Soc', None, gettextcallback=_fmt("{} %"))
        s.add_path('/Temperature', None, gettextcallback=_fmt("{} C"))
        s.add_path('/Status/Mode', "")
        s.add_path('/Status/FaultCode', 0)
        s.add_path('/Status/WarningCode', 0)

    def _bump(self):
        self._idx = (self._idx+1)%256
        self.svc['/UpdateIndex']=self._idx

    def publish(self, snap: Dict[str, Any]):
        s=self.svc
        s['/Connected']=1
        s['/Serial']=snap.get("Serial","")
        s['/Dc/0/Voltage']=snap["Batt_V"]
        s['/Dc/0/Current']=snap["Batt_A"]
        s['/Dc/0/Power']=snap["Batt_P"]
        s['/Soc']=snap["SoC"]
        s['/Temperature']=snap["Temp"]
        s['/Status/Mode']=snap["Mode"]
        s['/Status/FaultCode']=snap["Fault"]
        s['/Status/WarningCode']=snap["Warn"]
        self._bump()

def main():
    bat = BatteryService(device_instance=20)
    def tick():
        try:
            snap = collect_snapshot()
            bat.publish(snap)
        except Exception as e:
            log.error("poll error: %s", e)
            traceback.print_exc()
            try: bat.svc['/Connected']=0
            except Exception: pass
        return True
    GLib.timeout_add(int(POLL_SEC*1000), tick)
    GLib.MainLoop().run()

if __name__ == "__main__":
    main()

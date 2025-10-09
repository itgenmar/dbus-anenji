#!/usr/bin/env python3
# MultiPlus-style (VE.Bus) D-Bus service for Sumry inverter (Modbus RTU)
# Bus name: com.victronenergy.vebus.sumry_ttyUSB0  (auto-suffixed if taken)

import os, sys, time, logging, traceback
from typing import Dict, Any

logging.basicConfig(level=logging.INFO, format="%(levelname)s:%(name)s:%(message)s")
log = logging.getLogger("sumry-multiplus")

# ---- Vendored Victron velib_python ----
sys.path.insert(1, os.path.join(os.path.dirname(__file__), "ext", "velib_python"))
from vedbus import VeDbusService  # noqa: E402

# ---- DBus + GLib main loop ----
import gi  # noqa: E402
gi.require_version('GLib', '2.0')
from gi.repository import GLib  # noqa: E402
import dbus.mainloop.glib  # noqa: E402
dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
import dbus  # noqa: E402

# ---- Modbus / serial ----
import minimalmodbus  # noqa: E402
import serial  # noqa: E402

# ------------ Config (env-overridable) ------------
PORT     = os.environ.get("SUMRY_PORT", "/dev/ttyUSB0")
SLAVE_ID = int(os.environ.get("SUMRY_ID", "1"))
BAUD     = int(os.environ.get("SUMRY_BAUD", "9600"))
PARITY   = serial.PARITY_NONE          # device is 8N1
TIMEOUT  = float(os.environ.get("SUMRY_TIMEOUT", "1.5"))
POLL_SEC = float(os.environ.get("SUMRY_POLL", "2.0"))

SERVICE_BASE = os.environ.get("SUMRY_VEBUS_NAME", "com.victronenergy.vebus.sumry_ttyUSB0")
PRODUCT_NAME = "MultiPlus (Sumry bridge)"
FIRMWARE_VER = "1.2"
CONNECTION   = f"serial:{PORT}@{BAUD},8N1,id={SLAVE_ID}"

# ---------- Helpers ----------
def _fmt(fmt):
    def _cb(path, val):
        if val is None: return ""
        try: return fmt.format(val)
        except Exception: return str(val)
    return _cb

def pick_free_service_name(base: str) -> str:
    bus = dbus.SystemBus()
    if not bus.name_has_owner(base): return base
    i = 1
    while True:
        cand = f"{base}_{i}"
        if not bus.name_has_owner(cand): return cand
        i += 1

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
    time.sleep(0.02)
    return inv.read_register(addr, 0, functioncode=fc, signed=signed)

def r32(inv, addr, fc=3):
    hi = r16(inv, addr, False, fc)
    lo = r16(inv, addr+1, False, fc)
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

# ----- VE.Bus-ish Mode/State mapping -----
# Sumry Working Mode (reg 201): 0 PowerOn, 1 Standby, 2 Mains, 3 Off-Grid, 4 Bypass, 5 Charging, 6 Fault
# /Mode (VE.Bus Switch): 1=Charger only, 2=Inverter only, 3=On, 4=Off
def vebus_mode_from_wm(wm: int) -> int:
    if wm == 5:                 # Charging
        return 1                # Charger only
    if wm in (3, 2, 4):         # Off-Grid (inverting), Mains, Bypass
        return 3                # On
    return 4                    # Standby/PowerOn/Fault -> Off

# /State (info): 0 Off, 1 Low Power, 9 Inverting (simplified)
def vebus_state_from_wm(wm: int) -> int:
    if wm in (1, 0):            # Standby / PowerOn
        return 1
    if wm in (3, 2, 4, 5):      # Inverting / Mains / Bypass / Charging
        return 9
    if wm == 6:                 # Fault
        return 1
    return 1

# ---------- Poll ----------
def collect_snapshot() -> Dict[str, Any]:
    inv = mk_instrument()

    wm = r16(inv, 201, signed=False)
    mode_num  = vebus_mode_from_wm(wm)
    state_num = vebus_state_from_wm(wm)

    # AC-IN (mains) — registers 202..204
    in_v = r16(inv, 202, True) * 0.1
    in_f = r16(inv, 203, True) * 0.01
    in_p = r16(inv, 204, True)
    in_i = (in_p / in_v) if in_v else 0.0  # derive current; sign follows power

    # AC-OUT (L1) — registers 210..214
    out_v = r16(inv, 210, True) * 0.1
    out_i = r16(inv, 211, True) * 0.1
    out_f = r16(inv, 212, True) * 0.01
    out_p = r16(inv, 213, True)
    out_s = r16(inv, 214, True)

    # DC (battery) — 215..217 (+ optional SoC 229)
    dc_v  = r16(inv, 215, True) * 0.1
    dc_i  = r16(inv, 216, True) * 0.1
    dc_p  = r16(inv, 217, True)
    try:
        soc = r16(inv, 229, False)
    except Exception:
        soc = None

    # Active AC input: 0=AC in 1; 240=Disconnected
    active_input = 0 if wm in (2, 5, 4) and in_v > 30 else 240

    # Temps & identity
    t_dcdc = r16(inv, 226, True)
    t_inv  = r16(inv, 227, True)
    serial = read_ascii(inv, 186, 12)

    vebus_error = 0
    vebus_charge_state = 9  # Unknown

    def nz(v, fallback=0): return v if v is not None else fallback
    return {
        "ModeNum": nz(mode_num), "StateNum": nz(state_num),
        "ActiveInput": active_input,
        "Serial": serial or "",
        "In_V": nz(in_v*1.0), "In_I": nz(in_i*1.0), "In_F": nz(in_f*1.0), "In_P": nz(in_p),
        "Out_V": nz(out_v*1.0), "Out_I": nz(out_i*1.0), "Out_F": nz(out_f*1.0),
        "Out_P": nz(out_p),     "Out_S": nz(out_s),
        "Dc_V": nz(dc_v*1.0),   "Dc_I": nz(dc_i*1.0),   "Dc_P": nz(dc_p),
        "Soc": soc,
        "T_Dcdc": nz(t_dcdc),   "T_Inv": nz(t_inv),
        "VebusError": vebus_error, "VebusChargeState": vebus_charge_state,
    }

# ---------- Service ----------
class MultiPlusService:
    def __init__(self, device_instance=28):
        name = pick_free_service_name(SERVICE_BASE)
        self.svc = VeDbusService(name, register=False)
        self._idx = 0
        self._add_mandatory(device_instance)
        self.svc.register()
        self._add_values()
        log.info("registered ourselves on D-Bus as %s", name)

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
        # Phase count
        s.add_path('/Ac/NumberOfPhases', 1)

        # AC-IN (ActiveIn) + which input is active (0=AC in 1; 240=Disconnected)
        s.add_path('/Ac/ActiveIn/ActiveInput', 240)
        s.add_path('/Ac/ActiveIn/L1/V', None, gettextcallback=_fmt("{:.1f} V"))
        s.add_path('/Ac/ActiveIn/L1/I', None, gettextcallback=_fmt("{:.1f} A"))
        s.add_path('/Ac/ActiveIn/L1/F', None, gettextcallback=_fmt("{:.2f} Hz"))
        s.add_path('/Ac/ActiveIn/L1/P', None, gettextcallback=_fmt("{} W"))

        # AC-OUT (L1)
        s.add_path('/Ac/Out/L1/V', None, gettextcallback=_fmt("{:.1f} V"))
        s.add_path('/Ac/Out/L1/I', None, gettextcallback=_fmt("{:.1f} A"))
        s.add_path('/Ac/Out/L1/F', None, gettextcallback=_fmt("{:.2f} Hz"))
        s.add_path('/Ac/Out/L1/P', None, gettextcallback=_fmt("{} W"))
        s.add_path('/Ac/Out/L1/S', None, gettextcallback=_fmt("{} VA"))

        # DC (battery)
        s.add_path('/Dc/0/Voltage', None, gettextcallback=_fmt("{:.1f} V"))
        s.add_path('/Dc/0/Current', None, gettextcallback=_fmt("{:.1f} A"))
        s.add_path('/Dc/0/Power',   None, gettextcallback=_fmt("{} W"))
        s.add_path('/Soc', None, gettextcallback=_fmt("{} %"))

        # Switch & state
        s.add_path('/Mode', 4)    # 1=Charger only, 2=Inverter only, 3=On, 4=Off
        s.add_path('/State', 1)   # 0 Off, 1 Low Power, 9 Inverting (simplified)

        # VE.Bus extras
        s.add_path('/VebusError', 0)
        s.add_path('/VebusChargeState', 9)

        # Optional temps
        s.add_path('/Temperature/Dcdc', None, gettextcallback=_fmt("{} C"))
        s.add_path('/Temperature/Inverter', None, gettextcallback=_fmt("{} C"))

    def _bump(self):
        self._idx = (self._idx + 1) % 256
        self.svc['/UpdateIndex'] = self._idx

    def publish(self, snap: Dict[str, Any]):
        s=self.svc
        s['/Connected'] = 1
        s['/Serial']    = ""  # Sumry serial looks non-Victron; hide to keep UI tidy

        # AC-IN (ActiveIn)
        s['/Ac/ActiveIn/ActiveInput'] = snap["ActiveInput"]
        s['/Ac/ActiveIn/L1/V'] = snap["In_V"]
        s['/Ac/ActiveIn/L1/I'] = snap["In_I"]
        s['/Ac/ActiveIn/L1/F'] = snap["In_F"]
        s['/Ac/ActiveIn/L1/P'] = snap["In_P"]

        # AC-OUT
        s['/Ac/Out/L1/V'] = snap["Out_V"]
        s['/Ac/Out/L1/I'] = snap["Out_I"]
        s['/Ac/Out/L1/F'] = snap["Out_F"]
        s['/Ac/Out/L1/P'] = snap["Out_P"]
        s['/Ac/Out/L1/S'] = snap["Out_S"]

        # DC
        s['/Dc/0/Voltage'] = snap["Dc_V"]
        s['/Dc/0/Current'] = snap["Dc_I"]
        s['/Dc/0/Power']   = snap["Dc_P"]
        if snap["Soc"] is not None:
            s['/Soc'] = snap["Soc"]

        # Switch + state
        s['/Mode']  = snap["ModeNum"]     # 1=Charger only, 3=On, 4=Off
        s['/State'] = snap["StateNum"]

        # Extras
        s['/VebusError']       = snap["VebusError"]
        s['/VebusChargeState'] = snap["VebusChargeState"]

        # Temps
        s['/Temperature/Dcdc']     = snap["T_Dcdc"]
        s['/Temperature/Inverter'] = snap["T_Inv"]

        self._bump()

# ---------- Main ----------
def main():
    svc = MultiPlusService(device_instance=28)
    def tick():
        try:
            snap = collect_snapshot()
            svc.publish(snap)
        except Exception as e:
            log.error("poll error: %s", e)
            traceback.print_exc()
            try: svc.svc['/Connected'] = 0
            except Exception: pass
        return True
    GLib.timeout_add(int(POLL_SEC*1000), tick)
    GLib.MainLoop().run()

if __name__ == "__main__":
    main()

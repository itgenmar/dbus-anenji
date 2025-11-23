dbus-anenji

DBus interface for Anenji inverters on Venus OS, acting as a MultiPlus emulator (multiplus.py).
This implementation currently exposes inverter status, AC input, and AC output, with no control functions.

📦 Installation
1. Copy Files

Upload the project folder to:

/data/etc/


(/data persists across reboots on Venus OS.)

2. Install Required Packages
opkg update
opkg install python3-pip
pip3 install minimalmodbus

3. Configure the Serial Port

Identify your USB–RS485 device:

ls -l /dev/ttyUSB*


Then edit the main emulator script:

nano multiplus.py


Default device is:

/dev/ttyUSB1


Change this line to match your environment.

⚠️ DBus Serial Battery Conflict

If you are also running dbus-serialbattery, you must prevent it from using the same USB port.

In the serialbattery configuration, set:

EXCLUDED_DEVICES = /dev/ttyUSB1

(Adjust the device tty if yours is different.)

Once all is completed run ./install.sh

🔧 Known Issues
Incorrect DC Values at Startup

Some Anenji inverters report incorrect DC readings if they start while the internal fans are running.

Workaround used:

Install a switch to disable fans at startup

Allow the inverter to boot first

Re-enable fans after battery / PV / AC are connected

This ensures accurate DC voltage reporting.

🌞 PV Input (Not Implemented Yet)

The current version does not expose PV input data.
If anyone has Modbus register details for the Anenji PV side, contributions are welcome.

🧰 Hardware Setup

The inverter communicates via RS485 → USB.

Example adapter setup (see images in repository):

TX/RX A/B connected to inverter’s RS485 port

USB connected to the Venus OS device (e.g., Raspberry Pi / GX device)

✔️ Features Implemented
Feature	Status
Inverter output	✅ Working
AC input	✅ Working
AC output	✅ Working
DC voltage/current	⚠️ Inaccurate at startup (see notes)
PV input	❌ Not implemented
Control functions (on/off, charge, etc.)	❌ Not implemented
📄 Example Directory Structure
/data/etc/dbus-anenji/
│
├── multiplus.py
├── settings.json
├── README.md
└── other supporting scripts…

🤝 Contributions

Pull requests and register maps are welcome!
The goal is to eventually support:

PV input

Charge/Discharge control

Full MultiPlus emulation

Improved Modbus error handling

👤 Author

Created by Marius.

<img width="784" height="314" alt="image" src="https://github.com/user-attachments/assets/7164f96e-e199-419f-8bc2-9f34a1ec824a" />
<img width="781" height="311" alt="image" src="https://github.com/user-attachments/assets/698b4820-7cbc-49e4-9ebb-b12cc18e8b68" />
<img width="1306" height="823" alt="image" src="https://github.com/user-attachments/assets/9960e847-7421-4fc2-a99c-ef5e9bdee85c" />
<img width="1069" height="469" alt="image" src="https://github.com/user-attachments/assets/d70246bb-2296-41d0-9a44-e3d8bee8e347" />



dbus-anenji

DBus interface for Anenji inverters on Venus OS, acting as a MultiPlus emulator (multiplus.py).
This implementation currently exposes inverter status, AC input, and AC output, with control functions.
PV added - corect daily calculation reported in VRM. 


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

nano dbus-multiplus-emulator.py


Default device is:

/dev/ttyUSB3


Change this line to match your environment.

⚠️ DBus Serial Battery Conflict

If you are also running dbus-serialbattery, you must prevent it from using the same USB port.

In the serialbattery configuration, set:

EXCLUDED_DEVICES = /dev/ttyUSB3

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

🌞 PV Input Implemented

The current version does  expose PV input data.


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
PV input	✅ Working
Control functions (on/off, charge, etc.)	✅ Working
📄 Example Directory Structure
/data/etc/dbus-anenji/
│
├── dbus-multiplus-emulator.py.py
├── settings.json
├── README.md
└── other supporting scripts…

🤝 Contributions

Pull requests and register maps are welcome!
The goal was to eventually support: - Fixed

PV input - Done

Charge/Discharge control - Done
Charger only - Wrong naming Charges the battery and outputs power (Passtrough) 

Full MultiPlus emulation - Done

Improved Modbus error handling - Fixed

👤 Author

Created by Marius.
<img width="938" height="584" alt="image" src="https://github.com/user-attachments/assets/2b1713a1-158b-472d-a914-2b0cec45dbc8" />
<img width="335" height="435" alt="image" src="https://github.com/user-attachments/assets/3934b29f-c232-4265-a20b-2978b525a3ca" />

<img width="1184" height="785" alt="image" src="https://github.com/user-attachments/assets/327eb5e1-afc0-44b3-950a-da313d2850ec" />
<img width="977" height="532" alt="image" src="https://github.com/user-attachments/assets/359c90c2-e239-4959-9fe0-a778c8886533" />
<img width="1028" height="602" alt="image" src="https://github.com/user-attachments/assets/a2345188-3fe4-4180-890d-3c091dc0d923" />
<img width="1109" height="735" alt="image" src="https://github.com/user-attachments/assets/b5eb1f7f-b2d1-4bb2-b8b5-c1ef9a29b3d1" />
<img width="1055" height="684" alt="image" src="https://github.com/user-attachments/assets/6554b013-9596-49d4-b7e2-f5e52316b73b" />




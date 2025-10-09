# dbus-anenji

anenji invertor to venus os as multiplus = multiplus.py = Only Inverter side no controls


mkdir -p /data/venus-sumry-dbus/ext
cp -r /opt/victronenergy/velib_python /data/venus-sumry-dbus/ext/
opkg install python3 python3-pyserial
pip3 install --break-system-packages minimalmodbus



also PV as separated script  = pv_service.py


Runs on ttyUSB0

RS485 - USB

<img width="784" height="314" alt="image" src="https://github.com/user-attachments/assets/7164f96e-e199-419f-8bc2-9f34a1ec824a" />
<img width="781" height="311" alt="image" src="https://github.com/user-attachments/assets/698b4820-7cbc-49e4-9ebb-b12cc18e8b68" />
<img width="1069" height="469" alt="image" src="https://github.com/user-attachments/assets/d70246bb-2296-41d0-9a44-e3d8bee8e347" />



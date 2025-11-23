# dbus-anenji

anenji invertor to venus os as multiplus = multiplus.py = Only Inverter side no controls


Download the folder in  
/data/etc/
opkg update
opkg install python3-pip
pip3 install minimalmodbus
 
Fiind your TTYUSB and change in the script:

this version runs on ttyUSB1 just do nano and change it in the main dbus multi emulator

for the moment expose only inverter and ac input outpput. 

If someone has any experience to introduce the pv input please do. 
Also I have notice that if you are using dbus serial baterry you need to disable that in the config of the serialbattery with EXCLUDED_DEVICES = /dev/ttyUSB1 (change it to your port)

I have notice that the inverter doesnt show the correct values on DC if you turn it on with the fans one. i have used a swith and disabke them at startup (before connecting the battery,pv or ac)

Hope this helps 

Marius...
 
RS485 - USB

<img width="784" height="314" alt="image" src="https://github.com/user-attachments/assets/7164f96e-e199-419f-8bc2-9f34a1ec824a" />
<img width="781" height="311" alt="image" src="https://github.com/user-attachments/assets/698b4820-7cbc-49e4-9ebb-b12cc18e8b68" />
<img width="1069" height="469" alt="image" src="https://github.com/user-attachments/assets/d70246bb-2296-41d0-9a44-e3d8bee8e347" />



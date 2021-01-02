Compile application

cd <APP FOLDER>

make clean all FPGA_IMAGE=1

cp <APP NAME>.elf /media/<BOARD>/SOFTWARE/app.elf

touch /media/<BOARD>/reboot.txt


Open UART on linux

sudo minicom -D /dev/ttyUSB0 -b 38400


Change permission on device

sudo chmod 777 /dev/ttyUSB0

Write to UART from linux terminal

echo "string" >  /dev/ttyUSB0


----------------
- Modificar tamanho memoria SW
----------------
Modificar: m3designstart/software/common/scripts/cm3ds.ld
-> FLASH (rx) : ORIGIN = 0x0,  LENGTH = 0x20000
-> RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x20000
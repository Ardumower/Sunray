
a) How to hardware-debug using Raspberry PI4 and OpenOCD and gdb
-----------------------------------------------------------------

target: Adafruit Grand Central M4 (SAMD51P20A, 1024KB Flash, 256KB RAM)
interface: Raspberry PI4


1. Connect PI4 and Adafruit Grand Central M4 as follows:
  AGCM4 SWD  GND    ---- Raspberry PI GND
  AGCM4 SWD  SWCLK  ---- Raspberry PI GPIO 25
  AGCM4 SWD  SWDIO  ---- Raspberry PI GPIO 24
  AGCM4 SWD  RESET  ---- Raspberry PI GPIO 18
  
  (see AGCM4 schematics:  https://learn.adafruit.com/adafruit-grand-central/downloads
   and Raspberry pinout:  https://www.raspberrypi.org/documentation/usage/gpio/ )


2. On the PI4, disable the pull-ups for GPIO25, 24 and only activate pull-ups for GPIO18:
  sudo nano /etc/boot/config.txt  
  and add these lines:
  gpio=25=pn
  gpio=24=pn
  gpio=18=pu 
  
  
3. On the PI4, install OpenOCD:
  cd ~
  sudo apt-get update
  sudo apt-get install git autoconf libtool make pkg-config libusb-1.0-0 libusb-1.0-0-dev
  git clone http://openocd.zylin.com/openocd
  cd openocd
  ./bootstrap
  ./configure --enable-sysfsgpio --enable-bcm2835gpio
  make
  sudo make install


4. Create an interface config file (rpi4.cfg) with the following contents:

  adapter driver bcm2835gpio
  bcm2835gpio_peripheral_base 0xFE000000
  bcm2835gpio_speed_coeffs 236181 60
  bcm2835gpio_swd_nums 25 24
  bcm2835gpio_srst_num 18
  reset_config srst_only srst_push_pull


5. On the PI, start OpenOCD:

  sudo openocd -f rpi4.cfg -c "transport select swd" -c "adapter speed 1000"  -f atsame5x.cfg 
  (NOTE: additional targets can be found at /usr/local/share/openocd/scripts/target/)

  Output should be something like this:
  Info : Listening on port 6666 for tcl connections
  Info : Listening on port 4444 for telnet connections
  Info : BCM2835 GPIO JTAG/SWD bitbang driver
  Info : clock speed 2001 kHz
  Info : SWD DPIDR 0x2ba01477
  Info : atsame5.cpu: hardware has 6 breakpoints, 4 watchpoints
  Info : starting gdb server for atsame5.cpu on 3333
  Info : Listening on port 3333 for gdb connections
  
  If you don't get this output, use -c "debug_level 4"  for more verbose output.


6. On the PI, connect with GDB (assuming you have installed Arduino IDE Adafruit board files and compiled the sketch):

  /.arduino15/packages/adafruit/tools/arm-none-eabi-gcc/9-2019q4/bin/arm-none-eabi-gdb 
    -ex="target remote localhost:3333" /tmp/arduino_build_276904/sunray.ino.elf 

  Set a breakpoint in function 'loop' like this:
     break loop
  Run program:
     continue
  Get a backtrace of the last function calls:
     backtrace
  Delete all breakpoints 
     delete
  Reset MCU:   
     monitor reset halt
     

b) How to hardware-debug using JLink and gdb
---------------------------------------------

target: Adafruit Grand Central M4 (SAMD51P20A, 1024KB Flash, 256KB RAM)
interface: Segger JLink

1. Run JLinkGDBServer:
  "D:\Program Files (x86)\SEGGER\JLink\JLinkGDBServer.exe" -device ATSAMD51P20A -if swd -speed 4000 -port 3333

2. Connect with GDB:
  C:\Users\alex\AppData\Local\Arduino15\packages\adafruit\tools\arm-none-eabi-gcc\9-2019q4\bin>arm-none-eabi-gdb -ex="target remote localhost:3333" c:\users\alex\AppData\local\temp\arduino_build_408598\sunray.ino.elf
    
3. Flash with GDB:
  C:\Users\alex\AppData\Local\Arduino15\packages\adafruit\tools\arm-none-eabi-gcc\9-2019q4\bin>arm-none-eabi-gdb -ex="target remote localhost:3333" c:\users\alex\AppData\local\temp\arduino_build_408598\sunray.ino.elf -ex="load"
    
    
c) How to flash bootloader using JLink and OpenOCD
--------------------------------------------------
target: Adafruit Grand Central M4 (SAMD51P20A, 1024KB Flash, 256KB RAM)
interface: Segger JLink

1. Install openocd (for Windows: install WinUSB driver using UsbDriverTool)

2. Download bootloader (.bin) file:
https://github.com/adafruit/uf2-samdx1/releases

3. Create an OpenOCD file 'openocd.cfg' with the following contents:

source [find interface/jlink.cfg]
transport select swd
set CHIPNAME at91samd51p20a
source [find target/atsame5x.cfg]
init
reset 
halt
atsame5 bootloader 0
program bootloader-grandcentral_m4-v3.10.0.bin verify
atsame5 bootloader 16384
reset
shutdown

3. Run 'sudo openocd'
    
    

d) How to flash bootloader using JLink and JLink-Commander
----------------------------------------------------------
target: Adafruit Grand Central M4 (SAMD51P20A, 1024KB Flash, 256KB RAM)
interface: Segger JLink

1. Start JLink-Commander (JLink.exe) and enter:

J-Link> si 1 
J-Link> speed 4000 
J-Link> device ATSAMD51P20A
J-Link> r
J-Link> h
J-Link> erase
J-Link> loadbin bootloader-grandcentral_m4-v3.6.0.bin, 0x0
J-Link> verifybin bootloader-grandcentral_m4-v3.6.0.bin, 0x0
J-Link> r
J-Link> qc    
    

e) How to read fuse bits using JLink and JLink-Commander
----------------------------------------------------------
target: Adafruit Grand Central M4 (SAMD51P20A, 1024KB Flash, 256KB RAM)
interface: Segger JLink

1. Start JLink-Commander (JLink.exe) and enter:

J-Link> si 1 
J-Link> speed 4000 
J-Link> device ATSAMD51P20A
J-Link> mem8 0x00804000,0x2c

it will return the fuse data as follows:
00804000 = 39 92 9A F6 80 FF EC AE FF FF FF FF FF FF FF FF
00804010 = 10 40 80 00 FF FF FF FF FF FF FF FF FF FF FF FF
00804020 = FF FF FF FF FF FF FF FF FF FF FF FF

the BOOTPROT bits (bootloader protection) are located at 0x00804013:
J-Link> mem8 0x00804013,1
00804013 = 00

writing 0xF to it sets BOOTPROT size to zero (according to SAM5x datasheet):
J-Link> w1 0x00804013,0xF



    

How to hardware-debug using Raspberry PI4 and OpenOCD and gdb
------------------------------------------------------------------------

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

  Set a breakpoint like this:
     break loop
  Run program:
     continue
  Get a backtrace:
     backtrace

     
    
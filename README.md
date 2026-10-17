# Sunray firmware

# Table of contents
1. [Description](#description)
2. [Sunray for Ardumower](#sunray_ardumower)
3. [Sunray for SMARTMOW-DIY / owlPlatform / Alfred ](#sunray_alfred)
5. [Sunray Simulator](#sunray_sim)
6. [Further topics](#further_topics)
8. [Sunray App](#sunray_app)
9. [Sunray Husqvarna, STIHL, etc. conversion](#sunray_husqvarna)
   

## Description <a name="description"></a>
Sunray firmware is an alternative Firmware (experimental) for...

![sunray_platform](https://github.com/Ardumower/Sunray/assets/11735886/5d3d561a-d0eb-4028-98d9-260ea340c903)

Platform | Hardware required 
--- | ---
Ardumower | Ardumower kit mowing and gear motors, PCB 1.3, Adafruit Grand Central M4 (or Arduino Due) and ArduSimple RTK kit
Alfred | Alfred robot with Alfred RTK conversion kit (tiny Linux computer, IO-board, ArduSimple RTK kit, base antenna etc.)
SMARTMOW-DIY/owlRobotPlatform | Universal Do-It-Yourself robot mower with SMARTMOW-DIY/owlRobotPlatform hardware/PCB (More details: https://github.com/owlRobotics-GmbH/owlRobotPlatform)


The robot mower uses RTK to localize itself (without a perimeter wire)

How does it work?

Platform | Compilation 
--- | ---
Ardumower | The complete Sunray firmware and software drivers are compiled for a specific MCU (Due/M4)
Alfred | Sunray firmware is compiled for Linux. Additionally, a tiny serial robot driver is compiled for the specific Alfred MCU (STM32). The Linux Sunray firmware will communicate with this serial robot driver to control motors, read sensors etc.
owlRobotPlatform | Sunray firmware is compiled for Linux. The Linux Sunray firmware will communicate with the owlRobotPlatform hardware via CAN bus.
Simulator | The complete Sunray firmware and simulated hardware is compiled for Linux.

# Sunray for Ardumower <a name="sunray_ardumower"></a>

## Warning
All software and hardware, software and motor components are designed and optimized as a whole, if you try to replace or exclude some component not as designed, you risk to damage your hardware with the software

## Ardumower Wiki
Construction manual for the original Ardumower-based DIY robot mower:  http://wiki.ardumower.de/index.php?title=Ardumower_Sunray

## Adumower Demo video
https://www.youtube.com/watch?v=yDcmS5zdj90

## Ardumower Download
__WARNING__: Do not use the master version (via download button), that is 'code we work on' and it may be unstable - use one release version instead (click on 'releases' link below)!

https://github.com/Ardumower/Sunray/releases

# Sunray for SMARTMOW-DIY / owlPlatform / Alfred  <a name="sunray_alfred"></a>

NOTE: Below steps are only required if you want to compile a custom version of the 'Sunray for Alfred' (or SMARTMOW-DIY/owlPlatform) firmware. The code for all steps will require a Linux system (either the Alfred, a OrangePi5Pro (RaspberryPi) or some PC). For SMARTMOW-DIY/owlPlatform, it is assumed that you have installed SMARTMOW-DIY/owlPlatform drivers already as described here: https://github.com/owlRobotics-GmbH/owlRobotPlatform. NOTE: you can find a complete guide for the owlPlatform here: https://github.com/Ardumower/Sunray?tab=readme-ov-file#sunray_husqvarna

## How to install code and compile 'Sunray for SMARTMOW-DIY / owlPlatform  Alfred' (required only once)
Run this on your Linux terminal (in your Linux home folder):

```
## clone repository ##
sudo apt install git
cd ~
git clone https://github.com/Ardumower/Sunray.git

## make a customized copy of the Alfred config file (or olwPlatform config file)
cd ~/Sunray/linux
cp config_owlmower.h config.h   (for owlPlatform)
cp config_alfred.h config.h    (for Alfred)

## adjust your new 'config.h', then run service script and choose point 'Build sunray executable',
## when being asked, choose 'config.h' as config file 
./service.sh

## For Alfred: run service script and choose point 'Install sunray executable on Alfred',
## when being asked, enter your user password (see Sunray PDF manual for password)
##
## For owlPlatform: run service script and choose point 'Start sunray service'

./service.sh
```

## How to update installed code and re-compile 'Sunray for SMARTMOW-DIY / owlPlatform / Alfred'
```
## update repository ##
cd ~/Sunray
git pull

## run service script and choose point 'Rebuild sunray executable', when being asked,
## choose 'config.h' as config file 
cd ~/Sunray/linux
./service.sh

## For Alfred: run service script and choose point 'Install sunray executable on Alfred',
## when being asked, enter your user password (see Sunray PDF manual for password)
##
## For owlPlatform: run service script and choose point 'Stop sunray service',
## when being asked, enter your user password. Finally restart service script and choose
## 'Start sunray service'

./service.sh
```

## How to compile and flash 'Alfred MCU firmware'
NOTE: This step is only required if you want to compile a custom version of the Alfred MCU firmware. The Alfred MCU firmware can be compiled using Arduino compiler on the Alfred. The compiled Alfred MCU firmware will then be flashed to the Alfred MCU using OpenOCD.
```
cd sunray_install
sudo ./flash.sh
```
1. Choose 'Install Arduino IDE' (this step may not be required for Alfred SD card images)
2. Choose 'Build+Flash NGP firmware (Sunray-compatible)'


## How to solve USB serial port issues on a Raspberry PI (OS Lite 64 bit, Debian Bullseye)
Before running above commands, install required libs:
```
sudo apt-get -y install cmake
sudo apt-get -y install libbluetooth-dev
```
For OrangePi5Pro (Raspberry), you may have to adjust the serial path for the Alfred MCU UART connection in 'linux/config.h': 
```
#define SERIAL_ROBOT_PATH "/dev/ttyS0" 
```
You can find out the correct UART serial path using: 'dmesg | grep uart'. NOTE: you may have to stop running services accessing the UART serial path:
```
# find out processes accessing the UART serial path:
sudo lsof /dev/ttyS0
# list all running services:
sudo systemctl list-units --type=service --state=running
# stopping/disabling service
sudo systemctl stop serial-getty@ttyS0.service
sudo systemctl disable serial-getty@ttyS0.service
```
## How to use more robust Bit-bangling-based instead ARM-based I2C driver on a Raspberry PI (OS Lite 64 bit, Debian Bullseye)
The Raspberry CPU-based I2C driver has certain issues (e.g. missing clock stretching for BNO055, missing SCL recovery in noisy environment etc.) You can switch from the Raspberry ARM-I2C-driver to a more robust software-based driver (aka 'bit-bangling') like this:
1. Run 'sudo raspi-config' and disable the ARM I2C driver
2. Run 'sudo nano /boot/config.txt' and add this line to activate the software-based I2C driver:
dtoverlay=i2c-gpio,bus=1,i2c_gpio_sda=2,i2c_gpio_scl=3
3. Reboot ('sudo reboot')
4. Verify the I2C bus is working (e.g. a MPU 6050 IMU should be detected at address 69): 
sudo i2cdetect -y 1

## How to compile 'OpenOCD' on a Raspberry PI (OS Lite 64 bit, Debian Bullseye)
OpenOCD is used to flash the Alfred MCU firmware via GPIO interface (SWD emulation). Run this in your 'pi' home folder. The compiled binary ('openocd') can be found in folder 'src'. The binary will be called by the flash script ('~/sunray_install/flash.sh') to flash the Alfred MCU firmware. 
```
sudo apt-get -y install libusb-1.0-0
sudo apt-get -y install libusb-1.0-0-dev
sudo apt-get -y install pkg-config
sudo apt-get -y install libtool
git clone --recursive https://github.com/lupyuen/openocd-spi
cd openocd-spi
git remote add zorvalt https://github.com/Zorvalt/openocd-spi/
git pull zorvalt fix-multiple-gcc-10-errors
./boostrap
./configure --disable-internal-libjaylink --enable-sysfsgpio
make
```
Please note that the OpenOCD config ('~/sunray_install/config_files/openocd/swd-pi.ocd') has to be adjusted as the GPIO pins have different numbers:
```
# SWD banana-pi wiring:
# CON1-P18 	SWDIO   GPIO-10  (raspi GPIO24)
# CON1-P22 	SWCLK 	GPIO-47  (raspi GPIO25)
# CON1-P16 	SRST2 	GPIO-11  (raspi GPIO23)  -- main MCU
# CON1-P24 	SRST1 	GPIO-20  (raspi GPIO8)   -- perim MCU

sysfsgpio_swdio_num 24
sysfsgpio_swclk_num 25
sysfsgpio_srst_num 23
```

## How to configure Bluetooth BLE module on a Linux computer
NOTE: This step is only required if you don't use the Alfred Linux SD card image.

```
echo "----bluetooth devices----"
systemctl enable bluetooth.service
hcitool dev
# configure bluetooth BLE module
echo "----BLE config----"
echo 12 > /sys/kernel/debug/bluetooth/hci0/conn_min_interval  # 24   6
echo 20 > /sys/kernel/debug/bluetooth/hci0/conn_max_interval  # 40   6
echo 1 > /sys/kernel/debug/bluetooth/hci0/conn_latency       # 0    1
echo 153 > /sys/kernel/debug/bluetooth/hci0/adv_min_interval  # 0.625 ms units
echo 153 > /sys/kernel/debug/bluetooth/hci0/adv_max_interval  # 0.625 ms units
btmgmt -i hci0 power off
btmgmt -i hci0 le on
btmgmt -i hci0 bredr off
btmgmt -i hci0 connectable on
btmgmt -i hci0 name "alfred"
btmgmt -i hci0 advertising on
btmgmt -i hci0 power on
```

## How to compile 'Sunray Simulator' <a name="sunray_sim"></a>
The simulator will simulate an Ardumower/Alfred on a Linux computer. It uses the same firmware with simulator drivers for all robot hardware (motors, bumper, GPS, etc.). It will also simulate all network-based interfaces (HTTP, MQTT etc.) and support phone connections via Bluetooth BLE (see section above how to configure BLE module on Linux).

Run this on your Linux terminal:
```
## clone repository ##
cd ~
git clone https://github.com/Ardumower/Sunray.git
```

Now edit the file linux/config.h and uncomment only the simulation driver:
```
//#define DRV_SERIAL_ROBOT  1   // for Alfred
//#define DRV_ARDUMOWER     1   // keep this for Ardumower
#define DRV_SIM_ROBOT     1   // simulation
```

Finally, compile and run the simulator:
```
cd ~/Sunray/linux/build
rm -Rf *
cmake ..
make
sudo ./sunray
```
When uploading a map, simulator will set the robot position to the uploaded docking point. Using the Sunray App, you can simulate sensors like this:

Keyboard key | Sensor simulation 
--- | ---
o | Trigger robot obstacle sensor (bumper etc.)
r | Trigger robot rain sensor
l | Trigger robot battery low 

## Fixing issue: 'available(): not listening'
If you restart the Linux sunray process, it might be possible that the HTTP port is still not cleared from a previous session. A quick check before running the process will solve this:
```
echo "----waiting for TCP connections to be closed from previous sessions----"
echo "Waiting TCP port 80 to be closed..."
for _ in `seq 1 30`; do
  RES=$(netstat -ant | grep -w 80)
  if [ -z "$RES" ]; then
    break
  fi
  echo $RES
  sleep 2.0    
done;
```

Another issue might be that you are running the executable as normal user (and not 'root' user) - To allow a normal user to run a TCP server on port 80:
```
setcap 'cap_net_bind_service=+ep' /path/to/program 
```

## How to run motor test etc. on Linux
To run motor test, sensor test etc. on Linux you need to run the sunray executable manually (without the service). Once the executable is started and outputting console output, you simply enter the desired AT-commands and press ENTER:
```
AT+E  motor tet
AT+F  sensor test
```
Alternatively, you can find out the terminal name (with Linux command 'tty') and then send any text from another Linux terminal:
```
echo -e "AT+E\r\n"  > /dev/pts/1
```
 
## Further topics <a name="further_topics"></a>
Generating robot heatmaps (WiFi/GPS signal quality etc.):
https://forum.ardumower.de/threads/advanced-topic-generate-wifi-gps-heatmaps-with-sunray-on-alfred-or-ardumower-with-connected-raspberry-pi.25078/

# Sunray App Manual <a name="sunray_app"></a>
There exists an App to to control and monitor the Sunray firmware. More Details can be found in the Sunray App manual: https://drive.google.com/file/d/1NNrUEm2_amfMAjGY7ShcYtixtUgWGnyJ/view?usp=sharing

# Husqvarna, STIHL etc. conversion <a name="sunray_husqvarna"></a>
There exists an owlPlatform DIY conversion manual how to convert your Husqvarna, STIHL etc. to an DIY robot mower: https://docs.google.com/document/d/1rZi848AFimIRHJkMun1muZ-2nvr8JjXCmmPBuZ1BfJA/edit?usp=sharing










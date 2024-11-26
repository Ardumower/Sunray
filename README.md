# Sunray firmware

# Table of contents
1. [Description](#description)
2. [Sunray for Ardumower](#sunray_ardumower)
3. [Sunray for Alfred / owlPlatform](#sunray_alfred)
4. [Sunray Simulator](#sunray_sim)
5. [Further topics](#further_topics)
6. [Sunray ROS](#sunray_ros)
   

## Description <a name="description"></a>
Sunray firmware is an alternative Firmware (experimental) for...

![sunray_platform](https://github.com/Ardumower/Sunray/assets/11735886/5d3d561a-d0eb-4028-98d9-260ea340c903)

Platform | Hardware required 
--- | ---
Ardumower | Ardumower kit mowing and gear motors, PCB 1.3, Adafruit Grand Central M4 (or Arduino Due) and ArduSimple RTK kit
Alfred | Alfred robot with Alfred RTK conversion kit (tiny Linux computer, IO-board, ArduSimple RTK kit, base antenna etc.)
owlRobotPlatform | Universal Do-It-Yourself robot mower with owlRobotPlatform hardware/PCB (More details: https://github.com/owlRobotics-GmbH/owlRobotPlatform)


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
http://wiki.ardumower.de/index.php?title=Ardumower_Sunray

## Adumower Demo video
https://www.youtube.com/watch?v=yDcmS5zdj90

## Ardumower Download
__WARNING__: Do not use the master version (via download button), that is 'code we work on' and it may be unstable - use one release version instead (click on 'releases' link below)!

https://github.com/Ardumower/Sunray/releases

# Sunray for Alfred / owlPlatform <a name="sunray_alfred"></a>

NOTE: Below steps are only required if you want to compile a custom version of the 'Sunray for Alfred' (or owlPlatform) firmware. The code for all steps will require a Linux system (either the Alfred, a Raspberry PI or some PC).

## How to install code and compile 'Sunray for Alfred / owlPlatform' (required only once)
Run this on your Alfred Linux terminal (in your Alfred home folder):

```
## clone repository ##
cd ~
git clone https://github.com/Ardumower/Sunray.git

## make a customized copy of the Alfred config file (or olwPlatform config file)
cd ~/Sunray/alfred
cp config_alfred.h config.h    (for Alfred)
cp config_owlmower.h config.h   (for owlPlatform)

## adjust your new 'config.h', then run service script and choose point 'Build sunray executable',
## when being asked, choose 'config.h' as config file 
./service.sh

## For Alfred: run service script and choose point 'Install sunray executable on Alfred',
## when being asked, enter your user password (see Sunray PDF manual for password)
##
## For owlPlatform: run service script and choose point 'Start sunray service'

./service.sh
```

## How to update installed code and re-compile 'Sunray for Alfred / owlPlatform'
```
## update repository ##
cd ~/Sunray
git pull

## run service script and choose point 'Rebuild sunray executable', when being asked,
## choose 'config.h' as config file 
cd ~/Sunray/alfred
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
For Raspberry PI, you may have to adjust the serial path for the Alfred MCU UART connection in 'alfred/config.h': 
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

Now edit the file alfred/config.h and uncomment only the simulation driver:
```
//#define DRV_SERIAL_ROBOT  1   // for Alfred
//#define DRV_ARDUMOWER     1   // keep this for Ardumower
#define DRV_SIM_ROBOT     1   // simulation
```

Finally, compile and run the simulator:
```
cd ~/Sunray/alfred/build
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

## Fixing issue: 'error while loading shared libraries: libtf2_ros.so: cannot open shared object file: No such file or directory'
If running the Sunray firmware as an ROS node and getting above error message, you might have to run this:  
```
sudo ldconfig /opt/ros/melodic/lib/ 
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

## Sunray ROS (via Docker)  <a name="sunray_ros"></a>
The Sunray firmware can be compiled as ROS (robotic operating system) node, and ROS packages can then be used to replace the localization drivers (GPS/IMU) in Sunray. Typical ROS packages are LiDAR drivers, LiDAR-based SLAM (LiDAR mapping & localization) and LiDAR-based obstacle detection. Currently, the ROS system has been experimentally tested using these hardware combinations:

- Raspberry PI 5 and Livox MID-360 ( https://owlrobotics-store.company.site/products/LIDAR-Livox-MID-360-incl-Cable-p605042897 )

Demo videos ( https://www.youtube.com/watch?v=47-9z_iPiTs , https://www.youtube.com/watch?v=OYC_oKYsXts, https://www.youtube.com/watch?v=eZM_R9n57KA,  https://www.youtube.com/watch?v=vfD_GPRI-98 )

![Screenshot from 2024-07-24 11-05-29](https://github.com/user-attachments/assets/2daee793-c45e-4119-9ca0-a72ca4369e33)

Note that the LiDAR localization is not finished yet. You can use it, however a relocalization process (if your robot has no idea where it is) has to be triggered manually for now. Later versions will trigger the relocalization process automatically.

Because ROS is highly dependend on OS (e.g. you have to choose specific Ubuntu versions), and the used small computer is also dependend on another OS (e.g. Raspberry PI works best on Raspi OS), we have chosen to run ROS in a virtualization (Docker).
![image](https://github.com/user-attachments/assets/90dd6c8f-c8c0-4b9b-b82e-93b4a591d67e)

Steps to run Sunray as ROS node:

1. Make a copy of your existing config.h with another name (e.g. Sunray/alfred/config_myrobot.h), and activate these entries:
```
#define ROS_LAUNCH_FILE     "myrobot"  // the ROS robot launch file (you will choose in point 2 below)  
#define LIDAR_BUMPER_ENABLE true  // to use the LiDAR-based bumper (ground obstacle detection via LiDAR)
#define GPS_LIDAR           1     // to use LiDAR instead of RTK-GPS/IMU for localization
```
Depending on your options, the corresponding Sunray drivers will be compiled and used. 

![image](https://github.com/user-attachments/assets/a87f0991-6915-477c-945d-fadeb5f707c8)

2. In your 'Sunray/ros/src/sunray_node/launch' folder, make a copy of 'owlmower.launch' and rename it 'myrobot.launch'. This will be the ROS launch file for your robot.
3. If using the LiDAR bumper, adjust the LiDAR bumper parameters in your myrobot.launch (opening angle, max distance, max width, max height, near distance, near height, min obstacle size):
```
    <arg name="ground_lidar_angle_opening" default="180.0" />
    <arg name="ground_lidar_max_distance" default="0.5" />
    <arg name="ground_lidar_max_width" default="1.0" />
    <arg name="ground_lidar_max_height" default="0.5" />
    <arg name="ground_lidar_near_distance" default="0.3" />
    <arg name="ground_lidar_near_height" default="0.3" />
    <arg name="ground_lidar_min_obstacle_size" default="0.2" />
```
4. In your myrobot.launch, adjust the Sunray ROS localization mode:
Choose this if you don't want to use LiDAR localization (instead of RTK-GPS):
```
	<arg name="sunray_ros_mode" default="SIMPLE" />
```
Choose this if you want to use ROS localization:
```
	<arg name="sunray_ros_mode" default="LOCALIZATION" />
```

5. In your myrobot.launch, adjust the transformation from your LiDAR sensor position to the GPS position (between the wheels).Example:
![Screenshot from 2024-07-23 21-05-09](https://github.com/user-attachments/assets/97d6e772-5105-4e58-9e12-f11818a17b92)
NOTE: It can be tricky to find the correct parameters. Start with some parameters, then steer the robot (rotate), and look if the trajectory is correct. If not, further adjust your parameters.

5. In your 'Sunray/ros' folder, run 'service.sh' and choose:
- 'Docker install' to install docker on your Raspberry PI.
- 'Docker pull ROS image' to pull Ubuntu 18.04 docker image.
- 'Docker build container' to build the Docker container (the first layer on your docker image).
- 'Docker prepare ROS tools' to build all required tools in the Docker container (Livox SDK etc.).
- 'ROS compile' to build Sunray as ROS node (and required ROS nodes for LiDAR localization etc.).  
- 'ROS run test LIDAR' to test the LiDAR.
- 'ROS start LiDAR mapping' to start the LiDAR mapping. Connect to your robot, steer the robot around to show the LiDAR your environment (try to stay away 5m from the robot, so the mapping process creates better maps). 
- 'ROS stop LiDAR mapping' to stop the mapping. This will create a 3D pointcloud file (.PCD) use for localization.
- 'ROS run sunray localization' to start the LiDAR localization. Connect to your robot, and use Sunray as usual (create a new map, start mowing etc.).
- 'ROS trigger re-localization' to trigger a relocalization.

Here you can see how the localization pipeline works.
- Mapping: First the LiDAR points are gravity aligned, so that the Z axis looks away earth (if you ground is flat, it will be parallel to earth after this step). This ensures that the created map is parallel to earth. While steering the robot around, the gravity aligned LiDAR data is then used to iteratively build a complete LiDAR map of the environment. NOTE: there is no loop-closure involed, however in most cases it will create consistent maps (.PCD file).
- Localization: First the LiDAR points are (optionally) flipped upside-down (if your LiDAR is mounted upside-down). Then the corrected LiDAR data is used to localize against the PCD file. The result of the localization is two components: the LiDAR position (x, y, z) within the PCD file and the LiDAR orientation (yaw, pitch, roll). 
![image](https://github.com/user-attachments/assets/a02f9ff2-a7d8-430e-bf2d-c90ae4e4bf0c)
   







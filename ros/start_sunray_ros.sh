#!/bin/bash

# starts Sunray ROS node in (already prepared) Docker container


CONTAINER_NAME="ros1"

echo "EUID=$EUID"
echo "PWD=$PWD"
WHOAMI=`whoami`


if [ "$EUID" -ne 0 ]
  then echo "Please run as root (sudo)"
  exit
fi

# if [ ! -d "/home/pi/Sunray/alfred/build" ]; then
#  echo "install Sunray first!"
#  exit
# fi

if [[ `pidof sunray` != "" ]]; then
  echo "Sunray linux app already running! Exiting..."
  exit
fi

if [[ `pidof sunray_node` != "" ]]; then
  echo "Sunray ROS node already running! Exiting..."
  exit
fi


# starting dbus monitor
../ros/scripts/dbus_monitor.sh &


# -----------------------------------------
echo "trying setup CAN interface..."
echo "NOTE: you may have to edit boot config to enable CAN driver (see https://github.com/owlRobotics-GmbH/owlRobotPlatform)"
ip link set can0 up type can bitrate 1000000
retcode=$?
if [ $retcode -ne 0 ]; then
  echo "trying CAN-USB-bridge (SLCAN) ..."
  ls /dev/serial/by-id/usb-Raspberry_Pi_Pico*
  PICO_DEV=`ls /dev/serial/by-id/usb-Raspberry_Pi_Pico*`
  echo "PICO_DEV=$PICO_DEV"
  sudo slcand -o -s8 -t hw -S 3000000 $PICO_DEV
  sudo ip link set up slcan0
fi
#exit


# -----------------------------------------
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
#btmgmt -i hci0 name "bananapi $MAC"
#btmgmt -i hci0 name "owlMower"
BLE_NAME=`grep "BLE_NAME" ../sunray/config.h | cut -d'"' -f 2`
echo "BLE_NAME=$BLE_NAME"
btmgmt -i hci0 name "$BLE_NAME"
btmgmt -i hci0 advertising on
btmgmt -i hci0 power on
## btmgmt -i hci0 advinfo
# -----------------------------------------


echo "---configuring Ethernet for LiDAR---"
# add static IP address 
# Ethernet wired connection
CON=$(nmcli c | grep ethernet | head -c 20 | xargs )
echo "ETH CON: $CON"
nmcli c show "$CON" | grep ipv4.addresses
nmcli con mod "$CON" ipv4.addresses 192.168.1.102/24
nmcli con mod "$CON" ipv4.method manual    
nmcli con up "$CON"
# remove any old IP addresses
nmcli con mod "$CON" -ipv4.addresses "" -ipv4.gateway ""
nmcli con up "$CON"     
echo "CON: $CON"
nmcli c show "$CON" | grep ipv4.addresses
arp

# setup audio interface
# https://gavv.net/articles/pulseaudio-under-the-hood/
# https://www.freedesktop.org/wiki/Software/PulseAudio/Documentation/User/SystemWide/
# https://unix.stackexchange.com/questions/204522/how-does-pulseaudio-start
if ! command -v play &> /dev/null
then 
  echo "installing audio player..."
  apt install -y libsox-fmt-mp3 sox mplayer alsa-utils pulseaudio
fi
# add root to dialout group
echo "whoami: $WHOAMI"
echo "belonging to groups: "
groups
# show audio devices
#aplay -l
cat /proc/asound/cards
# restart pulseaudio daemon as root
#killall pulseaudio
#sleep 1
#pulseaudio -D --system --disallow-exit --disallow-module-loading
#pulseaudio --dump-conf
# sudo chmod 666 /var/run/pulse/native
# set default volume 
#adduser root dialout audio pulse-access pulse
#export PULSE_SERVER=unix:/var/run/pulse/native
echo "we will test host audio now... (you should hear a voice)"
../ros/scripts/dbus_send.sh -m Play -p ../tts/de/system_starting.mp3



echo "----waiting for TCP connections to be closed from previous sessions----"
echo "Waiting TCP port 80 to be closed..."
for _ in `seq 1 20`; do 
  RES=$(netstat -ant | grep -w 80 | grep LISTEN)
  #RES=$(lsofs -i:80)
  #RES=$(fuser 80/tcp) 
  if [ -z "$RES" ]; then
    break
  fi
  echo $RES
  PID=`fuser 80/tcp | cut -d' ' -f 2`
  echo "PID:$PID"
  ls -l /proc/$PID/exe
  # echo -n .  
  sleep 2.0     
done; 


echo "----starting sunray ros----"
#if [ -z "$SUNRAY_ROS_MODE" ]; then
#  SUNRAY_ROS_MODE=SIMPLE
#fi
if [ -z "$USE_BAG_FILE" ]; then
  USE_BAG_FILE=false
fi
if [ -z "$SUNRAY_ROS_LAUNCH" ]; then
  #SUNRAY_ROS_LAUNCH=owlmower.launch
  SUNRAY_ROS_LAUNCH=`grep "ROS_LAUNCH_FILE" ../sunray/config.h | cut -d'"' -f 2`
fi
if [ -z "$SUNRAY_ROS_RVIZ" ]; then
  SUNRAY_ROS_RVIZ=false
fi
echo "=====> SUNRAY_ROS_MODE: $SUNRAY_ROS_MODE <====="
echo "sunray_ros_launch: $SUNRAY_ROS_LAUNCH"
echo "start rviz: $SUNRAY_ROS_RVIZ"
echo "working dir: $PWD"    
echo "DISPLAY: $DISPLAY"  
echo "USE_BAG_FILE: $USE_BAG_FILE"
echo "BAG_FILE: $BAG_FILE"


WCON=$(nmcli c | grep wifi | head -1 | tail -c 12 | xargs )
echo "WIFI CON: $WCON"

WIP=`ifconfig $WCON | grep 'inet ' | awk -F'[: ]+' '{ print $3 }'`
echo "WIFI IP: $WIP"

# ROS_IP="$WIP"

# allow non-root to start http server 
#sudo setcap 'cap_net_bind_service=+ep' devel/lib/sunray_node/sunray_node

#docker stop $CONTAINER_NAME && docker start $CONTAINER_NAME && docker exec -t -it $CONTAINER_NAME \
#  bash -c 'mplayer /root/Sunray/tts/de/temperature_low_docking.mp3'  
#exit  

# allow docker to access host Xserver 
xhost +local:* 

# source ROS setup  
CMD="export DISPLAY=$DISPLAY"
if [[ $ROS_IP != "" ]]; then
  CMD+="; export ROS_IP=$ROS_IP"
fi 
CMD+="; export ROS_HOME=/root/Sunray/alfred"
CMD+="; . /ros_entrypoint.sh"
CMD+="; rosclean purge -y"
CMD+="; export ROSCONSOLE_CONFIG_FILE=/root/Sunray/ros/rosconsole.config"
CMD+="; export ROS_PYTHON_LOG_CONFIG_FILE=/root/Sunray/ros/python_logging.config" 
CMD+="  ; cd /root/Sunray/ros"
CMD+="  ; . devel/setup.bash"
CMD+="; setcap 'cap_net_bind_service=+ep' devel/lib/sunray_node/sunray_node"
CMD+="; cp -n -r src/sunray_node/config/camera_info /root/Sunray/alfred"
CMD+="; cd /root/Sunray/alfred"
CMD+="; pwd"
CMD+="; chmod o+x+r+w /root"
#CMD+="; groupadd -g 1000 pi || true"
#CMD+="; useradd -g 1000 -u 1000 pi || true"
CMD+="; roslaunch sunray_node run.launch sunray_ros_launch:=$SUNRAY_ROS_LAUNCH sunray_ros_mode:=$SUNRAY_ROS_MODE rviz:=$SUNRAY_ROS_RVIZ use_bag_file:=$USE_BAG_FILE bag_file:=$BAG_FILE map_pcd:=/root/PCD/dlio_map.pcd"
docker stop $CONTAINER_NAME && docker start $CONTAINER_NAME && docker exec -t -it $CONTAINER_NAME bash -c "$CMD" 

# rosnode kill -a ; sleep 3



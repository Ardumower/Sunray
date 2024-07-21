#!/bin/bash

# starts Sunray ROS node in (already prepared) Docker container


CONTAINER_NAME="ros1"

echo "EUID=$EUID"
echo "PWD=$PWD"



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
  #exit
fi


# -----------------------------------------
echo "setup CAN interface..."
echo "NOTE: you may have to edit boot config to enable CAN driver (see https://github.com/owlRobotics-GmbH/owlRobotPlatform)"
ip link set can0 up type can bitrate 1000000


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


# setup audio interface
# https://gavv.net/articles/pulseaudio-under-the-hood/
# https://www.freedesktop.org/wiki/Software/PulseAudio/Documentation/User/SystemWide/
# https://unix.stackexchange.com/questions/204522/how-does-pulseaudio-start
sudo ln -sf /var/run/pulse/native /tmp/pulse_socket
sudo chmod 666 /tmp/pulse_socket
sudo chmod 666 /var/run/pulse/native  
export PULSE_SERVER=unix:/var/run/pulse/native

if ! command -v play &> /dev/null
then 
  echo "installing audio player..."
  apt install -y libsox-fmt-mp3 sox mplayer alsa-utils pulseaudio
fi
# show audio devices
#aplay -l
cat /proc/asound/cards
# restart pulseaudio daemon as root
killall pulseaudio
sleep 1
pulseaudio -D --system --disallow-exit --disallow-module-loading
# set default volume 
amixer -D pulse sset Master 100%
#mplayer /home/pi/Sunray/tts/de/temperature_low_docking.mp3
#exit    


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
  # echo -n .  
  sleep 2.0     
done; 



echo "----starting sunray ros----"
echo "working dir:$PWD"    

WCON=$(nmcli c | grep wifi | head -1 | tail -c 12 | xargs )
echo "WIFI CON: $WCON"

WIP=`ifconfig $WCON | grep 'inet ' | awk -F'[: ]+' '{ print $3 }'`
echo "WIFI IP: $WIP"

# allow non-root to start http server 
#sudo setcap 'cap_net_bind_service=+ep' devel/lib/sunray_node/sunray_node

#docker stop $CONTAINER_NAME && docker start $CONTAINER_NAME && docker exec -t -it $CONTAINER_NAME \
#  bash -c 'mplayer /root/Sunray/tts/de/temperature_low_docking.mp3'  
#exit  

# source ROS setup  
docker stop $CONTAINER_NAME && docker start $CONTAINER_NAME && docker exec -t -it $CONTAINER_NAME \
  bash -c "export ROS_IP=$WIP ; export ROS_HOME=/root/Sunray/alfred ; . /ros_entrypoint.sh ; cd /root/Sunray/ros ; . devel/setup.bash ; setcap 'cap_net_bind_service=+ep' devel/lib/sunray_node/sunray_node ; cd /root/Sunray/alfred ; pwd ; roslaunch sunray_node run.launch" 

# rosnode kill -a ; sleep 3

#!/usr/bin/env bash


# build Sunray ROS node
cd build
cmake ..
make

cd build


echo "----bluetooth devices----"
systemctl enable bluetooth.service
hcitool dev
# configure bluetooth BLE module
echo "----BLE config----"
echo 12 > /sys/kernel/debug/bluetooth/hci0/conn_min_interval  # 24   6
echo 20 > /sys/kernel/debug/bluetooth/hci0/conn_max_interval  # 40   6
echo 1 > /sys/kernel/debug/bluetooth/hci0/conn_latency       # 0    1
btmgmt -i hci0 power off
btmgmt -i hci0 le on
btmgmt -i hci0 bredr off
btmgmt -i hci0 connectable on
btmgmt -i hci0 name "ROS robot"
btmgmt -i hci0 advertising on
btmgmt -i hci0 power on


# setup CAN bus
ip link set can0 up type can bitrate 1000000    


echo "---------all processes using the CAN bus------------"
sudo lsof | grep -i can_raw
echo "----------------------------------------------------"



# setup audio interface
# https://www.freedesktop.org/wiki/Software/PulseAudio/Documentation/User/SystemWide/
if ! command -v play &> /dev/null
then 
  echo "installing audio player..."
  apt install -y libsox-fmt-mp3 sox mplayer
fi
# show audio devices
aplay -l
# restart pulseaudio daemon as root
killall pulseaudio
pulseaudio -D --system --disallow-exit --disallow-module-loading
# set default volume 
amixer -D pulse sset Master 100%


echo "----waiting for TCP connections to be closed from previous sessions----"
echo "Waiting TCP port 80 to be closed..."
for _ in `seq 1 20`; do 
  RES=$(netstat -ant | grep -w 80)
  #RES=$(lsofs -i:80)
  #RES=$(fuser 80/tcp) 
  if [ -z "$RES" ]; then
    break
  fi
  echo $RES
  # echo -n .  
  sleep 2.0     
done; 



# run Sunray ROS node 
# allow non-root to start http server 
sudo setcap 'cap_net_bind_service=+ep' devel/lib/sunray_node/sunray_node
# source ROS setup
. devel/setup.bash
roslaunch sunray_node test.launch 




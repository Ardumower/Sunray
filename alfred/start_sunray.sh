#!/bin/bash

echo "EUID=$EUID"
echo "PWD=$PWD"

CMD=""



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
echo "----bluetooth devices----"
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
#btmgmt -i hci0 name "bananapi $MAC"
btmgmt -i hci0 name "bananapi"
btmgmt -i hci0 advertising on
btmgmt -i hci0 power on
## btmgmt -i hci0 advinfo
# -----------------------------------------


echo "----waiting for TCP connections to be closed from previous sessions----"
echo "Waiting TCP port 80 to be closed..."
for _ in `seq 1 10`; do 
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


echo "----starting sunray----"
echo "CMD=$CMD"
    


if [ -d "/home/pi/Sunray/alfred/build" ]; then
  # state and map files will be written here 
  cd /boot/sunray
  # pick sunray from here
  /usr/bin/stdbuf -oL -eL /home/pi/Sunray/alfred/build/sunray
else
  # pick sunray from here  
  /usr/bin/stdbuf -oL -eL $PWD/build/sunray
fi 

# debug mode
# exec gdbserver :1234 /home/pi/sunray_install/sunray "$@"
# debug executable with:  gdb /home/pi/sunray_install/sunray


#!/bin/bash

# start/stop sunray service
# start/stop camera streaming web service on "localhost:8081"

echo "EUID=$EUID"
echo "PWD=$PWD"


CMD=""


# start USB camera streaming web server 
function start_cam_service() {
  if [[ `pidof motion` != "" ]]; then
        echo "motion app already running! Exiting..."
        exit
  fi
  echo "starting motion service..."
  cp config_files/motion/motion.conf /etc/motion
  cp $PWD/config_files/motion/motion.service /etc/systemd/system/motion.service
  chmod 644 /etc/systemd/system/motion.service
  systemctl daemon-reload
  systemctl enable motion
  systemctl start motion
  systemctl --no-pager status motion
  echo "motion service started!"
}

function stop_cam_service() {
  echo "stopping motion service..."
  systemctl stop motion
  systemctl disable motion
  echo "motion service stopped!"
}

function start_sunray_service() {
  if [[ `pidof sunray` != "" ]]; then
        echo "Sunray linux app already running! Exiting..."
        exit
  fi
  # enable sunray service
  echo "starting sunray service..."
  #ln -s /home/pi/sunray_install/config_files/sunray.service /etc/systemd/system/sunray.service
  cp $PWD/config_files/sunray/sunray.service /etc/systemd/system/sunray.service
  chmod 644 /etc/systemd/system/sunray.service
  mkdir -p /boot/sunray
  chmod 644 /boot/sunray
  systemctl daemon-reload
  systemctl enable sunray
  systemctl start sunray
  systemctl --no-pager status sunray
  echo "sunray service started!"
}

function stop_sunray_service() {
  # disable sunray service
  echo "stopping sunray service..."
  systemctl stop sunray
  systemctl disable sunray
  echo "sunray service stopped!"
}

function start_log_service(){
  echo "starting logging service..."
  service rsyslog start
}

function stop_log_service(){
  echo "stopping logging service..."
  service rsyslog stop
}

function start_dm(){
  echo "starting display manager..."
  service lightdm start
}

function stop_dm(){
  echo "stopping display manager..."
  service lightdm stop
}

function list(){
  echo "listing services..."
  service --status-all
}

if [ "$EUID" -ne 0 ]
  then echo "Please run as root (sudo)"
  exit
fi

if [ ! -d "/etc/motion" ]; then
  echo installing motion...
  apt install motion
fi


# if [ ! -d "/home/pi/Sunray" ]; then
#  echo "install Sunray first!"
#  exit
# fi

#systemctl status sunray
#systemctl status motion


# show menu
PS3='Please enter your choice: '
options=("Start sunray service" 
  "Stop sunray service" 
  "Start camera service"  
  "Stop camera service"
  "Start logging service"
  "Stop logging service"
  "Start display manager"   
  "Stop display manager"   
  "List services"   
  "Quit")
select opt in "${options[@]}"
do
    case $opt in
        "Start sunray service")
            start_sunray_service
            break
            ;;
        "Stop sunray service")
            stop_sunray_service
            break
            ;;            
        "Start camera service")
            start_cam_service
            break
            ;;
        "Stop camera service")
            stop_cam_service
            break
            ;;
        "Start logging service")
            start_log_service
            break
            ;;
        "Stop logging service")
            stop_log_service
            break
            ;;
        "Start display manager")
            start_dm
            break
            ;;                                        
        "Stop display manager")
            stop_dm
            break
            ;;                                        
        "List services")
            list
            break
            ;;                                        
        "Quit")
            break
            ;;
        *) echo "invalid option $REPLY";;
    esac
done



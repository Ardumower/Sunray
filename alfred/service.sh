#!/bin/bash

# start/stop sunray service
# start/stop camera streaming web service on "localhost:8081"

echo "EUID=$EUID"
echo "PWD=$PWD"


CMD=""


function build_sunray() {
  if ! command -v cmake &> /dev/null
  then
    echo "installing required Linux libraries... (you may have to enter your sudo password)"  
    sudo apt-get -y install cmake
    sudo apt-get -y install libbluetooth-dev
  fi
  if [ "$EUID" -eq 0 ]
    then echo "Please run as non-root (not sudo)"
    exit
  fi

  PS3='Please choose config: '
  list=$(ls config*.h)
  IFS=$'\n'
  select CONFIG_FILE in $list 
    do test -n "$CONFIG_FILE" && break; 
  exit; 
  done
  echo "selected: $CONFIG_FILE"

  CONFIG_PATHNAME=$PWD/$CONFIG_FILE
  rm -f CMakeCache.txt
  rm -f cmake_install.cmake
  rm -Rf CMakeFiles
  cd build
  rm -Rf * 
  #exit
  cmake -D CONFIG_FILE=$CONFIG_PATHNAME ..
  make 
}


function rebuild_sunray() {
  if [ "$EUID" -eq 0 ]
    then echo "Please run as non-root (not sudo)"
    exit
  fi
  cd build
  make
}


function install_sunray_alfred() {
  if [ "$EUID" -eq 0 ]
    then echo "Please run as non-root (not sudo)"
    exit
  fi
  echo "NOTE: enter user password if being asked (see Sunray PDF manual for password)"
  echo "stopping sunray service..."
  sudo systemctl stop sunray
  echo "installing executable..."
  cp build/sunray ~/sunray_install/
  echo "starting sunray service..."
  sudo systemctl start sunray
}


# start USB camera streaming web server 
function start_cam_service() {
  if [ "$EUID" -ne 0 ]
    then echo "Please run as root (sudo)"
    exit
  fi
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
  if [ "$EUID" -ne 0 ]
    then echo "Please run as root (sudo)"
    exit
  fi
  echo "stopping motion service..."
  systemctl stop motion
  systemctl disable motion
  echo "motion service stopped!"
}

function start_sunray_service() {
  if [ "$EUID" -ne 0 ]
    then echo "Please run as root (sudo)"
    exit
  fi  
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
  if [ "$EUID" -ne 0 ]
    then echo "Please run as root (sudo)"
    exit
  fi
  # disable sunray service
  echo "stopping sunray service..."
  systemctl stop sunray
  systemctl disable sunray
  echo "sunray service stopped!"
}

function start_log_service(){
  if [ "$EUID" -ne 0 ]
    then echo "Please run as root (sudo)"
    exit
  fi  
  echo "starting logging service..."
  service rsyslog start
}

function stop_log_service(){
  if [ "$EUID" -ne 0 ]
    then echo "Please run as root (sudo)"
    exit
  fi
  echo "stopping logging service..."
  service rsyslog stop
}

function start_dm(){
  if [ "$EUID" -ne 0 ]
    then echo "Please run as root (sudo)"
    exit
  fi
  echo "starting display manager..."
  service lightdm start
}

function stop_dm(){
  if [ "$EUID" -ne 0 ]
    then echo "Please run as root (sudo)"
    exit
  fi
  echo "stopping display manager..."
  service lightdm stop
}

function list(){
  if [ "$EUID" -ne 0 ]
    then echo "Please run as root (sudo)"
    exit
  fi
  echo "listing services..."
  service --status-all
}

function showlog(){
  echo "show log..."
  journalctl -f -u sunray
}

function savelog(){
  echo "saving log.txt..."
  journalctl -u sunray > log.txt
}

function kernel_log(){
  echo "kernel log..."
  dmesg -wH
}

if [ ! -d "/etc/motion" ]; then
  echo installing motion...
  sudo apt install motion
fi


# if [ ! -d "/home/pi/Sunray" ]; then
#  echo "install Sunray first!"
#  exit
# fi

#systemctl status sunray
#systemctl status motion


# show menu
PS3='Please enter your choice: '
options=( 
  "Build sunray executable" "Rebuild sunray executable"
  "Install sunray executable on Alfred"
  "Start sunray service" "Stop sunray service" 
  "Start camera service" "Stop camera service"
  "Start logging service" "Stop logging service"
  "Start display manager" "Stop display manager"   
  "List services"   
  "Show log" "Save log" "Kernel log"
  "Quit")
select opt in "${options[@]}"
do
    case $opt in
        "Build sunray executable")
            build_sunray
            break
            ;;
        "Rebuild sunray executable")
            rebuild_sunray
            break
            ;;
        "Install sunray executable on Alfred")
            install_sunray_alfred
            break
            ;;
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
        "Show log")
            showlog
            break
            ;;     
        "Save log")
            savelog
            break
            ;;
        "Kernel log")
            kernel_log
            break
            ;;                                   
        "Quit")
            break
            ;;
        *) echo "invalid option $REPLY";;
    esac
done



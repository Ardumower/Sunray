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
  sudo rm ../sunray/config.h
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
  REPLACEPATH="/home/pi/Sunray/alfred"
  sed "s+$REPLACEPATH+$PWD+g" <$PWD/config_files/sunray/sunray.service.example >$PWD/config_files/sunray/sunray.service

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


function start_sunray() {
  if [ "$EUID" -ne 0 ]
    then echo "Please run as root (sudo)"
    exit
  fi
  echo "starting sunray... (press CTRL+C to stop)"
  ./start_sunray.sh
}

function start_sunray_motor_test() {
  if [ "$EUID" -ne 0 ]
    then echo "Please run as root (sudo)"
    exit
  fi
  echo "starting sunray... (press CTRL+C to stop)"
  ./start_sunray.sh <<< "AT+E"
}

function start_sunray_sensor_test() {
  if [ "$EUID" -ne 0 ]
    then echo "Please run as root (sudo)"
    exit
  fi
  echo "starting sunray... (press CTRL+C to stop)"  
  ./start_sunray.sh <<< "AT+F"
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

function list_services(){
  if [ "$EUID" -ne 0 ]
    then echo "Please run as root (sudo)"
    exit
  fi
  echo "listing services..."
  service --status-all
}

function show_log(){
  echo "show log... (press CTRL+C to stop)"
  journalctl -f -u sunray
}

function save_log(){
  echo "saving log.txt..."
  journalctl -u sunray > log.txt
}

function clear_log(){
  if [ "$EUID" -ne 0 ]
    then echo "Please run as root (sudo)"
    exit
  fi
  echo "clearing log..."
  journalctl --rotate --vacuum-time=1s -u sunray
}

function kernel_log(){
  echo "kernel log... (press CTRL+C to stop)"
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


compile_menu () {
    echo "Compile and test menu (NOTE: press CTRL+C to stop any pending actions)"
    options=(
        "Build sunray executable (you can choose config file)" 
        "Rebuild sunray executable (using last choosen config file)"
        "Run sunray executable"
        "Run sunray executable with motor test (WARNING: motors will start with full speed, jack-up your robot!)"
        "Run sunray executable with sensor test"
        "Back"
    )
    select option in "${options[@]}"; do
        case $option in
            ${options[0]})
                build_sunray
                break
            ;;
            ${options[1]})
                rebuild_sunray
                break
            ;;
            ${options[2]})
                start_sunray
                break
            ;;
            ${options[3]})
                start_sunray_motor_test
                break
            ;;
            ${options[4]})
                start_sunray_sensor_test
                break
            ;;
            ${options[5]})
                return
             ;;
            *) 
                echo invalid option
            ;;
        esac
    done
}



linux_services_menu () {
    echo "Linux services menu (NOTE: press CTRL+C to stop any pending actions)"
    options=(
        "Install sunray executable on existing Alfred file system (Alfred-only)" 
        "Start sunray service (as Linux autostart)"
        "Stop sunray service"
        "Start camera service (as Linux autostart)"
        "Stop camera service"
        "Start Linux logging service"
        "Stop Linux logging service"
        "Start display manager"
        "Stop display manager"
        "List services"
        "Back"
    )
    select option in "${options[@]}"; do
        case $option in
            ${options[0]})
                install_sunray_alfred
                break
            ;;
            ${options[1]})
                start_sunray_service
                break
            ;;
            ${options[2]})
                stop_sunray_service
                break
            ;;
            ${options[3]})
                start_cam_service
                break
            ;;
            ${options[4]})
                stop_cam_service
                break
            ;;
            ${options[5]})
                start_log_service
                break
            ;;
            ${options[6]})
                stop_log_service
                break
            ;;
            ${options[7]})
                start_dm
                break
            ;;
            ${options[8]})
                stop_dm
                break
            ;;
            ${options[9]})
                list_services
                break
            ;;
            ${options[10]})
                return
             ;;
            *) 
                echo invalid option
            ;;
        esac
    done
}



linux_logging_menu () {
    echo "Linux logging menu (NOTE: press CTRL+C to stop any pending actions)"
    options=(
        "Show log" 
        "Save log"
        "Clear log"
        "Kernel log"
        "Back"
    )
    select option in "${options[@]}"; do
        case $option in
            ${options[0]})
                show_log
                break
            ;;
            ${options[1]})
                save_log
                break
            ;;
            ${options[2]})
                clear_log
                break
            ;;
            ${options[3]})
                kernel_log
                break
            ;;
            ${options[4]})
                return
             ;;
            *) 
                echo invalid option
            ;;
        esac
    done
}



main_menu () {
    echo "Main menu (NOTE: press CTRL+C to stop any pending actions)"
    options=(
        "Compile and test menu"
        "Linux services menu"
        "Linux logging menu"
        "Quit"
    )
    select option in "${options[@]}"; do
        case $option in
            ${options[0]})
                compile_menu
                break
            ;;
            ${options[1]})
                linux_services_menu
                break
            ;;
            ${options[2]})
                linux_logging_menu
                break
             ;;
            ${options[3]})
                exit
             ;;
            *) 
                echo invalid option
            ;;
        esac
    done
}


# show main menu
while true
do 
  main_menu
done



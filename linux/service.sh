#!/bin/bash

# start/stop sunray service
# start/stop camera streaming web service on "localhost:8081"

echo "EUID=$EUID"
echo "PWD=$PWD"

KERNEL="$(uname -sr)"
CPU="$(uname -m)"
DISTRI="$(source /etc/os-release && echo "$ID")"

echo "CPU=$CPU  DISTRI=$DISTRI  KERNEL=$KERNEL"
 
cpu_ok=false
case "$CPU" in
  x86_64|amd64)          cpu_ok=true ;;   # Intel/AMD 64-bit
  aarch64|arm64)         cpu_ok=true ;;   # ARM 64-bit
  armv7l)                cpu_ok=true ;;   # ARM 32-bit
esac
if ! $cpu_ok; then
  echo "WARNING: probably unsupported CPU: $CPU (expected: x86_64/amd64, aarch64/arm64)"
fi

distri_ok=false
case "$DISTRI" in
  ubuntu)            distri_ok=true ;;   
  debian)            distri_ok=true ;;   
  raspbian)          distri_ok=true ;;   
esac
if ! $distri_ok; then
  echo "WARNING: probably unsupported distribution: $DISTRI (expected: ubuntu, debian, raspbian)"
fi

echo


CMD=""


if [ "$EUID" -eq 0 ]
  then echo "Please run as non-root (not sudo)"
  exit
fi


function build_sunray() {
  if ! command -v cmake &> /dev/null
  then
    echo "installing required Linux libraries... (you may have to enter your sudo password)"  
    sudo apt-get -y install cmake
    sudo apt-get -y install libbluetooth-dev libssl-dev
  fi
  #if [ "$EUID" -eq 0 ]
  #  then echo "Please run as non-root (not sudo)"
  #  exit
  #fi

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
  
  # Ensure build directory exists and cleanup safely inside it
  mkdir -p build
  cd build || { echo "ERROR: failed to enter build directory"; return 1; }
  if [[ "$(basename "$PWD")" != "build" ]]; then
    echo "ERROR: safety check failed (not in build dir), aborting cleanup"
    return 1
  fi
  # Remove previous build artifacts only inside build directory (including dotfiles)
  shopt -s dotglob nullglob
  rm -rf -- *
  shopt -u dotglob nullglob
  
  #exit
  cmake -D CONFIG_FILE=$CONFIG_PATHNAME ..
  make 
  cd ..
}


function rebuild_sunray() {
  #if [ "$EUID" -eq 0 ]
  #  then echo "Please run as non-root (not sudo)"
  #  exit
  #fi
  cd build
  make
  cd ..
}


function install_sunray_alfred() {
  #if [ "$EUID" -eq 0 ]
  #  then echo "Please run as non-root (not sudo)"
  #  exit
  #fi
  echo "NOTE: enter user password if being asked (see Sunray PDF manual for password)"
  echo "stopping sunray service..."
  sudo systemctl stop sunray
  echo "installing executable..."
  cp build/sunray ~/sunray_install/
  echo "starting sunray service..."
  sudo systemctl start sunray
}

# start RecoverCAN service 
function start_recovercan_service() {
  #if [ "$EUID" -ne 0 ]
  #  then echo "Please run as root (sudo)"
  #  exit
  #fi
  if [[ `pidof recovercan` != "" ]]; then
        echo "recovercan service already running! Exiting..."
        exit
  fi
  echo "starting recovercan service..."
  #cp config_files/motion/motion.conf /etc/motion

  REPLACEPATH="/home/pi/Sunray/linux"
  sed "s+$REPLACEPATH+$PWD+g" <$PWD/config_files/recovercan/recovercan.service.example >$PWD/config_files/recovercan/recovercan.service

  sudo cp $PWD/config_files/recovercan/recovercan.service /etc/systemd/system/recovercan.service
  sudo chmod 644 /etc/systemd/system/recovercan.service
  sudo systemctl daemon-reload
  sudo systemctl enable recovercan
  sudo systemctl start recovercan
  sudo systemctl --no-pager status recovercan
  echo "recovercan service started! (see log with: 'journalctl -f -u recovercan')"
}

function stop_recovercan_service() {
  #if [ "$EUID" -ne 0 ]
  #  then echo "Please run as root (sudo)"
  #  exit
  #fi
  echo "stopping recovercan service..."
  sudo systemctl stop recovercan
  sudo systemctl disable recovercan
  echo "recovercan service stopped!"
}


# start USB camera streaming web server 
function start_cam_service() {
  #if [ "$EUID" -ne 0 ]
  #  then echo "Please run as root (sudo)"
  #  exit
  #fi
  if [[ `pidof motion` != "" ]]; then
        echo "motion app already running! Exiting..."
        exit
  fi
  echo "starting motion service..."
  cp config_files/motion/motion.conf /etc/motion
  sudo cp $PWD/config_files/motion/motion.service /etc/systemd/system/motion.service
  sudo chmod 644 /etc/systemd/system/motion.service
  sudo systemctl daemon-reload
  sudo systemctl enable motion
  sudo systemctl start motion
  sudo systemctl --no-pager status motion
  echo "motion service started!"
}

function stop_cam_service() {
  #if [ "$EUID" -ne 0 ]
  #  then echo "Please run as root (sudo)"
  #  exit
  #fi
  echo "stopping motion service..."
  sudo systemctl stop motion
  sudo systemctl disable motion
  echo "motion service stopped!"
}

function start_sunray_service() {
  #if [ "$EUID" -ne 0 ]
  #  then echo "Please run as root (sudo)"
  #  exit
  #fi  
  if [[ `pidof sunray` != "" ]]; then
        echo "Sunray linux app already running! Exiting..."
        exit
  fi
  # enable sunray service
  echo "starting sunray service..."
  #ln -s /home/pi/sunray_install/config_files/sunray.service /etc/systemd/system/sunray.service
  REPLACEPATH="/home/pi/Sunray/linux"
  sed "s+$REPLACEPATH+$PWD+g" <$PWD/config_files/sunray/sunray.service.example >$PWD/config_files/sunray/sunray.service

  sudo cp $PWD/config_files/sunray/sunray.service /etc/systemd/system/sunray.service
  sudo chmod 644 /etc/systemd/system/sunray.service
  sudo mkdir -p /boot/sunray
  sudo chmod 644 /boot/sunray
  sudo systemctl daemon-reload
  sudo systemctl enable sunray
  sudo systemctl start sunray
  sudo systemctl --no-pager status sunray
  echo "sunray service started!"
}

function stop_sunray_service() {
  #if [ "$EUID" -ne 0 ]
  #  then echo "Please run as root (sudo)"
  #  exit
  #fi
  # disable sunray service
  echo "stopping sunray service..."
  sudo systemctl stop sunray
  sudo systemctl disable sunray
  echo "sunray service stopped!"
}


function start_sunray() {
  #if [ "$EUID" -ne 0 ]
  #  then echo "Please run as root (sudo)"
  #  exit
  #fi
  echo "starting sunray... (press CTRL+C to stop)"
  sudo ./start_sunray.sh
}

function start_sunray_motor_test() {
  #if [ "$EUID" -ne 0 ]
  #  then echo "Please run as root (sudo)"
  #  exit
  #fi
  echo "starting sunray... (press CTRL+C to stop)"
  sudo ./start_sunray.sh <<< "AT+E"
}

function start_sunray_sensor_test() {
  #if [ "$EUID" -ne 0 ]
  #  then echo "Please run as root (sudo)"
  #  exit
  #fi
  echo "starting sunray... (press CTRL+C to stop)"  
  sudo ./start_sunray.sh <<< "AT+F"
}



function start_log_service(){
  #if [ "$EUID" -ne 0 ]
  #  then echo "Please run as root (sudo)"
  #  exit
  #fi  
  echo "starting logging service..."
  sudo service rsyslog start
}

function stop_log_service(){
  #if [ "$EUID" -ne 0 ]
  #  then echo "Please run as root (sudo)"
  #  exit
  #fi
  echo "stopping logging service..."
  sudo service rsyslog stop
}

function start_dm(){
  #if [ "$EUID" -ne 0 ]
  #  then echo "Please run as root (sudo)"
  #  exit
  #fi
  echo "starting display manager..."
  sudo service lightdm start
}

function stop_dm(){
  #if [ "$EUID" -ne 0 ]
  #  then echo "Please run as root (sudo)"
  #  exit
  #fi
  echo "stopping display manager..."
  sudo service lightdm stop
}

function list_services(){
  #if [ "$EUID" -ne 0 ]
  #  then echo "Please run as root (sudo)"
  #  exit
  #fi
  echo "listing services..."
  sudo service --status-all
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
  #if [ "$EUID" -ne 0 ]
  #  then echo "Please run as root (sudo)"
  #  exit
  #fi
  echo "clearing log..."
  sudo journalctl --rotate --vacuum-time=1s -u sunray
}

function kernel_log(){
  echo "kernel log... (press CTRL+C to stop)"
  dmesg -wH
}


function update_local_repository(){
  if ! command -v git &> /dev/null
  then
    sudo apt-get -y install git
  fi
  git pull
  echo "restarting script file..."
  exec "$0" "$@"
}


function i2c_scanner(){
  if ! command -v i2cdetect &> /dev/null
  then
    sudo apt-get -y install i2c-tools
  fi
  echo "Note: there should be detected at least one I2C device (e.g. IMU at address 0x69)" 
  sudo i2cdetect -y 1
}

function show_network_interfaces(){
  if ! command -v ifconfig &> /dev/null
  then
    sudo apt-get -y install net-tools
  fi
  sudo ip link set can0 up type can bitrate 1000000
  echo "------Note: there should be listed a CAN interface (e.g. can0)------"
  ifconfig -s
  echo "------------"
}


function gpio_status(){
  if ! command -v gpio &> /dev/null
  then
    sudo apt-get -y install wiringpi
  fi
  echo "------Note: for owlPCB, physical pin12 should be V=0 (mode down)------"
  gpio readall
}


function linux_info(){
  if ! command -v screenfetch &> /dev/null
  then
    sudo apt-get -y install screenfetch
  fi
  screenfetch
}


function upgrade_linux_system(){
  read -p "Upgrade all linux system packages? (y/n): " answer
  if [[ "$answer" == "y" || "$answer" == "Y" ]]; then
    sudo apt-get update -y && sudo apt-get upgrade -y
  fi
}


function network_manager(){
  if ! command -v nmtui &> /dev/null
  then
    sudo apt-get -y install network-manager
  fi
  nmtui 
}


function fix_local_repository_file_permissions(){
  USER=$(whoami)
  echo "changing all files' owner to: $USER..."
  sudo chown -R $USER:$USER ../  
}


function install_anydesk(){
  if ! command -v wget &> /dev/null
  then
    sudo apt-get -y install wget
  fi
  CURDIR=$(pwd)
  cd /tmp
  wget https://download.anydesk.com/rpi/anydesk_7.0.2-1_arm64.deb
  sudo apt install -y ./anydesk_7.0.2-1_arm64.deb
  cd "$CURDIR"
}


if [ ! -d "/etc/motion" ]; then
  echo installing motion...
  sudo apt-get -y install motion
  stop_cam_service
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
        "Start RecoverCAN service (as Linux autostart)"
        "Stop RecoverCAN service"
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
                start_recovercan_service
                break
            ;;
            ${options[4]})
                stop_recovercan_service
                break
            ;;
            ${options[5]})
                start_cam_service
                break
            ;;
            ${options[6]})
                stop_cam_service
                break
            ;;
            ${options[7]})
                start_log_service
                break
            ;;
            ${options[8]})
                stop_log_service
                break
            ;;
            ${options[9]})
                start_dm
                break
            ;;
            ${options[10]})
                stop_dm
                break
            ;;
            ${options[11]})
                list_services
                break
            ;;
            ${options[12]})
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


linux_diag_menu () {
    echo "Linux diagnostics menu (NOTE: press CTRL+C to stop any pending actions)"
    options=(        
        "Linux info"
        "I2C scanner"
        "Show network/CAN interfaces"
        "GPIO status"
        "Network manager (WiFi etc.)"
        "Back"
    )
    select option in "${options[@]}"; do
        case $option in
            ${options[0]})
                linux_info
                break
            ;;
            ${options[1]})
                i2c_scanner
                break
            ;;
            ${options[2]})
                show_network_interfaces
                break
            ;;
            ${options[3]})
                gpio_status
                break
             ;;
            ${options[4]})
                network_manager
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




update_menu () {
    echo "Update menu (NOTE: press CTRL+C to stop any pending actions)"
    options=(
        "Update local Sunray repository from remote Github repository" 
        "Fix local Sunray repository file permissions"
        "Upgrade all Linux system packages"
        "Install AnyDesk"
        "Back"
    )
    select option in "${options[@]}"; do
        case $option in
            ${options[0]})
                update_local_repository
                break
            ;;
            ${options[1]})
                fix_local_repository_file_permissions
                break
            ;;
            ${options[2]})
                upgrade_linux_system
                break
            ;;
            ${options[3]})
                install_anydesk
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
        "Linux diagnostics menu"
        "Update menu"
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
                linux_diag_menu
                break
            ;;
            ${options[4]})
                update_menu
                break
            ;;
            ${options[5]})
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



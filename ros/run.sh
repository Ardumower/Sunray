#!/usr/bin/env bash


function docker_install {
  # install docker
  # Add Docker's official GPG key:
  sudo apt-get update
  sudo apt-get install ca-certificates curl
  sudo install -m 0755 -d /etc/apt/keyrings
  sudo curl -fsSL https://download.docker.com/linux/debian/gpg -o /etc/apt/keyrings/docker.asc
  sudo chmod a+r /etc/apt/keyrings/docker.asc

  # Add the repository to Apt sources:
  echo \
    "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/debian \
    $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
    sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
  sudo apt-get update

  sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
}

function docker_build_image {
  # create ROS docker image
  docker build --tag ros1 --file Dockerfile . 
}


function ros_compile {
  # build Sunray ROS node
  rm -Rf build
  rm -Rf devel
  catkin_make
}

function ros_run {
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
}



PS3='Please enter your choice: '
options=(
    "Docker install"
    "Docker build ROS image"
    "ROS build"    
    "ROS run"    
    "Quit")
select opt in "${options[@]}"
do
    case $opt in
        "Docker install")
            docker_install
            break
            ;;
        "Docker build ROS image")
            docker_build_image
            break
            ;;
        "ROS compile")
            ros_compile
            break
            ;;
        "ROS run")
            ros_run
            break
            ;;
        "Quit")
            break
            ;;
        *) echo "invalid option $REPLY";;
    esac
done




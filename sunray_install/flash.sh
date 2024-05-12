#!/bin/bash

# STM32 flash script (OpenOCD)

CONFIG=/home/pi/sunray_install/config_files/openocd
OPENOCD_PATH=/home/pi/sunray_install/openocd
INSTALL_PATH=/home/pi/sunray_install
DEV_PATH=/home/pi/ngp_rm18


chr() {
  [ "$1" -lt 256 ] || return 1
  printf "\\$(printf '%03o' "$1")"
}

ord() {
  LC_CTYPE=C printf '%d' "'$1"
}


function install_arduino_ide {
    if [ ! -d "/home/pi/arduino-1.8.19" ]; then
        echo "installing Arduino IDE..."        
        cd /home/pi/
        runuser -l pi -c 'wget https://downloads.arduino.cc/arduino-1.8.19-linuxarm.tar.xz'
        runuser -l pi -c 'tar -xvf arduino-1.8.19-linuxarm.tar.xz'
        cd arduino-1.8.19/
        sudo rm /usr/local/bin/arduino
        sudo ./install.sh
        sudo usermod -a -G dialout $USER
    fi
    echo "installing STMicroelectronics boards..."        
    # https://github.com/arduino/Arduino/blob/master/build/shared/manpage.adoc
    #runuser -l pi -c 'arduino --install-boards arduino:sam'
    runuser -l pi -c 'arduino --save-prefs --pref "boardsmanager.additional.urls=file:///home/pi/ngp_rm18/config_files/arduino/package_stmicroelectronics_index.json"'     
    runuser -l pi -c 'arduino --install-boards STMicroelectronics:stm32'
}

function build_sunray_compatible() {
    # https://github.com/arduino/Arduino/blob/master/build/shared/manpage.adoc
    echo "building sunray-compatible firmware for NGP mower..."
    #runuser -l pi -c 'arduino --version'
    #runuser -l pi -c 'arduino --get-pref sketchbook.path'    
    #runuser -l pi -c 'arduino --verbose --verify --board arduino:sam:arduino_due_x /home/pi/ngp_rm18/rm18/rm18'    
    # NOTE: options you may add:  --verbose
    runuser -l pi -c "rm -f /tmp/firmware/rm18.ino.bin"
    runuser -l pi -c 'arduino --pref build.path=/tmp/firmware --verify --board "STMicroelectronics:stm32:GenF1:pnum=GENERIC_F103VETX" /home/pi/ngp_rm18/rm18/rm18'
    if [ -f /tmp/firmware/rm18.ino.bin ]; then
      runuser -l pi -c "cp /tmp/firmware/rm18.ino.bin $DEV_PATH/firmware/"
      # install    
      if [ -d "$INSTALL_PATH" ]; then
        runuser -l pi -c "cp $DEV_PATH/firmware/rm18.ino.bin $INSTALL_PATH/firmware/"
      fi
      true
    else  
      echo "ERROR: no firmware binary was generated! Check above for errors!"
      false
    fi    
}


function build_openocd {
    echo "building openocd..."
    cd /home/pi/openocd-spi
    ./bootstrap
    ./configure --disable-internal-libjaylink --enable-sysfsgpio
    make
}

function make_cmd {
    local CMD=$1 
    CRC=0
    for letter in $(sed 's/./&\n/g' <(printf $CMD)); do
        CRC=$CRC+`ord $letter`
    done    
    CRC=`printf '%x' $(($CRC & 0xFF))`
    CMD="$CMD,0x$CRC\r\n"
    retval=$CMD
}

# test NGP sunray-compatible firmware communication via UART
function test_ngp {
    echo 'testing NGP sunray-compatible firmware communication via UART...'
    UART="/dev/ttyS1"
    stty -F $UART 19200
    # man ascii    
    make_cmd "AT+S"
    CMD_S=$retval
    make_cmd "AT+M,0,0,0"
    CMD_MF=$retval    
    make_cmd "AT+M,-50,-50,0"
    CMD_MB=$retval    
    for i in {1..100}
    do 
      timeout 0.5s cat -v < $UART &            
      echo "sending $CMD_S"
      echo -ne $CMD_S > $UART          
      
      echo "sending $CMD_MF"
      echo -ne $CMD_MF > $UART          
      sleep 1.0

      #echo "sending $CMD_MB"
      #echo -ne $CMD_MB > $UART                
      #sleep 10.0
    done
}

# test GPIO pins
# SWD banana-pi wiring:
# CON1-P18 	SWDIO   GPIO-10  (raspi GPIO24)
# CON1-P22 	SWCLK 	GPIO-47  (raspi GPIO25)
# CON1-P16 	SRST2 	GPIO-11  (raspi GPIO23)  -- main MCU
# CON1-P24 	SRST1 	GPIO-20  (raspi GPIO8)   -- perim MCU

# choose SWD pins for NGP main MCU via port-expander pca9555  
function mux_swd {
  #echo "setting host cpu freq..."
  #echo "1400000" > /sys/devices/system/cpu/cpufreq/policy0/scaling_max_freq
  echo "enabling GPIO SWD for ngp main mcu"
  sudo i2cset -y 1 0x20 0x06 0xbf  # configure output (CS6 = O0.6)
  #sudo i2cset -y 1 0x20 0x02 0x00  # choose perim MCU (CS6 = LOW)
  sudo i2cset -y 1 0x20 0x02 0x40  # choose main MCU (CS6 = HIGH)
}

function test_gpio_swd {
    mux_swd
    echo 'testing SWD GPIO pins...'
    for i in {1..100}
    do 
        echo "test $i"
        #sudo gpioset 0 10=0
        #sudo gpioset 0 47=0
        sudo gpioset 0 11=0
        sleep 2.0
        #sudo gpioset 0 10=1
        #sudo gpioset 0 47=1
        sudo gpioset 0 11=1
        sleep 2.0
    done
}



function test_gpio {
    echo 'testing GPIO pins...'
    # Banana Pi BPI-M4 has a 40-pin GPIO header Following is the Banana Pi GPIO Pinout: 
    # https://wiki.banana-pi.org/Banana_Pi_BPI-M4#GPIO_PIN_define
    # receiving pins
    #  idx    0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  
    PINS=(    3   5   7  11  12  13  15  16  18  22  24  26  29  31  32  33  35  36  37   )  
    GPIOS=(  17  16  21  42   3  43  44  11  10  47  20  22  41  40  50   4   2  53  34   )    
    COUNT=${#GPIOS[@]}
    LAST_GPIO=${GPIOS[COUNT-1]}        
    # set pin
    SET_PIN_IDX=0
    SET_PIN=${PINS[SET_PIN_IDX]}
    SET_GPIO=${GPIOS[SET_PIN_IDX]}    
    echo "number of GPIO pins to test: $COUNT"
    for SET_STATE in {0..1}
    do
        echo "***** setting pin $SET_PIN (GPIO $SET_GPIO) = $SET_STATE *****"
        sudo gpioset 0 $SET_GPIO=$SET_STATE                
        IDX=0
        for GPIO in "${GPIOS[@]}"
        do                 
            if [[ $GPIO == $SET_GPIO ]]; then   
                STATE='skipping sending pin'
            else
                STATE=`sudo gpioget 0 $GPIO`
            fi        
            PIN=${PINS[IDX]} 
            echo "reading pin $PIN (GPIO $GPIO) = $STATE"              
            IDX=$((IDX+1))
        done
    done
    echo 'test done'
    # make set pin input
    sudo gpioget 0 $SET_GPIO    
}


# toggle SWD reset pin (SRST)
function reset_swd {
    # reset via I2C GPIO
    #echo "RESET active"
    #sudo i2cset -y 1 0x21 0x06 0x00 # config all PORT0 pins output
    #sudo i2cset -y 1 0x21 0x02 0x00 # all PORT0 pins low
    #sleep 0.2

    #echo "RESET deactive"
    #sudo i2cset -y 1 0x21 0x02 0xff # all PORT0 pins high
    #sudo i2cset -y 1 0x21 0x06 0xff # config all PORT0 pins input
    #sleep 0.1

    # reset via GPIO pin
    echo "RESET"
    sudo gpioset 0 11=0
    sleep 0.2
    sudo gpioset 0 11=1
    sleep 0.2
}


function flash_original {
  cd $OPENOCD_PATH
  mux_swd
  #reset_swd  
  ./openocd -s $OPENOCD_PATH/tcl -f $CONFIG/swd-pi.ocd -f $CONFIG/flash-original.ocd
}

function flash_sunray_compatible {
  cd $OPENOCD_PATH
  mux_swd
  #reset_swd
  ./openocd -s $OPENOCD_PATH/tcl -f $CONFIG/swd-pi.ocd -f $CONFIG/flash-sunray-compat.ocd
}

function save_firmware {
  cd $OPENOCD_PATH
  mux_swd
  #reset_swd
  ./openocd -s $OPENOCD_PATH/tcl -f $CONFIG/swd-pi.ocd -f $CONFIG/save-firmware.ocd
  echo "Tip: compare binary files with 'cmp -l file1 file2'"
}


function build_flash_sunray_compatible {    
    if build_sunray_compatible $1; then 
        flash_sunray_compatible
    fi
}


if [ "$EUID" -ne 0 ]
  then echo "Please run as root (sudo)"
  exit
fi

#if [ ! -d "/home/pi/sunray_install" ]; then
#  echo "run install.sh first!"
#  exit
#fi

# parse (optional) command line options
if [ $# -ne 0 ]; then
  if [ $1 = "--help" ]; then   
    echo "flash --save               save firmware (dump)"
    echo "flash --flash sunray|ngp   flash sunray/ngp firmware"    
    exit
  fi
  if [ $1 = "--save" ]; then
    save_firmware
    exit
  elif [ $1 = "--flash" ]; then
    if [ $2 = "sunray" ]; then
        flash_sunray_compatible
        exit
    elif [ $2 = "ngp" ]; then    
        flash_original
        exit
    else
        exit
    fi
  fi
fi 


# show menu
PS3='Please enter your choice: '
options=("Build OpenOCD" "Save firmware (dump)" 
    "Flash NGP mower (original firmware)"  
    "Flash NGP mower (Sunray-compatible firmware)"  
    "Install Arduino IDE" "Build NGP firmare (Sunray-compatible)" 
    "Build+Flash NGP firmware (Sunray-compatible)"  
    "Test SWD GPIO pins"
    "Test GPIO pins" 
    "Test NGP firmware (Sunray-compatible) communication via UART"   
    "Reset SWD pin"
    "Quit")
select opt in "${options[@]}"
do
    case $opt in
        "Build OpenOCD")
            build_openocd
            break
            ;;
        "Save firmware (dump)")
            save_firmware
            break
            ;;            
        "Flash NGP mower (original firmware)")
            flash_original
            break
            ;;
        "Flash NGP mower (Sunray-compatible firmware)")
            flash_sunray_compatible
            break
            ;;
        "Install Arduino IDE")
            install_arduino_ide
            break
            ;;
        "Build NGP firmare (Sunray-compatible)")
            build_sunray_compatible
            break
            ;;        
        "Build+Flash NGP firmware (Sunray-compatible)")
            build_flash_sunray_compatible
            break
            ;;
        "Test SWD GPIO pins")
            test_gpio_swd
            break
            ;;
        "Test GPIO pins")
            test_gpio
            break
            ;;        
        "Test NGP firmware (Sunray-compatible) communication via UART")
            test_ngp
            break
            ;;
        "Reset SWD pin")
            reset_swd
            break
            ;;

        "Quit")
            break
            ;;
        *) echo "invalid option $REPLY";;
    esac
done


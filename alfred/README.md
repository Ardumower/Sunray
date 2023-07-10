# Sunray for Alfred

NOTE: Below steps are only required if you want to compile a custom version of the 'Sunray for Alfred' firmware.

## How to install code and compile 'Sunray for Alfred' (required only once)
Run this on your Alfred Linux terminal:

```
## clone repository ##
cd ~
git clone https://github.com/Ardumower/Sunray.git

## compile sunray (NOTE: 'make' will always copy config.h in current makefile folder into Sunray source folder) ##
cd ~/Sunray/alfred/build
rm -Rf *
cmake ..
make

## install new sunray executable ##
sudo systemctl stop sunray
cp sunray ~/sunray_install/
sudo systemctl start sunray
```

## How to update installed code and re-compile 'Sunray for Alfred'
```
## update repository ##
cd ~/Sunray
git pull

## compile sunray (NOTE: 'make' will always copy config.h in current makefile folder into Sunray source folder) ##
cd ~/Sunray/alfred/build
make

## install new sunray executable ##
sudo systemctl stop sunray
cp sunray ~/sunray_install/
sudo systemctl start sunray
```

## How to compile and flash 'Alfred MCU firmware'
NOTE: This step is only required if you want to compile a custom version of the Alfred MCU firmware. The Alfred MCU firmware can be compiled using Arduino compiler on the Alfred. The compiled Alfred MCU firmware will then be flashed to the Alfred MCU using OpenOCD.
```
cd sunray_install
sudo ./flash.sh
```
1. Choose 'Install Arduino IDE' (this step may not be required for Alfred SD card images)
2. Choose 'Build+Flash NGP firmware (Sunray-compatible)'


## How to compile 'Sunray for Alfred' on a Raspberry PI (OS Lite 64 bit, Debian Bullseye)
Before running above commands, install required libs:
```
sudo apt-get -y install cmake
sudo apt-get -y install libbluetooth-dev
```

## How to compile 'OpenOCD' on a Raspberry PI (OS Lite 64 bit, Debian Bullseye)
OpenOCD is used to flash the Alfred MCU firmware via GPIO interface (SWD emulation). Run this in your 'pi' home folder. The compiled binary ('openocd') can be found in folder 'src'. The binary will be called by the flash script ('~/sunray_install/flash.sh') to flash the Alfred MCU firmware. 
```
sudo apt-get -y install libtool
git clone --recursive https://github.com/lupyuen/openocd-spi
cd openocd-spi
git remote add zorvalt https://github.com/Zorvalt/openocd-spi/
git pull zorvalt fix-multiple-gcc-10-errors
./boostrap
./configure --disable-internal-libjaylink --enable-sysfsgpio
make
```
Please note that the OpenOCD config ('~/sunray_install/config_files/openocd/swd-pi.ocd') has to be adjusted as the GPIO pins have different numbers:
```
# SWD banana-pi wiring:
# CON1-P18 	SWDIO   GPIO-10  (raspi GPIO24)
# CON1-P22 	SWCLK 	GPIO-47  (raspi GPIO25)
# CON1-P16 	SRST2 	GPIO-11  (raspi GPIO23)  -- main MCU
# CON1-P24 	SRST1 	GPIO-20  (raspi GPIO8)   -- perim MCU

sysfsgpio_swdio_num 24
sysfsgpio_swclk_num 25
sysfsgpio_srst_num 23
```


## License
Ardumower Sunray 
Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH

Licensed GPLv3 for open source use
or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)
    

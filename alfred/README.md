# Sunray for Alfred

## How to install code and compile 'Sunray for Alfred' (required only once)
Run this on your Alfred:

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

## How to compile 'Sunray for Alfred' on a Raspberry PI (OS Lite 64 bit, Debian Bullseye)
Before running above commands, install required libs:
```
sudo apt-get -y install cmake
sudo apt-get -y install libbluetooth-dev
```

## How to compile 'OpenOCD' with SPI emulation on a Raspberry PI (OS Lite 64 bit, Debian Bullseye)
OpenOCD is used to flash the Alfred MCU firmware via GPIO interface (SPI emulation). Run this in your 'pi' home folder:
```
sudo apt-get -y install libtool
git clone --recursive https://github.com/lupyuen/openocd-spi
cd openocd-spi
./boostrap
./configure --disable-internal-libjaylink --enable-sysfsgpio
make
```

## License
Ardumower Sunray 
Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH

Licensed GPLv3 for open source use
or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)
    

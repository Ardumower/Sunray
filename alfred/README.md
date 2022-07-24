# Sunray for Alfred

## How to compile 'Sunray for Alred'
Run this on your Alfred:

```
## clone repository ##
cd ~
git clone https://github.com/Ardumower/Sunray.git

## compile sunray ##
cp ~/sunray_install/config_files/sunray/config.h ~/Sunray
cd ~/Sunray/alfred/build
rm -Rf *
cmake ..
make

## install new sunray executable ##
cp sunray ~/sunray_install/
cd ~/sunray_install
sudo ./service.sh     (to stop and start sunray service)
```
## License
Ardumower Sunray 
Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH

Licensed GPLv3 for open source use
or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)
    

#!/bin/bash

# update files in sunray_install path via online-updater

URL=http://grauonline.de/alexwww/ngp_sunray_firmware
HOME_PATH=/home/pi
INSTALL_PATH=$HOME_PATH/sunray_install
#DECRYPT_KEY=$INSTALL_PATH/config_files/decrypt_keys/zip-private.key
PKG_PATH=/tmp/sunray_ngp_firmware
SERVICE_ACTIVE=false
MD5=0
FILE=""
PKG=""


# if updater was started by a parent process (sunray), we have to close pipes, otherwise killing parent process kills us as well :-)
function close_pipes {
  echo "closing pipes..."
  logger "update.sh: closing pipes..."      
  for ((i=0;i<=254;i++));
  do
    eval "exec $i>&-"    
  done  
}


function find_update {
  echo "checking for updates..."  
  FILES=$(curl --silent "$URL/?C=M;O=D" | grep -o '<a href="[a-zA-Z]\+[^>"]*' | sed -ne 's/^<a href="\(.*\)/\1/p')
  #echo $FILES
  if [ $? -eq 0 ]; then    
    for file in $(echo $FILES | tr " " "\n")
    do
      echo "found version: $file"      
      FILE=$file
      URL="$URL/$FILE"
      #exit
      break
    done
  else
    echo "curl error - error finding updates!"
  fi  
}


function apply_update {
  echo "applying update..."
  logger "update.sh: applying update"        
  FILE=`basename $URL`
  PKG="$PKG_PATH/$FILE"    
  PKGDEC=$(echo $PKG | sed 's/.enc//')
  echo "EUID=$EUID"
  echo "PWD=$PWD"
  echo "url=$URL"
  echo "file=$FILE"
  echo "pkg=$PKG"
  echo "pkgdec=$PKGDEC"
  logger "update.sh: EUID=$EUID"
  logger "update.sh: PWD=$PWD"
  logger "update.sh: url=$URL"
  logger "update.sh: file=$FILE"
  logger "update.sh: pkg=$PKG"
  logger "update.sh: pkgdec=$PKGDEC"

  if $SERVICE_ACTIVE; then    
    logger "update.sh: stopping sunray service..."  
    echo "stopping sunray service..."
    logger "update.sh: stopping sunray service"
    systemctl stop sunray
  fi 
  logger "update.sh: killing any running sunray process..."      
  echo "killing any running sunray process..."
  killall sunray
  logger "update.sh: downloading update"          
  echo "downloading update..."      
  sudo i2cset -y 1 0x22 0x02 0x08  # choose LED 1 red
  runuser -l pi -c "rm -f $PKG_PATH/*"
  runuser -l pi -c "mkdir -p $PKG_PATH"
  runuser -l pi -c "curl --silent $URL --output $PKG" 
  if [ $? -eq 0 ]; then    
    sudo i2cset -y 1 0x22 0x02 0x20  # choose LED 2 red          
    if [ $PKG != $PKGDEC ]; then
      # file is encrypted (.enc)
      echo "decrypting..."
      logger "update.sh: decrypting"          
      #runuser -l pi -c "openssl rsautl -decrypt -inkey $DECRYPT_KEY -in $PKG -out $PKGDEC"
      runuser -l pi -c "openssl enc -d -aes-256-cbc -in $PKG -out $PKGDEC -pass pass:updateme"    
    fi
    if [ $? -eq 0 ]; then    
      echo "unzipping..."
      logger "update.sh: unzipping"
      runuser -l pi -c "unzip -q -o -d $HOME_PATH/ $PKGDEC"
      if [ $? -eq 0 ]; then
        #runuser -l pi -c "echo $APPLY > $HOME_PATH/sunray_version.txt" 
        NEW_MD5=$(md5sum $INSTALL_PATH/firmware/rm18.ino.bin | cut -f 1 -d " ")
        #echo "updater NGP firmware MD5:" $NEW_MD5
        #if [ "$NEW_MD5" != "$MD5" ]; then
          echo "--------------------------"
          echo "running flash tool..."
          logger "update.sh: running flash tool"
          sudo i2cset -y 1 0x22 0x02 0x02  # choose LED 3 red
          #echo "NOTE: updater contains a new NGP firmware! Run sudo ./flash.sh to flash new NGP firmware!"
          $INSTALL_PATH/flash.sh --flash sunray
        #fi
        echo "--------------------------"
        echo "update installed!"
        sudo i2cset -y 1 0x22 0x02 0x2a  # choose LED 1,2,3 red          
        logger "update.sh: update installed"      
      else
        logger "update.sh: unzip error"              
        echo "unzip error - update failed!"
      fi
    else
      logger "update.sh: decrypt error"
      echo "decrypt error - update failed!" 
    fi
  else
    logger "update.sh: curl error"          
    echo "curl error - update failed!"    
  fi
  
  if [ -f $PKG ]; then
    echo "removing downloaded package (.zip)..."
    rm $PKG
  fi
  
  if [ -f $PKGDEC ]; then
    echo "removing downloaded package (.zip.enc)..."
    rm $PKGDEC
  fi
  
  if $SERVICE_ACTIVE; then
    echo restarting sunray service...
    systemctl start sunray
    #systemctl restart sunray 
  fi
  logger "update.sh: done"
}


if [ ! "$EUID" -eq 0 ]; then
  echo "Please run as root (sudo ./update.sh)"
  exit
fi

# configure LEDs output
sudo i2cset -y 1 0x22 0x06 0x00  # configure output 

echo "=====update.sh====="
logger "=====update.sh======"
echo "home path: $HOME_PATH"
echo "install path: $INSTALL_PATH"
logger "update.sh: home path: $HOME_PATH"
logger "update.sh: install path: $INSTALL_PATH"

if [ -f $INSTALL_PATH/firmware/rm18.ino.bin ]; then
  MD5=$(md5sum $INSTALL_PATH/firmware/rm18.ino.bin | cut -f 1 -d " ")
else
  echo "warning: no ngp firmware binary found below install path!"
  logger "update.sh: warning: no ngp firmware binary found below install path!"
fi


#systemctl -q is-active sunray && echo YES || echo NO
if [ $(systemctl -q is-active sunray && echo 1) ]; then 
  SERVICE_ACTIVE=true
fi



echo "service running: $SERVICE_ACTIVE"
logger "update.sh: service running: $SERVICE_ACTIVE"

# parse (optional) command line options
if [ $# -ne 0 ]; then  
  if [ $1 = "--help" ]; then   
    echo "update --apply --url fileURL   (apply fileURL update)"        
    exit
  fi
  if [ $1 = "--apply" ]; then
    if [ $2 = "--url" ]; then
      PARAMS="update.sh $@"
      #echo "$PARAMS"
      logger "$PARAMS"  
      URL=$3
      close_pipes
      #sleep 3000.0           
      apply_update
      exit
    fi
  fi  
else
  find_update
  apply_update
fi 




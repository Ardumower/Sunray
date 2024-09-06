#!/bin/bash

# dbus monitor:
# - waits for specific messages (e.g. audio play request)

# run with:
#   dbus-send --system --dest=de.sunray.Bus --print-reply /de/sunray/Bus de.sunray.Bus.Play string:"/home/alex/Sunray/tts/de/testing_audio.mp3"


sudo killall dbus-monitor


if [[ `pidof dbus-monitor` != "" ]]; then
  echo "dbus_monitor.sh already running! Exiting..."
  exit
fi

# ls -l /proc/asound/ 
# systemctl --user status pulseaudio.service
# systemctl --user restart pulseaudio.service
# pulseaudio -k && sudo alsa force-reload
# mplayer ao -help


function run_as_user() {
  CMD=$1
  # echo "CMD:$CMD"
  if [ "$EUID" -eq 0 ]
  then 
    # root
    runuser $USER -c "export XDG_RUNTIME_DIR="/run/user/1000"; $CMD >/dev/null 2>&1" &
  else
    $CMD >/dev/null 2>&1 &
  fi
}


#USER=`whoami`
USER=`who | head -n1 | cut -d' ' -f1 | xargs`
#if [ -z "USER" ]; then
#  USER=`whoami`
#fi
#echo "USER: $USER"

run_as_user 'amixer -D pulse sset Master 100%'
#run_as_user "mplayer -ao alsa -volume 100 -af volume=5:1 /home/$USER/Sunray/tts/de/testing_audio.mp3"



# Überwache den dbus auf Nachrichten
sudo dbus-monitor --system "interface='de.sunray.Bus'" | while read -r line; do
  # Wenn eine bestimmte Methode aufgerufen wird (z.B. Play)
  if echo "$line" | grep -q "member=Play"; then
    # Extrahiere den Dateipfad (angenommen, der Pfad wird als Argument gesendet)
    read -r path_line
    filepath=$(echo "$path_line" | grep -oP '(?<=string ").*(?=")')
    #echo "filepath=$filepath"
    #echo "USER=$USER"
    # Prüfe, ob die Datei existiert und spiele sie ab
    filepath="${filepath/root/"home/$USER"}"      
    #echo "filepath=$filepath"

    if [ -f "$filepath" ]; then
      #echo "Playing $filepath..."
      #mplayer "$filepath"
      killall mplayer >/dev/null 2>&1
      #sleep 0.5
      # -volume 100 -af volume=5:1   //  volume 100% (-volume 100) and amplify by 5dB (-af volume=5:1)
      run_as_user "mplayer -ao alsa -volume 100 -af volume=5:1 $filepath"
      #echo "OK"
    else
      echo "File $filepath does not exist."
    fi
  fi
done



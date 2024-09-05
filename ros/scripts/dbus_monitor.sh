#!/bin/bash

# dbus-send --system --dest=de.sunray.Bus --print-reply \
#      /de/sunray/Bus de.sunray.Bus.Play string:"/path/to/yourfile.mp3"



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


USER=`who | cut -d: -f1 | xargs`
# echo "USER: $USER"

run_as_user 'amixer -D pulse sset Master 100%'
run_as_user "mplayer -ao alsa /home/$USER/Sunray/tts/de/testing_audio.mp3"



# Überwache den dbus auf Nachrichten
sudo dbus-monitor --system "interface='de.sunray.Bus'" | while read -r line; do
  # Wenn eine bestimmte Methode aufgerufen wird (z.B. Play)
  if echo "$line" | grep -q "member=Play"; then
    # Extrahiere den Dateipfad (angenommen, der Pfad wird als Argument gesendet)
    read -r path_line
    filepath=$(echo "$path_line" | grep -oP '(?<=string ").*(?=")')
    
    # Prüfe, ob die Datei existiert und spiele sie ab
    filepath="${filepath/root/"home/$USER"}"      
    if [ -f "$filepath" ]; then
      #echo "Playing $filepath..."
      #mplayer "$filepath"
      killall mplayer >/dev/null 2>&1
      #sleep 0.5
      run_as_user "mplayer -ao alsa $filepath"
      #echo "OK"
    else
      echo "File $filepath does not exist."
    fi
  fi
done



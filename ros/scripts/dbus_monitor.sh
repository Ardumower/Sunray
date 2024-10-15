#!/bin/bash

# dbus monitor:
# - waits for specific messages (e.g. audio play request)

# run with:
#   dbus-send --system --dest=de.sunray.Bus --print-reply /de/sunray/Bus de.sunray.Bus.Play string:"/home/alex/Sunray/tts/de/testing_audio.mp3"


# https://linuxconfig.org/how-to-install-pipewire-on-ubuntu-linux
# audio system installation:
#   sudo apt purge pulseaudio
#   sudo apt install --reinstall pipewire pipewire-audio-client-libraries
#   systemctl --user --now disable pulseaudio.service pulseaudio.socket
#   systemctl --user --now enable pipewire pipewire-pulse

# ----------------------------

if [  ]; then
  id
  killall mplayer >/dev/null 2>&1
  CMD="mplayer -nolirc -noconsolecontrols -really-quiet -volume 100 /home/pi/Sunray/tts/de/testing_audio.mp3"
  if [ "$EUID" -eq 0 ]; then
    runuser pi -c "export XDG_RUNTIME_DIR=\"/run/user/1000\"; $CMD" &
  else
    $CMD &
  fi
  exit
fi


echo "dbus_monitor started"

# sudo killall -s SIGKILL dbus-monitor
sudo killall -9 dbus-monitor


if [[ `pidof dbus-monitor` != "" ]]; then
  echo "dbus_monitor.sh already running! Exiting..."
  exit
fi

# ls -l /proc/asound/ 
# systemctl --user status pulseaudio.service
# systemctl --user restart pulseaudio.service
# pulseaudio -k && sudo alsa force-reload
# mplayer -ao help



function run_as_user() {
  CMD=$1
  #CMD+=" >/dev/null 2>&1"
  BLOCKING=$2
  #echo "EUID=$EUID USER:$USER CMD:$CMD BLOCKING:$BLOCKING"
  if [ "$EUID" -eq 0 ]
  then 
    # root
    # https://unix.stackexchange.com/questions/224370/how-to-stop-sudo-pam-messages-in-auth-log-for-a-specific-user
    if [ "$BLOCKING" = true ] ; then
      runuser $USER -c "export XDG_RUNTIME_DIR=\"/run/user/1000\"; $CMD" > /dev/null 2>&1
      #runuser $USER -c "$CMD" 
    else
      runuser $USER -c "export XDG_RUNTIME_DIR=\"/run/user/1000\"; $CMD" > /dev/null 2>&1 &    
      #runuser $USER -c "$CMD" &
    fi
  else
    if [ "$BLOCKING" = true ] ; then
      #eval " $CMD"
      $CMD > /dev/null 2>&1
    else
      #eval " $CMD" &
      $CMD > /dev/null 2>&1 &
    fi
  fi
  #echo "CMD done"
}


id
USER=`id -u -n 1000`
echo "USER: $USER"

#run_as_user "systemctl --user restart pulseaudio" true
#run_as_user "amixer cset numid=1 100%" true
#run_as_user 'amixer -D pulse sset Master 100%' true
#run_as_user "mplayer -nolirc -noconsolecontrols -really-quiet -volume 100 /home/$USER/Sunray/tts/de/testing_audio.mp3" true
#exit
#while :
#do
#  run_as_user "mplayer -nolirc -noconsolecontrols -really-quiet -volume 100 /home/$USER/Sunray/tts/de/testing_audio.mp3" true
#  sleep 2
#done



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
      #sleep 1.0
      # pkill mplayer >/dev/null 2>&1
      #killall -9 mplayer >/dev/null 2>&1
      #sleep 0.2
      # -volume 100 -af volume=5:1   //  volume 100% (-volume 100) and amplify by 5dB (-af volume=5:1)
      run_as_user "mplayer -nolirc -noconsolecontrols -really-quiet -volume 100 $filepath" false
      #echo "OK"
    #else
      #echo "File $filepath does not exist."
      #nop
    fi
  fi
done



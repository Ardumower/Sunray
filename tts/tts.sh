
# install with:
#   sudo apt install python3-pip sox libsox-fmt-mp3 xclip && pip3 install gTTS

# command line test:
#   gtts-cli "dies ist ein test" -l 'de' -o tts.mp3
#   play tts.mp3



# TTS string array:

if true; then 
    arr=(
        "unknown_event"                  "Systemfehler. Unbekanntes Ereignis."
        "system_starting"                "System wird hochgefahren. Bitte warten." 
        "system_started"                 "System wurde hochgefahren."
        "system_restarting"              "System wird neu gestartet."
        "system_shutting_down"           "System wird heruntergefahren."
        "system_starting_failed"         "Systemstart fehlgeschlagen."        
        "testing_audio"                  "Audio-Test."        
        "wifi_connected"                 "WLAN verbunden."
        "app_connected"                  "App verbunden."
        "imu_calibrating"                "Kalibriere IMU. Bitte warten."                 
        "error_gps_not_connected"        "Fehler! GPS nicht angeschlossen. Start nicht möglich."
        "error_imu_not_connected"        "Fehler! IMU nicht angeschlossen. Start nicht möglich."
        "error_imu_timeout"              "Fehler! IMU reagiert nicht. Ich versuche nochmal."
        "error_battery_undervoltage"     "Fehler! Batteriespannung zu niedrig. Bitte lade mich."        
        "error_traction_motor"           "Fehler Antriebsmotor! Ich versuche nochmal."
        "error_traction_motor_giveup"    "Fehler Antriebsmotor! Bitte erneut starten."        
        "error_traction_motor_overload"  "Fehler! Antriebs-Motor-Überlastung. Bitte erneut starten."
        "error_traction_motor"           "Fehler! Antriebs-Motor. Bitte erneut starten."
        "error_mow_motor"                "Fehler Mäh-Motor! Ich versuche nochmal."
        "error_mow_motor_overload"       "Fehler! Mäh-Motor-Überlastung. Bitter erneut starten."        
        "error_mow_motor_too_many"       "Zu viele Mäh-Motor-Fehler! Ich umfahre Hindernis."        
        "error_mow_motor_giveup"         "Fehler! Mäh-Motor. Bitte erneut starten."                
        "error_motor_overload"           "Fehler! Motor-Überlastung. Bitte erneut starten."
        "error_motor_error"              "Fehler! Motor-Fehler. Ich versuche nochmal."
        "error_motor_error_giveup"       "Fehler! Motor-Fehler. Bitte erneut starten."
        "error_odometry"                 "Fehler! Odometrie-Fehler. Bitte erneut starten."
        "error_no_dock_route"            "Fehler! Kein Nach-Hause-Pfad definiert."     
        "error_no_map_points"            "Fehler! Leere Karte. Bitte Karte hochladen."   
        "robot_tilted"                   "Ich wurde gekippt! Bitte erneut starten."
        "error_no_map_route_giveup"      "Kein Pfad zum Ziel gefunden. Bitte erneut starten."
        "error_no_map_route"             "Kein Pfad zum Ziel gefunden. Ich versuche nochmal."
        "no_gps_position"                "Keine GPS Position. Ich warte."
        "gps_jump"                       "GPS Position schwankt stark. Ich warte."
        "gps_bad"                        "GPS ist schlecht! Ich warte."
        "gps_restarted"                  "GPS neu gestartet. Bitte warten."
        "battery_low_dock"               "Meine Batterie ist leer! Ich fahre nach Hause."
        "mowing_completed"               "Mähen abgeschlossen! Ich fahre nach Hause."
        "bumper_obstacle"                "Bumper! Umfahre Hindernis."
        "lifted_obstacle"                "Angehoben! Umfahre Hindernis."
        "ground_obstacle"                "Hindernis am Boden! Umfahre Hindernis."
        "no_robot_motion_obstacle"       "Ich bewege mich nicht. Umfahre Hindernis."
        "no_gps_speed_obstacle"          "Keine GPS-Geschwindigkeit vorhanden. Umfahre Hindernis." 
        "motor_overload_reduce_speed"    "Motor-Überlastung. Ich reduziere die Geschwindigkeit."
        "angular_motion_timeout"         "Zeiüberschreitung beim Drehen. Umfahre Hindernis."
        "imu_no_rotation_obstacle"       "Keine IMU-Drehung feststellbar. Umfahre Hindernis."
        "imu_wheel_difference_obstacle"  "Unterschied zwischen IMU und Rad. Umfahre Hindernis."
        "rain_docking"                   "Es regnet. Ich fahre nach Hause."
        "temperature_out_of_range"       "Unpassende Aussen-Temperatur. Ich fahre nach Hause."
        "temperature_high_docking"       "Es ist zu heiss. Ich fahre nach Hause."
        "temperature_low_docking"        "Es ist zu kalt. Ich fahre nach Hause."        
        "user_stop"                      "STOP gedrückt! Ich warte."
        "user_start"                     "START gedrückt! Ich starte das Mähen."
        "user_dock"                      "DOCK gedrückt! Ich fahre nach Hause."
        "user_upload_map"                "Neue Mähkarte wurde hochgeladen."
        "user_timetable_upload"          "Zeitplan wurde hochgeladen."        
        "triggered_obstacle"             "Hindernis ausgelöst. Umfahre Hindernis."
        "idle_timeout"                   "Ich bin inaktiv. Bitte schalte mich aus."                      
        "charger_connected"              "Ladevorrichtung angeschlossen! Ich lade meine Batterie auf."        
        "dock_recovery"                  "Ladekontakt verloren. Ich versuche nochmal."
        "dock_recovery_giveup"           "Ladekontakt verloren. Bitte lade mich auf."
        "lidar_relocalization"           "LiDAR-Position unbekannt. Ich lokalisiere mich neu."
        "lidar_mapping_completed"        "LiDAR-Kartierung abgeschlossen."
    )
else
    arr=(
        "System wird hochgefahren. Bitte warten." 
    )
fi


IFS=""  # do not split array strings by spaces

FOLDER="$PWD/de" 

counter=1
for str in ${arr[@]}; do  
  if (( $counter % 2 == 0 ))     
  then
    if [ ! -f "$FOLDER/$key.mp3" ]; then
      printf "%-35s $str\n" $key      
      gtts-cli $str -l 'de' -o $FOLDER/_$key.mp3
      sox "$FOLDER/_$key.mp3" "$FOLDER/$key.mp3" pitch -400 speed 1.2  
      rm "$FOLDER/_$key.mp3"
      #play --volume 3.0 "$FOLDER/$key.mp3"
      #play --volume 1.0 "$FOLDER/$key.mp3"    
      #gtts-cli $str -l 'de' | play -t mp3 -   
    fi
  fi
  key="$str"
  counter=$(expr $counter  + 1)
done



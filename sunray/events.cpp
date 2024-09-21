// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

// robot events (log, audio etc.)


#include "events.h"
#include "config.h"

#ifdef __linux__
  #include <Process.h>
#endif


EventLogger Logger;


EventLogger::EventLogger()  {    
}

void EventLogger::event(EventCode evt) {
    event(evt, "");
}

// Event loggen und ausgeben
void EventLogger::event(EventCode evt, const String &additionalData) {
    #ifdef TTS_PATH 
        String evtText = "";
        switch (evt){
            case EVT_SYSTEM_STARTING:
                evtText = TXT_SYSTEM_STARTING;
                break;
            case EVT_SYSTEM_STARTED:
                evtText = TXT_SYSTEM_STARTED;
                break;
            case EVT_SYSTEM_RESTARTING:
                evtText = TXT_SYSTEM_RESTARTING;
                break;
            case EVT_SYSTEM_SHUTTING_DOWN:
                evtText = TXT_SYSTEM_SHUTTING_DOWN;
                break;
            case EVT_SYSTEM_STARTING_FAILED:
                evtText = TXT_SYSTEM_STARTING_FAILED;
                break;
            case EVT_WIFI_CONNECTED:
                evtText = TXT_WIFI_CONNECTED;
                break;
            case EVT_APP_CONNECTED:
                evtText = TXT_APP_CONNECTED;
                break;
            case EVT_IMU_CALIBRATING:
                evtText = TXT_IMU_CALIBRATING;
                break;
            case EVT_ERROR_GPS_NOT_CONNECTED:
                evtText = TXT_ERROR_GPS_NOT_CONNECTED;
                break; 
            case EVT_ERROR_IMU_NOT_CONNECTED:        
                evtText = TXT_ERROR_IMU_NOT_CONNECTED;
                break;
            case EVT_ERROR_IMU_TIMEOUT:
                evtText = TXT_ERROR_IMU_TIMEOUT;
                break;
            case EVT_ERROR_BATTERY_UNDERVOLTAGE:
                evtText = TXT_ERROR_BATTERY_UNDERVOLTAGE;
                break;
            case EVT_ERROR_TRACTION_MOTOR:
                evtText = TXT_ERROR_TRACTION_MOTOR;
                break;
            case EVT_ERROR_TRACTION_MOTOR_GIVEUP:
                evtText = TXT_ERROR_TRACTION_MOTOR_GIVEUP;
                break;
            case EVT_ERROR_TRACTION_MOTOR_OVERLOAD:
                evtText = TXT_ERROR_TRACTION_MOTOR_OVERLOAD;
                break;
            case EVT_ERROR_MOW_MOTOR:
                evtText = TXT_ERROR_MOW_MOTOR;
                break;
            case EVT_ERROR_MOW_MOTOR_OVERLOAD:
                evtText = TXT_ERROR_MOW_MOTOR_OVERLOAD;
                break;
            case EVT_ERROR_MOW_MOTOR_TOO_MANY:
                evtText = TXT_ERROR_MOW_MOTOR_TOO_MANY;
                break;
            case EVT_ERROR_MOW_MOTOR_GIVEUP:
                evtText = TXT_ERROR_MOW_MOTOR_GIVEUP;
                break;
            case EVT_ERROR_MOTOR_OVERLOAD:
                evtText = TXT_ERROR_MOTOR_OVERLOAD;
                break;
            case EVT_ERROR_MOTOR_ERROR:
                evtText = TXT_ERROR_MOTOR_ERROR;
                break;
            case EVT_ERROR_MOTOR_ERROR_GIVEUP:
                evtText = TXT_ERROR_MOTOR_ERROR_GIVEUP;
                break;
            case EVT_ERROR_ODOMETRY:
                evtText = TXT_ERROR_ODOMETRY;
                break;
            case EVT_ERROR_NO_DOCK_ROUTE:
                evtText = TXT_ERROR_NO_DOCK_ROUTE;
                break;
            case EVT_ROBOT_TILTED:
                evtText = TXT_ROBOT_TILTED;
                break;
            case EVT_ERROR_NO_MAP_ROUTE:
                evtText = TXT_ERROR_NO_MAP_ROUTE;
                break;
            case EVT_ERROR_NO_MAP_ROUTE_GIVEUP:
                evtText = TXT_ERROR_NO_MAP_ROUTE_GIVEUP;
                break;
            case EVT_ERROR_NO_MAP_POINTS:
                evtText = TXT_ERROR_NO_MAP_POINTS;
                break;
            case EVT_NO_GPS_POSITION:
                evtText = TXT_NO_GPS_POSITION;
                break;
            case EVT_GPS_JUMP:
                evtText = TXT_GPS_JUMP;
                break;
            case EVT_GPS_BAD:
                evtText = TXT_GPS_BAD;
                break;
            case EVT_GPS_RESTARTED:
                evtText = TXT_GPS_RESTARTED;
                break;
            case EVT_BATTERY_LOW_DOCK:
                evtText = TXT_BATTERY_LOW_DOCK;
                break;
            case EVT_MOWING_COMPLETED:
                evtText = TXT_MOWING_COMPLETED;
                break;
            case EVT_BUMPER_OBSTACLE:
                evtText = TXT_BUMPER_OBSTACLE;
                break;
            case EVT_LIDAR_BUMPER_OBSTACLE:
                evtText = TXT_LIDAR_BUMPER_OBSTACLE;
                break;
            case EVT_LIFTED_OBSTACLE:
                evtText = TXT_LIFTED_OBSTACLE;
                break;
            case EVT_GROUND_OBSTACLE:
                evtText = TXT_GROUND_OBSTACLE;
                break;
            case EVT_NO_ROBOT_MOTION_OBSTACLE:
                evtText = TXT_NO_ROBOT_MOTION_OBSTACLE;
                break;
            case EVT_NO_GPS_SPEED_OBSTACLE:
                evtText = TXT_NO_GPS_SPEED_OBSTACLE;
                break;
            case EVT_ANGULAR_MOTION_TIMEOUT_OBSTACLE:
                evtText = TXT_ANGULAR_MOTION_TIMEOUT_OBSTACLE;
                break;
            case EVT_IMU_NO_ROTATION_OBSTACLE:
                evtText = TXT_IMU_NO_ROTATION_OBSTACLE;
                break;
            case EVT_IMU_WHEEL_DIFFERENCE_OBSTACLE:
                evtText = TXT_IMU_WHEEL_DIFFERENCE_OBSTACLE;
                break;
            case EVT_TRIGGERED_OBSTACLE:
                evtText = TXT_TRIGGERED_OBSTACLE;
                break;
            case EVT_MOTOR_OVERLOAD_REDUCE_SPEED:
                evtText = TXT_MOTOR_OVERLOAD_REDUCE_SPEED;
                break;
            case EVT_RAIN_DOCKING:
                evtText = TXT_RAIN_DOCKING;
                break;
            case EVT_TEMPERATURE_HIGH_DOCKING:
                evtText = TXT_TEMPERATURE_HIGH_DOCKING;
                break;
            case EVT_TEMPERATURE_LOW_DOCKING:
                evtText = TXT_TEMPERATURE_LOW_DOCKING;
                break;
            case EVT_USER_STOP:
                evtText = TXT_USER_STOP;
                break;        
            case EVT_USER_START:
                evtText = TXT_USER_START;
                break;
            case EVT_USER_DOCK:
                evtText = TXT_USER_DOCK;
                break; 
            case EVT_USER_UPLOAD_MAP:
                evtText = TXT_USER_UPLOAD_MAP;
                break; 
            case EVT_IDLE_TIMEOUT:
                evtText = TXT_IDLE_TIMEOUT;
                break; 
            case EVT_CHARGER_CONNECTED:
                evtText = TXT_CHARGER_CONNECTED;
                break; 
            case EVT_DOCK_RECOVERY:
                evtText = TXT_DOCK_RECOVERY;
                break; 
            case EVT_DOCK_RECOVERY_GIVEUP:
                evtText = TXT_DOCK_RECOVERY_GIVEUP;
                break; 
            case EVT_LIDAR_RELOCALIZATION:
                evtText = TXT_LIDAR_RELOCALIZATION;
                break; 
            case EVT_LIDAR_MAPPING_COMPLETED:
                evtText = TXT_LIDAR_MAPPING_COMPLETED;
                break; 
            case EVT_USER_UPLOAD_TIME_TABLE:
                evtText = TXT_USER_UPLOAD_TIMETABLE;
                break;
            case EVT_AUDIO_TEST:
                evtText = TXT_AUDIO_TEST;
                break;
            case EVT_AUDIO_SHEEP:
                evtText = TXT_AUDIO_SHEEP;
                break;
            case EVT_AUDIO_BELL:
                evtText = TXT_AUDIO_BELL;
                break;
            case EVT_AUDIO_BEEP:
                evtText = TXT_AUDIO_BEEP;
                break;
            case EVT_AUDIO_TADA:
                evtText = TXT_AUDIO_TADA;
                break;

            default:
                evtText = TXT_UNKNOWN;
                break;
        }
        String timeStamp = getCurrentTimeStamp();
            
        String mp3File = TTS_PATH + evtText + ".mp3"; 
        playMP3(mp3File);
        if (additionalData != "") {
            evtText +=  " " + additionalData;
        }
    #endif    
}


// Aktuellen Zeitstempel bekommen
String EventLogger::getCurrentTimeStamp() {
    //std::time_t now = std::time(0);
    //char buf[80];
    //std::strftime(buf, sizeof(buf), "%Y-%m-%d %X", std::localtime(&now));
    //return String(buf);
    return "";
}

// MP3-Datei abspielen
void EventLogger::playMP3(String &filename) {
    #ifdef __linux__        
        //String command = "killall mplayer; mplayer ";
        // volume 100% (-volume 100) and amplify by 5dB (-af volume=5:1)
        String command = "../ros/scripts/dbus_send.sh -m Play -p ";
        command += filename;
        command += " &";
        //Process p;
        //CONSOLE.print("RUN: ");
        //CONSOLE.println(command.c_str());
        //p.runShellCommand(command.c_str());    
        system(command.c_str());
    #endif

}




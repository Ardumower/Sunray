// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

// robot events (log, audio etc.)

#ifndef EVENTS_H
#define EVENTS_H

#include <Arduino.h>

// keep text in flash memory (do not consume RAM memory for this)
#define TXT_UNKNOWN                         F("unknown_event")
#define TXT_SYSTEM_STARTING                 F("system_starting")
#define TXT_SYSTEM_STARTED                  F("system_started")
#define TXT_SYSTEM_RESTARTING               F("system_restarting")
#define TXT_SYSTEM_SHUTTING_DOWN            F("system_shutting_down")
#define TXT_SYSTEM_STARTING_FAILED          F("system_starting_failed")
#define TXT_WIFI_CONNECTED                  F("wifi_connected")
#define TXT_APP_CONNECTED                   F("app_connected")
#define TXT_IMU_CALIBRATING                 F("imu_calibrating")
#define TXT_ERROR_GPS_NOT_CONNECTED         F("error_gps_not_connected")
#define TXT_ERROR_IMU_NOT_CONNECTED         F("error_imu_not_connected")
#define TXT_ERROR_IMU_TIMEOUT               F("error_imu_timeout")
#define TXT_ERROR_BATTERY_UNDERVOLTAGE      F("error_battery_undervoltage")
#define TXT_ERROR_TRACTION_MOTOR            F("error_traction_motor")
#define TXT_ERROR_TRACTION_MOTOR_GIVEUP     F("error_traction_motor_giveup")
#define TXT_ERROR_TRACTION_MOTOR_OVERLOAD   F("error_traction_motor_overload")
#define TXT_ERROR_MOW_MOTOR                 F("error_mow_motor")
#define TXT_ERROR_MOW_MOTOR_OVERLOAD        F("error_mow_motor_overload")
#define TXT_ERROR_MOW_MOTOR_TOO_MANY        F("error_mow_motor_too_many")
#define TXT_ERROR_MOW_MOTOR_GIVEUP          F("error_mow_motor_giveup")
#define TXT_ERROR_MOTOR_OVERLOAD            F("error_motor_overload")
#define TXT_ERROR_MOTOR_ERROR               F("error_motor_error")
#define TXT_ERROR_MOTOR_ERROR_GIVEUP        F("error_motor_error_giveup")
#define TXT_ERROR_ODOMETRY                  F("error_odometry")
#define TXT_ERROR_NO_DOCK_ROUTE             F("error_no_dock_route")
#define TXT_ERROR_NO_MAP_POINTS             F("error_no_map_points")
#define TXT_ROBOT_TILTED                    F("robot_tilted")
#define TXT_ERROR_NO_MAP_ROUTE              F("error_no_map_route")
#define TXT_ERROR_NO_MAP_ROUTE_GIVEUP       F("error_no_map_route_giveup")
#define TXT_NO_GPS_POSITION                 F("no_gps_position")
#define TXT_GPS_JUMP                        F("gps_jump")
#define TXT_GPS_BAD                         F("gps_bad")
#define TXT_GPS_RESTARTED                   F("gps_restarted")
#define TXT_BATTERY_LOW_DOCK                F("battery_low_dock")
#define TXT_MOWING_COMPLETED                F("mowing_completed")
#define TXT_BUMPER_OBSTACLE                 F("bumper_obstacle")
#define TXT_LIDAR_BUMPER_OBSTACLE           F("lidar_bumper_obstacle")
#define TXT_LIFTED_OBSTACLE                 F("lifted_obstacle")
#define TXT_GROUND_OBSTACLE                 F("ground_obstacle")
#define TXT_TRIGGERED_OBSTACLE              F("triggered_obstacle")
#define TXT_NO_ROBOT_MOTION_OBSTACLE        F("no_robot_motion_obstacle")
#define TXT_NO_GPS_SPEED_OBSTACLE           F("no_gps_speed_obstacle")
#define TXT_ANGULAR_MOTION_TIMEOUT_OBSTACLE F("angular_motion_timeout")
#define TXT_IMU_NO_ROTATION_OBSTACLE        F("imu_no_rotation_obstacle")
#define TXT_IMU_WHEEL_DIFFERENCE_OBSTACLE   F("imu_wheel_difference_obstacle")
#define TXT_MOTOR_OVERLOAD_REDUCE_SPEED     F("motor_overload_reduce_speed")
#define TXT_RAIN_DOCKING                    F("rain_docking")
#define TXT_TEMPERATURE_HIGH_DOCKING        F("temperature_high_docking")
#define TXT_TEMPERATURE_LOW_DOCKING         F("temperature_low_docking")
#define TXT_TEMPERATURE_OUT_OF_RANGE_DOCK   F("temperature_out_of_range")
#define TXT_USER_STOP                       F("user_stop")
#define TXT_USER_START                      F("user_start")
#define TXT_USER_DOCK                       F("user_dock")
#define TXT_USER_UPLOAD_MAP                 F("user_upload_map")
#define TXT_USER_UPLOAD_TIMETABLE           F("user_timetable_map")
#define TXT_IDLE_TIMEOUT                    F("idle_timeout")
#define TXT_CHARGER_CONNECTED               F("charger_connected")
#define TXT_DOCK_RECOVERY                   F("dock_recovery")
#define TXT_DOCK_RECOVERY_GIVEUP            F("dock_recovery_giveup")
#define TXT_LIDAR_RELOCALIZATION            F("lidar_relocalization")
#define TXT_LIDAR_MAPPING_COMPLETED         F("lidar_mapping_completed")
#define TXT_AUDIO_TEST                      F("testing_audio")
#define TXT_AUDIO_SHEEP                     F("sheep")
#define TXT_AUDIO_BELL                      F("bell")
#define TXT_AUDIO_BEEP                      F("beep")
#define TXT_AUDIO_TADA                      F("tada")


// event codes 
enum EventCode {
    EVT_SYSTEM_STARTING,
    EVT_SYSTEM_STARTED,
    EVT_SYSTEM_RESTARTING,
    EVT_SYSTEM_SHUTTING_DOWN,
    EVT_SYSTEM_STARTING_FAILED,
    EVT_WIFI_CONNECTED,
    EVT_APP_CONNECTED,
    EVT_IMU_CALIBRATING,
    EVT_ERROR_GPS_NOT_CONNECTED,
    EVT_ERROR_IMU_NOT_CONNECTED,
    EVT_ERROR_IMU_TIMEOUT,
    EVT_ERROR_BATTERY_UNDERVOLTAGE,
    EVT_ERROR_TRACTION_MOTOR,
    EVT_ERROR_TRACTION_MOTOR_GIVEUP,
    EVT_ERROR_TRACTION_MOTOR_OVERLOAD,
    EVT_ERROR_MOW_MOTOR,
    EVT_ERROR_MOW_MOTOR_OVERLOAD,
    EVT_ERROR_MOW_MOTOR_TOO_MANY,
    EVT_ERROR_MOW_MOTOR_GIVEUP,
    EVT_ERROR_MOTOR_OVERLOAD,
    EVT_ERROR_MOTOR_ERROR,
    EVT_ERROR_MOTOR_ERROR_GIVEUP,
    EVT_ERROR_ODOMETRY,
    EVT_ERROR_NO_DOCK_ROUTE,
    EVT_ERROR_NO_MAP_POINTS,
    EVT_ROBOT_TILTED,
    EVT_ERROR_NO_MAP_ROUTE_GIVEUP,    
    EVT_ERROR_NO_MAP_ROUTE,
    EVT_NO_GPS_POSITION,
    EVT_GPS_JUMP,
    EVT_GPS_BAD,
    EVT_GPS_RESTARTED,
    EVT_BATTERY_LOW_DOCK,
    EVT_MOWING_COMPLETED,
    EVT_BUMPER_OBSTACLE,
    EVT_LIDAR_BUMPER_OBSTACLE,
    EVT_LIFTED_OBSTACLE,
    EVT_NO_ROBOT_MOTION_OBSTACLE,
    EVT_NO_GPS_SPEED_OBSTACLE,
    EVT_ANGULAR_MOTION_TIMEOUT_OBSTACLE,
    EVT_IMU_NO_ROTATION_OBSTACLE, 
    EVT_IMU_WHEEL_DIFFERENCE_OBSTACLE,
    EVT_MOTOR_OVERLOAD_REDUCE_SPEED,
    EVT_GROUND_OBSTACLE,
    EVT_TRIGGERED_OBSTACLE,
    EVT_RAIN_DOCKING,
    EVT_TEMPERATURE_HIGH_DOCKING,
    EVT_TEMPERATURE_LOW_DOCKING,
    EVT_TEMPERATURE_OUT_OF_RANGE_DOCK,
    EVT_USER_STOP,
    EVT_USER_START,
    EVT_USER_DOCK,
    EVT_USER_UPLOAD_MAP,
    EVT_USER_UPLOAD_TIME_TABLE,
    EVT_IDLE_TIMEOUT,
    EVT_CHARGER_CONNECTED,
    EVT_DOCK_RECOVERY,
    EVT_DOCK_RECOVERY_GIVEUP,
    EVT_LIDAR_RELOCALIZATION,
    EVT_LIDAR_MAPPING_COMPLETED,
    EVT_AUDIO_TEST,
    EVT_AUDIO_SHEEP,
    EVT_AUDIO_BELL,
    EVT_AUDIO_BEEP,
    EVT_AUDIO_TADA,
};


class EventLogger {
    public:
        EventLogger();
        void event(EventCode evt);    
        void event(EventCode evt, const String &additionalData);    
    private:
        String getCurrentTimeStamp();
        void playMP3(String &filename);    
};


extern EventLogger Logger;

#endif

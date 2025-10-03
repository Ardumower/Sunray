// Common type definitions extracted from robot.h to avoid circular includes
#ifndef SUNRAY_TYPES_H
#define SUNRAY_TYPES_H

// operation types
enum OperationType {
      OP_IDLE,      // idle
      OP_MOW,       // mowing
      OP_CHARGE,    // charging
      OP_ERROR,     // serious error
      OP_DOCK,      // go to docking
};

// sensor errors
enum Sensor {
      SENS_NONE,              // no error
      SENS_BAT_UNDERVOLTAGE,  // battery undervoltage
      SENS_OBSTACLE,          // obstacle triggered
      SENS_GPS_FIX_TIMEOUT,   // gps fix timeout
      SENS_IMU_TIMEOUT,       // imu timeout
      SENS_IMU_TILT,          // imut tilt
      SENS_KIDNAPPED,         // robot has been kidnapped (is no longer on planned track)
      SENS_OVERLOAD,          // motor overload
      SENS_MOTOR_ERROR,       // motor error
      SENS_GPS_INVALID,       // gps is invalid or not working
      SENS_ODOMETRY_ERROR,    // motor odometry error
      SENS_MAP_NO_ROUTE,      // robot cannot find a route to next planned point
      SENS_MEM_OVERFLOW,      // cpu memory overflow
      SENS_BUMPER,            // bumper triggered
      SENS_SONAR,             // ultrasonic triggered
      SENS_LIFT,              // lift triggered
      SENS_RAIN,              // rain sensor triggered
      SENS_STOP_BUTTON,       // emergency/stop button triggered
      SENS_TEMP_OUT_OF_RANGE, // temperature out-of-range triggered
};

#endif


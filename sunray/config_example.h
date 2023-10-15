// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

/* 
   WARNING: all software, hardware and motor components are designed and optimized as a whole, if you 
   try to replace or exclude some component not as designed, you risk to damage your hardware with 
   the software.

   see Wiki for installation details:
   http://wiki.ardumower.de/index.php?title=Ardumower_Sunray

   requirements:
   + Ardumower chassis and Ardumower kit mowing and gear motors   
   + Ardumower PCB 1.3/1.4 
   +   Adafruit Grand Central M4 (highly recommended) or Arduino Due 
   +   Ardumower BLE UART module (HM-10/CC2540/CC2541)
   +   optional: Ardumower IMU (MPU6050/MPU9150/MPU9250/MPU9255) - choose your IMU below
   +   optional: Ardumower WIFI (ESP8266 ESP-01 with stock firmware)   
   +   optional: HTU21D temperature/humidity sensor
   +   optional: sonar, bumperduino, freewheel sensor
   + Ardumower RTK (ublox F9P)


1. Rename file 'config_example.h' into 'config.h'

2. Configure the options below and finally compile and upload this project.


Adafruit Grand Central M4 NOTE: You have to add SDA, SCL pull-up resistors to the board 
and deactivate Due clone reset cicuit (JP13):
https://wiki.ardumower.de/index.php?title=Ardumower_Sunray#Adafruit_Grand_Central_M4
NOTE: If you get compilation errors with Adafruit Grand Central M4, you may have to downgrade 'Adafruit SAMD Boards' to version to 1.7.5.

Arduino Due UPLOAD NOTE:  

If using Arduino Due 'native' USB port for uploading, choose board 'Arduino Due native' in the 
Arduino IDE and COM port 'Arduino Due native port'. 

If using Arduino Due 'programming' USB port for uploading, choose board 'Arduino Due programming' in the 
Arduino IDE and COM port 'Arduino Due programming port'. 

Also, you may choose the serial port below for serial monitor output (CONSOLE).
   
*/



//#define DRV_SERIAL_ROBOT  1
#define DRV_ARDUMOWER     1   // keep this for Ardumower
//#define DRV_SIM_ROBOT     1   // simulation


// ------- Bluetooth4.0/BLE module -----------------------------------
// see Wiki on how to install the BLE module and configure the jumpers:
// https://wiki.ardumower.de/index.php?title=Ardumower_Sunray#Bluetooth_BLE_UART_module
#define ENABLE_PASS   1        // comment out to disable password authentication
#define PASS          123456   // choose password for WiFi/BLE communication (NOTE: has to match the connection password in the App!)

// -------- IMU sensor  ----------------------------------------------
// choose one MPU IMU (make sure to connect AD0 on the MPU board to 3.3v)
// verify in CONSOLE that your IMU was found (you will hear 8 buzzer beeps for automatic calibration at start)
// see Wiki for wiring, how to position the module, and to configure the I2C pullup jumpers:   
// https://wiki.ardumower.de/index.php?title=Ardumower_Sunray#IMU.2C_sensor_fusion

//#define MPU6050
//#define MPU9150
#define MPU9250   // also choose this for MPU9255
//#define BNO055
#define MPU_ADDR 0x69  // I2C address (0x68 if AD0=LOW, 0x69 if AD0=HIGH)

// should the mower turn off if IMU is tilt over? (yes: uncomment line, no: comment line)
#define ENABLE_TILT_DETECTION  1

// --------- lift sensor (e.g. Alfred mower) ---------------------------------------------
// should the lift sensor be enabled? (yes: uncomment line, no: comment line)
//#define ENABLE_LIFT_DETECTION  1
// should the lift sensor be used for obstacle avoidance (if not, mower will simply go into error if lifted)
#define LIFT_OBSTACLE_AVOIDANCE 1  


// ------- SD card map load/resume and logging ---------------------------------
// all serial console output can be logged to a (FAT32 formatted) SD card
// NOTE: for full log file inspections, we will need your sunray.ino.elf binary 
// (you can find out the location of the compiled .elf file while compiling with verbose compilation switched on 
//  via 'File->Preferences->Full output during compile') - detailed steps here:  
// https://wiki.ardumower.de/index.php?title=Ardumower_Sunray#SD_card_module
// https://wiki.ardumower.de/index.php?title=Ardumower_Sunray#SD_card_logging
//#define ENABLE_SD      1                 // enable SD card services (resuming, logging)? (uncomment to activate)
//#define ENABLE_SD_LOG  1                 // enable SD card logging? uncomment to activate (not recommended - WARNING: may slow down system!)
//#define ENABLE_SD_RESUME  1              // enable SD card map load/resume on reset? (uncomment to activate)


// ------ odometry -----------------------------------
// values below are for Ardumower chassis and Ardumower motors
// see Wiki on how to configure the odometry divider:
// https://wiki.ardumower.de/index.php?title=Ardumower_Sunray#PCB1.3_odometry_divider
// NOTE: It is important to verify your odometry is working accurate. 
// Follow the steps described in the Wiki to verify your odometry returns approx. 1 meter distance for 
// driving the same distance on the ground (without connected GPS): 
// https://wiki.ardumower.de/index.php?title=Ardumower_Sunray#Odometry_test
// https://forum.ardumower.de/threads/andere-r%C3%A4der-wie-config-h-%C3%A4ndern.23865/post-41732

// NOTE: if using non-default Ardumower chassis and your freewheel is at frontside (gear motors at backside), have may have to swap motor cables, 
// more info here: https://wiki.ardumower.de/index.php?title=Ardumower_Chassis_%27mountain_mod%27)
#define FREEWHEEL_IS_AT_BACKSIDE   true   // default Ardumower: true   (change to false, if your freewheel is at frontside) - this is used for obstacle avoidance
#define WHEEL_BASE_CM         36         // wheel-to-wheel distance (cm)        
#define WHEEL_DIAMETER        250        // wheel diameter (mm)                 
#define MOWER_SIZE            60         // mower / chassis size / length in cm

//#define ENABLE_ODOMETRY_ERROR_DETECTION  true    // use this to detect odometry erros
#define ENABLE_ODOMETRY_ERROR_DETECTION  false

// choose ticks per wheel revolution :
// ...for the 36mm diameter motor (blue cap)  https://www.marotronics.de/2-x-36er-DC-Planeten-Getriebemotor-24-Volt-mit-HallIC-30-33-RPM-8mm-Welle
//#define TICKS_PER_REVOLUTION  1310 / 2    // odometry ticks per wheel revolution 

// ...for the 36mm diameter motor (black cap)  https://www.marotronics.de/MA36-DC-Planeten-Getriebemotor-24-Volt-mit-HallIC-30-33-RPM-8mm-Welle-ab-2-Stueck-Staffelpreis
// #define TICKS_PER_REVOLUTION 975 / 2

// ...for the newer 42mm diameter motor (green connector) https://www.marotronics.de/MA42-DC-Planeten-Getriebemotor-24-Volt-mit-HallIC-30-33-RPM-8mm-Welle-ab-2-Stueck-Staffelpreis
// #define TICKS_PER_REVOLUTION  696 / 2    // odometry ticks per wheel revolution 

// ...for the older 42mm diameter motor (white connector)  https://wiki.ardumower.de/images/d/d6/Ardumower_chassis_inside_ready.jpg
#define TICKS_PER_REVOLUTION  1050 / 2    // odometry ticks per wheel revolution 

// ...for the brushless motor april 2021   https://wiki.ardumower.de/index.php?title=Datei:BLUnit.JPG
//#define TICKS_PER_REVOLUTION  1300 / 2    // 1194/2  odometry ticks per wheel revolution

// #define TICKS_PER_REVOLUTION  304     // odometry ticks per wheel revolution (RM18)


// ----- gear motors --------------------------------------------------
// for brushless motors, study the sections (drivers, adapter, protection etc.) in the Wiki (https://wiki.ardumower.de/index.php?title=DIY_Brushless_Driver_Board)
//#define MOTOR_DRIVER_BRUSHLESS   1     // uncomment this for new brushless motor drivers
//#define MOTOR_DRIVER_BRUSHLESS_MOW_DRV8308  1 // uncomment for brushless DRV8308 driver and mowing motor 
//#define MOTOR_DRIVER_BRUSHLESS_MOW_A4931  1    // uncomment for brushless A3931 driver and mowing motor
//#define MOTOR_DRIVER_BRUSHLESS_MOW_BLDC8015A 1  // uncomment for brushless BLDC8015A driver and mowing motor
//#define MOTOR_DRIVER_BRUSHLESS_MOW_JYQD 1  // uncomment for brushless JYQD driver and mowing motor (https://forum.ardumower.de/threads/jyqd-treiber-und-sunray.24811/)
//#define MOTOR_DRIVER_BRUSHLESS_MOW_OWL 1  // uncomment for brushless owlDrive mowing motor 
//#define MOTOR_DRIVER_BRUSHLESS_GEARS_DRV8308  1   // uncomment for brushless DRV8308 driver and gear/traction motors 
//#define MOTOR_DRIVER_BRUSHLESS_GEARS_A4931  1    // uncomment for brushless A4931 driver and gear/traction motors
//#define MOTOR_DRIVER_BRUSHLESS_GEARS_BLDC8015A 1   // uncomment for brushless BLDC8015A driver and gear/traction motors
//#define MOTOR_DRIVER_BRUSHLESS_GEARS_JYQD 1   // uncomment for brushless JYQD driver and gears/traction motor
//#define MOTOR_DRIVER_BRUSHLESS_GEARS_OWL 1   // uncomment for brushless owlDrive gears/traction motor


#define MOTOR_FAULT_CURRENT 6.0    // gear motors fault current (amps)
#define MOTOR_TOO_LOW_CURRENT 0.005   // gear motor too low current (amps)
#define MOTOR_OVERLOAD_CURRENT 0.8    // gear motors overload current (amps)

//#define USE_LINEAR_SPEED_RAMP  true      // use a speed ramp for the linear speed
#define USE_LINEAR_SPEED_RAMP  false      // do not use a speed ramp 

// motor speed control (PID coefficients) - these values are tuned for Ardumower motors
// general information about PID controllers: https://wiki.ardumower.de/index.php?title=PID_control
#define MOTOR_PID_KP     2.0    // do not change 2.0 (for non-Ardumower motors or if the motor speed control is too fast you may try: KP=1.0, KI=0, KD=0)
#define MOTOR_PID_KI     0.03   // do not change 0.03
#define MOTOR_PID_KD     0.03   // do not change 0.03

//#define MOTOR_LEFT_SWAP_DIRECTION 1  // uncomment to swap left motor direction
//#define MOTOR_RIGHT_SWAP_DIRECTION 1  // uncomment to swap right motor direction


// ----- mowing motor -------------------------------------------------
// NOTE: motor drivers will indicate 'fault' signal if motor current (e.g. due to a stall on a molehole) or temperature is too high for a 
// certain time (normally a few seconds) and the mower will try again and set a virtual obstacle after too many tries
// On the other hand, the overload detection will detect situations the fault signal cannot detect: slightly higher current for a longer time 

//#define MAX_MOW_PWM 200  // use this to permanently reduce mowing motor power (255=max)

#define MOW_FAULT_CURRENT 8.0       // mowing motor fault current (amps)
#define MOW_TOO_LOW_CURRENT 0.005   // mowing motor too low current (amps)
#define MOW_OVERLOAD_CURRENT 2.0    // mowing motor overload current (amps)

// should the direction of mowing motor toggle each start? (yes: true, no: false)
#define MOW_TOGGLE_DIR       true
//#define MOW_TOGGLE_DIR       false

// should the error on motor overload detection be enabled?
#define ENABLE_OVERLOAD_DETECTION  true    // robot will stop on overload
//#define ENABLE_OVERLOAD_DETECTION  false    // robot will slow down on overload

// should the motor fault (error) detection be enabled? 
#define ENABLE_FAULT_DETECTION  true
//#define ENABLE_FAULT_DETECTION  false       // use this if you keep getting 'motor error'

#define ENABLE_RPM_FAULT_DETECTION  true     // use mow rpm signal to detect a motor fault (requires mowing motor with rpm output!)
//#define ENABLE_RPM_FAULT_DETECTION  false     // do not use mow rpm signal to detect a motor fault

// should the robot trigger obstacle avoidance on motor errors if motor recovery failed?
#define ENABLE_FAULT_OBSTACLE_AVOIDANCE true  

// shall the mow motor be activated for normal operation? Deactivate this option for GPS tests and path tracking running tests
#define ENABLE_MOW_MOTOR true // Default is true, set false for testing purpose to switch off mow motor permanently

// ------ WIFI module (ESP8266 ESP-01 with ESP firmware 2.2.1) --------------------------------
// NOTE: all settings (maps, absolute position source etc.) are stored in your phone - when using another
// device for the WIFI connection (PC etc.), you will have to transfer those settings (share maps via app, 
// re-enter absolute position source etc) !
// see Wiki on how to install the WIFI module and configure the WIFI jumpers:
// https://wiki.ardumower.de/index.php?title=Ardumower_Sunray#Bluetooth_BLE_UART_module

#define START_AP  false             // should WIFI module start its own access point? 
#define WIFI_IP   192,168,2,15      // choose IP e.g. 192,168,4,1  (comment out for dynamic IP/DHCP) - NOTE: use commans instead of points
#define WIFI_SSID "myssid"            // choose WiFi network ID
#define WIFI_PASS "mypassword"      // choose WiFi network password

// client (app) --->  server (robot)
#define ENABLE_SERVER true          // must be enabled if robot should act as server (recommended)
//#define ENABLE_SERVER false           // must be disabled if robot should act as client (requires external relay server)

// a relay server allows to access the robot via the Internet by transferring data from app to robot and vice versa (not available yet, highly experimental)
// client (app) --->  relay server  <--- client (robot)
#define ENABLE_RELAY false            // must be enabled to use relay server
#define RELAY_USER "username"         // choose a unique user name/number!
#define RELAY_MACHINE "robot1"        // choose a unique robot id
#define RELAY_HOST "grauonline.net"   // relay server name
#define RELAY_PORT 5000               // relay server port 

//#define ENABLE_UDP 1                // enable console for UDP? (for developers only)
#define UDP_SERVER_IP   192,168,2,56     // remote UDP IP and port to connect to
#define UDP_SERVER_PORT 4210

// --------- NTRIP client (linux only, highly experimental) ---------------------------------
//#define ENABLE_NTRIP 1            // must be activated to use Linux NTRIP
#define NTRIP_HOST "195.227.70.119"   // sapos nrw
#define NTRIP_PORT 2101
#define NTRIP_MOUNT "VRS_3_4G_NW"
#define NTRIP_USER "user"
#define NTRIP_PASS "pass"

// ------ MQTT (for ESP8266 only, highly experimental - ENABLE_SERVER must be set to false for this to work :-/ ) -----------------------------
// you can access your robot using a MQTT broker - choose a topic prefix for your robot below - available MQTT topics:
// robot1/cmd           (cmd can be: start, stop, dock)
// robot1/op            (current robot operation as text)
// robot1/gps/sol       (current gps solution as text)
// robot1/gps/pos       (current gps position as text)
// ... lot of other information -> see comm.cpp or check with your MQTT Explorer
//#define ENABLE_MQTT  true                           // start MQTT client?  (true for yes, false for no)
#define ENABLE_MQTT  false
#define MQTT_TOPIC_PREFIX  "robot1"                 // the MQTT topic prefix for your robot 
#define MQTT_SERVER  "192.168.2.47"                 // your MQTT broker IP or hostname (e.g. "broker.mqtt-dashboard.com")
#define MQTT_PORT  1883
#define MQTT_USER "user"
#define MQTT_PASS "pass"

// ------ ultrasonic sensor -----------------------------
// see Wiki on how to install the ultrasonic sensors: 
// https://wiki.ardumower.de/index.php?title=Ardumower_Sunray#Ultrasonic_sensor

#define SONAR_INSTALLED 1              // uncomment if ultrasonic sensors are installed
//#define SONAR_ENABLE true              // should ultrasonic sensor be used?
#define SONAR_ENABLE false
#define SONAR_TRIGGER_OBSTACLES true     // should sonar be used to trigger obstacles? if not, mower will only slow down
#define SONAR_LEFT_OBSTACLE_CM   10      // stop mowing operation below this distance (cm) 
#define SONAR_CENTER_OBSTACLE_CM 10      // stop mowing operation below this distance (cm) 
#define SONAR_RIGHT_OBSTACLE_CM  10      // stop mowing operation below this distance (cm) 

// ------ rain sensor ----------------------------------------------------------
//#define RAIN_ENABLE true                 // if activated, mower will dock when rain sensor triggers
#define RAIN_ENABLE false

// ------ time-of-flight distance sensor (VL53L0X) -----------------------------
// do not use this sensor (not recommended)
//#define TOF_ENABLE true
#define TOF_ENABLE false
#define TOF_OBSTACLE_CM 100      // stop mowing operation below this distance (cm) 


// ------ bumper sensor (bumperduino, freewheel etc.) ----------------
// see Wiki on how to install bumperduino or freewheel sensor:
// https://wiki.ardumower.de/index.php?title=Bumper_sensor
// https://wiki.ardumower.de/index.php?title=Free_wheel_sensor
//#define BUMPER_ENABLE true
#define BUMPER_ENABLE false
#define BUMPER_DEADTIME 1000  // linear motion dead-time (ms) after bumper is allowed to trigger
#define BUMPER_TRIGGER_DELAY  0 // bumper must be active for (ms) to trigger
#define BUMPER_MAX_TRIGGER_TIME 30  // if bumpersensor stays permanent triggered mower will stop with bumper error (time in seconds; 0 = disabled)

// ----- battery charging current measurement (INA169) --------------
// the Marotronics charger outputs max 1.5A 
// ( https://www.marotronics.de/Ladegeraete-fuer-den-Ardumower-Akkus-24V-mit-Status-LED-auch-fuer-Li-Ion-Akkus )
// so we are using the INA169 in non-bridged mode (max. 2.5A)
// ( https://www.marotronics.de/INA169-Analog-DC-Current-Sensor-Breakout-60V-25A-5A-Marotronics )

#define CURRENT_FACTOR 0.5     // PCB1.3 (non-bridged INA169, max. 2.5A)
//#define CURRENT_FACTOR 1.0   // PCB1.3 (bridged INA169, max. 5A)
//#define CURRENT_FACTOR 1.98   // PCB1.4 (non-bridged INA169, max. 2.5A)
//#define CURRENT_FACTOR 2.941  // PCB1.4 (bridged INA169, max. 5A)

#define GO_HOME_VOLTAGE   21.5  // start going to dock below this voltage
// The battery will charge if both battery voltage is below that value and charging current is above that value.
#define BAT_FULL_VOLTAGE  28.7  // start mowing again at this voltage
#define BAT_FULL_CURRENT  0.2   // start mowing again below this charging current (amps)

// https://wiki.ardumower.de/index.php?title=Ardumower_Sunray#Automatic_battery_switch_off
#define BAT_SWITCH_OFF_IDLE  false         // switch off if idle (JP8 must be set to autom.)
#define BAT_SWITCH_OFF_UNDERVOLTAGE  true  // switch off if undervoltage (JP8 must be set to autom.)


// ------ GPS ------------------------------------------
// ------- RTK GPS module -----------------------------------
// see Wiki on how to install the GPS module and configure the jumpers:
// https://wiki.ardumower.de/index.php?title=Ardumower_Sunray#Bluetooth_BLE_UART_module
//
// NOTE: if you experience GPS checksum errors, try to increase UART FIFO size:
// 1. Arduino IDE->File->Preferences->Click on 'preferences.txt' at the bottom
// 2. Locate file 'packages/arduino/hardware/sam/xxxxx/cores/arduino/RingBuffer.h'
//    (for Adafruit Grand Central M4: 'packages\adafruit\hardware\samd\xxxxx\cores\arduino\RingBuffer.h')
// change:     #define SERIAL_BUFFER_SIZE 128     into into:     #define SERIAL_BUFFER_SIZE 1024

//#define GPS_USE_TCP 1                    // comment out for serial gps, activate for TCP client-based GPS
//#define GPS_SKYTRAQ  1               // comment out for ublox gps, uncomment for skytraq gps/NMEA

#define REQUIRE_VALID_GPS  true       // mower will pause if no float and no fix GPS solution during mowing (recommended)
//#define REQUIRE_VALID_GPS  false    // mower will continue to mow if no float or no fix solution (not recommended)

#define GPS_SPEED_DETECTION true  // will detect obstacles via GPS feedback (no speed)  - recommended
//#define GPS_SPEED_DETECTION false

// detect if robot is actually moving (obstacle detection via GPS feedback)
#define GPS_MOTION_DETECTION          true    // if robot is not moving trigger obstacle avoidance (recommended)
//#define GPS_MOTION_DETECTION        false   // ignore if robot is not moving
#define GPS_MOTION_DETECTION_TIMEOUT  8      // timeout for motion (secs)

// configure ublox f9p with optimal settings (will be stored in f9p RAM only)
// NOTE: due to a PCB1.3 bug GPS_RX pin is not working and you have to fix this by a wire:
// https://wiki.ardumower.de/index.php?title=Ardumower_Sunray#PCB1.3_GPS_pin_fix_and_wire_fix   (see 'GPS wire fix')
#define GPS_REBOOT_RECOVERY  true // allow GPS receiver rebooting (recommended - requires GPS wire fix above! otherwise firmware will stuck at boot!)
//#define GPS_REBOOT_RECOVERY   false  // do not allow rebooting GPS receiver (no GPS wire fix required)

#define GPS_CONFIG   true     // configure GPS receiver (recommended - requires GPS wire fix above! otherwise firmware will stuck at boot!)
//#define GPS_CONFIG   false  // do not configure GPS receiver (no GPS wire fix required)

#define GPS_CONFIG_FILTER   true     // use signal strength filter? (recommended to get rid of 'FIX jumps') - adjust filter settings below
//#define GPS_CONFIG_FILTER   false     // use this if you have difficulties to get a FIX solution (uses ublox default filter settings)
#define CPG_CONFIG_FILTER_MINELEV  10   // Min SV elevation degree: 14 (high elevation, less robust), 10 (low elevation, robust) 
#define CPG_CONFIG_FILTER_NCNOTHRS 10   // C/N0 Threshold #SVs: 10 (robust), 6 (less robust)
#define CPG_CONFIG_FILTER_CNOTHRS  30   // 30 dbHz (robust), 13 dbHz (less robust)


// ------ obstacle detection and avoidance  -------------------------

#define ENABLE_PATH_FINDER  true     // path finder calculates routes around exclusions and obstacles 
//#define ENABLE_PATH_FINDER  false
#define ALLOW_ROUTE_OUTSIDE_PERI_METER 1.0   // max. distance (m) to allow routing from outside perimeter 
// (increase if you get 'no map route' errors near perimeter)

#define OBSTACLE_DETECTION_ROTATION true // detect robot rotation stuck (requires IMU)
// #define OBSTACLE_DETECTION_ROTATION false   // NOTE: recommended to turn this off for slope environment   

#define OBSTACLE_AVOIDANCE true   // try to find a way around obstacle
//#define OBSTACLE_AVOIDANCE false  // stop robot on obstacle
#define OBSTACLE_DIAMETER 1.2   // choose diameter of obstacles placed in front of robot (m) for obstacle avoidance
#define DISABLE_MOW_MOTOR_AT_OBSTACLE true // switch off mow motor while escape at detected obstacle; set false if mow motor shall not be stopped at detected obstacles

// detect robot being kidnapped? robot will try GPS recovery if distance to tracked path is greater than a certain value
// (false GPS fix recovery), and if that fails go into error 
#define KIDNAP_DETECT true  // recommended
//#define KIDNAP_DETECT false
#define KIDNAP_DETECT_ALLOWED_PATH_TOLERANCE 1.0  // allowed path tolerance (m) 
#define KIDNAP_DETECT_ALLOWED_PATH_TOLERANCE_DOCK_UNDOCK 0.2  // allowed path tolerance (m) 
#define KIDNAP_DETECT_DISTANCE_DOCK_UNDOCK 2  // distance from dock in (m) to use KIDNAP_DETECT_ALLOWED_PATH_TOLERANCE_DOCK_UNDOCK

// ------ docking --------------------------------------
// is a docking station available?
#define DOCKING_STATION true   // use this if docking station available and mower should dock automatically
//#define DOCKING_STATION false    // mower will just stop after mowing instead of docking automatically 

#define DOCK_IGNORE_GPS false     // use GPS fix in docking station and IMU for GPS float/invalid
//#define DOCK_IGNORE_GPS true     // ignore GPS fix in docking station and use IMU-only (use this if robot gets false GPS fixes in your docking station)

#define DOCK_AUTO_START true     // robot will automatically continue mowing after docked automatically
//#define DOCK_AUTO_START false      // robot will not automatically continue mowing after docked automatically

//#define DOCK_RETRY_TOUCH true   // robot will retry touching docking contacts (max. 1cm) if loosing docking contacts during charging
#define DOCK_RETRY_TOUCH false   // robot will not retry touching docking contacts (max. 1cm) if loosing docking contacts during charging

#define DOCK_UNDOCK_TRACKSLOW_DISTANCE 5 // set distance (m) from dock for trackslow (speed limit)

#define UNDOCK_IGNORE_GPS_DISTANCE 2 // set distance (m) from dock to ignore gps while undocking

// ---- path tracking -----------------------------------

// below this robot-to-target distance (m) a target is considered as reached
#define TARGET_REACHED_TOLERANCE 0.05

// stanley control for path tracking - determines gain how fast to correct for lateral path errors
#define STANLEY_CONTROL_P_NORMAL  3.0   // 3.0 for path tracking control (angular gain) when mowing
#define STANLEY_CONTROL_K_NORMAL  1.0   // 1.0 for path tracking control (lateral gain) when mowing

#define STANLEY_CONTROL_P_SLOW    3.0   // 3.0 for path tracking control (angular gain) when docking tracking
#define STANLEY_CONTROL_K_SLOW    0.1   // 0.1 for path tracking control (lateral gain) when docking tracking


// ----- other options --------------------------------------------

// button control (turns on additional features via the POWER-ON button)
#define BUTTON_CONTROL true      // additional features activated (press-and-hold button for specific beep count: 
                                 //  1 beep=stop, 6 beeps=start, 5 beeps=dock, 3 beeps=R/C mode ON/OFF, 9 beeps=shutdown, 12 beeps=WiFi WPS
//#define BUTTON_CONTROL false   // additional features deactivated

//#define USE_TEMP_SENSOR true  // only activate if temp sensor (htu21d) connected
#define USE_TEMP_SENSOR false  

#define DOCK_OVERHEAT_TEMP 90    // if temperature above this degreeC, mower will dock 
#define DOCK_TOO_COLD_TEMP 5    // if temperature below this degreeC, mower will dock 

// activate support for model R/C control?
// use PCB pin 'mow' for R/C model control speed and PCB pin 'steering' for R/C model control steering, 
// also connect 5v and GND and activate model R/C control via PCB P20 start button for 3 sec.
// more details: https://wiki.ardumower.de/index.php?title=Ardumower_Sunray#R.2FC_model
//#define RCMODEL_ENABLE 1  // uncomment line to turn on R/C control

#define BUZZER_ENABLE 1 // uncomment to disable


// ------ experimental options  -------------------------

// drive curves smoothly?
//#define SMOOTH_CURVES  true
#define SMOOTH_CURVES  false

// --------- serial monitor output (CONSOLE) ------------------------
// which Arduino Due USB port do you want to your for serial monitor output (CONSOLE)?
// Arduino Due native USB port  => choose SerialUSB
// Arduino Due programming port => choose Serial
#if defined (__arm__) && defined (__SAM3X8E__) // Arduino Due compatible
  #define BOARD "Arduino Due"
  #define CONSOLE SerialUSB   // Arduino Due: do not change (used for Due native USB serial console)
#elif __SAMD51__
  #define BOARD "Adafruit Grand Central M4"
  #define CONSOLE Serial      // Adafruit Grand Central M4 
#elif __linux__ 
  #define BOARD "Linux"
  #define CONSOLE Console 
#else
  #ifdef __cplusplus
    #error "ERROR: you need to choose either Arduino Due or Adafruit GCM4 in Arduino IDE"
  #endif
#endif

// ------- serial ports and baudrates---------------------------------
#define CONSOLE_BAUDRATE    115200    // baudrate used for console
//#define CONSOLE_BAUDRATE    921600  // baudrate used for console
#define BLE_BAUDRATE    115200        // baudrate used for BLE
#define BLE_NAME      "Ardumower"     // name for BLE module
#define GPS_BAUDRATE  115200          // baudrate for GPS RTK module
#define WIFI_BAUDRATE 115200          // baudrate for WIFI module
#define ROBOT_BAUDRATE 115200         // baudrate for Linux serial robot (non-Ardumower)

#ifdef __SAM3X8E__                 // Arduino Due
  #define WIFI Serial1
  #define ROBOT Serial1
  #define BLE Serial2
  #define GPS Serial3
  //#define GPS Serial                // only use this for .ubx logs (sendgps.py)
#elif __SAMD51__                      // Adafruit Grand Central M4 
  #define WIFI Serial2 
  #define ROBOT Serial2               
  #define BLE Serial3
  #define GPS Serial4
#elif __linux__ 
  #define WIFI SerialWIFI                
  #define SERIAL_WIFI_PATH "/dev/null"  
  #define LINUX_BLE       // comment to disable BLE
  #define BLE SerialBLE             
  #define SERIAL_BLE_PATH "/dev/null"    // dummy serial device    
  #define GPS SerialGPS
  #define SERIAL_GPS_PATH "/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00"  
  #define GPS_HOST "127.0.0.1"  
  #define GPS_PORT 2947  
  #define ROBOT SerialROBOT
  #define SERIAL_ROBOT_PATH "/dev/ttyUSB1"  
  #define NTRIP SerialNTRIP
  #define SERIAL_NTRIP_PATH "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_00000000-if00-port0"    
#endif



// ------- I2C addresses -----------------------------
#define DS1307_ADDRESS B1101000
#define AT24C32_ADDRESS B1010000


// ------- PCB1.3/Due settings -------------------------
#define IOREF 3.3   // I/O reference voltage 

// ------ hardware pins---------------------------------------
// no configuration needed here

#ifdef __linux__
  // ...
#else
  #define pinMotorEnable  37         // EN motors enable
  #define pinMotorLeftPWM 5          // M1_IN1 left motor PWM pin
  #define pinMotorLeftDir 31         // M1_IN2 left motor Dir pin
  #define pinMotorLeftSense A1       // M1_FB  left motor current sense
  #define pinMotorLeftFault 25       // M1_SF  left motor fault
                                                              
  #define pinMotorRightPWM  3        // M2_IN1 right motor PWM pin
  #define pinMotorRightDir 33        // M2_IN2 right motor Dir pin
  #define pinMotorRightSense A0      // M2_FB  right motor current sense
  #define pinMotorRightFault 27      // M2_SF  right motor fault
                                      
  #define pinMotorMowPWM 2           // M1_IN1 mower motor PWM pin (if using MOSFET, use this pin)
  #define pinMotorMowDir 29          // M1_IN2 mower motor Dir pin (if using MOSFET, keep unconnected)
  #define pinMotorMowSense A3        // M1_FB  mower motor current sense  
  #define pinMotorMowFault 26        // M1_SF  mower motor fault   (if using MOSFET/L298N, keep unconnected)
  #define pinMotorMowEnable 28       // EN mower motor enable      (if using MOSFET/L298N, keep unconnected)
  #define pinMotorMowRpm A11
      
  #define pinFreeWheel 8             // front/rear free wheel sensor    
  #define pinBumperLeft 39           // bumper pins
  #define pinBumperRight 38

  #define pinDropLeft 45           // drop pins                                                                                          Dropsensor - Absturzsensor
  #define pinDropRight 23          // drop pins                                                                                          Dropsensor - Absturzsensor

  #define pinSonarCenterTrigger 24   // ultrasonic sensor pins
  #define pinSonarCenterEcho 22
  #define pinSonarRightTrigger 30    
  #define pinSonarRightEcho 32
  #define pinSonarLeftTrigger 34         
  #define pinSonarLeftEcho 36
  #define pinPerimeterRight A4       // perimeter
  #define pinDockingReflector A4     // docking IR reflector
  #define pinPerimeterLeft A5

  #define pinLED 13                  // LED
  #define pinBuzzer 53               // Buzzer
  //#define pinTilt 35                 // Tilt sensor (required for TC-G158 board)  
  #define pinLift 35                 // Lift sensor (marked as 'Tilt' on PCB1.3/1.4) 
  #define pinButton 51               // digital ON/OFF button
  #define pinBatteryVoltage A2       // battery voltage sensor
  #define pinBatterySwitch 4         // battery-OFF switch   
  #define pinChargeVoltage A9        // charging voltage sensor
  #define pinChargeCurrent A8        // charge current sensor
  #define pinChargeRelay 50          // charge relay
  #define pinRemoteMow 12            // remote control mower motor
  #define pinRemoteSteer 11          // remote control steering 
  #define pinRemoteSpeed 10          // remote control speed
  #define pinRemoteSwitch 52         // remote control switch
  #define pinVoltageMeasurement A7   // test pin for your own voltage measurements
  #if defined(_SAM3XA_)              // Arduino Due
    #define pinOdometryLeft DAC0     // left odometry sensor
    #define pinOdometryRight CANRX   // right odometry sensor  
    #define pinReservedP46 CANTX     // reserved
    #define pinReservedP48 DAC1      // reserved
  #else                              // Adafruit Grand Central M4 
    #define pinOdometryLeft A12      // left odometry sensor
    #define pinOdometryRight A14     // right odometry sensor 
    #define pinReservedP46 A15       // reserved
    #define pinReservedP48 A13       // reserved
  #endif
  #define pinLawnFrontRecv 40        // lawn sensor front receive
  #define pinLawnFrontSend 41        // lawn sensor front sender 
  #define pinLawnBackRecv 42         // lawn sensor back receive
  #define pinLawnBackSend 43         // lawn sensor back sender 
  #define pinUserSwitch1 46          // user-defined switch 1
  #define pinUserSwitch2 47          // user-defined switch 2
  #define pinUserSwitch3 48          // user-defined switch 3
  #define pinRain 44                 // rain sensor
  #define pinReservedP14 A7          // reserved
  #define pinReservedP22 A6          // reserved
  #define pinReservedP26 A10         // reserved

  #ifndef SDCARD_SS_PIN
    #if defined(_SAM3XA_)              // Arduino Due
      #define SDCARD_SS_PIN pinUserSwitch1
    #else
      #define SDCARD_SS_PIN 4
    #endif
  #endif
#endif

// IMU (compass/gyro/accel): I2C  (SCL, SDA) 
// Bluetooth: Serial2 (TX2, RX2)
// GPS: Serial3 (TX3, RX3) 
// WIFI: Serial1 (TX1, RX1) 

#define DEBUG(x) CONSOLE.print(x)
#define DEBUGLN(x) CONSOLE.println(x)

#if defined(ENABLE_SD_LOG)
  #define CONSOLE sdSerial         
#elif defined(ENABLE_UDP)
  #define CONSOLE udpSerial         
#endif


// the following will be used by Arduino library RingBuffer.h - to verify this Arduino library file:
// 1. Arduino IDE->File->Preferences->Click on 'preferences.txt' at the bottom
// 2. Locate file 'packages/arduino/hardware/sam/xxxxx/cores/arduino/RingBuffer.h
  
#define SERIAL_BUFFER_SIZE 1024

#ifdef BNO055
  #define MPU9250   // just to make mpu driver happy to compile something
#endif

#ifdef __cplusplus
  #include "udpserial.h"
  #include "sdserial.h"
  #include "src/agcm4/adafruit_grand_central.h"
  #ifdef __linux__
    #include "src/linux/linux.h"    
    #include <Console.h>
  #endif
#endif

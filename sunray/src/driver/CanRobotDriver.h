// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

// owlRobotics platform mower: robot (with motor drivers, battery, bumper etc.) connected and controlled via CAN bus

#ifndef CAN_ROBOT_DRIVER_H
#define CAN_ROBOT_DRIVER_H

#include <Arduino.h>
#include "RobotDriver.h"
#include "../../config.h"
#include "../../types.h"

#ifdef __linux__
  #include <Process.h>
  #include "../../linuxcan.h"
#else 
  #include "../../can.h"
#endif

#ifndef ENABLE_CAN_DISPLAY
#define ENABLE_CAN_DISPLAY 0
#endif


// -----CAN frame data types----------------

#define OWL_RELAIS_MSG_ID    400  // owlRelais PCB
#define OWL_DRIVE_MSG_ID     300  // owlDrive PCB 
#define OWL_CONTROL_MSG_ID   200  // owlControl PCB 
#define OWL_RECEIVER_MSG_ID  100  // owlReceiver PCB 
#define OWL_DISPLAY_MSG_ID   500  // owlDisplay (grafische Anzeige)

#define MY_NODE_ID 61

// owlDrive PCB 
#define LEFT_MOTOR_NODE_ID    1 
#define RIGHT_MOTOR_NODE_ID   2
#define MOW1_MOTOR_NODE_ID    3
#define MOW2_MOTOR_NODE_ID    4
#define MOW3_MOTOR_NODE_ID    5
#define MOW4_MOTOR_NODE_ID    6
#define MOW5_MOTOR_NODE_ID    7

//#define MOW_MOTOR_COUNT   5    // defined in config.h


#define MOW_HEIGHT_MOTOR_NODE_ID     8


#define CONTROL_NODE_ID        1// owlControl PCB
#define DISPLAY_NODE_ID        1// owlDisplay node (shared bus, distinct message ID)

#define RELAIS_1_NODE_ID   1 // owlRelais PCB
#define RELAIS_2_NODE_ID   2

#define RECEIVER_PUSHBOX_NODE_ID    3 // owlReceiver PCB 


typedef union canNodeType_t {
    uint8_t byteVal[2];
    struct __attribute__ ((__packed__)) {
        uint16_t sourceNodeID : 6;   // 6 bits for source node ID (valid node IDs: 1-62)
        uint16_t destNodeID   : 6;   // 6 bits for destination node ID (valid node IDs: 1-62, value 63 means all nodes)
        uint16_t reserved     : 4;   // 4 bits reserved
    } sourceAndDest;
} __attribute__((packed)) canNodeType_t;

// what action to do...
enum canCmdType_t: uint8_t {
    can_cmd_info       = 0,  // broadcast something
    can_cmd_request    = 1,  // request something
    can_cmd_set        = 2,  // set something
    can_cmd_save       = 3,  // save something        
};

namespace owldrv {
  // which variable to use for the action...
  enum canValueType_t: uint8_t {
      can_val_target          = 0, // target
      can_val_voltage         = 1, // voltage
      can_val_current         = 2, // current
      can_val_velocity        = 3, // velocity
      can_val_angle           = 4, // angle
      can_val_motion_ctl_mode = 5, // motion control mode
      can_val_cfg_mem         = 6, // config memory
      can_val_motor_enable    = 7, // motor enable state
      can_val_pAngleP         = 8, // angle P controller
      can_val_velocityLimit   = 9, // max. velocity of the position control (rad/s)
      can_val_pidVelocityP    = 10, // velocity P   
      can_val_pidVelocityI    = 11, // velocity I   
      can_val_pidVelocityD    = 12, // velocity D
      can_val_pidVelocityRamp = 13, // velocity PID output ramp  (max. output change/s)
      can_val_lpfVelocityTf   = 14, // velocity low-pass filtering time constant (sec)
      can_val_error           = 15, // error status
      can_val_upload_firmware = 16, // upload file (to upload new firmware)
      can_val_firmware_crc    = 17, // firmware flash memory CRC (to verify firmware integrity)        
      can_val_firmware_ver    = 18, // firmware version
      can_val_broadcast_rx_enable  = 19, // broadcast receive enable state       
      can_val_fifo_target     = 20, // add target (to drive within one clock duration) to FIFO 
      can_val_endswitch_allow_pos_neg_dtargets = 21, // pos/neg delta targets allowed at end-switch?
      can_val_reboot          = 22, // reboot MCU
      can_val_endswitch       = 23, // end-switch status
      can_val_fifo_clock      = 24, // FIFO clock signal (process FIFO)
      can_val_control_error   = 25, // control error (setpoint-actual)
      can_val_fifo_target_ack_result_val = 26, // which variable to send in an 'can_val_fifo_target' acknowledge     
      can_val_detected_supply_voltage = 27,  // detected supply voltage
      can_val_angle_add       = 28,  // add angle 
      can_val_pwm_speed       = 29,  // pwm-speed (-1.0...1.0  =  classic motor controller compatiblity)
      can_val_odo_ticks       = 30,  // odometry ticks (encoder ticks   =  classic motor controller compatiblity)
      can_val_misc_sensor1    = 31,  // miscellaneous sensor1 state
      can_val_misc_sensor2    = 32,  // miscellaneous sensor2 state  
      can_val_total_current   = 33,  // total current of all motor phases (low-pass filtered) 
      can_val_device_id       = 34,  // device ID (0..63)
  };
}

namespace owlctl {
  // which variable to use for the action...
  enum canValueType_t: uint8_t {
      can_val_error             = 1, // error status
      can_val_battery_voltage   = 2, // voltage
      can_val_bumper_state      = 3, // bumper status
      can_val_stop_button_state = 4, // STOP button state
      can_val_buzzer_state      = 5, // buzzer state
      can_val_rain_state        = 6, // rain state
      can_val_charger_voltage   = 7, // charger voltage      
      can_val_lift_state        = 8, // lift sensor state      
      can_val_slow_down_state   = 9, // slow-down state
      can_val_ip_address        = 10, // IP address
      can_val_device_id         = 11,
      can_val_power_off_state   = 12, // power-off pin state
      can_val_power_off_command = 13, // schedule power-off
  };

  enum powerOffState_t: uint8_t {
      power_off_inactive = 0,
      power_off_active = 1,
      power_off_shutdown_pending = 2,
  };
}

namespace owldisplay {
  enum valueType_t : uint8_t {
      can_val_sat_summary      = 0x10,
      can_val_rtk_age          = 0x11,
      can_val_wifi_signal      = 0x12,
      can_val_ip_address       = 0x14,
      can_val_battery_voltage  = 0x40,
      can_val_battery_current  = 0x41,
      can_val_map_progress     = 0x50,
      can_val_state_code       = 0x51,
      can_val_status_message   = 0x52
  };

  enum stateCode_t : uint8_t {
      state_unknown = 0,
      state_mow     = 1,
      state_dock    = 2,
      state_idle    = 3,
      state_charge  = 4,
      state_error   = 5
  };
}

namespace owlrecv {

  // which variable to use for the action...
  enum canValueType_t: uint8_t {
      can_val_button_state     = 1, // button state
      can_val_axis_x1          = 2, // x1-axis state
      can_val_axis_y1          = 3, // y1-axis state
      can_val_axis_z1          = 4, // z1-axis state
      can_val_axis_x2          = 5, // x2-axis state
      can_val_axis_y2          = 6, // y2-axis state
      can_val_axis_z2          = 7, // z2-axis state
      can_val_battery_voltage  = 8, // battery voltage 
      can_val_device_id        = 9,
  };

}  // namespace

namespace owlrls {

  // which variable to use for the action...
  enum canValueType_t: uint8_t {
    can_val_device_id         = 0, // info value
    can_val_error             = 1, // error status
    can_val_relais_state      = 2, // relais state
    can_val_relais_countdown  = 3, // set time or get time left
  };
} // namespace owlrls

// motor driver error values
enum errType_t: uint8_t {
    err_ok           = 0, // everything OK
    err_no_comm      = 1, // no CAN communication
    err_no_settings  = 2, // no settings
    err_undervoltage = 3, // undervoltage triggered
    err_overvoltage  = 4, // overvoltage triggered
    err_overcurrent  = 5, // overcurrent triggered
    err_overtemp     = 6, // over-temperature triggered    
};

// which data the variable has, CAN data can be different variants 
typedef union {
    uint8_t byteVal[4];  // either 4 bytes
    int32_t intValue;    // either integer (4 bytes)
    float floatVal;      // either float (4 bytes)
    struct __attribute__ ((__packed__)) ofs_val_t {   // either short (2 bytes) offset and 1 byte
        uint16_t ofsVal;
        uint8_t  byteVal;
    } ofsAndByte;
} __attribute__((packed)) canDataType_t;



class CanRobotDriver: public RobotDriver {
  public:
    String robotID;
    String mcuFirmwareName;
    String mcuFirmwareVersion;
    int requestLeftPwm;
    int requestRightPwm;
    bool requestReleaseBrakesWhenZero;
    int requestMowPwm;   
    int requestMowHeightMillimeter;         
    unsigned long encoderTicksLeft;
    unsigned long encoderTicksRight;
    unsigned long encoderTicksMow[MOW_MOTOR_COUNT];
    bool mcuCommunicationLost;
    bool mowFault[MOW_MOTOR_COUNT];
    bool leftMotorFault;
    bool rightMotorFault;
    float batteryVoltage;
    float chargeVoltage;
    float chargeCurrent;
    float mowCurr[MOW_MOTOR_COUNT];
    float motorLeftCurr;
    float motorRightCurr;
    float motorHeightAngleCurr;
    float motorHeightAngleEndswitch;
    bool motorHeightAngleEndswitchSet;
    bool motorHeightFoundEndswitch;
    bool resetMotorTicks;
    float batteryTemp;
    float cpuTemp;
    bool triggeredLeftBumper;
    bool triggeredRightBumper;
    bool triggeredLift;
    bool triggeredRain;
    bool triggeredStopButton;
    bool triggeredSlowDown;
    bool triggeredPushboxStopButton;
    unsigned long nextDisplayStateTime;
    OperationType lastDisplayOpSent;
    String lastIpSent;
    bool lastIpSentValid;
    uint8_t lastIpSentBytes[4];
    void begin() override;
    void run() override;
    bool getRobotID(String &id) override;
    bool getMcuFirmwareVersion(String &name, String &ver) override;
    float getCpuTemperature() override;
    void requestMotorDrivePwm(int leftPwm, int rightPwm, bool requestReleaseBrakesWhenZero);
    void requestMotorMowPwm(int mowPwm);
    void requestMotorMowCurrent();
    void requestMowHeight(int mowHeightMillimeter);
    void requestMotorErrorStatus();
    void requestSummary();
    void requestPushboxState();        
    void requestVersion();
    void updateCpuTemperature();
    void updateWifiConnectionState();
    void updateWifiSignalStrength();
    virtual void sendIpAddress() override;
    void updateDisplayTelemetry();
    void requestPowerOffState();
    void requestManagedShutdown(uint8_t delaySeconds);
    void sendPowerOffCommand(uint8_t delaySeconds);
    uint8_t getPowerOffDelaySeconds() const;
    void setSimulatePowerOffHang(bool flag, unsigned long durationSeconds = 0);
    void setSimulatePowerOffHangFor(unsigned long durationSeconds){ setSimulatePowerOffHang(true, durationSeconds); }
    bool getSimulatePowerOffHang() const;
    void sendCanData(int msgId, int destNodeId, canCmdType_t cmd, int val, canDataType_t data);
    void sendDisplayOperation(OperationType op);
  protected:    
    bool ledPanelInstalled;
    #ifdef __linux__
      LinuxCAN can;
      Process cpuTempProcess;
      Process wifiStatusProcess;
      Process wifiSignalProcess;
      Process ipAddressToStringProcess;
    #else  
      CAN can; // dummy, so compiler doesn't complain on other platforms
    #endif    
    String cmd;
    String cmdResponse;
    unsigned long nextMotorTime;    
    unsigned long nextSummaryTime;
    unsigned long motorHeightFoundEndswitchTime;
    unsigned long nextCheckErrorTime;
    unsigned long nextConsoleTime;
    unsigned long nextMowTime;    
    unsigned long nextTempTime;
    unsigned long nextWifiTime;
    unsigned long nextLedTime;    
    unsigned long nextDisplayTelemetryTime;
    unsigned long powerOffLogTime;
    unsigned long powerOffCommandSendTime;
    bool powerOffCommandSent;
    bool powerOffCommandAccepted;
    bool linuxShutdownIssued;
    bool simulatePowerOffHang;
    bool simulatePowerOffHangNotified;
    unsigned long simulatePowerOffHangUntil;
    bool simulatePowerOffHangConfigured;
    unsigned long simulatePowerOffHangConfiguredDuration;
    bool simulatePowerOffHangCommandPending;
    unsigned long simulatePowerOffHangCommandTime;
    bool simulatePiSelfShutdownPending;
    unsigned long simulatePiSelfShutdownTime;
    owlctl::powerOffState_t powerOffState;
    uint8_t powerOffDelaySeconds;
    bool satSummarySent;
    uint8_t lastSatStatus;
    uint8_t lastSatUsed;
    uint8_t lastSatTotal;
    bool rtkAgeSent;
    uint16_t lastRtkAgeTenths;
    bool mapProgressSent;
    uint16_t lastMapCount;
    uint8_t lastMapPercent;
    enum class PowerOffDecisionTrigger : uint8_t {
      None = 0,
      ExternalPin,
      InternalRequest
    };
    bool powerOffDecisionPending;
    unsigned long powerOffDecisionStartTime;
    unsigned long powerOffDecisionDeadline;
    uint8_t powerOffDecisionDelaySeconds;
    PowerOffDecisionTrigger powerOffDecisionTrigger;
    int consoleCounter;
    int cmdMotorCounter;
    int cmdSummaryCounter;
    int cmdMotorResponseCounter;
    int cmdSummaryResponseCounter;
    int16_t lastWifiSignalDbm = -127;
    void sendSerialRequest(String s);
    void processResponse();
    void motorResponse();
    void summaryResponse();
    void versionResponse();
    void handlePowerOffState(owlctl::powerOffState_t remoteState, uint8_t activeSeconds, uint8_t configuredDelay);
    void handlePowerOffCommandAck(uint8_t acceptedFlag, uint8_t delaySeconds);
    void startPowerOffDecision(uint8_t delaySeconds, PowerOffDecisionTrigger trigger);
    void cancelPowerOffDecision(PowerOffDecisionTrigger trigger);
    void processPowerOffDecision();
    bool readyForManagedShutdown(PowerOffDecisionTrigger trigger);
    void processPendingPowerOffCommand();
};

class CanMotorDriver: public MotorDriver {
  public:        
    unsigned long lastEncoderTicksLeft;
    unsigned long lastEncoderTicksRight; 
    unsigned long lastEncoderTicksMow[MOW_MOTOR_COUNT];
    CanRobotDriver &canRobot;
    CanMotorDriver(CanRobotDriver &sr);
    void begin() override;
    void run() override;
    void setMowHeight(int mowHeightMillimeter) override;
    void setMotorPwm(int leftPwm, int rightPwm, int mowPwm, bool releaseBrakesWhenZero) override;
    void getMotorFaults(bool &leftFault, bool &rightFault, bool &mowFault) override;
    void resetMotorFaults()  override;
    void getMotorCurrent(float &leftCurrent, float &rightCurrent, float &mowCurrent) override;
    void getMotorEncoderTicks(int &leftTicks, int &rightTicks, int &mowTicks) override;
};

class CanBatteryDriver : public BatteryDriver {
  public:   
    float batteryTemp;
    bool mcuBoardPoweredOn;
    unsigned long nextTempTime;
    unsigned long nextADCTime;
    bool adcTriggered;
    unsigned long linuxShutdownTime;
    bool owlPowerOffNotified;
    #ifdef __linux__
      Process batteryTempProcess;
    #endif
    CanRobotDriver &canRobot;
    CanBatteryDriver(CanRobotDriver &sr);
    void begin() override;
    void run() override;    
    float getBatteryVoltage() override;
    float getChargeVoltage() override;
    float getChargeCurrent() override;    
    float getBatteryTemperature() override;    
    virtual void enableCharging(bool flag) override;
    virtual void keepPowerOn(bool flag) override;
    void updateBatteryTemperature();
};

class CanBumperDriver: public BumperDriver {
  public:    
    CanRobotDriver &canRobot;
    CanBumperDriver(CanRobotDriver &sr);
    void begin() override;
    void run() override;
    bool nearObstacle() override;
    bool obstacle() override;
    bool getLeftBumper() override;
    bool getRightBumper() override;
    void getTriggeredBumper(bool &leftBumper, bool &rightBumper) override;  	  		    
};

class CanStopButtonDriver: public StopButtonDriver {
  public:    
    CanRobotDriver &canRobot;
    CanStopButtonDriver(CanRobotDriver &sr);
    void begin() override;
    void run() override;
    bool triggered() override;  	  		    
};

class CanRainSensorDriver: public RainSensorDriver {
  public:    
    CanRobotDriver &canRobot;
    CanRainSensorDriver(CanRobotDriver &sr);    
    void begin() override;
    void run() override;
    bool triggered() override;  
};

class CanLiftSensorDriver: public LiftSensorDriver {
  public:    
    CanRobotDriver &canRobot;
    CanLiftSensorDriver(CanRobotDriver &sr);    
    void begin() override;
    void run() override;
    bool triggered() override;  
};

class CanBuzzerDriver: public BuzzerDriver {
  public:    
    CanRobotDriver &canRobot;
    CanBuzzerDriver(CanRobotDriver &sr);    
    void begin() override;
    void run() override;
    void noTone() override;
    void tone(int freq) override;  
};

class CanRelaisDriver: public RelaisDriver {
  public:
    CanRobotDriver &canRobot;
    CanRelaisDriver(CanRobotDriver &sr);
    //std::vector<int> deviceIds;
    bool relaisState;
    void begin() override;
    void run() override;
    void setRelaisState(int relais_node_id, bool state) override;
    bool getRelaisState(int relais_node_id);
    void setRelaisStateCountdown(int relais_node_id, bool state, unsigned long countdown) override;
    //unsigned long getRelaisStateCountdown(int relais_node_id) override;
    //unsigned long getRelaisStateCountdownRemaining(int relais_node_id) override;

};

#endif

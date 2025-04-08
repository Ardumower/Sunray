// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

// motor driver base, battery driver base, bumper driver base

#ifndef ROBOT_DRIVER_H
#define ROBOT_DRIVER_H

#include "../../gps.h"
#include <Client.h>

class RobotDriver {
  public:    
    // ---- led states -----           
    bool ledStateWifiInactive;
    bool ledStateWifiConnected;
    bool ledStateGpsFix;
    bool ledStateGpsFloat;
    bool ledStateShutdown;
    bool ledStateError;
    virtual void begin() = 0;
    virtual void run() = 0;
    virtual bool getRobotID(String &id) = 0;
    virtual bool getMcuFirmwareVersion(String &name, String &ver) = 0;    
    virtual float getCpuTemperature() = 0;
};

class MotorDriver {
  public:    
    virtual void begin() = 0;
    virtual void run() = 0;

    // set mowing height in millimeters
    virtual void setMowHeight(int mowHeightMillimeter) = 0;        
    // set pwm (0-255), positive: forward, negative: backwards
    virtual void setMotorPwm(int leftPwm, int rightPwm, int mowPwm, bool releaseBrakesWhenZero) = 0;
    // get motor faults
    virtual void getMotorFaults(bool &leftFault, bool &rightFault, bool &mowFault) = 0;
    // reset motor faults
    virtual void resetMotorFaults() = 0;
    // get motor currents (ampere)
    virtual void getMotorCurrent(float &leftCurrent, float &rightCurrent, float &mowCurrent) = 0;
    // get motor encoder ticks
    virtual void getMotorEncoderTicks(int &leftTicks, int &rightTicks, int &mowTicks) = 0; 
};



class BatteryDriver {
  public:    
    virtual void begin() = 0;
    virtual void run() = 0;
    
    // read battery voltage
    virtual float getBatteryVoltage() = 0;
    // read battery temperature (degC) 
    virtual float getBatteryTemperature() = 0;
    // read charge voltage
    virtual float getChargeVoltage() = 0;
    // read charge current (amps)
    virtual float getChargeCurrent() = 0;
    // enable battery charging
    virtual void enableCharging(bool flag) = 0;
    // keep system on or power-off
    virtual void keepPowerOn(bool flag) = 0;  	  		    
};

class BumperDriver {
  public:    
    virtual void begin() = 0;
    virtual void run() = 0;
    virtual bool nearObstacle() = 0;
    virtual bool obstacle() = 0;
    virtual bool getLeftBumper() = 0;
    virtual bool getRightBumper() = 0;
    
    // get triggered bumper
    virtual void getTriggeredBumper(bool &leftBumper, bool &rightBumper) = 0;  	  		    
};

class StopButtonDriver {
  public:    
    virtual void begin() = 0;
    virtual void run() = 0;
    virtual bool triggered() = 0;  	  		    
};

class LiftSensorDriver {
  public:    
    virtual void begin() = 0;
    virtual void run() = 0;
    virtual bool triggered() = 0;  	  		    
};

class RainSensorDriver {
  public:    
    virtual void begin() = 0;
    virtual void run() = 0;
    virtual bool triggered() = 0;  	  		    
};

class ImuDriver {
  public:
    float quatW; // quaternion
    float quatX; // quaternion
    float quatY; // quaternion
    float quatZ; // quaternion        
    float roll; // euler radiant
    float pitch; // euler radiant
    float yaw;   // euler radiant
    bool imuFound;   
    // detect module (should update member 'imuFound')
    virtual void detect() = 0;             
    // try starting module with update rate 5 Hz (should return true on success)
    virtual bool begin() = 0;    
    virtual void run() = 0;
    // check if data has been updated (should update members roll, pitch, yaw)
    virtual bool isDataAvail() = 0;
    // reset module data queue (should reset module FIFO etc.)         
    virtual void resetData() = 0;        
};

class BuzzerDriver {
  public:    
    virtual void begin() = 0;
    virtual void run() = 0;
    virtual void noTone() = 0;  	  		      
    virtual void tone(int freq) = 0;
};

class GpsDriver {
  public:
    bool isRelocalizing = false;  // should robot wait for localization device until it gets its position?
    unsigned long iTOW; //  An interval time of week (ITOW), ms since Saturday/Sunday transition
    int numSV;         // #signals tracked 
    int numSVdgps;     // #signals tracked with DGPS signal
    double lon;        // deg
    double lat;        // deg
    double height;     // m
    float relPosN;     // m
    float relPosE;     // m
    float relPosD;     // m
    float heading;     // rad
    float groundSpeed; // m/s
    float accuracy;    // m
    float hAccuracy;   // m
    float vAccuracy;   // m
    SolType solution;    
    bool solutionAvail; // should bet set true if received new solution 
    unsigned long dgpsAge;
    unsigned long chksumErrorCounter;
    unsigned long dgpsChecksumErrorCounter;
    unsigned long dgpsPacketCounter;
    int year;          // UTC time year (1999..2099)
    int month;         // UTC time month (1..12)
    int day;           // UTC time day (1..31)
    int hour;          // UTC time hour (0..23)
    int mins;          // UTC time minute (0..59)
    int sec;           // UTC time second (0..60) (incl. leap second)
    int dayOfWeek;     // UTC dayOfWeek (0=Monday)
    String nmeaGGAMessage; // last NMEA-GGA message from GPS receiver
    // start tcp receiver
    virtual void begin(Client &client, char *host, uint16_t port) = 0;
    // start serial receiver          
    virtual void begin(HardwareSerial& bus,uint32_t baud) = 0;
    // should process receiver data
    virtual void run() = 0;    
    // should configure receiver    
    virtual bool configure() = 0; 
    // should reboot receiver
    virtual void reboot() = 0;

    // should send raw data to GPS receiver
    virtual void send(const uint8_t *buffer, size_t size) = 0;
    // should send RTCM data to GPS receiver
    virtual void sendRTCM(const uint8_t *buffer, size_t size) = 0;

    // generate NMEA GGA message
    virtual String generateGGA(int hour, int min, int sec, double lon, double lat, double height){
      // "$GNGGA,082947.40,5408.81295,N,01239.42452,E,1,12,0.67,34.2,M,41.1,M,,*77"
      char buffer[32];
      float lonint, latint;            
      float lonfrac = modff(lon, &lonint); // get fractional and integral parts
      float latfrac = modff(lat, &latint);
      int londeg = fabs(lonint);
      int latdeg = fabs(latint);
      float lonmins = roundf(fabs(lonfrac*60)*100000)/100000.0;
      float latmins = roundf(fabs(latfrac*60)*100000)/100000.0;
      String s = "GNGGA,";
      //UTC time HHMMSS.0      
      sprintf(buffer, "%02d%02d%02d.0", hour, min, sec);
      s += buffer;
      s += ",";     
      // coordinates in dddmm.mmmmm    (degrees, minutes and decimal minutes)
      sprintf(buffer, "%03d%08.5f", latdeg, latmins);
      s += buffer;      
      s += ",";            
      if (lat >= 0) s+= "N";
        else s += "S";
      s += ",";
      sprintf(buffer, "%03d%08.5f", londeg, lonmins);      
      s += buffer;      
      s += ",";      
      if (lon >= 0) s+= "E";
        else s += "W";
      s += ",1,12,0.67,";
      sprintf(buffer, "%.1f", height);
      s += buffer;
      s += ",M,46.1,M,,";    
      int crc = 0;
       // the first $ sign and the last two bytes of original CRC + the * sign
      for (int i = 0; i < s.length(); i++) crc ^= s[i];      
      sprintf(buffer, "%02X", crc);
      s = "$" + s + "*";
      s += buffer;
      return s;
    }

    // decodes iTOW into hour, min, sec and dayOfWeek(0=Monday)
    virtual void decodeTOW(){ 
      long towMin = iTOW / 1000 / 60;  // convert milliseconds to minutes since GPS week start      
      dayOfWeek = ((towMin / 1440)+6) % 7; // GPS week starts at Saturday/Sunday transition   
      unsigned long totalMin = towMin % 1440; // total minutes of current day  
      hour = totalMin / 60; 
      mins = totalMin % 60; 
    }
};



#endif


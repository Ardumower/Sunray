// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)


#include "BnoDriver.h"
#include "../../config.h"
#include "../../i2c.h"
//#include <EEPROM.h>


#define BNO055_SAMPLERATE_DELAY_MS (200)


/**************************************************************************/
/*
    Display the raw calibration offset and radius data
    */
/**************************************************************************/
void BnoDriver::displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
    CONSOLE.print(F("Accelerometer: "));
    CONSOLE.print(calibData.accel_offset_x); CONSOLE.print(F(" "));
    CONSOLE.print(calibData.accel_offset_y); CONSOLE.print(F(" "));
    CONSOLE.print(calibData.accel_offset_z); CONSOLE.print(F(" "));

    CONSOLE.print(F("\nGyro: "));
    CONSOLE.print(calibData.gyro_offset_x); CONSOLE.print(F(" "));
    CONSOLE.print(calibData.gyro_offset_y); CONSOLE.print(F(" "));
    CONSOLE.print(calibData.gyro_offset_z); CONSOLE.print(F(" "));

    CONSOLE.print(F("\nMag: "));
    CONSOLE.print(calibData.mag_offset_x); CONSOLE.print(F(" "));
    CONSOLE.print(calibData.mag_offset_y); CONSOLE.print(F(" "));
    CONSOLE.print(calibData.mag_offset_z); CONSOLE.print(F(" "));

    CONSOLE.print(F("\nAccel Radius: "));
    CONSOLE.print(calibData.accel_radius);

    CONSOLE.print(F("\nMag Radius: "));
    CONSOLE.print(calibData.mag_radius);
}


/**************************************************************************/
/*
    Display some basic info about the sensor status
*/
/**************************************************************************/
void BnoDriver::displaySensorStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  CONSOLE.println("");
  CONSOLE.print(F("System Status: 0x"));
  CONSOLE.println(system_status, HEX);
  CONSOLE.print(F("Self Test:     0x"));
  CONSOLE.println(self_test_results, HEX);
  CONSOLE.print(F("System Error:  0x"));
  CONSOLE.println(system_error, HEX);
  CONSOLE.println("");
  //delay(500);
}

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void BnoDriver::displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  CONSOLE.println(F("------------------------------------"));
  CONSOLE.print  (F("Sensor:       ")); CONSOLE.println(sensor.name);
  CONSOLE.print  (F("Driver Ver:   ")); CONSOLE.println(sensor.version);
  CONSOLE.print  (F("Unique ID:    ")); CONSOLE.println(sensor.sensor_id);
  CONSOLE.print  (F("Max Value:    ")); CONSOLE.print(sensor.max_value); CONSOLE.println(F(" xxx"));
  CONSOLE.print  (F("Min Value:    ")); CONSOLE.print(sensor.min_value); CONSOLE.println(F(" xxx"));
  CONSOLE.print  (F("Resolution:   ")); CONSOLE.print(sensor.resolution); CONSOLE.println(F(" xxx"));
  CONSOLE.println(F("------------------------------------"));
  CONSOLE.println("");
  //delay(500);
}

/**************************************************************************/
/*
    Display sensor calibration status
    */
/**************************************************************************/
void BnoDriver::displayCalStatus(void)
{
    /* Get the four calibration values (0..3) */
    /* Any sensor data reporting 0 should be ignored, */
    /* 3 means 'fully calibrated" */
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);

    /* The data should be ignored until the system calibration is > 0 */
    CONSOLE.print("\t");
    if (!system)
    {
        CONSOLE.print("! ");
    }

    /* Display the individual values */
    CONSOLE.print(F("Sys:"));
    CONSOLE.print(system, DEC);
    CONSOLE.print(F(" G:"));
    CONSOLE.print(gyro, DEC);
    CONSOLE.print(F(" A:"));
    CONSOLE.print(accel, DEC);
    CONSOLE.print(F(" M:"));
    CONSOLE.print(mag, DEC);
}



void BnoDriver::readCalibration(){
    int eeAddress = 0;
    long bnoID;
    bool foundCalib = false;

    //EEPROM.get(eeAddress, bnoID);
    
    sensor_t sensor;

    /*
    *  Look for the sensor's unique ID at the beginning oF EEPROM.
    *  This isn't foolproof, but it's better than nothing.
    */
    bno.getSensor(&sensor);
    if (bnoID != sensor.sensor_id)
    {
        CONSOLE.println(F("\nNo Calibration Data for this sensor exists in EEPROM"));
        delay(500);
    }
    else
    {
        #ifndef FORCE_CALIB_IMU
          CONSOLE.println(F("\nFound Calibration for this sensor in EEPROM."));        
          eeAddress += sizeof(long);
          //EEPROM.get(eeAddress, calibrationData);
  
          displaySensorOffsets(calibrationData);
  
          CONSOLE.println(F("\n\nRestoring Calibration data to the BNO055..."));
          bno.setSensorOffsets(calibrationData);
  
          CONSOLE.println(F("\n\nCalibration data loaded into BNO055"));
          foundCalib = true;
        #endif
    }

    //delay(1000);

    /* Display some basic information on this sensor */
    displaySensorDetails();

    /* Optional: Display current status */
    displaySensorStatus();

    bno.setExtCrystalUse(true);

    sensors_event_t event;
    bno.getEvent(&event);
    if (foundCalib){
        /*CONSOLE.println("Move sensor slightly to calibrate magnetometers");
        while (!bno.isFullyCalibrated())
        {
            bno.getEvent(&event);
            delay(BNO055_SAMPLERATE_DELAY_MS);
        }*/
    }
    else if (false)
    {
        CONSOLE.println(F("Please Calibrate Sensor: "));
        while (!bno.isFullyCalibrated())
        {
            bno.getEvent(&event);

            CONSOLE.print(F("X: "));
            CONSOLE.print(event.orientation.x, 4);
            CONSOLE.print(F("\tY: "));
            CONSOLE.print(event.orientation.y, 4);
            CONSOLE.print(F("\tZ: "));
            CONSOLE.print(event.orientation.z, 4);

            /* Optional: Display calibration status */
            displayCalStatus();

            /* New line for the next sample */
            CONSOLE.println("");

            /* Wait the specified delay before requesting new data */
            delay(BNO055_SAMPLERATE_DELAY_MS);
        }
        
        CONSOLE.println(F("\nFully calibrated!"));
        CONSOLE.println("--------------------------------");
        CONSOLE.println(F("Calibration Results: "));
        adafruit_bno055_offsets_t newCalib;
        bno.getSensorOffsets(newCalib);
        displaySensorOffsets(newCalib);

        CONSOLE.println(F("\n\nStoring calibration data to EEPROM..."));

        eeAddress = 0;
        bno.getSensor(&sensor);
        bnoID = sensor.sensor_id;

        //EEPROM.put(eeAddress, bnoID);

        eeAddress += sizeof(long);
        //EEPROM.put(eeAddress, newCalib);
        CONSOLE.println(F("Data stored to EEPROM."));

        CONSOLE.println(F("\n--------------------------------\n"));        
    }    
}


BnoDriver::BnoDriver(){    
    nextUpdateTime = 0;
}

void BnoDriver::selectChip(){
    #ifdef __linux__
        //CONSOLE.println("selecting I2C mux device 7...");
        // select chip via TCA9548A (I2C device7)
        //I2CwriteTo(0x70, 0, 1 << 7);
        Wire.beginTransmission(0x70);
        Wire.write(1 << 7);
        Wire.endTransmission(); 
    #endif
}

void BnoDriver::detect(){
  // detect BNO055  
  selectChip();

  if (!bno.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS))  // IMU fusion only
  {
    //if (!bno.begin(Adafruit_BNO055::OPERATION_MODE_NDOF))  // compass fusion (fast calibration)
    //if (!bno.begin(Adafruit_BNO055::OPERATION_MODE_NDOF_FMC_OFF))  // compass fusion (slow calibration)
    CONSOLE.println(F("BNO055 not found - Did you choose the correct IMU in config.h?"));          
    imuFound = false;
    return;
  }
  CONSOLE.println("BNO055 found!");  
  imuFound = true;  
}


bool BnoDriver::begin(){ 
    CONSOLE.println("using imu driver: BnoDriver");
    selectChip();
    readCalibration();
    return true;
}


void BnoDriver::run(){
}


bool BnoDriver::isDataAvail(){
    if (millis() < nextUpdateTime) return false;
    nextUpdateTime = millis() + IMU_FIFO_RATE; // 5 Hz
    sensors_event_t event; 
    
    selectChip();
    bno.getEvent(&event);              
    //bno.getCalibration(&msgTele.calSystem, &msgTele.calGyro, &msgTele.calAccel, &msgTele.calMag);            
    imu::Quaternion quat = bno.getQuat();           // Request quaternion data from BNO055
    quatW = quat.w();
    quatX = quat.x();
    quatY = quat.y();
    quatZ = quat.z();
    roll = event.orientation.z / 180.0 * PI;
    pitch = event.orientation.y / 180.0 * PI;
    yaw = -event.orientation.x / 180.0 * PI;
    return true;
}         
    
void BnoDriver::resetData(){
    
}





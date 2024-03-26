// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)


#include "MpuDriver.h"
#include "../../config.h"
#include "../../i2c.h"



MpuDriver::MpuDriver(){    
}

void MpuDriver::selectChip(){
}

void MpuDriver::detect(){
  // detect MPUxxxx  
  uint8_t data = 0;
  selectChip();
  I2CreadFrom(MPU_ADDR, 0x75, 1, &data, 1); // whoami register
  CONSOLE.print(F("MPU ID=0x"));
  CONSOLE.println(data, HEX);     
  #if defined MPU6050 || defined MPU9150       
    if (data == 0x68) {
        CONSOLE.println("MPU6050/9150 found");
        imuFound = true;
        return;
    } else if (data == 0x72) {
        CONSOLE.println("MPU6052 found");
        imuFound = true;
        return;
    }
  #endif
  #if defined MPU9250 
    if (data == 0x73) {
        CONSOLE.println("MPU9255 found");
        imuFound = true;
        return;
    } else if (data == 0x71) {
        CONSOLE.println("MPU9250 found");
        imuFound = true;
        return;
    }
  #endif
  imuFound = false;
  CONSOLE.println(F("MPU6050/9150/9250/9255 not found - Did you connect AD0 to 3.3v and choose it in config.h?"));          
}


bool MpuDriver::begin(){ 
    CONSOLE.println("using imu driver: MpuDriver");
    //selectChip();
    if (mpu.begin() != INV_SUCCESS){
        return false;
    }
    //mpu.setAccelFSR(2);	      
    mpu.dmpBegin(DMP_FEATURE_6X_LP_QUAT  // Enable 6-axis quat
               |  DMP_FEATURE_GYRO_CAL // Use gyro calibration
             //  | DMP_FEATURE_SEND_RAW_ACCEL
              , IMU_FIFO_RATE); // Set DMP FIFO rate
    // DMP_FEATURE_LP_QUAT can also be used. It uses the 
    // accelerometer in low-power mode to estimate quat's.
    // DMP_FEATURE_LP_QUAT and 6X_LP_QUAT are mutually exclusive    
    //mpu.dmpSetOrientation(orientationMatrix);
    return true;
}


void MpuDriver::run(){
}


bool MpuDriver::isDataAvail(){
    //selectChip();
    bool avail = (mpu.fifoAvailable() > 0);    
    if (!avail) return false;
    //CONSOLE.println("fifoAvailable");
    // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
    if ( mpu.dmpUpdateFifo() != INV_SUCCESS) return false;
    // computeEulerAngles can be used -- after updating the
    // quaternion values -- to estimate roll, pitch, and yaw
    //  toEulerianAngle(imu.calcQuat(imu.qw), imu.calcQuat(imu.qx), imu.calcQuat(imu.qy), imu.calcQuat(imu.qz), imu.roll, imu.pitch, imu.yaw);
    quatW = mpu.qw;
    quatX = mpu.qx;
    quatY = mpu.qy;
    quatZ = mpu.qz;
    mpu.computeEulerAngles(false);      
    //CONSOLE.print(imu.ax);
    //CONSOLE.print(",");
    //CONSOLE.print(imu.ay);
    //CONSOLE.print(",");
    //CONSOLE.println(imu.az);
    roll = mpu.roll;
    pitch = mpu.pitch;
    yaw = mpu.yaw;    
    return true;
}         
    
void MpuDriver::resetData(){
    //selectChip();
    mpu.resetFifo();
}




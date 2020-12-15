// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

// motor driver base

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H



class MotorDriver {
  public:    
    virtual void begin();
    virtual void run();
    
    // set pwm (0-255), positive: forward, negative: backwards
    virtual void setMotorPwm(int leftPwm, int rightPwm, int mowPwm);
    // get motor faults
    virtual void getMotorFaults(bool &leftFault, bool &rightFault, bool &mowFault);
    // reset motor faults
    virtual void resetMotorFaults();
    // get motor currents (ampere)
    virtual void getMotorCurrent(float &leftCurrent, float &rightCurrent, float &mowCurrent);
    // get motor encoder ticks
    virtual void getMotorEncoderTicks(int &leftTicks, int &rightTicks, int &mowTicks); 
};


#endif

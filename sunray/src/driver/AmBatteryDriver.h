// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

// Ardumower battery driver

#ifndef AM_BATTERY_DRIVER_H
#define AM_BATTERY_DRIVER_H


#include "BatteryDriver.h"


class AmBatteryDriver : public BatteryDriver {
  public:    
    void begin() override;
    void run() override;
    
    // read battery voltage
    float getBatteryVoltage() override;
    // read charge voltage
    float getChargeVoltage() override;
    // read charge current
    float getChargeCurrent() override;    
    // enable battery charging
    virtual void enableCharging(bool flag) override;
    // keep system on or power-off
    virtual void keepPowerOn(bool flag) override;
  protected:
  	float batteryFactor;
    float currentFactor;

};


#endif

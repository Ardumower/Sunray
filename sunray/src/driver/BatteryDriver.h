// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

// battery driver base

#ifndef BATTERY_DRIVER_H
#define BATTERY_DRIVER_H



class BatteryDriver {
  public:    
    virtual void begin();
    virtual void run();
    
    // read battery voltage
    virtual float getBatteryVoltage();
    // read charge voltage
    virtual float getChargeVoltage();
    // read charge current (amps)
    virtual float getChargeCurrent();
    // enable battery charging
    virtual void enableCharging(bool flag);
    // keep system on or power-off
    virtual void keepPowerOn(bool flag);  	  		    
};


#endif

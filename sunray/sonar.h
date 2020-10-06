// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

// HC-SR04 ultrasonic sensor driver (2cm - 400cm)
// for 3 sensors, optimized for speed: based on hardware interrupts (no polling)
// up to 100 Hz measurements tested

#ifndef SONAR_H
#define SONAR_H



class Sonar {
    public:      
		bool enabled;
	    int triggerLeftBelow;
        int triggerCenterBelow;
        int triggerRightBelow;
	    void begin();            
        void run();
	    bool obstacle();	    
		bool nearObstacle();
		unsigned int distanceLeft; // cm
		unsigned int distanceRight;
		unsigned int distanceCenter;  		
		bool verboseOutput; 
    protected:                 
		unsigned int convertCm(unsigned int echoTime);
		unsigned long nearObstacleTimeout;
};



#endif


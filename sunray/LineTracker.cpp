// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)


#include "LineTracker.h"
#include "robot.h"
#include "StateEstimator.h"
#include "helper.h"
#include "pid.h"
#include "src/op/op.h"


//PID pidLine(0.2, 0.01, 0); // not used
//PID pidAngle(2, 0.1, 0);  // not used

float stanleyTrackingNormalK = STANLEY_CONTROL_K_NORMAL;
float stanleyTrackingNormalP = STANLEY_CONTROL_P_NORMAL;    
float stanleyTrackingSlowK = STANLEY_CONTROL_K_SLOW;
float stanleyTrackingSlowP = STANLEY_CONTROL_P_SLOW;    

float setSpeed = 0.1; // linear speed (m/s)

bool rotateLeft = false;
bool rotateRight = false;
bool angleToTargetFits = false;
bool targetReached = false;
float trackerDiffDelta = 0;
bool stateKidnapped = false;

int dockGpsRebootState;                   // Svol0: status for gps-reboot at specified docking point by undocking action
bool blockKidnapByUndocking;              // Svol0: kidnap detection is blocked by undocking without gps
unsigned long dockGpsRebootTime;          // Svol0: retry timer for gps-fix after gps-reboot
unsigned long dockGpsRebootFixCounter;    // Svol0: waitingtime for fix after gps-reboot
unsigned long dockGpsRebootFeedbackTimer; // Svol0: timer to generate acustic feedback
bool dockGpsRebootDistGpsTrg = false;     // Svol0: trigger to check solid gps-fix position (no jump)
bool allowDockLastPointWithoutGPS = false;  // Svol0: allow go on docking by loosing gps fix
bool warnDockWithoutGpsTrg = false;            // Svol0: Trigger for warnmessage

// control robot velocity (linear,angular) to track line to next waypoint (target)
// uses a stanley controller for line tracking
// https://medium.com/@dingyan7361/three-methods-of-vehicle-lateral-control-pure-pursuit-stanley-and-mpc-db8cc1d32081
void trackLine(bool runControl){  
  Point target = maps.targetPoint;
  Point lastTarget = maps.lastTargetPoint;
  float linear = 1.0;  
  bool mow = true;
  if (stateOp == OP_DOCK) mow = false;
  float angular = 0;      
  float targetDelta = pointsAngle(stateX, stateY, target.x(), target.y());      
  if (maps.trackReverse) targetDelta = scalePI(targetDelta + PI);
  targetDelta = scalePIangles(targetDelta, stateDelta);
  trackerDiffDelta = distancePI(stateDelta, targetDelta);                         
  lateralError = distanceLineInfinite(stateX, stateY, lastTarget.x(), lastTarget.y(), target.x(), target.y());        
  float distToPath = distanceLine(stateX, stateY, lastTarget.x(), lastTarget.y(), target.x(), target.y());        
  float targetDist = maps.distanceToTargetPoint(stateX, stateY);
  
  float lastTargetDist = maps.distanceToLastTargetPoint(stateX, stateY);  

  // limitation for setSpeed //SOew
  if (setSpeed > MOTOR_MAX_SPEED) setSpeed = MOTOR_MAX_SPEED;

  if (SMOOTH_CURVES)
    targetReached = (targetDist < 0.2);    
  else 
    targetReached = (targetDist < TARGET_REACHED_TOLERANCE);    
  
  
  if ( (motor.motorLeftOverload) || (motor.motorRightOverload) || (motor.motorMowOverload) ){
    linear = OVERLOADSPEED; // see config.h  
  }   
          
  // allow rotations only near last or next waypoint or if too far away from path
  if ( (targetDist < 0.5) || (lastTargetDist < 0.5) ||  (fabs(distToPath) > 0.5) ) {
    if (SMOOTH_CURVES)
      angleToTargetFits = (fabs(trackerDiffDelta)/PI*180.0 < 120);          
    else     
      angleToTargetFits = (fabs(trackerDiffDelta)/PI*180.0 < 20);   
  } else angleToTargetFits = true;

               
  if (!angleToTargetFits){
    // angular control (if angle to far away, rotate to next waypoint)
    linear = 0;
    //angular = 29.0 / 180.0 * PI; //  29 degree/s (0.5 rad/s);

    // different angular speed by docking and undocking action
    // Svol0: only use "DOCKANGULARSPEED" by docking and undocking if "trackSlow" is activ
    if (((maps.isDocking()) || (maps.isUndocking())) && (maps.trackSlow)) angular = DOCKANGULARSPEED;
    else angular = ROTATETOTARGETSPEED;    
    
    if ((!rotateLeft) && (!rotateRight)){ // decide for one rotation direction (and keep it)
      if (trackerDiffDelta < 0) rotateLeft = true;
        else rotateRight = true;
    }        
    if (rotateLeft) angular *= -1;            
    if (fabs(trackerDiffDelta)/PI*180.0 < 90){
      rotateLeft = false;  // reset rotate direction
      rotateRight = false;
    }    
  } 
  else {
    // line control (stanley)    
    bool straight = maps.nextPointIsStraight();

    #if USE_LINEAR_SPEED_RAMP
      // linear speed ramp needs more distance to stop at high speeds
      const float closeToTargetLimit = 1.0 * setSpeed;
    #else
      const float closeToTargetLimit = 0.25;
    #endif
        
    if (maps.trackSlow) {
      // planner forces slow tracking (e.g. docking etc)
      linear = TRACKSLOWSPEED;  // see config.h           
    } else if (     ((setSpeed > 0.2) && (maps.distanceToTargetPoint(stateX, stateY) < closeToTargetLimit) && (!straight))   // approaching
          || ((linearMotionStartTime != 0) && (millis() < linearMotionStartTime + 1000))                      // leaving  
       ) 
    {
      linear = APPROACHWAYPOINTSPEED; // reduce speed when approaching/leaving waypoints          
    } 
    else {
      if (gps.solution == SOL_FLOAT)        
        linear = min(setSpeed, FLOATSPEED); // reduce speed for float solution
      else
        linear = setSpeed;         // desired speed
      if (sonar.nearObstacle()) linear = SONARSPEED; // slow down near obstacles
    }      
    //angula                                    r = 3.0 * trackerDiffDelta + 3.0 * lateralError;       // correct for path errors 
    
    //Mapping of Stanley Control Parameters in relation to actual Setpoint value of speed
    //Values need to be multiplied, because map() function does not work well with small range decimals
    float CurrSpeed = motor.linearSpeedSet * 1000;                                                    
    CurrSpeed = abs(CurrSpeed);
    float k = map(CurrSpeed, MOTOR_MIN_SPEED*1000, MOTOR_MAX_SPEED*1000, STANLEY_CONTROL_K_SLOW*1000, STANLEY_CONTROL_K_NORMAL*1000);  //MOTOR_MIN_SPEED and MOTOR_MAX_SPEED from config.h
    float p = map(CurrSpeed, MOTOR_MIN_SPEED*1000, MOTOR_MAX_SPEED*1000, STANLEY_CONTROL_P_SLOW*1000, STANLEY_CONTROL_P_NORMAL*1000);  //MOTOR_MIN_SPEED and MOTOR_MAX_SPEED from config.h
    k = k / 1000;                                                                                     
    p = p / 1000;                                                
    
    angular =  p * trackerDiffDelta + atan2(k * lateralError, (0.001 + fabs(motor.linearSpeedSet)));       // correct for path errors           
    /*pidLine.w = 0;              
    pidLine.x = lateralError;
    pidLine.max_output = PI;
    pidLine.y_min = -PI;
    pidLine.y_max = PI;
    pidLine.compute();
    angular = -pidLine.y;   */
    //CONSOLE.print(lateralError);        
    //CONSOLE.print(",");        
    //CONSOLE.println(angular/PI*180.0);            
    if (maps.trackReverse) linear *= -1;   // reverse line tracking needs negative speed
    if (!SMOOTH_CURVES) angular = max(-PI/16, min(PI/16, angular)); // restrict steering angle for stanley
  }
  // check some pre-conditions that can make linear+angular speed zero
  if (fixTimeout != 0){
    if (millis() > lastFixTime + fixTimeout * 1000.0){
      activeOp->onGpsFixTimeout();        
    }       
  }     

  if ((gps.solution == SOL_FIXED) || (gps.solution == SOL_FLOAT)){        
    warnDockWithoutGpsTrg = false;    // Svol0: reset warnmessage trigger
    if (abs(linear) > 0.06) {
      if ((millis() > linearMotionStartTime + 5000) && (stateGroundSpeed < 0.03)){
        // if in linear motion and not enough ground speed => obstacle
        //if ( (GPS_SPEED_DETECTION) && (!maps.isUndocking()) ) { 
        if (GPS_SPEED_DETECTION) {         
          CONSOLE.println("gps no speed => obstacle!");
          triggerObstacle();
          return;
        }
      }
    }  
  } else {
    // no gps solution
    if (REQUIRE_VALID_GPS){
      // Svol0: continue docking if gps solution gets lost by driving to the last point (normal if dockingstation is under a roof)
      if (allowDockLastPointWithoutGPS == true){
        if (!warnDockWithoutGpsTrg){
          CONSOLE.println("WARN: Continue docking with no gps solution!");
          warnDockWithoutGpsTrg = true;
        }
      } else {
        CONSOLE.println("WARN: no gps solution!");
        activeOp->onGpsNoSignal();
      }
    }
  }

  // gps-jump/false fix check
  if (KIDNAP_DETECT){
    float allowedPathTolerance = KIDNAP_DETECT_ALLOWED_PATH_TOLERANCE;     
    if ( maps.isUndocking() ) allowedPathTolerance = 0.2;
//    if (fabs(distToPath) > allowedPathTolerance){ // actually, this should not happen (except on false GPS fixes or robot being kidnapped...)
    // Svol0: changed for GPS-Reboot at a
    if ((fabs(distToPath) > allowedPathTolerance) && (!blockKidnapByUndocking)){ // actually, this should not happen (except on false GPS fixes or robot being kidnapped...)
      if (!stateKidnapped){
        stateKidnapped = true;
        activeOp->onKidnapped(stateKidnapped);
      }            
    } else {
      if (stateKidnapped) {
        stateKidnapped = false;
        activeOp->onKidnapped(stateKidnapped);        
      }
    }
  }

  // reboot gps by undocking at a specified docking point (please see "DOCK_POINT_GPS_REBOOT" in config.h) //SOew
  if (dockGpsRebootState > 0){   // status dockGpsReboot: 0= off, 1= reset gps, 2= wait for gps-fix, 3= check for stable gps-fix
    switch (dockGpsRebootState){
      
      case 1:
        // reboot gps to get new GPS fix
        gps.reboot();   // reboot gps to get new GPS fix
        dockGpsRebootTime = millis() + 5000; // load check timer for gps-fix with 5sec
        dockGpsRebootFixCounter = 0;
        dockGpsRebootState = 2;
        CONSOLE.println("robot.cpp  dockGpsRebootState - start gps-reboot");
        break;

      case 2:
        // wait for gps-fix solution
        if (dockGpsRebootTime <= millis()){
          if (gps.solution == SOL_FIXED){
            maps.setLastTargetPoint(stateX, stateY);  // Manipulate last target point to avoid "KIDNAP DETECT"
            dockGpsRebootState = 3;
            dockGpsRebootFeedbackTimer  = millis();
            dockGpsRebootTime = millis(); // load check timer for stable gps-fix
            dockGpsRebootDistGpsTrg = false; // reset trigger
            CONSOLE.print("robot.cpp  dockGpsRebootState - got gps-fix after ");
            CONSOLE.print(dockGpsRebootFixCounter);
            CONSOLE.println(" sec");     
          }
          else {
            dockGpsRebootTime += 5000; // load check timer for gps-fix with 5sec
            dockGpsRebootFixCounter += 5;  // add 5 seconds
            if (!buzzer.isPlaying()) buzzer.sound(SND_TILT, true);
            CONSOLE.print("robot.cpp  dockGpsRebootState - still no gps-fix after ");
            CONSOLE.print(dockGpsRebootFixCounter);
            CONSOLE.println(" sec");     
          }
        }
        break;

      case 3:
        // wait if gps-fix position stays stable for at least 20sec
        if ((gps.solution == SOL_FIXED) && (millis() - dockGpsRebootTime > 20000)){
          dockGpsRebootState      = 0; // finished
          blockKidnapByUndocking  = false;  // enable Kidnap detection
          CONSOLE.println("robot.cpp  dockGpsRebootState - gps-pos is stable; continue undocking;");
        }
        if (gps.solution != SOL_FIXED) dockGpsRebootState = 2; // wait for gps-fix again
        if (dockGpsRebootDistGpsTrg == true){ // gps position is changing to much
          dockGpsRebootDistGpsTrg = false; // reset trigger
          dockGpsRebootTime = millis();
          CONSOLE.print("robot.cpp  dockGpsRebootState - gps-pos is moving; timereset after");
          CONSOLE.print((millis() - dockGpsRebootTime));
          CONSOLE.println("msec");
          if (!buzzer.isPlaying()) buzzer.sound(SND_ERROR, true);               
        }
        if (dockGpsRebootFeedbackTimer <= millis()){
          dockGpsRebootFeedbackTimer = millis() + 5000;
          if (!buzzer.isPlaying()) buzzer.sound(SND_READY, true);
        }
        break;
        
      case 10:
        // Wenn ohne GPS fix oder float das undocking gestartet wird, muss bei erreichen von fix oder float die "linearMotionStartTime" resetet werden, um "gps no speed => obstacle!" zu vermeiden
        resetLinearMotionMeasurement();
        break;
        
    } // switch (dockGpsRebootState)
    if (dockGpsRebootState < 10) {
      // stop mower
      linear = 0;
      angular = 0;        
      mow = false;
    }
  } //if (dockGpsRebootState > 0)
     
  if (mow)  {  // wait until mowing motor is running
    if (millis() < motor.motorMowSpinUpTime + MOW_SPINUPTIME){  // see config.h -> "MOW_SPINUPTIME"
      if (!buzzer.isPlaying()) buzzer.sound(SND_WARNING, true);
      linear = 0;
      angular = 0;   
    }
  }

  if (runControl){
    motor.setLinearAngularSpeed(linear, angular);      
    if (detectLift()) mow = false; // in any case, turn off mower motor if lifted 
    motor.setMowState(mow);    
  }

  if (targetReached){
    if (maps.wayMode == WAY_MOW){
      maps.clearObstacles(); // clear obstacles if target reached
      motorErrorCounter = 0; // reset motor error counter if target reached
      stateSensor = SENS_NONE; // clear last triggered sensor
    }
    bool straight = maps.nextPointIsStraight();
    if (!maps.nextPoint(false)){
      // finish        
      activeOp->onNoFurtherWaypoints();      
    } else {      
      // next waypoint          
      //if (!straight) angleToTargetFits = false;      
    }
  }  
}

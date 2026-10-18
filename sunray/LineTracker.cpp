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
#include "Stats.h"
#include "events.h"

#ifdef DOCK_REFLECTOR_STRIPS
extern "C" bool cameraReflectorStripsFound();
extern "C" float cameraReflectorStripsHorizontalError();
extern "C" float cameraReflectorStripsConfidence();
extern "C" int cameraReflectorStripsStripCount();
extern "C" unsigned long cameraReflectorStripsAgeMs();
#endif

//PID pidLine(0.2, 0.01, 0); // not used
//PID pidAngle(2, 0.1, 0);  // not used
static Polygon circle(8);

// control robot velocity (linear,angular) to track line to next waypoint (target)
// uses a stanley controller for line tracking
// https://medium.com/@dingyan7361/three-methods-of-vehicle-lateral-control-pure-pursuit-stanley-and-mpc-db8cc1d32081
void LineTracker::trackLine(bool runControl){  
  Point target = maps.targetPoint;
  Point lastTarget = maps.lastTargetPoint;
  float linear = 1.0;  
  bool mow = true;
  if (stateEstimator.stateOp == OP_DOCK) mow = false;
  float angular = 0;      
  float targetDelta = pointsAngle(stateEstimator.stateX, stateEstimator.stateY, target.x(), target.y());      
  if (maps.trackReverse) targetDelta = scalePI(targetDelta + PI);  
  targetDelta = scalePIangles(targetDelta, stateEstimator.stateDelta);
  trackerDiffDelta = distancePI(stateEstimator.stateDelta, targetDelta);                         
  stateEstimator.lateralError = distanceLineInfinite(stateEstimator.stateX, stateEstimator.stateY, lastTarget.x(), lastTarget.y(), target.x(), target.y());        
  float distToPath = distanceLine(stateEstimator.stateX, stateEstimator.stateY, lastTarget.x(), lastTarget.y(), target.x(), target.y());        

  float lineDist = maps.distanceToTargetPoint(lastTarget.x(), lastTarget.y());
  /*if ((abs(lineDist-lastLineDist ) > 0.0) || (abs(distToPath) > 0.5)) {
    CONSOLE.print("distToPath=");
    CONSOLE.print(distToPath);
    CONSOLE.print(" x=");
    CONSOLE.print(stateEstimator.stateX);
    CONSOLE.print(" y=");    
    CONSOLE.print(stateEstimator.stateY);
    CONSOLE.print(" lastX=");    
    CONSOLE.print(lastTarget.x());
    CONSOLE.print(" lastY=");    
    CONSOLE.print(lastTarget.y());
    CONSOLE.print(" tgX=");    
    CONSOLE.print(target.x());
    CONSOLE.print(" tgY=");    
    CONSOLE.println(target.y());
    lastLineDist = lineDist;
  }*/
  float targetDist = maps.distanceToTargetPoint(stateEstimator.stateX, stateEstimator.stateY);
  
  float lastTargetDist = maps.distanceToLastTargetPoint(stateEstimator.stateX, stateEstimator.stateY);  
  if (SMOOTH_CURVES)
    targetReached = (targetDist < 0.2);    
  else 
    targetReached = (targetDist < TARGET_REACHED_TOLERANCE);

  
  // allow rotations only near last or next waypoint or if too far away from path
  // it might race between rotating mower and targetDist check below
  // if we race we still have rotateLeft or rotateRight true
  if ( (targetDist < 0.5) || (lastTargetDist < 0.5) || (fabs(distToPath) > 0.5) ||
       rotateLeft || rotateRight ) {
    if (SMOOTH_CURVES)
      angleToTargetFits = (fabs(trackerDiffDelta)/PI*180.0 < 120);
    else     
      angleToTargetFits = (fabs(trackerDiffDelta)/PI*180.0 < TARGET_ANGLE_TOLERANCE);
  } else {
    // while tracking the mowing line do allow rotations if angle to target increases (e.g. due to gps jumps)
    angleToTargetFits = (fabs(trackerDiffDelta)/PI*180.0 < 45);       
    //angleToTargetFits = true;
  }

  //if (!angleToTargetFits) CONSOLE.println("!angleToTargetFits");

  if (!angleToTargetFits){
    // angular control (if angle to far away, rotate to next waypoint)
    linear = 0;
    angular = 29.0 / 180.0 * PI; //  29 degree/s (0.5 rad/s);               
     // decide for one rotation direction (and keep it)
    if ((!rotateLeft) && (!rotateRight)) {
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
    bool trackslow_allowed = true;
    bool rtkDockApproach = false;
    float desiredSpeed = stateEstimator.setSpeed;
#ifdef DOCK_RTK_SPEED
    rtkDockApproach = stateEstimator.stateOp == OP_DOCK && stateEstimator.stateLocalizationMode == LOC_GPS;
    if (rtkDockApproach) desiredSpeed = DOCK_RTK_SPEED;
#endif

    rotateLeft = false;
    rotateRight = false;

    // in case of docking or undocking - check if trackslow is allowed
    if ( maps.isUndocking() || maps.isDocking() ) {
        float dockX = 0;
        float dockY = 0;
        float dockDelta = 0;
        maps.getDockingPos(dockX, dockY, dockDelta);
        float dist_dock = distance(dockX, dockY, stateEstimator.stateX, stateEstimator.stateY);
        // only allow trackslow if we are near dock (below DOCK_UNDOCK_TRACKSLOW_DISTANCE)
        if (dist_dock > DOCK_UNDOCK_TRACKSLOW_DISTANCE) {
            trackslow_allowed = false;
        }
    }

    if (maps.trackSlow && trackslow_allowed && !rtkDockApproach) {
      // planner forces slow tracking (e.g. docking etc)
      linear = DOCK_LINEAR_SPEED; // 0.1           
    } else if (     ((desiredSpeed > 0.2) && (maps.distanceToTargetPoint(stateEstimator.stateX, stateEstimator.stateY) < 0.5) && (!straight))   // approaching
          || ((stateEstimator.linearMotionStartTime != 0) && (millis() < stateEstimator.linearMotionStartTime + 3000))                      // leaving  
       ) 
    {
      linear = 0.1; // reduce speed when approaching/leaving waypoints          
      //CONSOLE.println("SLOW: approach")
    } 
    else {
      if ((stateEstimator.stateLocalizationMode == LOC_GPS) && (gps.solution == SOL_FLOAT)){        
        linear = min(desiredSpeed, 0.1); // reduce speed for float solution
        //CONSOLE.println("SLOW: float");
      } else
        linear = desiredSpeed;         // desired speed
      #ifdef DRV_CAN_ROBOT
        CanRobotDriver::UltrasonicMotionDirection ultrasonicDirection = CanRobotDriver::ultrasonic_motion_any;
        if (fabs(linear) > 0.001) {
          ultrasonicDirection = maps.trackReverse ? CanRobotDriver::ultrasonic_motion_reverse : CanRobotDriver::ultrasonic_motion_forward;
        }
        robotDriver.setUltrasonicMotionDirectionHint(ultrasonicDirection);
      #endif
      if (bumperDriver.nearObstacle()){
        linear = 0.1;  // slow down near obstacles 
        //CONSOLE.println("SLOW: BUMPER");      
      }

      if (sonar.nearObstacle()) {
        linear = 0.1; // slow down near obstacles
        //CONSOLE.println("SLOW: sonar");      
      }
    }      
    // slow down speed in case of overload and overwrite all prior speed 
    if ( (motor.motorLeftOverload) || (motor.motorRightOverload) || (motor.motorMowOverload) ){
      if (!printmotoroverload) {
          Logger.event(EVT_MOTOR_OVERLOAD_REDUCE_SPEED);
          CONSOLE.println("motor overload detected: reducing linear speed");
      }
      printmotoroverload = true;
      linear = min(linear, MOTOR_OVERLOAD_SPEED);  
      //CONSOLE.println("SLOW: overload");
    } else {
      printmotoroverload = false;
    }   
          
    //angula                                    r = 3.0 * trackerDiffDelta + 3.0 * lateralError;       // correct for path errors 
    float k = stanleyTrackingNormalK; // STANLEY_CONTROL_K_NORMAL;
    float p = stanleyTrackingNormalP; // STANLEY_CONTROL_P_NORMAL;    
    if (maps.trackSlow && trackslow_allowed && !rtkDockApproach) {
      k = stanleyTrackingSlowK; //STANLEY_CONTROL_K_SLOW;   
      p = stanleyTrackingSlowP; //STANLEY_CONTROL_P_SLOW;          
    }
    angular =  p * trackerDiffDelta + atan2(k * stateEstimator.lateralError, (0.001 + fabs(motor.linearSpeedSet)));       // correct for path errors           
    /*pidLine.w = 0;              
    pidLine.x = stateEstimator.lateralError;
    pidLine.max_output = PI;
    pidLine.y_min = -PI;
    pidLine.y_max = PI;
    pidLine.compute();
    angular = -pidLine.y;   */
    //CONSOLE.print(stateEstimator.lateralError);        
    //CONSOLE.print(",");        
    //CONSOLE.println(angular/PI*180.0);            
    if (maps.trackReverse) linear *= -1;   // reverse line tracking needs negative speed
    // restrict steering angle for stanley  (not required anymore after last state estimation bugfix)
    //if (!SMOOTH_CURVES) angular = max(-PI/16, min(PI/16, angular)); 
  }
  // check some pre-conditions that can make linear+angular speed zero
  if ((stateEstimator.stateLocalizationMode == LOC_GPS) && (stateEstimator.fixTimeout != 0)){
    if (millis() > stateEstimator.lastFixTime + stateEstimator.fixTimeout * 1000.0){
      activeOp->onGpsFixTimeout();        
    }           
  }     

  if (stateEstimator.stateLocalizationMode == LOC_GPS){
    if  ((gps.solution == SOL_FIXED) || (gps.solution == SOL_FLOAT)){        
      if (abs(linear) > 0.06) {
        if ((millis() > stateEstimator.linearMotionStartTime + 5000) && (stateEstimator.stateGroundSpeed < 0.03)){
          // if in linear motion and not enough ground speed => obstacle
          //if ( (GPS_SPEED_DETECTION) && (!maps.isUndocking()) ) { 
          if (GPS_SPEED_DETECTION) {         
            CONSOLE.println("gps no speed => obstacle!");
            stats.statMowGPSNoSpeedCounter++;
            Logger.event(EVT_NO_GPS_SPEED_OBSTACLE);
            triggerObstacle();
            return;
          }
        }
      }  
    } else {
      // no gps solution
      if (REQUIRE_VALID_GPS){
        CONSOLE.println("WARN: no gps solution!");
        activeOp->onGpsNoSignal();
      }
    }
  }

  if (stateEstimator.stateLocalizationMode == LOC_GUIDANCE_SHEET){
      if (!buzzer.isPlaying()) buzzer.sound(SND_WARNING, true);
      angular = 0;
  }
  #ifdef DOCK_REFLECTOR_STRIPS
  if (stateEstimator.stateLocalizationMode == LOC_REFLECTOR_STRIPS){
    mow = false;
    const bool reflectorFound = cameraReflectorStripsFound();
    const float imageError = cameraReflectorStripsHorizontalError();
    const float confidence = cameraReflectorStripsConfidence();
    if (!reflectorFound || !stateEstimator.reflectorDockingGate.mayDrive()) {
      if (runControl) motor.stopImmediately(false);
      // Never continue on stale pixels. The detector applies the configured
      // timeout, so a short camera hiccup causes a controlled stop.
      linear = 0;
      angular = 0;
    } else {
      if (!buzzer.isPlaying()) buzzer.sound(SND_WARNING, true);
      angular = -DOCK_REFLECTOR_STRIPS_ANGULAR_P * imageError;
      angular = max(-DOCK_REFLECTOR_STRIPS_MAX_ANGULAR,
                    min(DOCK_REFLECTOR_STRIPS_MAX_ANGULAR, angular));
      const float absError = fabs(imageError);
      if (absError > DOCK_REFLECTOR_STRIPS_MAX_ERROR_FOR_DRIVE) {
        linear = 0; // rotate until the strip is safely in the field of view
      } else {
        const float speedScale = max(0.35f, 1.0f - absError);
        linear = abs(DOCK_LINEAR_SPEED) * speedScale;
        // Keep the camera facing the marker throughout the whole dock path.
        const bool reverse = maps.isUndocking() ? DOCK_FRONT_SIDE : !DOCK_FRONT_SIDE;
        if (reverse) linear *= -1;
      }
    }

    static unsigned long nextReflectorStatusTime = 0;
    if (millis() >= nextReflectorStatusTime) {
      nextReflectorStatusTime = millis() + 1000;
      CONSOLE.print("dock reflectors: found="); CONSOLE.print(reflectorFound);
      CONSOLE.print(" strips="); CONSOLE.print(cameraReflectorStripsStripCount());
      CONSOLE.print(" confidence="); CONSOLE.print(confidence);
      CONSOLE.print(" error="); CONSOLE.print(imageError);
      CONSOLE.print(" age="); CONSOLE.print(cameraReflectorStripsAgeMs());
      CONSOLE.print(" gpsPos="); CONSOLE.print((int)stateEstimator.cameraGpsPosition.activeQuality(millis()));
      CONSOLE.print(" lin="); CONSOLE.print(linear);
      CONSOLE.print(" ang="); CONSOLE.println(angular);
    }
  }
  #endif

  // gps-jump/false fix check
  if (KIDNAP_DETECT){
    float allowedPathTolerance = KIDNAP_DETECT_ALLOWED_PATH_TOLERANCE;     
    if ( maps.isUndocking() || maps.isDocking() ) {
        float dockX = 0;
        float dockY = 0;
        float dockDelta = 0;
        maps.getDockingPos(dockX, dockY, dockDelta);
        float dist = distance(dockX, dockY, stateEstimator.stateX, stateEstimator.stateY);
        // check if current distance to docking station is below
        // KIDNAP_DETECT_DISTANCE_DOCK_UNDOCK to trigger KIDNAP_DETECT_ALLOWED_PATH_TOLERANCE_DOCK_UNDOCK
        if (dist < KIDNAP_DETECT_DISTANCE_DOCK_UNDOCK) {
            allowedPathTolerance = KIDNAP_DETECT_ALLOWED_PATH_TOLERANCE_DOCK_UNDOCK;
        }
    }    
    if ((stateEstimator.stateLocalizationMode == LOC_GPS) && (fabs(distToPath) > allowedPathTolerance)){ // actually, this should not happen (except on false GPS fixes or robot being kidnapped...)
      if (!stateKidnapped){
        stateKidnapped = true;
        CONSOLE.print("KIDNAP_DETECT: stateKidnapped=");
        CONSOLE.print(stateKidnapped);
        CONSOLE.print(" distToPath=");
        CONSOLE.println(distToPath);
        activeOp->onKidnapped(stateKidnapped);
      }            
    } else {
      if (stateKidnapped) {
        stateKidnapped = false;
        CONSOLE.print("KIDNAP_DETECT: stateKidnapped=");
        CONSOLE.print(stateKidnapped);
        CONSOLE.print(" distToPath=");
        CONSOLE.println(distToPath);
        activeOp->onKidnapped(stateKidnapped);        
      }
    }
  }
   
  // in any case, turn off mower motor if lifted 
  // also, if lifted, do not turn on mowing motor so that the robot will drive and can do obstacle avoidance 
  if (detectLift()) mow = false;
  
  if (mow)  { 
    if (millis() < motor.motorMowSpinUpTime + 10000){
       // wait until mowing motor is running
      if (!buzzer.isPlaying()) buzzer.sound(SND_WARNING, true);
      linear = 0;
      angular = 0;          
    }
  }

  if (runControl){
    if (angleToTargetFits != langleToTargetFits) {
        //CONSOLE.print("angleToTargetFits: ");
        //CONSOLE.print(angleToTargetFits);
        //CONSOLE.print(" trackerDiffDelta: ");
        //CONSOLE.println(trackerDiffDelta);
        langleToTargetFits = angleToTargetFits;
    }


    motor.setLinearAngularSpeed(linear, angular);      
    motor.setMowState(mow);    
  }

  //if (!maps.isTargetingLastDockPoint()){
  if (stateEstimator.stateLocalizationMode != LOC_REFLECTOR_STRIPS){
    if (targetReached){
      rotateLeft = false;
      rotateRight = false;
      activeOp->onTargetReached();
      bool straight = maps.nextPointIsStraight();
      if (!maps.nextPoint(false,stateEstimator.stateX,stateEstimator.stateY)){
        // finish        
        activeOp->onNoFurtherWaypoints();      
      } else {      
        // next waypoint          
        //if (!straight) angleToTargetFits = false;      
      }
    }
  }  
}

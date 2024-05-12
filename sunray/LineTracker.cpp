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


//PID pidLine(0.2, 0.01, 0); // not used
//PID pidAngle(2, 0.1, 0);  // not used
Polygon circle(8);

float stanleyTrackingNormalK = STANLEY_CONTROL_K_NORMAL;
float stanleyTrackingNormalP = STANLEY_CONTROL_P_NORMAL;    
float stanleyTrackingSlowK = STANLEY_CONTROL_K_SLOW;
float stanleyTrackingSlowP = STANLEY_CONTROL_P_SLOW;    

float setSpeed = 0.1; // linear speed (m/s)
bool rotateLeft = false;
bool rotateRight = false;
bool angleToTargetFits = false;
bool langleToTargetFits = false;
bool targetReached = false;
float trackerDiffDelta = 0;
bool stateKidnapped = false;
bool printmotoroverload = false;
bool trackerDiffDelta_positive = false;


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
      angleToTargetFits = (fabs(trackerDiffDelta)/PI*180.0 < 20);
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

    rotateLeft = false;
    rotateRight = false;

    // in case of docking or undocking - check if trackslow is allowed
    if ( maps.isUndocking() || maps.isDocking() ) {
        float dockX = 0;
        float dockY = 0;
        float dockDelta = 0;
        maps.getDockingPos(dockX, dockY, dockDelta);
        float dist_dock = distance(dockX, dockY, stateX, stateY);
        // only allow trackslow if we are near dock (below DOCK_UNDOCK_TRACKSLOW_DISTANCE)
        if (dist_dock > DOCK_UNDOCK_TRACKSLOW_DISTANCE) {
            trackslow_allowed = false;
        }
    }

    if (maps.trackSlow && trackslow_allowed) {
      // planner forces slow tracking (e.g. docking etc)
      linear = 0.1;           
    } else if (     ((setSpeed > 0.2) && (maps.distanceToTargetPoint(stateX, stateY) < 0.5) && (!straight))   // approaching
          || ((linearMotionStartTime != 0) && (millis() < linearMotionStartTime + 3000))                      // leaving  
       ) 
    {
      linear = 0.1; // reduce speed when approaching/leaving waypoints          
      //CONSOLE.println("SLOW: approach")
    } 
    else {
      if (gps.solution == SOL_FLOAT){        
        linear = min(setSpeed, 0.1); // reduce speed for float solution
        //CONSOLE.println("SLOW: float");
      } else
        linear = setSpeed;         // desired speed
      if (sonar.nearObstacle()) {
        linear = 0.1; // slow down near obstacles
        //CONSOLE.println("SLOW: sonar");      
      }
    }      
    // slow down speed in case of overload and overwrite all prior speed 
    if ( (motor.motorLeftOverload) || (motor.motorRightOverload) || (motor.motorMowOverload) ){
      if (!printmotoroverload) {
          CONSOLE.println("motor overload detected: reduce linear speed to 0.1");
      }
      printmotoroverload = true;
      linear = 0.1;  
      //CONSOLE.println("SLOW: overload");
    } else {
      printmotoroverload = false;
    }   
          
    //angula                                    r = 3.0 * trackerDiffDelta + 3.0 * lateralError;       // correct for path errors 
    float k = stanleyTrackingNormalK; // STANLEY_CONTROL_K_NORMAL;
    float p = stanleyTrackingNormalP; // STANLEY_CONTROL_P_NORMAL;    
    if (maps.trackSlow && trackslow_allowed) {
      k = stanleyTrackingSlowK; //STANLEY_CONTROL_K_SLOW;   
      p = stanleyTrackingSlowP; //STANLEY_CONTROL_P_SLOW;          
    }
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
    // restrict steering angle for stanley  (not required anymore after last state estimation bugfix)
    //if (!SMOOTH_CURVES) angular = max(-PI/16, min(PI/16, angular)); 
  }
  // check some pre-conditions that can make linear+angular speed zero
  if (fixTimeout != 0){
    if (millis() > lastFixTime + fixTimeout * 1000.0){
      activeOp->onGpsFixTimeout();        
    }       
  }     

  if ((gps.solution == SOL_FIXED) || (gps.solution == SOL_FLOAT)){        
    if (abs(linear) > 0.06) {
      if ((millis() > linearMotionStartTime + 5000) && (stateGroundSpeed < 0.03)){
        // if in linear motion and not enough ground speed => obstacle
        //if ( (GPS_SPEED_DETECTION) && (!maps.isUndocking()) ) { 
        if (GPS_SPEED_DETECTION) {         
          CONSOLE.println("gps no speed => obstacle!");
          statMowGPSNoSpeedCounter++;
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

  // gps-jump/false fix check
  if (KIDNAP_DETECT){
    float allowedPathTolerance = KIDNAP_DETECT_ALLOWED_PATH_TOLERANCE;     
    if ( maps.isUndocking() || maps.isDocking() ) {
        float dockX = 0;
        float dockY = 0;
        float dockDelta = 0;
        maps.getDockingPos(dockX, dockY, dockDelta);
        float dist = distance(dockX, dockY, stateX, stateY);
        // check if current distance to docking station is below
        // KIDNAP_DETECT_DISTANCE_DOCK_UNDOCK to trigger KIDNAP_DETECT_ALLOWED_PATH_TOLERANCE_DOCK_UNDOCK
        if (dist < KIDNAP_DETECT_DISTANCE_DOCK_UNDOCK) {
            allowedPathTolerance = KIDNAP_DETECT_ALLOWED_PATH_TOLERANCE_DOCK_UNDOCK;
        }
    }
    if (fabs(distToPath) > allowedPathTolerance){ // actually, this should not happen (except on false GPS fixes or robot being kidnapped...)
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

  if (targetReached){
    rotateLeft = false;
    rotateRight = false;
    activeOp->onTargetReached();
    bool straight = maps.nextPointIsStraight();
    if (!maps.nextPoint(false,stateX,stateY)){
      // finish        
      activeOp->onNoFurtherWaypoints();      
    } else {      
      // next waypoint          
      //if (!straight) angleToTargetFits = false;      
    }
  }  
}



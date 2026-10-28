// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#ifdef __linux__
#include <CameraAvoidanceRuntime.h>
#endif
#include "op.h"
#include <Arduino.h>
#include "../../robot.h"
#include "../../StateEstimator.h"
#include "../../LineTracker.h"
#include "../../Stats.h"
#include "../../map.h"
#include "../../events.h"
#include "../navigation/DockMotionLimit.h"
#ifndef DOCK_MAX_WHEEL_SPEED
#define DOCK_MAX_WHEEL_SPEED 0.02f
#endif
#ifdef DOCK_REFLECTOR_STRIPS
extern "C" bool cameraReflectorStripsFound();
extern "C" float cameraReflectorStripsHorizontalError();
#endif

DockOp::DockOp(){
  lastMapRoutingFailed = false;
  mapRoutingFailedCounter = 0;
  dockReasonRainTriggered = false;
  dockReasonRainAutoStartTime = 0;
  dockContactAdvanceActive = false;
  dockContactAdvanceStartLeftTicks = 0;
  dockContactAdvanceStartRightTicks = 0;
  dockContactAdvanceStopTime = 0;
}


String DockOp::name(){
    return "Dock";
}


bool DockOp::cameraGuidanceActive() const {
#ifdef DOCK_REFLECTOR_STRIPS
  return stateEstimator.reflectorDockingGate.active() ||
      stateEstimator.stateLocalizationMode==LOC_REFLECTOR_STRIPS ||
      cameraSearch.active() || dockContactAdvanceActive;
#else
  // Preserve the existing obstacle stop for non-camera docking configs.
  return true;
#endif
}

bool DockOp::requestCameraDocking(){
#ifdef DOCK_REFLECTOR_STRIPS
  if (stateEstimator.stateOp != OP_IDLE && stateEstimator.stateOp != OP_ERROR) return false;
  float x,y,heading;
  if (maps.dockPoints.numPoints < 2 || !maps.getDockingPos(x,y,heading)) return false;
  motor.stopImmediately(true);
  motor.setMowState(false);
  cameraEntryRequested = true;
  setOperation(OP_DOCK);
  return true;
#else
  return false;
#endif
}

// Used only at command entry and during an already authorized marker search.
// A fresh marker in the camera is the orientation reference; map yaw can drift.
bool DockOp::tryCameraEntry(bool confirmedCameraEntry){
#ifdef DOCK_REFLECTOR_STRIPS
  bool automaticCameraEntry = false;
  float dockX,dockY,dockHeading;
  if (maps.dockPoints.numPoints >= 2 &&
      maps.getDockingPos(dockX,dockY,dockHeading)) {
    const int handoff = ReflectorDockingGate::handoffIndex(maps.dockPoints.numPoints);
    const bool nearPath = ReflectorDockingGate::nearCameraPath(
        stateEstimator.stateX,stateEstimator.stateY,
        maps.dockPoints.points[handoff].x(),maps.dockPoints.points[handoff].y(),dockX,dockY);
    // A command may resume camera docking only on the station side of the
    // handoff. The 2 m endpoint circle alone also includes earlier dock points.
    // A search already authorized at the handoff may acquire within its radius.
    const bool stationSide = cameraSearch.active() || DockMarkerSearch::eligible(
        stateEstimator.stateX,stateEstimator.stateY,
        maps.dockPoints.points[handoff].x(),maps.dockPoints.points[handoff].y(),dockX,dockY);
    // Found already includes consecutive-frame stability and freshness checks.
    automaticCameraEntry = ReflectorDockingGate::cameraEntryAllowed(
        cameraReflectorStripsFound(),nearPath && stationSide,cameraReflectorStripsHorizontalError());
  }
  if (confirmedCameraEntry || automaticCameraEntry){
    cameraSearchAvailable=false;
    maps.shouldDock = true;
    maps.shouldMow = false;
    maps.shouldRetryDock = false;
    maps.wayMode = WAY_DOCK;
    maps.dockPointsIdx = ReflectorDockingGate::handoffIndex(maps.dockPoints.numPoints);
    maps.targetPoint.assign(maps.dockPoints.points[maps.dockPointsIdx]);
    maps.setLastTargetPoint(stateEstimator.stateX,stateEstimator.stateY);
    maps.trackReverse = !DOCK_FRONT_SIDE;
    stateEstimator.reflectorDockingGate.confirmDockEntry(millis());
    stateEstimator.dockReflectorStripsActive = true;
    stateEstimator.dockReflectorStripsUndockCompleted = false;
    stateEstimator.stateLocalizationMode = LOC_REFLECTOR_STRIPS;
    motor.stopImmediately(true);
    lastMapRoutingFailed = false;
    mapRoutingFailedCounter = 0;
    CONSOLE.println(automaticCameraEntry
        ? "dock: stable marker on station side of handoff, acquiring camera without GPS approach"
        : "dock: operator confirmed entry, acquiring camera marker without GPS approach");
    return true;
  }
#endif
  return false;
}

bool DockOp::startCameraSearch(bool atHandoff){
#ifdef DOCK_REFLECTOR_STRIPS
  float dockX,dockY,heading;
  if (maps.dockPoints.numPoints < 2 || !maps.getDockingPos(dockX,dockY,heading)) return false;
  const int handoff=ReflectorDockingGate::handoffIndex(maps.dockPoints.numPoints);
  const float hx=maps.dockPoints.points[handoff].x(),hy=maps.dockPoints.points[handoff].y();
  const float dx=stateEstimator.stateX-hx,dy=stateEstimator.stateY-hy;
  const bool reachedHandoff=atHandoff && maps.dockPointsIdx>=handoff &&
      dx*dx+dy*dy<=DOCK_REFLECTOR_STRIPS_HANDOFF_RADIUS*DOCK_REFLECTOR_STRIPS_HANDOFF_RADIUS;
  if (!reachedHandoff && !DockMarkerSearch::eligible(stateEstimator.stateX,stateEstimator.stateY,
      hx,hy,dockX,dockY)) return false;
  cameraSearchAvailable=false;
  motor.stopImmediately(true);
  motor.setMowState(false);
  maps.shouldDock=true;maps.shouldMow=false;maps.shouldRetryDock=false;
  maps.wayMode=WAY_FREE; // no waypoint tracking while searching
  stateEstimator.reflectorDockingGate.reset();
  stateEstimator.dockReflectorStripsActive=false;
  stateEstimator.dockReflectorStripsUndockCompleted=false;
  cameraSearch.start(stateEstimator.stateDelta,millis());
  CONSOLE.println("dock: searching marker in place on station side of handoff (max one turn)");
  return true;
#else
  return false;
#endif
}

bool DockOp::runCameraSearch(){
#ifdef DOCK_REFLECTOR_STRIPS
  if (cameraSearch.state==DockMarkerSearch::Inactive) return false;
  battery.resetIdle();
  motor.setMowState(false);
  // Physical obstacle checks precede marker acquisition and any movement.
  if (detectObstacle() || shouldStop) return true;
  detectSensorMalfunction();
  if (shouldStop) return true;
  if (tryCameraEntry(false)) {cameraSearch.reset();return true;}
  const auto phase=cameraSearch.update(stateEstimator.stateDelta,millis());
  if (phase==DockMarkerSearch::Finished || phase==DockMarkerSearch::Stalled){
    motor.stopImmediately(true);
    CONSOLE.println(phase==DockMarkerSearch::Finished
        ? "dock: full search turn without marker, stopping in Idle"
        : "dock: search rotation stalled or pose invalid, stopping in Idle");
    changeOp(idleOp,false);
    return true;
  }
  if (phase==DockMarkerSearch::Settling){motor.stopImmediately(true);return true;}
  float linear=0,angular=.2f;
  limitDockWheelSpeed(linear,angular,motor.wheelBaseCm/100.0f,DOCK_MAX_WHEEL_SPEED);
  motor.enableTractionMotors(true);
  motor.setLinearAngularSpeed(0,angular,false);
  return true;
#else
  return false;
#endif
}

void DockOp::begin(){
    lineTracker.resetDockSlipRecovery();
  cameraSearch.reset();
  cameraSearchAvailable=true;
  const bool confirmedCameraEntry = cameraEntryRequested;
  cameraEntryRequested = false; // one-shot, never persist across commands/reboots
  dockContactAdvanceActive = false;
  if (previousOp == &chargeOp){
    battery.setIsDocked(true);
    changeOp(chargeOp);
    return;
  }

  bool error = false;
  bool routingFailed = false;

  motor.setLinearAngularSpeed(0,0);
  motor.setMowState(false);

  if (((initiatedByOperator) && (previousOp == &idleOp)) || (lastMapRoutingFailed))  maps.clearObstacles();

  CONSOLE.print("OP_DOCK");
  CONSOLE.print(" initiatedByOperator=");
  CONSOLE.print(initiatedByOperator);
  CONSOLE.print(" dockReasonRainTriggered=");
  CONSOLE.println(dockReasonRainTriggered);

  if (tryCameraEntry(confirmedCameraEntry)) return;
  if (startCameraSearch()) return;

  // plan route to next target point 

  if (maps.startDocking(stateEstimator.stateX, stateEstimator.stateY)){       
    if (maps.nextPoint(true, stateEstimator.stateX, stateEstimator.stateY)) {
      maps.repeatLastMowingPoint();
      stateEstimator.lastFixTime = millis();                
      maps.setLastTargetPoint(stateEstimator.stateX, stateEstimator.stateY);        
      //stateSensor = SENS_NONE;                  
    } else {
      error = true;
      CONSOLE.println("error: no waypoints!");
      //op = stateOp;                
    }
  } else error = true;
  if (error){
    stateEstimator.stateSensor = SENS_MAP_NO_ROUTE;
    //op = OP_ERROR;
    routingFailed = true;        
    motor.setMowState(false);
  }

  if (routingFailed){
    lastMapRoutingFailed = true; 
    mapRoutingFailedCounter++;    
    if (mapRoutingFailedCounter > 60){
      CONSOLE.println("error: too many routing errors!");
      stateEstimator.stateSensor = SENS_MAP_NO_ROUTE;
      changeOp(errorOp);      
    } else {    
      gpsRebootRecoveryOp.rebootGpsOnBegin = false;
      changeOp(gpsRebootRecoveryOp, true);
    }
  } else {
    lastMapRoutingFailed = false;
    mapRoutingFailedCounter = 0;
  }

}



void DockOp::end(){
  if (cameraSearch.state!=DockMarkerSearch::Inactive) motor.stopImmediately(true);
  cameraSearch.reset();
  if (dockContactAdvanceActive) motor.setLinearAngularSpeed(0, 0, false);
  dockContactAdvanceActive = false;
}

void DockOp::run(){
#ifdef DOCK_TEST_TIMEOUT_MS
    if (DOCK_TEST_TIMEOUT_MS > 0 && millis()-startTime >= DOCK_TEST_TIMEOUT_MS) {
        motor.stopImmediately(true);
        CONSOLE.println("dock: local test deadline reached, stopping in Idle");
        changeOp(idleOp, false);
        return;
    }
#endif
    if (bumper.obstacle() && stopOnDockingObstacle(SENS_BUMPER)) return;
    // First electrical contact starts a bounded straight odometry advance.
    // It must precede camera search/handoff and must not require a visible marker.
#ifdef DOCK_CONTACT_ADVANCE_DISTANCE
    if (dockContactAdvanceActive){
        long leftTicks = (long)(motor.motorLeftTicks - dockContactAdvanceStartLeftTicks);
        long rightTicks = (long)(motor.motorRightTicks - dockContactAdvanceStartRightTicks);
        float distanceCm = ((float)(abs(leftTicks) + abs(rightTicks))) / (2.0 * motor.ticksPerCm);
        bool timedOut = millis() >= dockContactAdvanceStopTime;
        if ((distanceCm >= DOCK_CONTACT_ADVANCE_DISTANCE * 100.0) || timedOut){
            motor.setLinearAngularSpeed(0, 0, false);
            dockContactAdvanceActive = false;
            CONSOLE.print("dock: contact advance finished distance=");
            CONSOLE.print(distanceCm / 100.0);
            CONSOLE.print("m timeout=");
            CONSOLE.println(timedOut);
            battery.setIsDocked(true);
            changeOp(chargeOp);
            return;
        }

        float speed = DOCK_FRONT_SIDE ? DOCK_LINEAR_SPEED : -DOCK_LINEAR_SPEED;
        motor.enableTractionMotors(true);
        motor.setLinearAngularSpeed(speed, 0, false);
        detectSensorMalfunction();
        battery.resetIdle();
        return;
    }
#endif
    if (runCameraSearch()) return;
#ifdef DOCK_REFLECTOR_STRIPS
    const auto cameraPhase=stateEstimator.reflectorDockingGate.phase;
    const bool markerFound=cameraReflectorStripsFound();
    if (ReflectorDockingGate::shouldSearch(cameraPhase,cameraSearchAvailable,markerFound) &&
        startCameraSearch(true)) return;
    // A later tracking loss waits for reacquisition; it must not start a new scan.
    if ((cameraPhase==ReflectorDockingGate::AcquireDock || cameraPhase==ReflectorDockingGate::Dock) && markerFound)
        cameraSearchAvailable=false;
    // Once GPS approach has begun, only the route-aware StateEstimator gate
    // may hand over at the penultimate dock point (with configured tolerance).
    // Seeing a marker near the final segment must not skip earlier dock points.
    // begin() can change maps after this cycle's state estimation.
    if (maps.isDocking() && stateEstimator.reflectorDockingGate.phase == ReflectorDockingGate::Inactive) {
        motor.stopImmediately(false);
        return;
    }
#endif


    if (!detectObstacle()){
        detectObstacleRotation();                              
    }
    if (shouldStop) return; // an obstacle transition must not be overwritten by tracking
    // line tracking
    lineTracker.trackLine(true);       
    detectSensorMalfunction(); 
    battery.resetIdle();
}


void DockOp::onChargerConnected(){
#ifdef DOCK_CONTACT_ADVANCE_DISTANCE
    if (DOCK_CONTACT_ADVANCE_DISTANCE > 0){
        // Contact bounce must not restart the distance or extend the deadline.
        if (dockContactAdvanceActive) return;
        dockContactAdvanceStartLeftTicks = motor.motorLeftTicks;
        dockContactAdvanceStartRightTicks = motor.motorRightTicks;

        float speed = abs(DOCK_LINEAR_SPEED);
        unsigned long timeoutDuration = 5000;
        if (speed > 0.001){
            unsigned long expectedDuration = (unsigned long)(DOCK_CONTACT_ADVANCE_DISTANCE / speed * 1000.0);
            if (expectedDuration * 3 > timeoutDuration) timeoutDuration = expectedDuration * 3;
        }
        dockContactAdvanceStopTime = millis() + timeoutDuration;
        dockContactAdvanceActive = true;

        CONSOLE.print("dock: charger contact, advancing another ");
        CONSOLE.print(DOCK_CONTACT_ADVANCE_DISTANCE);
        CONSOLE.println("m using odometry");
        return;
    }
#endif

    Op::onChargerConnected();
}


void DockOp::onTargetReached(){
    CONSOLE.println("DockOp::onTargetReached");
    if (maps.wayMode == WAY_MOW){
      maps.clearObstacles(); // clear obstacles if target reached
      stateEstimator.motorErrorCounter = 0; // reset motor error counter if target reached
      stateEstimator.stateSensor = SENS_NONE; // clear last triggered sensor
    }
}


void DockOp::onGpsFixTimeout(){
#ifdef DOCK_REFLECTOR_STRIPS
    if (stateEstimator.dockReflectorStripsActive) return;
#endif
    if (activateDeadReckoningNearDock()) return;
    if (REQUIRE_VALID_GPS){
      stateEstimator.stateSensor = SENS_GPS_FIX_TIMEOUT;
      changeOp(gpsWaitFixOp, true);
    }
}

void DockOp::onGpsNoSignal(){
#ifdef DOCK_REFLECTOR_STRIPS
    if (stateEstimator.dockReflectorStripsActive) return;
#endif
    if (activateDeadReckoningNearDock()) return;
    if (REQUIRE_VALID_GPS){
      stateEstimator.stateSensor = SENS_GPS_INVALID;
      changeOp(gpsWaitFloatOp, true);
    }
}

bool DockOp::activateDeadReckoningNearDock(){
#ifdef DOCK_IGNORE_GPS_DISTANCE
    if ((!maps.isDocking()) || (!maps.isTargetingLastDockPoint())) return false;
    float dockDistance = getDockDistance();
    if (dockDistance > DOCK_IGNORE_GPS_DISTANCE) return false;
    if (!stateEstimator.dockGpsIgnored){
      CONSOLE.print("dock: GPS unavailable, activating IMU/odometry at distance=");
      CONSOLE.println(dockDistance);
      Logger.event(EVT_DOCK_IGNORING_GPS);
    }
    stateEstimator.dockGpsIgnored = true;
    stateEstimator.stateLocalizationMode = LOC_IMU_ODO_ONLY;
    return true;
#else
    return false;
#endif
}

void DockOp::onKidnapped(bool state){
    if (state){
        stateEstimator.stateSensor = SENS_KIDNAPPED;      
        motor.setLinearAngularSpeed(0,0, false); 
        motor.setMowState(false);    
        changeOp(kidnapWaitOp, true); 
    }
}


void DockOp::onObstacleRotation(){
#ifdef __linux__
    if(CameraAvoidanceRuntime::enabled() && !cameraGuidanceActive() && stateEstimator.stateLocalizationMode==LOC_GPS){onObstacle();return;}
#endif
    if (stopOnDockingObstacle(SENS_OBSTACLE)) return;
    CONSOLE.println("error: rotation error due to obstacle!");    
    stats.statMowObstacles++;   
    stateEstimator.stateSensor = SENS_OBSTACLE;
    changeOp(errorOp);    
}

void DockOp::onObstacle(){
    if (stopOnDockingObstacle(SENS_OBSTACLE)) return;
#ifdef __linux__
    if(CameraAvoidanceRuntime::enabled() && !cameraGuidanceActive() && stateEstimator.stateLocalizationMode==LOC_GPS){
      motor.stopImmediately(true);
      ++stats.statMowObstacles;
      CONSOLE.println("dock approach: obstacle, reverse and replan");
      changeOp(escapeReverseOp, true);
      return;
    }
#endif
    if (battery.chargerConnected()) {
      CONSOLE.println("triggerObstacle: ignoring, because charger connected");      
      return;
    }
    if ((!DOCK_DETECT_OBSTACLE_IN_DOCK) && (maps.isBetweenLastAndNextToLastDockPoint())) {
      //CONSOLE.println("triggerObstacle: ignoring, because in dock");      
      return;
    }
    CONSOLE.println("triggerObstacle");      
    stats.statMowObstacles++;      
    if (maps.isDocking()) {    
        if (maps.retryDocking(stateEstimator.stateX, stateEstimator.stateY)) {
            changeOp(escapeReverseOp, true);                      
            return;
        }
    } 
    if ((OBSTACLE_AVOIDANCE) && (maps.wayMode != WAY_DOCK)){    
        changeOp(escapeReverseOp, true);      
    } else {     
        stateEstimator.stateSensor = SENS_OBSTACLE;
        CONSOLE.println("error: obstacle!");
        changeOp(errorOp);                
    }
}

/*void DockOp::onChargerConnected(){            
  battery.setIsDocked(true); // Test me   
  changeOp(chargeOp);
}*/


void DockOp::onNoFurtherWaypoints(){
  CONSOLE.println("docking finished!");
  battery.setIsDocked(true);    
  changeOp(idleOp); 
}

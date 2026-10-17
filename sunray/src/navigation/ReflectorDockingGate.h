#pragma once
#include <cmath>

// Hardware-free handoff policy. Camera acquisition never starts a search turn:
// the robot remains stopped until the marker is stable after the mandatory stop.
class ReflectorDockingGate {
public:
  enum Phase { Inactive, Approach, AcquireDock, Dock, AcquireUndock, Undock, Complete, Blocked };
  Phase phase = Inactive;
  unsigned long stoppedAt = 0;
  float undockLimit = 0;
  bool wasDocking = false, wasUndocking = false;

  static int handoffIndex(int count) { return count >= 2 ? count-2 : -1; }

  static bool shouldSearch(Phase phase,bool available,bool markerFound){
    return phase==AcquireDock && available && !markerFound;
  }

  void reset(){phase=Inactive;wasDocking=wasUndocking=false;}
  void confirmDockEntry(unsigned long now){reset();phase=AcquireDock;wasDocking=true;stoppedAt=now;}
  bool active() const {return phase==AcquireDock || phase==Dock || phase==AcquireUndock || phase==Undock || phase==Blocked;}
  bool mayDrive() const {return phase==Dock || phase==Undock;}
  bool complete() const {return phase==Complete;}
  static bool resumeUndocking(float distance,float exitRadius,bool inCorridor,bool knownDockPath,float facing){
    return std::isfinite(distance) && distance>exitRadius && exitRadius>0 &&
        inCorridor && (knownDockPath || facing>0.7071f);
  }
  static bool nearSegment(float x,float y,float ax,float ay,float bx,float by,float radius){
    const float dx=bx-ax,dy=by-ay,length2=dx*dx+dy*dy;
    if(!std::isfinite(length2) || length2<0.0025f || radius<=0)return false;
    const float t=((x-ax)*dx+(y-ay)*dy)/length2;
    if(t<0 || t>1)return false;
    const float ex=x-ax-t*dx,ey=y-ay-t*dy;
    return ex*ex+ey*ey<=radius*radius;
  }
  // Distance to the finite final dock segment, including its endpoints.
  // This wider entry corridor does not change the 1 m RTK/camera handoff.
  static bool nearCameraPath(float x,float y,float ax,float ay,float bx,float by){
    if(!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(ax)||!std::isfinite(ay)||
       !std::isfinite(bx)||!std::isfinite(by))return false;
    const float dx=bx-ax,dy=by-ay,length2=dx*dx+dy*dy;
    if(!std::isfinite(length2)||length2<0.0025f)return false;
    float t=((x-ax)*dx+(y-ay)*dy)/length2;
    t=t<0?0:(t>1?1:t);
    const float ex=x-ax-t*dx,ey=y-ay-t*dy;
    return ex*ex+ey*ey<=2.0f*2.0f;
  }
  static bool cameraEntryAllowed(bool markerFound,bool nearPath,float imageError){
    return markerFound && nearPath && std::isfinite(imageError) && std::fabs(imageError)<=1.0f;
  }
  // Unsigned counters decrement in reverse. Compute the shorter modular
  // displacement before converting to float, including across counter wrap.
  static float tickDisplacement(unsigned long current,unsigned long start){
    const unsigned long forward=current-start,reverse=start-current;
    return float(forward<reverse?forward:reverse);
  }

  void update(bool docking,bool undocking,bool routeValid,
              float x,float y,float handoffX,float handoffY,float travel,
              bool markerFound,unsigned long now,float entryRadius,float exitRadius,unsigned long stopMs,bool dockingRouteReady=true){
    if(!docking && !undocking){reset();return;}
    const bool newDock=docking&&!wasDocking,newUndock=undocking&&!wasUndocking;
    wasDocking=docking;wasUndocking=undocking;
    if(!routeValid || !std::isfinite(travel) || travel<0 || entryRadius<=0 || exitRadius<=0 || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(handoffX) || !std::isfinite(handoffY)){
      phase=Blocked;return;
    }
    const float dx=x-handoffX,dy=y-handoffY,distance=std::sqrt(dx*dx+dy*dy);
    if(newDock)phase=Approach;
    if(newUndock){
      phase=AcquireUndock;stoppedAt=now;
      // A missed exit must stop, never cause unlimited reverse travel.
      undockLimit=distance*1.5f+1.0f;
    }
    if(docking && phase==Approach && dockingRouteReady && distance<=entryRadius){
      phase=AcquireDock;stoppedAt=now;
    }
    if(undocking && phase!=Complete && phase!=Blocked){
      if(distance<=exitRadius){phase=Complete;return;}
      if(travel>=undockLimit){phase=Blocked;return;}
    }
    if(phase==Dock && !markerFound){phase=AcquireDock;stoppedAt=now;}
    if(phase==Undock && !markerFound){phase=AcquireUndock;stoppedAt=now;}
    if((phase==AcquireDock || phase==AcquireUndock) && markerFound && now-stoppedAt>=stopMs){
      phase=phase==AcquireDock?Dock:Undock;
    }
  }
};

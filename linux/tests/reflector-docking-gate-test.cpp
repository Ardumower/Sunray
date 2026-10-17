#include "src/navigation/ReflectorDockingGate.h"
#include <cassert>
#include "src/navigation/DockMotionLimit.h"
#include <climits>
#include <initializer_list>
#include <limits>
#include <cstdio>
int main(){
 assert(ReflectorDockingGate::nearSegment(11.2f,-1.24f,11.89f,-1.38f,8.18f,.11f,.75f));
 assert(!ReflectorDockingGate::nearSegment(11.2f,2,11.89f,-1.38f,8.18f,.11f,.75f));
 assert(!ReflectorDockingGate::nearSegment(7,0,8,0,14,0,.75f));
 assert(!ReflectorDockingGate::nearSegment(8,0,8,0,8,0,.75f));
 assert(ReflectorDockingGate::resumeUndocking(3,2,true,true,0));
 assert(ReflectorDockingGate::resumeUndocking(3,2,true,false,.95f));
 assert(!ReflectorDockingGate::resumeUndocking(2,2,true,true,1));
 assert(!ReflectorDockingGate::resumeUndocking(3,2,false,false,1));
 assert(ReflectorDockingGate::resumeUndocking(3,2,true,false,1));
 assert(!ReflectorDockingGate::resumeUndocking(3,2,true,false,-1));
 assert(ReflectorDockingGate::tickDisplacement(100,100)==0);
 assert(ReflectorDockingGate::tickDisplacement(101,100)==1);
 assert(ReflectorDockingGate::tickDisplacement(99,100)==1);
 assert(ReflectorDockingGate::tickDisplacement(ULONG_MAX,0)==1);
 assert(ReflectorDockingGate::tickDisplacement(0,ULONG_MAX)==1);
 assert(ReflectorDockingGate::tickDisplacement(ULONG_MAX-4,5)==10);
 for(float l:{-.5f,-.02f,0.f,.02f,.5f})for(float a:{-1.f,0.f,1.f}){
  float v=l,w=a;limitDockWheelSpeed(v,w,.48f,.02f);
  assert(std::fabs(v+w*.24f)<=.020001f && std::fabs(v-w*.24f)<=.020001f);
  assert(std::fabs(v*a-w*l)<.00001f);
 }
 ReflectorDockingGate g;
 auto update=[&](bool dock,bool undock,float x,bool found,unsigned long t,float travel=0,int idx=0,bool valid=true){g.update(dock,undock,valid,idx,x,0,0,0,travel,found,t,.25f,2.f,500);};
 update(true,false,4,true,100);assert(g.phase==g.Approach&&!g.active());
 update(true,false,.2,true,200);assert(g.phase==g.AcquireDock&&!g.mayDrive());
 update(true,false,.2,true,699);assert(!g.mayDrive());
 update(true,false,.2,true,700);assert(g.mayDrive());
 update(true,false,1,false,720);assert(g.active()&&!g.mayDrive());
 update(true,false,1,false,3000);assert(!g.mayDrive());
 update(true,false,1,true,3020);assert(g.mayDrive());
 update(true,false,1,false,3040);update(true,false,1,true,3060);assert(!g.mayDrive());
 update(false,false,0,false,3100);assert(!g.active());
 update(true,false,3,false,3200,0,1);assert(g.phase==g.AcquireDock);
 update(false,false,0,false,3300);
 update(false,true,6,true,3400);assert(g.phase==g.AcquireUndock);
 update(false,true,6,true,3899);assert(!g.mayDrive());
 update(false,true,6,true,3900);assert(g.phase==g.Undock);
 update(false,true,4.5,true,4000,1.5);assert(g.mayDrive()&&!g.complete());
 update(false,true,3,false,4100,3);assert(!g.mayDrive());
 update(false,true,3,true,4600,3);assert(g.mayDrive());
 update(false,true,2,true,4700,4);assert(g.complete()&&!g.active());
 update(false,true,2.2,false,4800,4);assert(g.complete());
 update(false,false,0,false,4900);assert(g.phase==g.Inactive);
 update(false,true,5,true,5000);update(false,true,4,true,5500,9);assert(g.phase==g.Blocked);
 update(false,true,1,true,6000,9);assert(g.phase==g.Blocked);
 update(false,false,0,false,6100);update(true,false,0,true,6200,0,0,false);assert(g.phase==g.Blocked);
 update(true,false,0,true,7000);assert(g.phase==g.Blocked);
 update(false,false,0,false,7100);update(false,true,5,true,7200,std::numeric_limits<float>::quiet_NaN());assert(g.phase==g.Blocked);
 update(false,false,0,false,0);update(true,false,0,true,ULONG_MAX-199);update(true,false,0,true,299);assert(!g.mayDrive());update(true,false,0,true,300);assert(g.mayDrive());
 g.reset();g.confirmDockEntry(100);
 update(true,false,1.55,true,599);assert(g.phase==g.AcquireDock&&!g.mayDrive());
 update(true,false,1.55,true,600);assert(g.phase==g.Dock);
 update(true,false,1.55,false,620);assert(!g.mayDrive());
 update(false,false,0,false,700);update(true,false,1.55,true,800);assert(g.phase==g.Approach);
 std::puts("PASS first-point stop, reacquisition, undock 2m handoff, reset, blocked route/travel, timer wrap");
}

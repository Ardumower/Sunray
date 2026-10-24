#include <cassert>
#include <cstdio>
#include <initializer_list>
#include "../../sunray/src/navigation/DockSlipRecovery.h"
using R=DockSlipRecovery;
R::Input sample(uint32_t t) { return {t,true,false,0,-.12f,.3f,0,t*.00002f}; }
void step(R& r,R::Input i) { float l=.02f,a=.08f;r.update(i,l,a); }
int main(){
 R r;step(r,sample(0));step(r,sample(11999));assert(r.phase()==R::Tracking);
 step(r,sample(12000));assert(r.phase()==R::Settling);
 float l=.02f,a=.08f;r.update(sample(12500),l,a);assert(l==0&&a==0);
 step(r,sample(12700));assert(r.phase()==R::Turning);
 r.update(sample(12800),l,a);assert(l==0&&a>.08f&&a<.09f);
 auto i=sample(14000);i.yaw=.11f;step(r,i);assert(r.phase()==R::Tracking);
 i=sample(14100);i.yaw=.11f;step(r,i);
 i=sample(26100);i.yaw=.11f;step(r,i);assert(r.phase()==R::Settling);
 r.update(i,l,a);assert(l==0&&a==0);
 r.reset();step(r,sample(0));step(r,sample(12000));step(r,sample(12700));r.update(sample(15200),l,a);assert(l==0&&std::fabs(a-.20f)<.0001f);
 step(r,sample(17699));assert(r.phase()==R::Turning);
 step(r,sample(17700));assert(r.phase()==R::Settling);
 step(r,sample(18400));assert(r.phase()==R::Turning);
 for(unsigned t=23400;t<100000;t+=5700){step(r,sample(t));assert(r.phase()==R::Settling);step(r,sample(t+700));assert(r.phase()==R::Turning);}
 for(int mode=0;mode<4;++mode){
   R moving;
   for(unsigned t=0;t<=30000;t+=20){auto v=sample(t);
     if(mode==0)v.imageSpan=.3f+t*.000002f; // real translation
     if(mode==1)v.yaw=t*.00001f; // real rotation
     if(mode==2)v.imageError=t*.00001f; // lateral camera motion
     if(mode==3)v.rightTravel=0; // not enough evidence of slip
     step(moving,v);
   }
   assert(moving.phase()==R::Tracking);
 }
 for(int bumper=0;bumper<2;++bumper){R x;step(x,sample(0));step(x,sample(12000));auto v=sample(12100);v.camera=bumper;v.bumper=bumper;
   float speed=.02f,turn=.08f;x.update(v,speed,turn);assert(speed==0&&turn==0);
   assert(x.phase()==(bumper?R::Failed:R::Settling));
 }
 R paused;step(paused,sample(0));step(paused,sample(12000));step(paused,sample(12700));
 auto lost=sample(90000);lost.camera=false;float speed=.02f,turn=.08f;paused.update(lost,speed,turn);
 assert(speed==0&&turn==0&&paused.phase()==R::Turning);
 step(paused,sample(91000));assert(paused.phase()==R::Settling);
 step(paused,sample(91700));assert(paused.phase()==R::Turning);
 // A confirmed small rotation reaching the centre must not overshoot to 6deg.
 for(float error : {-.01f,.01f}) {
   R aligned;step(aligned,sample(0));step(aligned,sample(12000));step(aligned,sample(12700));
   auto v=sample(13700);v.imageError=error;v.yaw=.04f;
   float speed=.02f,turn=.08f;aligned.update(v,speed,turn);
   assert(aligned.phase()==R::Tracking && speed==0 && turn==0);
 }
 // Image jitter without gyro motion cannot prematurely end recovery.
 R jitter;step(jitter,sample(0));step(jitter,sample(12000));step(jitter,sample(12700));
 auto j=sample(13700);j.imageError=0;j.yaw=.001f;step(jitter,j);
 assert(jitter.phase()==R::Turning);
 R wrap;auto v=sample(0);v.now=0xfffff000;step(wrap,v);v.now+=12000;v.rightTravel=.3;step(wrap,v);assert(wrap.phase()==R::Settling);
 puts("PASS dock slip: stationary encoder travel, visual/gyro progress, settle, bounded pivot, repeated bounded attempts, bumper/marker loss and clock wrap");
}

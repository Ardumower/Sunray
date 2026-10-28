#include "src/navigation/CameraAvoidance.h"
#include <cassert>
#include <iostream>
using namespace CameraAvoidance;
static bool allowed(Point p){return p.x>-5&&p.x<20&&std::abs(p.y)<8;}
void scenario(const Polygon& truth,const char* name){
 Planner p;Pose pose{};Point goal{10,0};double lastTurn=0,travelled=0,minClearance=100;int stops=0;bool arrived=false;
 for(int tick=1;tick<1800;tick++){
  std::array<uint8_t,3600> mask;mask.fill(1);
  for(int y=0;y<45;y++)for(int x=0;x<80;x++){
   Point q;if(p.project(x,y,pose,q)&&inside(q,truth))mask[y*80+x]=3;
  }
  // Flickering boundary cells do not move the whole stored obstacle every frame.
  if(tick%7==0)for(int i=0;i<3600;i+=37)mask[i]=1;
  p.observe(mask.data(),tick*100,pose);
  float v=.35,w=0;
  if(!p.plan(pose,goal,tick*100,allowed))v=w=0;
  else p.steer(pose,.35,.1,allowed,lastTurn,v,w);
  if(v<.01)stops++;
  pose.yaw=wrap(pose.yaw+w*.1);pose.x+=v*.1*std::cos(pose.yaw);pose.y+=v*.1*std::sin(pose.yaw);travelled+=v*.1;
  minClearance=std::min(minClearance,separation({pose.x,pose.y},truth));
  assert(minClearance>=std::hypot(.8,1.)/2);
  if(distance({pose.x,pose.y},goal)<.22){arrived=true;break;}
 }
 std::cout<<name<<" arrived="<<arrived<<" travel="<<travelled<<" clearance="<<minClearance<<" stopped_ticks="<<stops<<" pose="<<pose.x<<","<<pose.y<<" obstacles="<<p.obstacles.size()<<std::endl;
 assert(arrived);assert(travelled<18);
}
void escapeAndRejoin(){
 Planner p;Polygon obstacle{{-1,-1},{1,-1},{1,1},{-1,1}};
 p.obstacles.push_back({obstacle,1});Pose pose{.8,0,0};double turn=0;
 for(int tick=1;tick<160;tick++){
  const double before=signedClearance({pose.x,pose.y},obstacle);
  assert(p.plan(pose,{4,0},tick*100,allowed));float v=0,w=0;
  p.steer(pose,.35,.1,allowed,turn,v,w);
  pose.yaw=wrap(pose.yaw+w*.1);pose.x+=v*.1*std::cos(pose.yaw);pose.y+=v*.1*std::sin(pose.yaw);
  assert(signedClearance({pose.x,pose.y},obstacle)>=before-.001);
 }
 assert(!p.contains({pose.x,pose.y}));
 p.reset();p.obstacles.push_back({{{3,-1},{7,-1},{7,1},{3,1}},1});pose={};turn=0;
 Point lineStart{0,0},goal{18,0},join=p.rejoinPoint(pose,lineStart,goal);bool rejoined=false;
 for(int tick=1;tick<1600;tick++){
  if(distance({pose.x,pose.y},join)<.3)join=p.rejoinPoint(pose,lineStart,goal);
  float v=0,w=0;
  if(p.plan(pose,join,tick*100,allowed))p.steer(pose,.35,.1,allowed,turn,v,w);
  pose.yaw=wrap(pose.yaw+w*.1);pose.x+=v*.1*std::cos(pose.yaw);pose.y+=v*.1*std::sin(pose.yaw);
  if(pose.x>10 && pose.x<12 && std::abs(pose.y)<.2)rejoined=true;
  if(distance({pose.x,pose.y},goal)<.22)break;
 }
 std::cout<<"escape and early mowing-line rejoin="<<rejoined<<" final="<<pose.x<<","<<pose.y<<std::endl;
 assert(rejoined);assert(distance({pose.x,pose.y},goal)<.22);
}
int main(){scenario({{3,-1},{7,-1},{7,1},{3,1}},"asphalt runway");scenario({{3,-.5},{4,-.5},{4,.5},{3,.5}},"model aircraft");escapeAndRejoin();}

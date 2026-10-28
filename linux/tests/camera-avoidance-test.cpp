#include "src/navigation/CameraAvoidance.h"
#include <cassert>
#include <iostream>
#include <chrono>
using namespace CameraAvoidance;
static bool allowed(Point p){return std::abs(p.x)<50&&std::abs(p.y)<50;}
int main(){
 // A short approved entry into the perimeter is allowed; leaving its corridor
 // or extending the outside allowance is not. Camera obstacles still apply.
 auto entry=[](Point q){return q.x>=0 || entryCorridor(q,{-.56,0},{2,0},1.);};
 assert(entry({-.56,0}));assert(!entry({-.3,.2}));assert(!entry({-1.6,0}));
 Planner ingress;assert(ingress.plan({-.56,0,0},{2,0},1,entry));
 ingress.obstacles.push_back({{{-.4,-.2},{-.2,-.2},{-.2,.2},{-.4,.2}},1});
 assert(!ingress.plan({-.56,0,0},{2,0},2,entry));
 Planner p;
 // A goal deeper inside an enclosing contour must not prevent a separate
 // escape manoeuvre. Restore normal planning after reaching free space.
 Planner escape;escape.obstacles.push_back({{{-1,-1},{1,-1},{1,1},{-1,1}},1});
 assert(!escape.plan({.8,0,0},{0,0},100,allowed));
 assert(escape.planEscape({.8,0,0},allowed));
 assert(escape.route.size()==1 && !escape.contains(escape.route.front()));
 assert(escape.clear({.8,0},escape.route.front(),allowed));
 assert(!escape.clear({.8,0},{0,0},allowed));
 // A short clear waypoint does not impose the old distance/2 speed cap.
 Planner cruise;assert(cruise.plan({}, {.6,0},100,allowed));
 float cv=0,cw=0;double ct=0;cruise.steer({},.6,.02,allowed,ct,cv,cw);
 assert(cv>.59f);
 // Reach a boundary waypoint without the two-second prediction overshooting
 // the boundary and deadlocking before targetReached can advance the route.
 Planner boundary;Pose approaching{.3,0,0};double turn=0;float v=0,w=0;
 auto edge=[](Point q){return q.x<=1.000001 && q.x>=0 && std::abs(q.y)<=1;};
 for(int tick=0;tick<150 && distance({approaching.x,approaching.y},{1,0})>.1;tick++){
   assert(boundary.plan(approaching,{1,0},100+tick,edge));
   boundary.steer(approaching,.5,.05,edge,turn,v,w);
   assert(v>0);approaching.x+=v*.05;assert(approaching.x<=1.0);
 }
 assert(distance({approaching.x,approaching.y},{1,0})<=.1);
 std::array<uint8_t,3600> mask;mask.fill(1);
 for(int y=29;y<42;y++)for(int x=30;x<50;x++)mask[y*80+x]=3;
 p.observe(mask.data(),100,{});assert(p.obstacles.empty());
 p.observe(mask.data(),200,{});assert(p.obstacles.size()==1);
 const auto old=p.obstacles[0].points;
 for(int y=29;y<42;y++)for(int x=25;x<55;x++)mask[y*80+x]=3;
 p.observe(mask.data(),300,{});assert(p.obstacles.size()==1);
 for(auto pt:old)assert(separation(pt,p.obstacles[0].points)<.001);
 p.reset();assert(p.obstacles.empty());
 mask.fill(1);mask[35*80+40]=0;
 p.observe(mask.data(),400,{});p.observe(mask.data(),500,{});
 assert(p.obstacles.size()==1); // confirmed thin pole, one image column
 p.reset();
 p.obstacles.push_back({{{2,-1},{4,-1},{4,1},{2,1}},100});
 assert(p.plan({}, {7,0},1000,allowed));assert(p.active);assert(p.route.size()>=2);
 Point prev{};for(auto pt:p.route){assert(p.clear(prev,pt,allowed));prev=pt;}
 const auto searches=p.searchRuns;
 for(int tick=1;tick<100;tick++){
   p.revision++; // repeated perception refreshes must not invalidate a safe route
   assert(p.plan({}, {7,0},1000+tick*20,allowed));
 }
 assert(p.searchRuns==searches);
 // A fresh obstacle on the robot's path stops immediately, even with a cached route.
 p.obstacles.push_back({{{1,-60},{1.5,-60},{1.5,60},{1,60}},1});
 assert(!p.plan({}, {7,0},3100,allowed));
 const auto failedSearches=p.searchRuns;
 for(int tick=1;tick<25;tick++)assert(!p.plan({}, {7,0},3100+tick*20,allowed));
 assert(p.searchRuns==failedSearches);
 assert(!p.plan({}, {7,0},3600,allowed));assert(p.searchRuns==failedSearches+1);
 // Long asphalt rectangle: route goes around it and shortcuts along its side,
 // rather than tracing its complete outline or zig-zagging back into it.
 p.reset();p.obstacles.push_back({{{2,-1},{7,-1},{7,1},{2,1}},100});
 assert(p.plan({}, {9,0},2000,allowed));assert(p.route.size()<=5);
 prev={};for(auto pt:p.route){assert(p.clear(prev,pt,allowed));prev=pt;}
 assert(p.plan({2.2,0,0},{9,0},3000,allowed)); // escape an enclosing camera contour
 assert(!p.clear({2.2,0},{5,0},allowed)); // no deeper penetration
 assert(!p.plan({}, {3,0},4000,allowed));
 // Restore full clearance immediately after escaping; another obstacle remains forbidden.
 assert(p.plan({0,3,0},{9,0},4500,allowed));assert(!p.clear({0,3},{3,0},allowed));
 p.reset();assert(p.plan({}, {7,0},5000,allowed));assert(!p.active);assert(p.route.size()==1);
 // A map boundary forbids escaping through a narrow passage.
 p.obstacles.push_back({{{2,-5},{3,-5},{3,5},{2,5}},100});
 assert(!p.plan({}, {7,0},6000,[](Point q){return q.y>-3&&q.y<3&&q.x>-1&&q.x<10;}));
 std::cout<<"PASS mask confirmation, contour growth, footprint clearance, shortcuts, blocked goals and map bounds\n";
}

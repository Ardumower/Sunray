#pragma once
// Sparse, camera-derived obstacle contours. No persistent occupancy map.
// Geometry and planning are independent of Arduino/Godot for deterministic tests.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <vector>
namespace CameraAvoidance {
struct Point { double x=0,y=0; Point()=default; Point(double x,double y):x(x),y(y){} };
struct Pose { double x=0,y=0,yaw=0; };
inline double distance(Point a,Point b){return std::hypot(a.x-b.x,a.y-b.y);}
inline double cross(Point a,Point b,Point c){return (b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x);}
inline double wrap(double a){return std::atan2(std::sin(a),std::cos(a));}
using Polygon=std::vector<Point>;
inline Polygon hull(Polygon p){
 std::sort(p.begin(),p.end(),[](Point a,Point b){return a.x==b.x?a.y<b.y:a.x<b.x;});
 p.erase(std::unique(p.begin(),p.end(),[](Point a,Point b){return distance(a,b)<.005;}),p.end());
 if(p.size()<3)return p;
 Polygon h;
 for(auto v:p){while(h.size()>1&&cross(h[h.size()-2],h.back(),v)<=0)h.pop_back();h.push_back(v);}
 const size_t lower=h.size();
 for(int i=int(p.size())-2;i>=0;--i){while(h.size()>lower&&cross(h[h.size()-2],h.back(),p[i])<=0)h.pop_back();h.push_back(p[i]);}
 h.pop_back();return h;
}
inline double segmentDistance(Point p,Point a,Point b){
 double dx=b.x-a.x,dy=b.y-a.y,d=dx*dx+dy*dy;
 double t=d?std::max(0.,std::min(1.,((p.x-a.x)*dx+(p.y-a.y)*dy)/d)):0;
 return distance(p,{a.x+t*dx,a.y+t*dy});
}
inline bool entryCorridor(Point p,Point start,Point goal,double maxOutside){
 return maxOutside>0 && distance(p,start)<=maxOutside && segmentDistance(p,start,goal)<=.10;
}
inline bool inside(Point p,const Polygon& poly){
 bool in=false;if(poly.size()<3)return false;
 for(size_t i=0,j=poly.size()-1;i<poly.size();j=i++){
  auto a=poly[i],b=poly[j];
  if((a.y>p.y)!=(b.y>p.y)&&p.x<(b.x-a.x)*(p.y-a.y)/(b.y-a.y)+a.x)in=!in;
 }
 return in;
}
inline double separation(Point p,const Polygon& poly){
 if(inside(p,poly))return 0;
 double d=1e9;for(size_t i=0;i<poly.size();i++)d=std::min(d,segmentDistance(p,poly[i],poly[(i+1)%poly.size()]));return d;
}
// Negative inside, positive outside: unlike separation(), this retains the
// direction of progress when an updated camera contour encloses the robot.
inline double signedClearance(Point p,const Polygon& poly){
 double d=1e9;for(size_t i=0;i<poly.size();++i)d=std::min(d,segmentDistance(p,poly[i],poly[(i+1)%poly.size()]));
 return inside(p,poly)?-d:d;
}
struct Config {
 double height=.4,forward=.5,hfov=75,downPitch=0,width=.8,length=1.,margin=.20;
 double range=5.,resolution=.20; // conservative near-field ground projection
};
struct Obstacle { Polygon points; uint64_t seen=0; };
class Planner {
public:
 Config cfg;
 std::vector<Obstacle> obstacles;
 std::vector<Point> route;
 Point routeStart;
 bool active=false,blocked=false,noRoute=false;
 unsigned revision=0;
 unsigned searchRuns=0;
 // Crop away sky/horizon; this projection applies to ground contact/boundaries.
 bool project(int x,int y,Pose pose,Point& result) const {
  const double f=40/std::tan(cfg.hfov*3.141592653589793/360);
  const double right=(x+.5-40)/f, down=(y+.5-22.5)/f;
  const double pitch=cfg.downPitch*3.141592653589793/180;
  const double vertical=down*std::cos(pitch)+std::sin(pitch);
  if(vertical<=.025)return false;
  const double front=cfg.height*(std::cos(pitch)-down*std::sin(pitch))/vertical+cfg.forward;
  const double left=-cfg.height*right/vertical;
  if(front<.6||front>cfg.range||std::abs(left)>cfg.range)return false;
  result={pose.x+std::cos(pose.yaw)*front-std::sin(pose.yaw)*left,
          pose.y+std::sin(pose.yaw)*front+std::cos(pose.yaw)*left};return true;
 }
 void reset(){obstacles.clear();route.clear();pending.clear();active=blocked=noRoute=false;escapeEnabled=false;lastSearchFailed=false;side=0;lastStamp=lastPlan=0;revision++;}
 // Two consecutive spatially overlapping observations suppress flickering masks.
 void observe(const uint8_t* labels,uint64_t stamp,Pose pose){
  if(!stamp||stamp<=lastStamp)return;
  lastStamp=stamp;
  std::array<bool,3600> visited{};std::vector<Polygon> detected;
  for(int seed=0;seed<3600;seed++){
   if(visited[seed]||labels[seed]==1||labels[seed]==255)continue;
   Point ignored;if(!project(seed%80,seed/80,pose,ignored))continue;
   Polygon points;std::vector<int> todo{seed};visited[seed]=true;
   for(size_t q=0;q<todo.size();q++){
    int i=todo[q],x=i%80,y=i/80;Point p;
    if(project(x,y,pose,p))points.push_back(p);
    for(auto offset:std::array<Point,4>{{{1,0},{-1,0},{0,1},{0,-1}}}){
     int xx=x+int(offset.x),yy=y+int(offset.y),n=yy*80+xx;
     if(xx<0||xx>=80||yy<0||yy>=45||visited[n]||labels[n]==1||labels[n]==255)continue;
     if(!project(xx,yy,pose,p))continue;
     visited[n]=true;todo.push_back(n);
    }
   }
   if(points.empty() || (points.size()<3 && labels[seed]!=0))continue;
   // Only the visible ground-contact boundary has a valid ground-plane range.
   // Projecting the top of a raised object would invent a huge obstacle behind
   // it, which grows incorrectly as the camera turns around it.
   std::array<int,80> bottom;bottom.fill(-1);
   for(int i:todo)bottom[i%80]=std::max(bottom[i%80],i/80);
   points.clear();double nearest=1e9;
   for(int x=0;x<80;x++)if(bottom[x]>=0){Point q;if(project(x,bottom[x],pose,q))nearest=std::min(nearest,distance(q,{pose.x,pose.y}));}
   for(int x=0;x<80;x++)if(bottom[x]>=0){
    Point contact;if(!project(x,bottom[x],pose,contact))continue;
    const double r=distance(contact,{pose.x,pose.y});
    // Partial silhouette fragments near the top of a tall object are not
    // ground contacts. Grow the near boundary as more of it becomes visible.
    if(r>nearest+1.2)continue;
    // Retain very narrow poles: a single image column still occupies width.
    points.push_back({contact.x-.05*std::sin(pose.yaw),contact.y+.05*std::cos(pose.yaw)});
    points.push_back({contact.x+.05*std::sin(pose.yaw),contact.y-.05*std::cos(pose.yaw)});
    if(r>.01)points.push_back({contact.x+.20*(contact.x-pose.x)/r,contact.y+.20*(contact.y-pose.y)/r});
   }
   auto contour=hull(points);if(contour.size()<3)continue;
   detected.push_back(contour);
   bool confirmed=false;
   for(auto& old:pending)if(touches(contour,old,.25)){confirmed=true;break;}
   if(confirmed)merge(contour,stamp);
  }
  pending=std::move(detected);
  // Do not erase a contour merely because it leaves the camera field of view.
  for(size_t i=0;i<obstacles.size();){
   if(stamp-obstacles[i].seen>120000){obstacles.erase(obstacles.begin()+i);revision++;}else ++i;
  }
 }
 bool blockedBy(Point p,const Polygon& poly) const {
  const double clearance=signedClearance(p,poly),required=radius();
  if(clearance>=required)return false;
  if(!escapeEnabled)return true;
  const double initial=signedClearance(escapeOrigin,poly);
  // An obstacle already enclosing the start may be exited, but must never be
  // penetrated more deeply. All other obstacles retain full footprint clearance.
  return initial>=required || clearance<initial-.0001;
 }
 bool traversable(Point p) const {for(const auto& o:obstacles)if(blockedBy(p,o.points))return false;return true;}
 bool contains(Point p) const {for(auto& o:obstacles)if(separation(p,o.points)<radius())return true;return false;}
 double radius() const {return std::hypot(cfg.width,cfg.length)/2+cfg.margin;}
 bool clear(Point a,Point b,const std::function<bool(Point)>& allowed) const {
  int n=std::max(1,int(std::ceil(distance(a,b)/.10)));
  for(int i=0;i<=n;i++){
   Point p{a.x+(b.x-a.x)*i/n,a.y+(b.y-a.y)*i/n};
   if(!traversable(p)||!allowed(p))return false;
  }return true;
 }
 // Aim back onto the original mowing segment ahead of the robot, beyond
 // the visible blocked portion, rather than diagonally at its distant endpoint.
 Point rejoinPoint(Pose pose,Point from,Point goal) const {
  const double length=distance(from,goal);if(length<.01)return goal;
  const double ux=(goal.x-from.x)/length,uy=(goal.y-from.y)/length;
  const double progress=std::max(0.,std::min(length,(pose.x-from.x)*ux+(pose.y-from.y)*uy));
  double join=std::min(length,progress+2.0);
  for(double t=progress;t<=std::min(length,progress+10.);t+=.1)
   if(contains({from.x+ux*t,from.y+uy*t}))join=std::min(length,t+1.0);
  return {from.x+ux*join,from.y+uy*join};
 }
 // Plan through free space, then shortcut to the furthest visible route point.
 // Never require following every vertex of an obstacle contour.
 bool plan(Pose pose,Point goal,uint64_t now,const std::function<bool(Point)>& allowed){
  Point start{pose.x,pose.y};blocked=false;noRoute=false;escapeOrigin=start;escapeEnabled=contains(start);
  if(clear(start,goal,allowed)){route={goal};routeStart=start;active=false;lastSearchFailed=false;side=0;return true;}
  active=true;
  const bool sameGoal=distance(goal,lastGoal)<.001;
  // Revalidate the remaining route against current observations. A new mask
  // alone does not justify running A* again, even when its contour grew.
  if(sameGoal && !route.empty()){
   int visible=-1;
   for(int i=int(route.size())-1;i>=0;--i)if(clear(start,route[i],allowed)){visible=i;break;}
   bool valid=visible>=0;
   for(size_t i=visible>=0?size_t(visible)+1:route.size();i<route.size()&&valid;++i)
    valid=clear(route[i-1],route[i],allowed);
   if(valid && distance(start,route[visible])>.25){lastSearchFailed=false;return true;}
  }
  // Failed or invalidated routes stop immediately, but expensive replanning
  // runs at most twice per second for an unchanged target.
  if(lastPlan && sameGoal && now-lastPlan<500){route.clear();blocked=true;noRoute=lastSearchFailed;return false;}
  lastPlan=now;lastGoal=goal;lastSearchFailed=false;searchRuns++;
  const int N=101,C=N/2;const double step=cfg.resolution;
  const double gx=(goal.x-start.x)/step+C,gy=(goal.y-start.y)/step+C;
  const int tx=std::max(1,std::min(N-2,int(std::round(gx)))),ty=std::max(1,std::min(N-2,int(std::round(gy))));
  auto point=[&](int i){return Point{start.x+(i%N-C)*step,start.y+(i/N-C)*step};};
  struct Bounds {const Polygon* polygon;double x0,y0,x1,y1;};
  std::vector<Bounds> nearby;
  const double clearance=radius(),extent=C*step;
  for(const auto& o:obstacles){
   Bounds b{&o.points,1e9,1e9,-1e9,-1e9};
   for(auto p:o.points){b.x0=std::min(b.x0,p.x);b.y0=std::min(b.y0,p.y);b.x1=std::max(b.x1,p.x);b.y1=std::max(b.y1,p.y);}
   b.x0-=clearance;b.y0-=clearance;b.x1+=clearance;b.y1+=clearance;
   if(b.x1>=start.x-extent&&b.x0<=start.x+extent&&b.y1>=start.y-extent&&b.y0<=start.y+extent)nearby.push_back(b);
  }
  std::vector<int8_t> occupancy(N*N,-1);
  auto free=[&](int i){
   if(i<0||i>=N*N)return false;
   if(occupancy[i]<0){
    auto p=point(i);bool safe=allowed(p);
    for(const auto& b:nearby)if(safe && p.x>=b.x0 && p.x<=b.x1 && p.y>=b.y0 && p.y<=b.y1 && blockedBy(p,*b.polygon))safe=false;
    occupancy[i]=safe;
   }
   return occupancy[i]==1;
  };
  const int source=C*N+C,dest=ty*N+tx;
  if(!free(source)||!free(dest)){route.clear();blocked=true;noRoute=lastSearchFailed=true;return false;}
  using Entry=std::pair<double,int>;std::priority_queue<Entry,std::vector<Entry>,std::greater<Entry>> open;
  std::vector<double> costs(N*N,1e20);std::vector<int> parent(N*N,-1);costs[source]=0;open.push({0,source});
  int end=-1;unsigned expanded=0;
  while(!open.empty()&&expanded++<16000){
   int i=open.top().second;double score=open.top().first;open.pop();
   if(score>costs[i]+distance(point(i),point(dest))+.001&&i!=source)continue;
   if(i==dest){end=i;break;}
   for(int dy=-1;dy<=1;dy++)for(int dx=-1;dx<=1;dx++){
    if(!dx&&!dy)continue;int x=i%N+dx,y=i/N+dy,j=y*N+x;
    if(x<0||x>=N||y<0||y>=N||!free(j))continue;
    if(dx&&dy&&(!free(i+dx)||!free(i+dy*N)))continue;
    Point p=point(j);double lateral=cross(start,goal,p)/std::max(.1,distance(start,goal));
    double penalty=side&&lateral*side<-.15?.10:0;
    double cost=costs[i]+step*std::hypot(dx,dy)+penalty;
    if(cost>=costs[j])continue;costs[j]=cost;parent[j]=i;
    open.push({cost+distance(p,point(dest)),j});
   }
  }
  if(end<0){route.clear();blocked=true;noRoute=lastSearchFailed=true;return false;}
  std::vector<Point> path;for(int i=end;i!=source;i=parent[i]){if(i<0){blocked=true;return false;}path.push_back(point(i));}
  std::reverse(path.begin(),path.end());route.clear();routeStart=start;Point from=start;
  for(size_t i=0;i<path.size();){size_t far=i;for(size_t j=path.size();j-->i;)if(clear(from,path[j],allowed)){far=j;break;}route.push_back(path[far]);from=path[far];i=far+1;}
  if(!side&&route.size()){double d=cross(start,goal,route.front());if(std::abs(d)>.1)side=d>0?1:-1;}
  return true;
 }
 // Escape is independent of an unreachable mowing target. Pick a nearby
 // improving direction using the same obstacle and map checks, never wait for
 // an enclosing contour to expire before trying to leave it.
 bool planEscape(Pose pose,const std::function<bool(Point)>& allowed){
  Point start{pose.x,pose.y};escapeOrigin=start;escapeEnabled=contains(start);
  if(!escapeEnabled || !allowed(start))return false;
  Point best;double bestScore=-1e9;bool have=false;
  for(double length=.5;length<=3.0;length+=.5)for(int i=0;i<24;++i){
   const double heading=pose.yaw+i*3.141592653589793/12;
   Point candidate{start.x+length*std::cos(heading),start.y+length*std::sin(heading)};
   if(!clear(start,candidate,allowed))continue;
   double improvement=1e9;
   for(const auto& obstacle:obstacles){
    const double initial=signedClearance(start,obstacle.points);
    if(initial<radius())improvement=std::min(improvement,signedClearance(candidate,obstacle.points)-initial);
   }
   if(improvement<.1)continue;
   const double score=(contains(candidate)?0:10)+std::min(improvement,2.)-length*.2-std::abs(wrap(heading-pose.yaw))*.2;
   if(score>bestScore){bestScore=score;best=candidate;have=true;}
  }
  if(!have)return false;
  route={best};routeStart=start;active=true;blocked=noRoute=false;lastSearchFailed=false;return true;
 }
 void steer(Pose pose,double desired,double dt,const std::function<bool(Point)>& allowed,
            double& previousAngular,float& linear,float& angular) const {
  Point from{pose.x,pose.y};
  // Follow the furthest collision-free waypoint, not the obstacle outline.
  size_t far=0;for(size_t i=0;i<route.size();i++)if(clear(from,route[i],allowed))far=i;
  Point look=route[far];double d=distance(from,look);
  double error=wrap(std::atan2(look.y-pose.y,look.x-pose.x)-pose.yaw);
  // No blanket 0.1 m/s slowdown: preserve commanded speed on gentle detours.
  // Very sharp turns use rotation; tight curves are physically speed limited.
  double speed=std::max(0.,desired)*std::max(0.,std::cos(error));
  // Keep cruise speed through intermediate aims. Validate only up to this
  // aim, since the following route segment takes over there. Limit only very
  // close approaches to avoid jumping past a waypoint in one control update.
  speed=std::min(speed,d/std::max(.15,dt*2));
  if(std::abs(error)>1.0)speed=0;
  double turn=speed>0?2*speed*std::sin(error)/std::max(.6,std::min(1.4,d)):error;
  turn=std::max(-.65,std::min(.65,turn));
  turn=std::max(previousAngular-dt*1.2,std::min(previousAngular+dt*1.2,turn));
  // Validate the commanded arc as well as the straight shortcut. A curved
  // controller must not cut through an inflated obstacle at a waypoint.
  Point prev=from;bool arcSafe=true;
  const double horizon=speed>.001?std::min(2.,d/speed):.1;
  const int samples=std::max(1,int(std::ceil(horizon/.1)));
  for(int sample=1;sample<=samples;++sample){
    const double t=horizon*sample/samples;
    Point next=std::abs(turn)<.001?Point{pose.x+speed*t*std::cos(pose.yaw),pose.y+speed*t*std::sin(pose.yaw)}:
      Point{pose.x+speed/turn*(std::sin(pose.yaw+turn*t)-std::sin(pose.yaw)),pose.y-speed/turn*(std::cos(pose.yaw+turn*t)-std::cos(pose.yaw))};
    if(!clear(prev,next,allowed)){arcSafe=false;break;}prev=next;
  }
  if(!arcSafe){
   speed=0;turn=std::max(-.65,std::min(.65,error));
   // A tiny yaw command may round to zero in the wheel driver, permanently
   // pinning the robot next to a contour. Make a definite corrective pivot.
   if(std::abs(turn)<.15)turn=error<0?-.15:.15;
  }
  linear=speed;angular=turn;previousAngular=turn;
 }
 // Adopt only a computed path; current perception always stays on the control
 // thread and must not be overwritten with an older worker snapshot.
 void adoptPath(const Planner& other){
  route=other.route;routeStart=other.routeStart;active=other.active;blocked=other.blocked;noRoute=other.noRoute;
  lastPlan=other.lastPlan;lastGoal=other.lastGoal;side=other.side;lastSearchFailed=other.lastSearchFailed;
 }
 void setControlPose(Pose pose){escapeOrigin={pose.x,pose.y};escapeEnabled=contains(escapeOrigin);}
 bool visibleRoute(Pose pose,const std::function<bool(Point)>& allowed) const {
  for(auto p:route)if(distance({pose.x,pose.y},p)>.01 && clear({pose.x,pose.y},p,allowed))return true;
  return false;
 }
 uint64_t observationStamp() const{return lastStamp;}
private:
 uint64_t lastStamp=0,lastPlan=0;Point lastGoal;int side=0;
 std::vector<Polygon> pending;
 Point escapeOrigin;bool escapeEnabled=false,lastSearchFailed=false;
 static bool touches(const Polygon& a,const Polygon& b,double gap){
  for(auto p:a)if(separation(p,b)<=gap)return true;
  for(auto p:b)if(separation(p,a)<=gap)return true;return false;
 }
 void merge(Polygon p,uint64_t now){
  for(size_t i=0;i<obstacles.size();){
   if(touches(p,obstacles[i].points,.25)){
    p.insert(p.end(),obstacles[i].points.begin(),obstacles[i].points.end());p=hull(p);
    obstacles.erase(obstacles.begin()+i);
   }else ++i;
  }
  // Limit memory and planning cost. Conservative bounds instead of dropping vertices inward.
  if(p.size()>24){double x0=1e9,y0=1e9,x1=-1e9,y1=-1e9;for(auto v:p){x0=std::min(x0,v.x);y0=std::min(y0,v.y);x1=std::max(x1,v.x);y1=std::max(y1,v.y);}p={{x0,y0},{x1,y0},{x1,y1},{x0,y1}};}
  if(obstacles.size()>=24)return;
  obstacles.push_back({p,now});revision++;
 }
};
}

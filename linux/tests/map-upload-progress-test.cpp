#include "src/navigation/MowingUploadProgress.h"
#include <cassert>
#include <cstdio>
#include <vector>
#include <limits>
struct Point {
 float px=0,py=0;
 Point(){} Point(float x,float y):px(x),py(y){}
 float x(){return px;} float y(){return py;}
 void setXY(float x,float y){px=x;py=y;}
 void assign(Point &p){*this=p;}
};
struct Polygon {
 int numPoints=0;
 std::vector<Point> points;
 bool alloc(int count){if(count<0)return false;numPoints=count;points.resize(count);return true;}
 void dealloc(){numPoints=0;points.clear();}
};
struct PolygonList {
 std::vector<Polygon> polygons;
 bool alloc(int count){polygons.resize(count);return true;}
 void dealloc(){polygons.clear();}
};
struct Console {template<class T> void print(T){} template<class T> void println(T){} void println(){}} CONSOLE;
struct Log {void event(int){}} Logger;
const int EVT_USER_UPLOAD_MAP=1;
int memoryCorruptions=0,memoryAllocErrors=0;
int freeMemory(){return 100000;}
enum WayType {WAY_PERIMETER,WAY_EXCLUSION,WAY_DOCK,WAY_MOW,WAY_FREE};
struct Map {
 Polygon points,perimeterPoints,dockPoints,mowPoints,freePoints,pathFinderNodes;
 PolygonList exclusions,obstacles,pathFinderObstacles;
 MowingUploadProgress uploadProgress;
 int mowPointsIdx=0,dockPointsIdx=0,freePointsIdx=0,exclusionPointsCount=0,mapCRC=0,savedIndex=-1;
 int calcMapCRC(){return 0;} void dump(){} void save(){savedIndex=mowPointsIdx;}
 void finishedUploadingMap();void clearMap();bool setPoint(int,float,float);bool setWayCount(WayType,int);bool setExclusionLength(int,int);
};
#include "map-upload-production.inc"
static std::vector<Point> route{{1,1},{2,2},{3,3},{4,4}};
static Map original(){Map m;m.mowPoints.alloc(route.size());m.mowPoints.points=route;m.mowPointsIdx=2;return m;}
static void upload(Map &m,std::vector<Point> next,bool exclusion=false,int dockCount=2){
 std::vector<Point> all{{-1,-1},{5,-1},{5,5}};
 if(exclusion)for(int i=0;i<3;++i)all.push_back(Point(i,-2));
 for(int i=0;i<dockCount;++i)all.push_back(Point(10+i,20+i));
 all.insert(all.end(),next.begin(),next.end());
 for(unsigned i=0;i<all.size();++i)assert(m.setPoint(i,all[i].px,all[i].py));
 assert(m.mowPointsIdx==0); // never use old progress with partially received data
 assert(m.setWayCount(WAY_PERIMETER,3));
 assert(m.setWayCount(WAY_EXCLUSION,exclusion?3:0));
 assert(m.setWayCount(WAY_DOCK,dockCount));
 assert(m.setWayCount(WAY_MOW,next.size()));
 assert(m.setWayCount(WAY_FREE,0)); // used to reset a restored index again
 if(exclusion){assert(m.mowPointsIdx==0);assert(m.setExclusionLength(0,3));}
}
int main(){
 for(bool exclusion:{false,true}){
  Map m=original();upload(m,route,exclusion,4);assert(m.mowPointsIdx==2&&m.savedIndex==2);
  upload(m,route,exclusion,2);assert(m.mowPointsIdx==2);
  auto changed=route;changed[2].px+=.02f;upload(m,changed,exclusion);assert(m.mowPointsIdx==0);
  m=original();changed=route;std::swap(changed[0],changed[1]);upload(m,changed,exclusion);assert(m.mowPointsIdx==0);
  m=original();changed=route;changed.pop_back();upload(m,changed,exclusion);assert(m.mowPointsIdx==0);
  m=original();upload(m,{},exclusion);assert(m.mowPointsIdx==0);
 }
 Map m=original();m.clearMap();upload(m,route);assert(m.mowPointsIdx==0);
 m=original();assert(m.setPoint(0,99,99));upload(m,route);assert(m.mowPointsIdx==0);
 m=original();m.mowPointsIdx=99;upload(m,route);assert(m.mowPointsIdx==0);
 m=original();m.mowPointsIdx=-1;upload(m,route);assert(m.mowPointsIdx==0);
 m=original();m.mowPoints.points[0].px=std::numeric_limits<float>::quiet_NaN();upload(m,route);assert(m.mowPointsIdx==0);
 std::puts("PASS actual map upload: preserved ordered mowing route, changed dock/perimeter/exclusions, final count ACK, reorder/change/resize/reset/partial/invalid cases");
}

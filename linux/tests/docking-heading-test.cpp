// Small hardware-free fixture for the production Map::getDockingPos body.
#include <cassert>
#include <cmath>
#include <cstdio>
#include <vector>
struct Point {
  float px,py;
  float x(){return px;}
  float y(){return py;}
};
struct Map {
  struct { std::vector<Point> points; int numPoints; } dockPoints;
  bool getDockingPos(float&,float&,float&,int idx=-1);
  Map(std::initializer_list<Point> points) {dockPoints.points=points;dockPoints.numPoints=points.size();}
};
static float pointsAngle(float x,float y,float xx,float yy){return std::atan2(yy-y,xx-x);}
#include "docking-heading-production.inc"
static void heading(Map map,float expected,int idx=-1){
  float x=-999,y=-999,delta=-999;
  assert(map.getDockingPos(x,y,delta,idx));
  const auto endpoint=map.dockPoints.points[idx<0?map.dockPoints.numPoints-1:idx];
  assert(x==endpoint.px && y==endpoint.py);
  assert(std::fabs(delta-expected)<1e-5f);
}
static void invalid(Map map,int idx=-1){
  float x=12,y=34,delta=1.23f;
  assert(!map.getDockingPos(x,y,delta,idx));
  assert(x==12 && y==34 && delta==1.23f);
}
int main(){
  const float pi=std::acos(-1.0f);
  heading({{14.1f,-2.16f},{11.89f,-1.38f},{8.18f,.11f},{8.18f,.11f}},std::atan2(1.49f,-3.71f));
  heading({{0,0},{1,0}},0);
  heading({{0,0},{0,1}},pi/2);
  heading({{0,0},{-1,0}},pi);
  heading({{0,0},{0,-1}},-pi/2);
  heading({{0,0},{1,0},{1,0},{1,0}},0);
  heading({{0,0},{1,.001f},{1,0}},0);
  heading({{0,0},{1,0},{1,0},{2,1}},0,2);
  heading({{0,0},{0,0},{1,0}},pi,0);
  invalid({});invalid({{0,0}});invalid({{1,1},{1,1},{1,1}});
  invalid({{0,0},{.001f,.001f}});
  invalid({{0,0},{1,0}},2);invalid({{0,0},{1,0}},-2);
  std::puts("PASS docking heading: real duplicate route, axes, repeated/near endpoints, indexed lookup, invalid routes preserve state");
}

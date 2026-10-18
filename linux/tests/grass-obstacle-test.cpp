#include "src/camera/GrassObstacle.h"
#include <cassert>
#include <algorithm>
#include <cstdio>
int main(){
 assert(GrassObstacle::fresh(100,1099));assert(!GrassObstacle::fresh(100,1100));
 assert(!GrassObstacle::fresh(0,10));assert(!GrassObstacle::fresh(100,99));
 uint8_t labels[3600];std::fill(labels,labels+3600,1);
 assert(!GrassObstacle::detect(labels));
 // Geometry: upper three quarters excluded, top 70% wide, bottom full width.
 assert(!GrassObstacle::contains(.5f,.74f));
 assert(!GrassObstacle::contains(.14f,.75f));assert(GrassObstacle::contains(.16f,.75f));
 assert(GrassObstacle::contains(.84f,.75f));assert(!GrassObstacle::contains(.86f,.75f));
 assert(GrassObstacle::contains(0,1));assert(GrassObstacle::contains(1,1));
 assert(GrassObstacle::border(0,359));assert(GrassObstacle::border(639,359));
 assert(GrassObstacle::border(320,270));assert(!GrassObstacle::border(320,310));
 int cells[3600],total=0;
 for(int y=0;y<45;y++)for(int x=0;x<80;x++){
   if(GrassObstacle::cellInside(x,y))cells[total++]=y*80+x;
   else labels[y*80+x]=0;
 }
 assert(total>700&&total<850);
 assert(!GrassObstacle::detect(labels)); // All excluded cells are obstacles.
 const int threshold=(total+9)/10;
 for(int i=0;i<threshold-1;i++)labels[cells[i]]=0;
 assert(!GrassObstacle::detect(labels));
 labels[cells[threshold-1]]=0;assert(GrassObstacle::detect(labels));
 for(int i=0;i<total;i++)labels[cells[i]]=i<total*7/100?0:1;
 assert(!GrassObstacle::detect(labels,false));assert(GrassObstacle::detect(labels,true));
 for(int i=0;i<total;i++)labels[cells[i]]=3;
 assert(!GrassObstacle::detect(labels,true)); // Non-grass alone is not obstacle class.
 assert(GrassObstacle::limit(.5,true,true,false,true,true)==.1f);
 assert(GrassObstacle::limit(.02,true,true,false,true,true)==.02f);
 assert(GrassObstacle::limit(.5,false,true,false,true,true)==.5f);
 assert(GrassObstacle::limit(.5,true,false,false,true,true)==.5f);
 assert(GrassObstacle::limit(.5,true,true,true,true,true)==.5f);
 assert(GrassObstacle::limit(.5,true,true,false,true,false)==.5f);
 assert(GrassObstacle::limit(0,true,true,false,true,true)==0);
 uint8_t image[160*120*3];std::fill(image,image+sizeof(image),23);
 GrassObstacle::drawContour(image,160,120,1);
 assert(image[(119*160)*3]==255&&image[(119*160)*3+1]==0);
 assert(image[(100*160+80)*3]==23); // Camera image inside remains unchanged.
 GrassObstacle::drawContour(image,160,120,0);assert(image[(119*160)*3+1]==255);
 GrassObstacle::drawContour(image,160,120,-1);assert(image[(119*160)*3]==255&&image[(119*160)*3+1]==180);
 puts("PASS ROI, hysteresis, class selection, opt-in, automatic mode, direction, freshness and no acceleration");
}

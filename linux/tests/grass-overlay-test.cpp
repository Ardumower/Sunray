#include "camera/GrassOverlay.h"
#include <fstream>
#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
int main(int argc,char**argv){
 if(argc!=3)return 2;
 std::vector<uint8_t> input(640*360*3);std::ifstream f(argv[1],std::ios::binary);if(!f.read((char*)input.data(),input.size()))return 3;
 std::vector<float> ref(5*45*80);std::ifstream r(argv[2],std::ios::binary);if(!r.read((char*)ref.data(),ref.size()*4))return 4;
 GrassOverlay overlay;for(int k=1;k<=50;k++)overlay.submit(input.data(),640,360,k);
 std::vector<uint8_t> result;uint64_t stamp=0;double ms=0;
 for(int n=0;n<500;n++){if(overlay.latest(result,stamp,ms)&&stamp==50)break;std::this_thread::sleep_for(std::chrono::milliseconds(10));}
 if(stamp!=50||result.size()!=input.size())return 5;
 const uint8_t colors[5][3]={{235,65,65},{45,210,65},{160,90,220},{230,155,40},{70,130,230}};
 for(int y=0;y<360;y++)for(int x=0;x<640;x++){
   int cell=(y/8)*80+x/8,best=0;for(int c=1;c<5;c++)if(ref[c*3600+cell]>ref[best*3600+cell])best=c;
   for(int c=0;c<3;c++){int i=(y*640+x)*3+c;if(result[i]!=(input[i]*20+colors[best][c]*80)/100)return 6;}
 }
 std::cout<<"PASS latest frame 50, exact overlay matches independent RKNN reference; inference "<<ms<<" ms\n";
}

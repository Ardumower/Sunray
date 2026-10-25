#!/usr/bin/env python3
"""Check source-frame time semantics and render the production timestamp overlay."""
from pathlib import Path
import subprocess
import tempfile
import sys

root = Path(__file__).resolve().parents[2]
source = (root / 'sunray/src/camera/CameraStreamer.cpp').read_text()
drawing = source[source.index('static inline void putPixel('):source.index('#else // !__linux__')]
text = source[source.index('static void drawText('):source.index('static void drawVerticalArrow(')]
# Every real view (normal/grass/texture/auxiliary cameras) is stamped at the
# common encoder, while model input stays clean and the error image is explicit.
send = source[source.index('auto sendView ='):source.index('sendView(outRGB,')]
assert send.index('drawFrameTimestamp(') < send.index('jpegEncodeRGB(')
assert 'frameClock.label(stamp)' in send and 'uint64_t stamp' in send
fallback = source[source.index('void CameraStreamer::buildAndSendFrame()'):]
assert 'drawFrameTimestamp(' in fallback and 'CAM ERROR' in fallback

harness = r'''
#include "src/camera/FrameTimestamp.h"
#include "src/camera/StreamGeometry.h"
#include <algorithm>
#include <cassert>
#include <cstring>
#include <vector>
#include <fstream>
#include <iostream>
#include <cstdlib>
'''
tests = r'''
int main(int argc,char** argv){
 // Widescreen, 4:3 and portrait cameras keep all image edges without
 // exceeding the requested transport size. No 16:9 -> 4:3 distortion.
 auto size=CameraStream::fit(640,360,160,120);
 assert(size.width==160 && size.height==90);
 size=CameraStream::fit(640,480,160,120);
 assert(size.width==160 && size.height==120);
 size=CameraStream::fit(360,640,160,120);
 assert(size.width==67 && size.height==120);
 size=CameraStream::fit(1920,1080,320,240);
 assert(size.width==320 && size.height==180);
 setenv("TZ","UTC",1);tzset();
 using namespace std::chrono;
 CameraFrameClock clock(1000,system_clock::time_point(milliseconds(10000)));
 assert(clock.label(1234)=="00:00:10.234");
 assert(clock.label(600)=="00:00:09.600"); // delayed inference retains capture time
 assert(clock.label(1235)!=clock.label(1234)); // sub-second frames are distinct
 for(int i=0;i<100;++i)assert(clock.label(600)=="00:00:09.600"); // no send-time refresh
 CameraFrameClock midnight(1000,system_clock::time_point(milliseconds(86399500)));
 assert(midnight.label(1499)=="23:59:59.999");
 assert(midnight.label(1500)=="00:00:00.000");
 const uint64_t wrap=uint64_t(1)<<32;
 CameraFrameClock longUptime(wrap+1000,system_clock::time_point(milliseconds(10000)));
 assert(longUptime.label(wrap+1234)==clock.label(1234));
 for(int width:{32,160,320,640}){
  int height=width/2;std::vector<uint8_t> pixels(size_t(width)*height*3+32,77);
  drawFrameTimestamp(pixels.data()+16,width,height,"17:23:45.678");
  for(int i=0;i<16;++i)assert(pixels[i]==77&&pixels[pixels.size()-1-i]==77);
  assert(std::count(pixels.begin()+16,pixels.end()-16,255)>0);
 }
 if(argc>1){
  const int w=320,h=180;std::vector<uint8_t> preview(w*h*3);
  for(int y=0;y<h;++y)for(int x=0;x<w;++x){size_t i=(y*w+x)*3;preview[i]=x*200/w;preview[i+1]=y*170/h;preview[i+2]=70;}
  drawText(preview.data(),w,h,8,8,"CAM",2,255,255,255);
  drawFrameTimestamp(preview.data(),w,h,"17:23:45.678");
  std::ofstream out(argv[1],std::ios::binary);out<<"P6\n"<<w<<" "<<h<<"\n255\n";out.write((char*)preview.data(),preview.size());
 }
 std::cout<<"PASS capture-time milliseconds, delayed/repeated results, midnight, 32-bit header wrap, all-view encoder and clipped rendering\n";
}
'''
with tempfile.TemporaryDirectory() as work:
    path = Path(work)
    (path / 'test.cpp').write_text(harness + drawing + text + tests)
    subprocess.run(['c++', '-std=c++11', '-I', str(root / 'sunray'), str(path / 'test.cpp'), '-o', str(path / 'test')], check=True)
    subprocess.run([str(path / 'test'), *sys.argv[1:]], check=True)

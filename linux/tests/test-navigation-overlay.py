#!/usr/bin/env python3
"""Exercise the production snapshot publisher and renderer without robot hardware."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
source = (root / 'sunray/src/camera/OverlayData.cpp').read_text()
publisher = source[source.index('namespace { std::atomic<int>'):].replace('#endif', '')
streamer = (root / 'sunray/src/camera/CameraStreamer.cpp').read_text()
pixels = streamer[streamer.index('static inline void putPixel'):streamer.index('#else // !__linux__')]
text = streamer[streamer.index('static void drawText'):streamer.index('static void drawVerticalArrow')]
render = streamer[streamer.index('static void drawNavigationOverlay'):streamer.index('static bool jpegDecodeToRGB')]
program = r'''
#include <atomic>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>
#include <algorithm>
#include <fstream>
#include "sunray/src/camera/NavigationOverlay.h"
#include "sunray/src/navigation/ReflectorDockingGate.h"
enum { OP_IDLE, OP_MOW, OP_CHARGE, OP_ERROR, OP_DOCK };
enum { LOC_GPS, LOC_IMU_ODO_ONLY, LOC_GUIDANCE_SHEET, LOC_REFLECTOR_STRIPS };
enum { SOL_INVALID, SOL_FLOAT, SOL_FIXED };
struct { int solution=SOL_FIXED; } gps;
struct { ReflectorDockingGate reflectorDockingGate; int stateLocalizationMode=LOC_GPS; } stateEstimator;
struct Op { int mode=OP_IDLE; int getGoalOperationType(){return mode;} } op;
Op* activeOp=&op;
struct { struct { bool value=false; bool active(){return value;} } cameraSearch; } dockOp;
''' + publisher + pixels + text + render + r'''
int main(int argc,char**argv){
 using namespace NavigationOverlay;
 auto check=[](Mode mode){cameraUpdateNavigationOverlay();assert(cameraGetNavigationMode()==mode);};
 check(Hidden);
 op.mode=OP_MOW;check(RtkFix);
 gps.solution=SOL_FLOAT;check(RtkFloat);
 gps.solution=SOL_INVALID;check(GpsWait);
 stateEstimator.stateLocalizationMode=LOC_IMU_ODO_ONLY;check(Odometry);
 stateEstimator.stateLocalizationMode=LOC_GUIDANCE_SHEET;check(Guidance);
 stateEstimator.stateLocalizationMode=LOC_REFLECTOR_STRIPS;check(CameraWait);
 auto& gate=stateEstimator.reflectorDockingGate;
 for(auto phase:{ReflectorDockingGate::Dock,ReflectorDockingGate::Undock}){gate.phase=phase;check(Camera);}
 for(auto phase:{ReflectorDockingGate::AcquireDock,ReflectorDockingGate::AcquireUndock}){gate.phase=phase;check(CameraWait);}
 gate.phase=ReflectorDockingGate::Blocked;check(CameraStop);
 op.mode=OP_DOCK;dockOp.cameraSearch.value=true;check(CameraSearch);
 for(int mode:{OP_IDLE,OP_CHARGE,OP_ERROR}){op.mode=mode;check(Hidden);}
 for(int w:{160,320,640}){
  const int h=w*3/4;
  for(int mode=Hidden;mode<=Guidance;++mode){
   navigationMode.store(mode);
   std::vector<uint8_t> rgb(w*h*3,150);drawNavigationOverlay(rgb.data(),w,h);
   bool changed=std::any_of(rgb.begin(),rgb.end(),[](uint8_t x){return x!=150;});
   assert(changed==(mode!=Hidden));
   if(argc>1){std::ofstream f(std::string(argv[1])+"/nav-"+std::to_string(w)+"-"+std::to_string(mode)+".ppm",std::ios::binary);
    f<<"P6\n"<<w<<" "<<h<<"\n255\n";f.write((char*)rgb.data(),rgb.size());}
  }
 }
}
'''
with tempfile.TemporaryDirectory() as tmp:
    cpp = Path(tmp) / 'test.cpp'
    cpp.write_text(program)
    binary = Path(tmp) / 'test'
    subprocess.run(['c++', '-std=c++14', '-fsanitize=address,undefined', '-I', str(root), str(cpp), '-o', str(binary)], check=True)
    import sys
    subprocess.run([str(binary)] + sys.argv[1:], check=True)
print('Navigation snapshot and overlay rendering passed (160/320/640 pixels).')

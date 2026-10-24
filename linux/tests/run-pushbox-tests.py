#!/usr/bin/env python3
"""Exercise the production CAN button handler without connecting to a robot."""
import pathlib, subprocess, tempfile
root = pathlib.Path(__file__).resolve().parents[2]
s = (root/'sunray/src/driver/CanRobotDriver.cpp').read_text()
a = s.index('case owlrecv::can_val_button_state: {')
b = s.index('\n              }', a)
handler = s[a:b]
source = r'''
#include <cassert>
#include <cstdint>
#include <cstdio>
#define RECEIVER_PUSHBOX_NODE_ID 3
#define MY_NODE_ID 1
#define BIN 2
namespace owlrecv { const int can_val_button_state=1; }
enum { OP_MOW, OP_DOCK, OP_IDLE, EVT_AUDIO_SHEEP, SND_READY };
struct Output { template<class... T> void print(T...){} template<class... T> void println(T...){} } CONSOLE;
unsigned actions=0, sounds=0;
void setOperation(int,bool){++actions;}
struct { void event(int){++actions;} } Logger;
struct { void sound(int,bool){++sounds;} } buzzer;
uint8_t lastPushboxButtons=0;
void packet(uint8_t state,int src=3,int dst=1,int len=8){
 struct { int can_dlc; } frame{len};
 struct { struct { int sourceNodeID,destNodeID; } sourceAndDest; } node{{src,dst}};
 union { uint8_t byteVal[4]; int intValue; } data;
 data.byteVal[0]=state; data.byteVal[1]=0xab; data.byteVal[2]=0xcd; data.byteVal[3]=0xef;
 switch(1){
''' + handler + r'''
 }
}
int main(){
 packet(1); for(int i=0;i<100;++i) packet(1);
 assert(actions==1 && sounds==1);
 packet(0); packet(1); assert(actions==2);
 packet(0); packet(4,2); packet(4,3,2); packet(4,3,1,4);
 assert(actions==2);
 packet(4); packet(4); assert(actions==3);
 packet(0); packet(8); assert(actions==4);
 packet(0); assert(sounds==4); // dirty upper CAN bytes are ignored
 puts("PASS Sunray single edge, hold/retries, release/repress, source/destination/length and dirty upper bytes");
}
'''
with tempfile.TemporaryDirectory() as tmp:
 p=pathlib.Path(tmp); (p/'test.cpp').write_text(source)
 subprocess.run(['c++','-std=c++14',str(p/'test.cpp'),'-o',str(p/'test')],check=True)
 subprocess.run([str(p/'test')],check=True)

#!/usr/bin/env python3
"""Run production frame parser against delayed and incomplete network packets."""
from pathlib import Path
import subprocess,tempfile
root=Path(__file__).resolve().parents[2]
s=(root/'sunray/src/net/WebSocketClient.cpp').read_text()
parser=s[s.index('bool WebSocketClient::readExact('):]
harness=r'''
#include <cassert>
#include <cstdint>
#include <string>
#include <vector>
#include <iostream>
using String=std::string;
unsigned long clockMs=0;
unsigned long millis(){return clockMs;}
void delay(unsigned long n){clockMs+=n;}
struct Log {template<class T> void print(T){} template<class T> void println(T){}} CONSOLE;
struct WsLock {explicit WsLock(int&) {}};
struct FakeClient {
 std::vector<std::pair<unsigned long,uint8_t>> data;size_t pos=0;
 bool available(){return pos<data.size()&&clockMs>=data[pos].first;}
 int read(){return available()?data[pos++].second:-1;}
 void write(uint8_t){}
};
struct WebSocketClient {
 FakeClient _client;int _mtx=0;bool online=true;
 bool connectedUnlocked(){return online;}
 void closeUnlocked(){online=false;}
 bool readExact(uint8_t*,size_t,unsigned long);
 bool pollText(String&);
};
'''
tests=r'''
int main(){
 // Header bytes arrive 200 ms apart; payload is delayed by 8 seconds.
 {clockMs=0;WebSocketClient w;w._client.data={{0,0x81},{200,3},{8000,'P'},{8200,'B'},{9000,'!'}};
 String out;assert(w.pollText(out));assert(out=="PB!");assert(w.online);assert(clockMs==9000);}
 // An incomplete frame closes at a bounded deadline; it is never dispatched.
 {clockMs=0;WebSocketClient w;w._client.data={{0,0x81},{200,3},{400,'P'}};
 String out;assert(!w.pollText(out));assert(!w.online);assert(clockMs==30000);}
 // Missing second header byte cannot leave the stream parser misaligned.
 {clockMs=0;WebSocketClient w;w._client.data={{0,0x81}};
 String out;assert(!w.pollText(out));assert(!w.online);assert(clockMs==30000);}
 std::cout<<"PASS production WebSocket parser: delayed header/payload, complete-frame deadline, incomplete-header recovery\n";
}
'''
with tempfile.TemporaryDirectory() as work:
 p=Path(work);(p/'test.cpp').write_text(harness+parser+tests)
 subprocess.run(['c++','-std=c++14',str(p/'test.cpp'),'-o',str(p/'test')],check=True)
 subprocess.run([str(p/'test')],check=True)

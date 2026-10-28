#include "../../sunray/src/net/CloudWorker.h"
#include "../src/LinuxBoardName.h"
#include <cassert>
#include <iostream>
#include <future>
using Clock=std::chrono::steady_clock;
struct Fake {
 std::atomic<int> gate{0},entered{0},connects{0},receives{0},sends{0},binaries{0};
 std::atomic<bool> up{false};
 std::thread::id owner;
 void block(int phase){entered=phase;while(gate==phase)std::this_thread::sleep_for(std::chrono::milliseconds(1));}
 CloudWorker::Transport io(){return {
 [this]{owner=std::this_thread::get_id();++connects;block(1);up=true;return true;},
 [this]{assert(owner==std::this_thread::get_id());return up.load();},
 [this]{assert(owner==std::this_thread::get_id());block(4);up=false;},
 [this](std::string& s){assert(owner==std::this_thread::get_id());block(2);if(receives++==0){s="AT+V";return true;}return false;},
 [this](const std::string&){assert(owner==std::this_thread::get_id());++sends;block(3);return true;},
 [this](const unsigned char*,size_t){assert(owner==std::this_thread::get_id());++binaries;block(5);return true;}};}
};
void until(const std::function<bool()>& f){auto end=Clock::now()+std::chrono::seconds(3);while(!f()){assert(Clock::now()<end);std::this_thread::sleep_for(std::chrono::milliseconds(1));}}
void controlProbe(CloudWorker& w){auto a=Clock::now();for(int i=0;i<1000;i++){CloudWorker::Request r;w.take(r);w.reply(0,"");unsigned char b=0;w.camera(&b,1);}auto ms=std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now()-a).count();std::cout<<"1000 control polls under stalled network: "<<ms<<" ms\n";assert(ms<50);}
int main(){
 // Waiting for a slow control/status response must not pause camera delivery.
 {Fake f;CloudWorker w(f.io(),5);w.start();CloudWorker::Request request;
  until([&]{return w.take(request);});
  for(int i=1;i<=5;++i){unsigned char frame=static_cast<unsigned char>(i);w.camera(&frame,1);until([&]{return f.binaries>=i;});}
  assert(f.sends==0); // video arrived before the control response was available
  until([&]{return w.reply(request.session,"V,delayed");});until([&]{return f.sends==1;});}
 for(int phase:{1,2,3,4,5}){
  Fake f;f.gate=phase;CloudWorker w(f.io(),5);w.start();
  if(phase>=3){CloudWorker::Request r;until([&]{return w.take(r);});assert(r.text=="AT+V");until([&]{return w.reply(r.session,"V,test");});
   if(phase==4)f.up=false;
   if(phase==5){unsigned char frame=1;w.camera(&frame,1);}}
  until([&]{return f.entered==phase;});controlProbe(w);f.gate=0;
 }
 // Auxiliary training RPCs stay on the network thread, even if local I/O stalls.
 {Fake f;auto io=f.io();std::atomic<bool> entered{false},release{false};
  io.auxiliary=[&](const std::string&,std::string& answer){entered=true;while(!release.load())std::this_thread::sleep_for(std::chrono::milliseconds(1));answer="TRAIN:reply";return true;};
  CloudWorker w(io,5);w.start();until([&]{return entered.load();});controlProbe(w);
  CloudWorker::Request r;assert(!w.take(r));release=true;until([&]{return f.sends==1;});}
 // Distinct camera views retain their own newest frame under stalled I/O.
 {Fake f;f.receives=1;f.gate=2;auto io=f.io();std::atomic<int> seen{0};
  io.binary=[&](const unsigned char* data,size_t length){assert(length==17);assert(data[16]==2);seen.fetch_or(data[7]==101?2:1);return true;};
  CloudWorker w(io,5);w.start();until([&]{return f.entered==2;});
  unsigned char raw[17]={'S','R','B','X',1,1,0x80,0},texture[17]={'S','R','B','X',1,1,0x80,101};
  raw[16]=1;w.camera(raw,17);texture[16]=1;w.camera(texture,17);
  raw[16]=2;w.camera(raw,17);texture[16]=2;w.camera(texture,17);
  f.gate=0;until([&]{return seen.load()==3;});}
 // A quiet LTE connection survives missing replies beyond the former 15 s limit.
 {Fake f;f.receives=1;CloudWorker w(f.io(),5);w.start();until([&]{return f.connects==1;});
  std::this_thread::sleep_for(std::chrono::seconds(16));assert(f.connects==1);assert(w.connected());controlProbe(w);}
 // Expired commands are never executed after a control pause.
 {Fake f;CloudWorker w(f.io(),5);w.start();until([&]{return f.receives>0;});std::this_thread::sleep_for(std::chrono::milliseconds(550));CloudWorker::Request r;assert(!w.take(r));}
 // Disconnect/reconnect clears session state, rejects old replies.
 {Fake f;CloudWorker w(f.io(),5);w.start();CloudWorker::Request first;until([&]{return w.take(first);});until([&]{return w.reply(first.session,"first");});until([&]{return f.sends==1;});f.up=false;until([&]{return f.connects>=2;});f.receives=0;CloudWorker::Request second;until([&]{return w.take(second);});assert(first.session!=second.session);assert(w.reply(first.session,"obsolete"));assert(f.sends==1);until([&]{return w.reply(second.session,"second");});until([&]{return f.sends==2;});}
 auto a=Clock::now();for(int i=0;i<1000;i++)assert(!linuxBoardName().empty());assert(Clock::now()-a<std::chrono::milliseconds(50));
 std::cout<<"PASS: connect/read/write/close/camera isolation, stale commands, sessions, cached board name\n";
}

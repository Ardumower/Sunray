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
 for(int phase:{1,2,3,4,5}){
  Fake f;f.gate=phase;CloudWorker w(f.io(),5);w.start();
  if(phase>=3){CloudWorker::Request r;until([&]{return w.take(r);});assert(r.text=="AT+V");until([&]{return w.reply(r.session,"V,test");});
   if(phase==4)f.up=false;
   if(phase==5){unsigned char frame=1;w.camera(&frame,1);}}
  until([&]{return f.entered==phase;});controlProbe(w);f.gate=0;
 }
 // Expired commands are never executed after a control pause.
 {Fake f;CloudWorker w(f.io(),5);w.start();until([&]{return f.receives>0;});std::this_thread::sleep_for(std::chrono::milliseconds(550));CloudWorker::Request r;assert(!w.take(r));}
 // Disconnect/reconnect clears session state, rejects old replies.
 {Fake f;CloudWorker w(f.io(),5);w.start();CloudWorker::Request first;until([&]{return w.take(first);});until([&]{return w.reply(first.session,"first");});until([&]{return f.sends==1;});f.up=false;until([&]{return f.connects>=2;});f.receives=0;CloudWorker::Request second;until([&]{return w.take(second);});assert(first.session!=second.session);assert(w.reply(first.session,"obsolete"));assert(f.sends==1);until([&]{return w.reply(second.session,"second");});until([&]{return f.sends==2;});}
 auto a=Clock::now();for(int i=0;i<1000;i++)assert(!linuxBoardName().empty());assert(Clock::now()-a<std::chrono::milliseconds(50));
 std::cout<<"PASS: connect/read/write/close/camera isolation, stale commands, sessions, cached board name\n";
}

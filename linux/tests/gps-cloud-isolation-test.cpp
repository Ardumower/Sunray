// Production UBLOX parser + CloudWorker, with an in-memory serial receiver.
// No robot, radio, cloud service or motor commands are used by this test.
#include "../../sunray/src/net/CloudWorker.h"
#include <cassert>
#include <deque>
#include <iostream>
#include "../../sunray/src/ublox/ublox.h"
#include "../../sunray/events.h"
#undef min
#undef max
#undef round
#undef yield

using Clock = std::chrono::steady_clock;
static unsigned long clockOffset = 0;
unsigned long millis() {
  static const auto start = Clock::now();
  return clockOffset + std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now()-start).count();
}
void delay(uint32_t ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }
void delayMicroseconds(uint32_t us) { std::this_thread::sleep_for(std::chrono::microseconds(us)); }
void thread_yield() { std::this_thread::yield(); }
EventLogger::EventLogger() {}
void EventLogger::event(EventCode) { assert(false && "unexpected hardware configuration"); }
EventLogger Logger;

struct Receiver : HardwareSerial {
  std::deque<uint8_t> input;
  uint32_t generation=0;
  uint32_t connectionGeneration() const override { return generation; }
  int available() override { return input.size(); }
  int read() override { assert(!input.empty()); auto c=input.front(); input.pop_front(); return c; }
  size_t write(uint8_t) override { assert(false && "must not configure/reset receiver"); return 0; }
  void position(uint32_t tow, uint8_t solution, bool corrupt=false) {
    // UBX-NAV-RELPOSNED v1, 64-byte payload, including genuine UBX checksum.
    std::vector<uint8_t> p(70, 0);
    p[0]=0xb5; p[1]=0x62; p[2]=1; p[3]=0x3c; p[4]=64; p[6]=1;
    for (int i=0;i<4;++i) p[10+i]=(tow>>(8*i))&255;
    p[66]=7 | (solution<<3); // gnssFixOK, diffSoln, relPosValid, carrSoln
    uint8_t a=0,b=0;
    for (size_t i=2;i<p.size();++i) { a+=p[i]; b+=a; }
    p.push_back(a); p.push_back(corrupt ? b^1 : b);
    input.insert(input.end(),p.begin(),p.end());
  }
};

struct Network {
  std::atomic<int> blocked{0};
  std::atomic<bool> release{false},up{false};
  int phase;
  bool delivered=false;
  explicit Network(int phase):phase(phase) {}
  void stall(int at) {
    if (phase!=at) return;
    blocked=at;
    while(!release.load()) std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  CloudWorker::Transport transport() { return {
    [this]{stall(1); up=true; return true;},
    [this]{return up.load();},
    [this]{stall(4); up=false;},
    [this](std::string& s){stall(2); if(!delivered){delivered=true;s="AT+V";return true;} return false;},
    [this](const std::string&){stall(3);return true;},
    [this](const unsigned char*,size_t){stall(5);return true;}
  }; }
};

int main() {
  Receiver serial;
  UBLOX gps;
  gps.begin(serial,115200); // GPS_CONFIG=false in the isolated test build
  uint32_t tow=1000;
  for (int phase : {1,2,3,4,5}) {
    Network network(phase);
    CloudWorker worker(network.transport(),5);
    worker.start();
    auto deadline=Clock::now()+std::chrono::seconds(3);
    while(network.blocked!=phase) {
      assert(Clock::now()<deadline);
      CloudWorker::Request request;
      if(worker.take(request)) {
        while(!worker.reply(request.session,"V,test")) {
          assert(Clock::now()<deadline);
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        if(phase==4) network.up=false;
        if(phase==5) { unsigned char frame=0; worker.camera(&frame,1); }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    unsigned count=0;
    unsigned long worst=0, nextPosition=millis();
    auto until=Clock::now()+std::chrono::milliseconds(3400); // > GPS's 3 s watchdog
    do {
      const auto before=millis();
      if(before>=nextPosition) {
        tow+=200; serial.position(tow,SOL_FIXED); nextPosition=before+200; ++count;
      }
      gps.run();
      CloudWorker::Request request;
      worker.take(request);
      assert(gps.solution==SOL_FIXED);
      assert(gps.iTOW==tow);
      worst=std::max(worst,millis()-before);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    } while(Clock::now()<until);
    assert(count>=16); assert(worst<50);
    network.release=true;
    worker.stopAndJoin();
    std::cout<<"PASS GPS FIX during network phase "<<phase<<": "<<count<<" positions, worst poll "<<worst<<" ms\n";
  }
  // Safety remains intact: real receiver INVALID, FLOAT, silence and bad CRC.
  serial.position(++tow,SOL_INVALID); gps.run(); assert(gps.solution==SOL_INVALID);
  serial.position(++tow,SOL_FLOAT); gps.run(); assert(gps.solution==SOL_FLOAT);
  serial.position(++tow,SOL_FIXED); gps.run(); assert(gps.solution==SOL_FIXED);
  clockOffset+=3100;
  serial.position(++tow,SOL_FIXED,true); gps.run();
  assert(gps.solution==SOL_INVALID); assert(gps.chksumErrorCounter>0);
  serial.position(++tow,SOL_FIXED); gps.run(); assert(gps.solution==SOL_FIXED);
  clockOffset+=3100; gps.run(); assert(gps.solution==SOL_INVALID);
  serial.position(++tow,SOL_FIXED); gps.run(); assert(gps.solution==SOL_FIXED);
  ++serial.generation; // GPS_CONFIG=false must also survive USB reconnect without TX
  serial.position(++tow,SOL_FIXED); gps.run();
  assert(!gps.isConfiguring() && gps.solution==SOL_FIXED);
  clockOffset+=11000; gps.run();
  assert(!gps.isConfiguring() && gps.solution==SOL_INVALID);
  serial.position(++tow,SOL_FIXED); gps.run(); assert(gps.solution==SOL_FIXED);
  std::cout<<"PASS receiver INVALID/FLOAT, corrupt packets, missing GPS, reconnect and GPS_CONFIG=false\n";
}

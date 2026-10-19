#include <cassert>
#include <vector>
#include <iostream>
#include "../../sunray/src/ublox/SignalDiagnostics.h"
int main() {
  UbloxSignalDiagnostics d;
  assert(!d.available);
  std::vector<char> packet(8 + 12*16, 0); packet[5] = 12;
  for (int i=0; i<12; ++i) {
    packet[8+i*16]=i%7;
    packet[9+i*16]=i+1;
    packet[10+i*16]=i%2;
    packet[14+i*16]=i*4;
    packet[18+i*16]=(i%2) ? 9 : 1;
  }
  assert(d.update(packet.data(),packet.size(),1234));
  assert(d.available && d.receivedAt==1234 && d.reported==12);
  assert(d.nonzero==11 && d.used==6 && d.sum==264 && d.maximum==44);
  assert(d.topCount==8 && d.top[0].sv==12 && d.top[0].cno==44);
  assert(d.top[7].sv==5 && d.top[7].cno==16);
  // Truncated, unsupported-version and inconsistent-count frames preserve last valid sample.
  assert(!d.update(packet.data(),packet.size()-1,2000));
  packet[4]=1; assert(!d.update(packet.data(),packet.size(),2000)); packet[4]=0;
  packet[5]=13; assert(!d.update(packet.data(),packet.size(),2000));
  assert(!d.update(packet.data(),4,2000));
  assert(d.receivedAt==1234 && d.maximum==44);
  packet.assign(8+16,0); packet[5]=1;
  assert(d.update(packet.data(),packet.size(),3000));
  assert(d.reported==1 && d.nonzero==0 && d.sum==0 && d.topCount==0 && d.maximum==0);
  packet.assign(8,0); assert(d.update(packet.data(),packet.size(),4000));
  assert(d.reported==0 && d.available && d.receivedAt==4000);
  std::cout << "PASS NAV-SIG strengths, top8, no reception, empty and malformed frames\n";
}

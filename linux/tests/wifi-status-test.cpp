// Include the implementation to inject a stalled probe into the private worker.
#include "../src/LinuxWifiStatus.cpp"
#include <cassert>
#include <cstdlib>
#include <iostream>

int main() {
  // run-cloud-tests.sh supplies a fake wpa_cli in this test's PATH. Exercise
  // the production command, parser and real process timeout without WiFi.
  setenv("WIFI_TEST_STATE", "COMPLETED", 1);
  assert(queryWifiStatus().connected);
  setenv("WIFI_TEST_STATE", "INACTIVE", 1);
  assert(queryWifiStatus().inactive);
  setenv("WIFI_TEST_STATE", "FAIL", 1);
  assert(!queryWifiStatus().connected);
  setenv("WIFI_TEST_STATE", "STALL", 1);
  auto queryStart=std::chrono::steady_clock::now();
  assert(!queryWifiStatus().connected);
  auto queryTime=std::chrono::steady_clock::now()-queryStart;
  assert(queryTime>=std::chrono::seconds(1) && queryTime<std::chrono::seconds(4));
  std::cout<<"PASS actual WiFi probe parser, failure and bounded process timeout\n";
  std::atomic<bool> entered{false}, release{false};
  std::atomic<int> phase{0};
  WifiStatusWorker worker([&] {
    entered=true;
    while(!release.load()) std::this_thread::sleep_for(std::chrono::milliseconds(1));
    const int step=phase.load();
    return LinuxWifiStatus{step==0,step==1};
  }, 1);
  auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
  while(!entered.load()) { assert(std::chrono::steady_clock::now()<deadline); std::this_thread::yield(); }
  // The exact production snapshot path must remain responsive even when the
  // status probe hangs longer than the GPS watchdog.
  unsigned polls=0;
  auto until=std::chrono::steady_clock::now()+std::chrono::milliseconds(3400);
  do {
    auto started=std::chrono::steady_clock::now();
    auto value=worker.snapshot();
    assert(!value.connected && !value.inactive);
    assert(std::chrono::steady_clock::now()-started<std::chrono::milliseconds(50));
    ++polls;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  } while(std::chrono::steady_clock::now()<until);
  release=true;
  for(int state : {0,1,2}) {
    phase=state;
    deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
    for (;;) {
      auto value=worker.snapshot();
      if(value.connected==(state==0) && value.inactive==(state==1)) break;
      assert(std::chrono::steady_clock::now()<deadline);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  assert(polls>500);
  std::cout<<"PASS WiFi status stall: "<<polls<<" control polls; connected/inactive/failure transitions\n";
}

#include "src/camera/LatestFrameWorker.h"
#include <cassert>
#include <chrono>
#include <future>
#include <vector>

int main(){
  using namespace std::chrono;
  std::promise<void> entered,release,finished;
  auto released=release.get_future().share();
  auto started=entered.get_future();auto done=finished.get_future();
  std::vector<int> processed;
  const auto caller=std::this_thread::get_id();
  LatestFrameWorker<int> worker([&](const int& frame){
    assert(std::this_thread::get_id()!=caller);
    processed.push_back(frame);
    if(frame==1){entered.set_value();released.wait();}
    if(frame==100)finished.set_value();
  });
  worker.submit(1);
  assert(started.wait_for(seconds(2))==std::future_status::ready);
  // The processor is deliberately blocked. Capture must still submit frames.
  auto submissions=std::async(std::launch::async,[&]{for(int i=2;i<=100;++i)worker.submit(i);});
  assert(submissions.wait_for(seconds(2))==std::future_status::ready);
  release.set_value();
  assert(done.wait_for(seconds(2))==std::future_status::ready);
  worker.stop();worker.stop();worker.submit(101);
  assert((processed==std::vector<int>{1,100}));
}

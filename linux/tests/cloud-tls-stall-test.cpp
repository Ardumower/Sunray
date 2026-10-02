#include "../../sunray/src/net/CloudWorker.h"
#include <cassert>
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include "../src/BridgeSecureClient.h"
#undef min
#undef max
unsigned long millis(){static auto a=std::chrono::steady_clock::now();return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-a).count();}
int main(){
 int listener=socket(AF_INET,SOCK_STREAM,0);assert(listener>=0);
 sockaddr_in address={};address.sin_family=AF_INET;address.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
 assert(bind(listener,(sockaddr*)&address,sizeof(address))==0);assert(listen(listener,1)==0);
 socklen_t length=sizeof(address);assert(getsockname(listener,(sockaddr*)&address,&length)==0);
 std::atomic<bool> accepted{false},release{false},finished{false};
 std::thread server([&]{int s=accept(listener,nullptr,nullptr);assert(s>=0);accepted=true;
  // Start a TLS record but never complete it: TCP readable does not imply SSL_read can finish.
  unsigned char partial[]={0x16,0x03,0x03,0x00,0x10,0x02};send(s,partial,sizeof(partial),MSG_NOSIGNAL);
  while(!release.load())std::this_thread::sleep_for(std::chrono::milliseconds(1));close(s);
 });
 BridgeSecureClient client("SYSTEM","","","localhost");
 CloudWorker::Transport io;
 io.connect=[&]{bool ok=client.connect("127.0.0.1",ntohs(address.sin_port));finished=true;return ok;};
 io.connected=[&]{return bool(client.connected());};io.close=[&]{client.stop();};
 io.receive=[](std::string&){return false;};io.send=[](const std::string&){return true;};io.binary=[](const unsigned char*,size_t){return true;};
 auto started=millis();unsigned polls=0;unsigned long worst=0;
 {CloudWorker worker(io,10000);worker.start();
  while(!finished.load() && millis()-started<5000){auto a=millis();CloudWorker::Request r;worker.take(r);worker.reply(0,"");worst=std::max(worst,millis()-a);++polls;std::this_thread::sleep_for(std::chrono::milliseconds(5));}
  assert(accepted.load());assert(finished.load());assert(polls>100);assert(worst<50);
 }
 release=true;server.join();close(listener);
 std::cout<<"PASS real partial TLS record: "<<millis()-started<<" ms network stall, "<<polls<<" control polls, worst "<<worst<<" ms\n";
}

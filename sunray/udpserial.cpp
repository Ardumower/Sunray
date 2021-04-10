// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "udpserial.h"
#include "config.h"
#ifdef __linux__
  #include <BridgeUdp.h>
#else
  #include "src/esp/WiFiEsp.h"
  #include "src/esp/WiFiEspUdp.h"
#endif

#if defined(_SAM3XA_)                 // Arduino Due
  #define CONSOLE SerialUSB
#else
  #define CONSOLE Serial
#endif


#if !defined(NO_GLOBAL_INSTANCES) && !defined(NO_GLOBAL_SERIAL)
UdpSerial udpSerial;
#endif

unsigned int localPort = 4211;  // local port 
IPAddress remoteIP(UDP_SERVER_IP);


WiFiEspUDP Udp;
char packetBuffer[100];          // buffer to packet
int packetIdx = 0;
bool udpStarted = false;
bool udpActive = false;


void UdpSerial::begin(unsigned long baud){  
  CONSOLE.begin(baud);
}  

void UdpSerial::beginUDP(){  
  Udp.begin(localPort);  
  Udp.beginPacket(remoteIP, UDP_SERVER_PORT);
  // Udp.endPacket();
  udpStarted = true;  
}

size_t UdpSerial::write(uint8_t data){
  if ((udpStarted) && (!udpActive)) {
    udpActive = true;
    packetBuffer[packetIdx] = char(data);
    packetIdx++;
    if (packetIdx == 99){
      packetBuffer[packetIdx] = '\0';      
      Udp.write(packetBuffer);              
      packetIdx = 0;            
    }
    udpActive = false;
  }  
  CONSOLE.write(data);
  return 1; 
}
  
  
int UdpSerial::available(){
  return CONSOLE.available();
}


int UdpSerial::read(){
  return CONSOLE.read();
}


int UdpSerial::peek(){
  return CONSOLE.peek();
}

void UdpSerial::flush(){  
  CONSOLE.flush();    
}




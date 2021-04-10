  // Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "sdserial.h"
#include <SD.h>



#ifdef _SAM3XA_                 // Arduino Due
  #define CONSOLE SerialUSB
#elif __SAMD51__
  #define CONSOLE Serial
#else
  #include <Console.h>
  #define CONSOLE Console
#endif


#if !defined(NO_GLOBAL_INSTANCES) && !defined(NO_GLOBAL_SERIAL)
SDSerial sdSerial;
#endif



void SDSerial::begin(unsigned long baud){  
  logFileName = "";
  packetIdx = 0;
  sdStarted = false;
  sdActive = false;
  CONSOLE.begin(baud);
}  

void SDSerial::beginSD(){  
  
  for (int i=1000; i < 9000; i++){
    logFileName = "log";
    logFileName += i;
    logFileName += ".txt";    
    if (!SD.exists(logFileName)) {
      CONSOLE.print("logfile: ");
      CONSOLE.println(logFileName);    
      sdStarted = true;
      break; 
      //SD.remove(UPDATE_FILE);
      //updateFile.close();
      //uint32_t updateSize = updateFile.size();
      //updateFile.seek(SDU_SIZE);      
    }
  }  
}

size_t SDSerial::write(uint8_t data){
  if ((sdStarted) && (!sdActive)) {
    sdActive = true;
    packetBuffer[packetIdx] = char(data);
    packetIdx++;
    if (packetIdx == 99){
      packetBuffer[packetIdx] = '\0';            
      logFile = SD.open(logFileName, FILE_WRITE);
      if (logFile){        
        logFile.write(packetBuffer);              
        logFile.flush();
        logFile.close();            
      } else {
        CONSOLE.println("ERROR opening file for writing");
      }
      packetIdx = 0;            
    }
    sdActive = false;
  }  
  CONSOLE.write(data);
  return 1; 
}
  
  
int SDSerial::available(){
  return CONSOLE.available();
}


int SDSerial::read(){
  return CONSOLE.read();
}


int SDSerial::peek(){
  return CONSOLE.peek();
}

void SDSerial::flush(){  
  CONSOLE.flush();    
}




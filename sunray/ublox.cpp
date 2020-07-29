// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "Arduino.h"
#include "ublox.h"
#include "config.h"


/* uBlox object, input the serial bus and baud rate */
UBLOX::UBLOX(HardwareSerial& bus,uint32_t baud)
{
  _bus = &bus;
	_baud = baud;
  debug = false;
  verbose = false;
}

/* starts the serial communication */
void UBLOX::begin()
{
	this->state    = GOT_NONE;
  this->msgclass = -1;
  this->msgid    = -1;
  this->msglen   = -1;
  this->chka     = -1;
  this->chkb     = -1;
  this->count    = 0;
  this->dgpsAge  = 0;
  this->solutionAvail = false;
  this->numSV    = 0;
  this->accuracy  =0;
  this->chksumErrorCounter = 0;
  this->dgpsChecksumErrorCounter = 0;
  this->dgpsPacketCounter = 0;
	// begin the serial port for uBlox	
  _bus->begin(_baud);
}



void UBLOX::parse(int b)
{
  if (debug) CONSOLE.print(b, HEX);
  if (debug) CONSOLE.print(",");
  if ((b == 0xB5) && (this->state == GOT_NONE)) {

      if (debug) CONSOLE.println("\n");
      this->state = GOT_SYNC1;
  }

  else if ((b == 0x62) && (this->state == GOT_SYNC1)) {

      this->state = GOT_SYNC2;
      this->chka = 0;
      this->chkb = 0;
  }

  else if (this->state == GOT_SYNC2) {

      this->state = GOT_CLASS;
      this->msgclass = b;
      this->addchk(b);
  }

  else if (this->state == GOT_CLASS) {

      this->state = GOT_ID;
      this->msgid = b;
      this->addchk(b);
  }

  else if (this->state == GOT_ID) {

      this->state = GOT_LENGTH1;
      this->msglen = b;
      this->addchk(b);
  }

  else if (this->state == GOT_LENGTH1) {

      this->state = GOT_LENGTH2;
      this->msglen += (b << 8);
      if (debug) {
        CONSOLE.print("payload size ");
        CONSOLE.print(this->msglen, HEX);
        CONSOLE.print(",");
      }
      this->count = 0;
      this->addchk(b);
  }

  else if (this->state == GOT_LENGTH2) {

      this->addchk(b);
      if (this->count < sizeof(this->payload)){
        this->payload[this->count] = b; 
      } 
      this->count += 1;

      if (this->count >= this->msglen) {

          this->state = GOT_PAYLOAD;
      }
  }

  else if (this->state == GOT_PAYLOAD) {

      if (b == this->chka){
        this->state = GOT_CHKA;
      } else {        
        CONSOLE.print("ublox chka error, msgclass=");
        CONSOLE.print(this->msgclass, HEX);
        CONSOLE.print(", msgid=");
        CONSOLE.print(this->msgid, HEX);
        CONSOLE.print(", msglen=");
        CONSOLE.print(this->msglen, HEX);        
        CONSOLE.print(": ");
        CONSOLE.print(b, HEX);
        CONSOLE.print("!=");
        CONSOLE.println(this->chka, HEX);
        this->state = GOT_NONE;                
        this->chksumErrorCounter++;
      }
  }

  else if (this->state == GOT_CHKA) {

      if (b == this->chkb) {
          this->dispatchMessage();
          this->state = GOT_NONE;
      }

      else {
          CONSOLE.print("ublox chkb error, msgclass=");
          CONSOLE.print(this->msgclass, HEX);
          CONSOLE.print(", msgid=");
          CONSOLE.print(this->msgid, HEX);
          CONSOLE.print(", msglen=");
          CONSOLE.print(this->msglen, HEX);        
          CONSOLE.print(": ");
          CONSOLE.print(b, HEX);
          CONSOLE.print("!=");
          CONSOLE.println(this->chkb, HEX);
          this->state = GOT_NONE;
          this->chksumErrorCounter++;
      }
  }
}

void UBLOX::addchk(int b) {

    this->chka = (this->chka + b) & 0xFF;
    this->chkb = (this->chkb + this->chka) & 0xFF;
}
    

void UBLOX::dispatchMessage() {
    switch (this->msgclass){
      case 0x01:
        switch (this->msgid) {
          case 0x07:
            { // UBX-NAV-PVT
              iTOW = (unsigned long)this->unpack_int32(0);
              numSV = this->unpack_int8(23);               
              if (verbose) CONSOLE.println("UBX-NAV-PVT");
            }
            break;
          case 0x12:
            { // UBX-NAV-VELNED
              iTOW = (unsigned long)this->unpack_int32(0);
              groundSpeed = ((double)((unsigned long)this->unpack_int32(20))) / 100.0;
              heading = ((double)this->unpack_int32(24)) * 1e-5 / 180.0 * PI;
              //CONSOLE.print("heading:");
              //CONSOLE.println(heading);
              if (verbose) CONSOLE.println("UBX-NAV-VELNED");
            }
            break;
          case 0x14: 
            { // UBX-NAV-HPPOSLLH
              iTOW = (unsigned long)this->unpack_int32(4);
              lon = (1e-7  * (this->unpack_int32(8)   +  (this->unpack_int8(24) * 1e-2)));
              lat = (1e-7  * (this->unpack_int32(12)  +  (this->unpack_int8(25) * 1e-2)));
              height = (1e-3 * (this->unpack_int32(16) +  (this->unpack_int8(26) * 1e-2))); // HAE (WGS84 height)
              //height = (1e-3 * (this->unpack_int32(20) +  (this->unpack_int8(27) * 1e-2))); // MSL height
              hAccuracy = ((double)((unsigned long)this->unpack_int32(28))) * 0.1 / 1000.0;
              vAccuracy = ((double)((unsigned long)this->unpack_int32(32))) * 0.1 / 1000.0;
              accuracy = sqrt(sq(hAccuracy) + sq(vAccuracy));
              // long hMSL = this->unpack_int32(16);
              //unsigned long hAcc = (unsigned long)this->unpack_int32(20);
              //unsigned long vAcc = (unsigned long)this->unpack_int32(24);                            
              if (verbose) {
                CONSOLE.print("UBX-NAV-HPPOSLLH ");
                CONSOLE.print("lon=");
                CONSOLE.print(lon,8);
                CONSOLE.print("  lat=");
                CONSOLE.println(lat,8);                      
              }
            }
            break;            
          case 0x43:
            { // UBX-NAV-SIG
              if (verbose) CONSOLE.print("UBX-NAV-SIG ");
              iTOW = (unsigned long)this->unpack_int32(0);
              int numSigs = this->unpack_int8(5);
              float ravg = 0;
              float rmax = 0;
              float rmin = 9999;
              float rsum = 0;    
              int crcnt = 0;
              for (int i=0; i < numSigs; i++){                
                float prRes = ((float)((short)this->unpack_int16(12+16*i))) * 0.1;
                float cno = ((float)this->unpack_int8(14+16*i));
                int qualityInd = this->unpack_int8(15+16*i);                                
                int sigFlags = (unsigned short)this->unpack_int16(18+16*i);                                
                if ((sigFlags & 3) == 1){
                  if ((sigFlags & 128) != 0){
                    /*CONSOLE.print(sigFlags);
                    CONSOLE.print(",");                                
                    CONSOLE.print(qualityInd);
                    CONSOLE.print(",");                                
                    CONSOLE.print(prRes);
                    CONSOLE.print(",");
                    CONSOLE.println(cno); */
                    rsum += fabs(prRes);
                    rmax = max(rmax, fabs(prRes));
                    rmin = min(rmin, fabs(prRes));
                    crcnt++;
                  }                    
                }                
              }
              ravg = rsum/((float)numSigs);
              if (verbose){
                CONSOLE.print("sol=");
                CONSOLE.print(solution);              
                CONSOLE.print("\t");
                CONSOLE.print("hAcc=");
                CONSOLE.print(hAccuracy);
                CONSOLE.print("\tvAcc=");
                CONSOLE.print(vAccuracy);
                CONSOLE.print("\t#");
                CONSOLE.print(crcnt);
                CONSOLE.print("/");
                CONSOLE.print(numSigs);
                CONSOLE.print("\t");
                CONSOLE.print("rsum=");
                CONSOLE.print(rsum);
                CONSOLE.print("\t");
                CONSOLE.print("ravg=");
                CONSOLE.print(ravg);
                CONSOLE.print("\t");
                CONSOLE.print("rmin=");
                CONSOLE.print(rmin);
                CONSOLE.print("\t");
                CONSOLE.print("rmax=");
                CONSOLE.println(rmax); 
              }              
            }
            break;
          case 0x3C: 
            { // UBX-NAV-RELPOSNED              
              iTOW = (unsigned long)this->unpack_int32(4);
              relPosN = ((float)this->unpack_int32(8))/100.0;
              relPosE = ((float)this->unpack_int32(12))/100.0;
              relPosD = ((float)this->unpack_int32(16))/100.0;              
              solution = (UBLOX::SolType)((this->unpack_int32(60) >> 3) & 3);              
              solutionAvail = true;
              if (verbose){
                CONSOLE.print("UBX-NAV-RELPOSNED ");
                CONSOLE.print("n=");
                CONSOLE.print(relPosN,2);
                CONSOLE.print("  e=");
                CONSOLE.println(relPosE,2);                       
              }              
            }
            break;            
        }
        break;      
      case 0x02:
        switch (this->msgid) {
          case 0x32: 
            { // UBX-RXM-RTCM              
              if (verbose) CONSOLE.println("UBX-RXM-RTCM");
              bool flags = (unsigned long)this->unpack_int8(1);
              if (flags & 1) dgpsChecksumErrorCounter++;
              dgpsPacketCounter++;
              dgpsAge = millis();
            }
            break;            
        }
        break;
    }    
}

long UBLOX::unpack_int32(int offset) {

    return this->unpack(offset, 4);
}

long UBLOX::unpack_int16(int offset) {

    return this->unpack(offset, 2);
}

long UBLOX::unpack_int8(int offset) {

    return this->unpack(offset, 1);
}

long UBLOX::unpack(int offset, int size) {

    long value = 0; // four bytes on most Arduinos

    for (int k=0; k<size; ++k) {
        value <<= 8;
        value |= (0xFF & this->payload[offset+size-k-1]);
    }

    return value;
 }
    
  
  
/* parse the uBlox data */
void UBLOX::run()
{
	// read a byte from the serial port	  
  if (!_bus->available()) return;
  while (_bus->available()) {		
    byte data = _bus->read();        
		parse(data);
    //if (data == 0xB5) CONSOLE.println("\n");
    //CONSOLE.print(data, HEX);
    //CONSOLE.print(",");
	}	
}



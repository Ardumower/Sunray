// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "Arduino.h"
#include "ublox.h"
#include "../../config.h"
#include "SparkFun_Ublox_Arduino_Library.h" 


SFE_UBLOX_GPS configGPS; // used for f9p module configuration only


// used to send .ubx log files via 'sendgps.py' to Arduino (also set GPS to Serial in config for this)
//#define GPS_DUMP   1    


UBLOX::UBLOX()
{
  debug = false;
  verbose = false;
  useTCP = false;
  solutionTimeout = 0;
  #ifdef GPS_DUMP
    verbose = true;
  #endif
}

void UBLOX::begin(){
  CONSOLE.println("using gps driver: UBLOX");
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
  this->numSVdgps    = 0;
  this->accuracy  =0;
  this->chksumErrorCounter = 0;
  this->dgpsChecksumErrorCounter = 0;
  this->dgpsPacketCounter = 0;
}

void UBLOX::begin(Client &client, char *host, uint16_t port){
  CONSOLE.println("UBLOX::begin tcp");
   useTCP = true;
  _client = &client;
  if(!client.connect(host,port)){
    CONSOLE.print("Cannot connect to ");
    CONSOLE.print(host);
    CONSOLE.print(":");
    CONSOLE.println(port);
  }
  // start streaming-in
  begin(); 
}   


/* starts the serial communication */
void UBLOX::begin(HardwareSerial& bus,uint32_t baud)
{	
  CONSOLE.println("UBLOX::begin serial");
  _bus = &bus;
	_baud = baud;  
	// begin the serial port for uBlox	
  _bus->begin(_baud);
   // start streaming-in
  begin(); 
  if (GPS_CONFIG){
    configure();
  }
}


bool UBLOX::configure(){
  CONSOLE.println("trying to connect to ublox f9p...");
  CONSOLE.println("NOTE: if GPS is not responding either set 'GPS_CONFIG=false' in config.h or perform GPS wire fix (see Wiki)");
  //configGPS.enableDebugging(CONSOLE, false);
  
  while(true){
    CONSOLE.print("trying baud ");
    CONSOLE.println(_baud);        
    if (configGPS.begin(*_bus)) break;    
    CONSOLE.println(F("ERROR: GPS receiver is not responding"));            
    CONSOLE.println("trying baud 38400");    
    _bus->begin(38400);
    if (configGPS.begin(*_bus)) {
      configGPS.setVal32(0x40520001, _baud, VAL_LAYER_RAM);  // CFG-UART1-BAUDRATE   (Ardumower)
      _bus->begin(_baud);
      break;                
    }
    _bus->begin(_baud);
    CONSOLE.println(F("ERROR: GPS receiver is not responding"));                
  }
        
  CONSOLE.println("GPS receiver found!");
    
  CONSOLE.println("ublox f9p: sending GPS rover configuration...");

  int timeout = 2000;
  int idx = 0;
  int succeeded = 0;
  for (int idx=0; idx < 3; idx++){
    for (int i=1; i < 3; i++){
      bool setValueSuccess = true;
      CONSOLE.print("idx=");
      CONSOLE.print(idx);
      CONSOLE.print("...");
      if (idx == 0){
        // ----- USB messages (Ardumower) -----------------  
        setValueSuccess &= configGPS.newCfgValset8(0x20910009, 0, VAL_LAYER_RAM); // CFG-MSGOUT-UBX_NAV_PVT_USB    (off)
        setValueSuccess &= configGPS.addCfgValset8(0x20910090, 0); // CFG-MSGOUT-UBX_NAV_RELPOSNED_USB  (off)
        setValueSuccess &= configGPS.addCfgValset8(0x20910036, 0); // CFG-MSGOUT-UBX_NAV_HPPOSLLH_USB   (off)
        setValueSuccess &= configGPS.addCfgValset8(0x20910045, 0); // CFG-MSGOUT-UBX_NAV_VELNED_USB     (off)
        setValueSuccess &= configGPS.addCfgValset8(0x2091026b, 0); // CFG-MSGOUT-UBX_RXM_RTCM_USB   (off)
        setValueSuccess &= configGPS.addCfgValset8(0x20910348, 0); // CFG-MSGOUT-UBX_NAV_SIG_USB   (off)
        setValueSuccess &= configGPS.addCfgValset8(0x2091005e, 0); // CFG-MSGOUT-UBX_NAV_TIMEUTC_USB   (off)
        
        // ----- uart1 messages (Ardumower) -----------------          
        setValueSuccess &= configGPS.addCfgValset8(0x20910007, 0); // CFG-MSGOUT-UBX_NAV_PVT_UART1   (off)
        setValueSuccess &= configGPS.addCfgValset8(0x2091008e, 0); // CFG-MSGOUT-UBX_NAV_RELPOSNED_UART1  (off)
        setValueSuccess &= configGPS.addCfgValset8(0x20910034, 0); // CFG-MSGOUT-UBX_NAV_HPPOSLLH_UART1   (off)
        setValueSuccess &= configGPS.addCfgValset8(0x20910043, 0); // CFG-MSGOUT-UBX_NAV_VELNED_UART1     (off)
        setValueSuccess &= configGPS.addCfgValset8(0x20910269, 0); // CFG-MSGOUT-UBX_RXM_RTCM_UART1   (off)
        setValueSuccess &= configGPS.addCfgValset8(0x20910346, 0); // CFG-MSGOUT-UBX_NAV_SIG_UART1   (off)
        setValueSuccess &= configGPS.sendCfgValset8(0x2091005c, timeout); // CFG-MSGOUT-UBX_NAV_TIMEUTC_UART1   (off)
      }
      else if (idx == 1){
        setValueSuccess &= configGPS.newCfgValset8(0x209100a8, 0, VAL_LAYER_RAM); // CFG-MSGOUT-NMEA_ID_DTM_UART2  (off)
        setValueSuccess &= configGPS.addCfgValset8(0x209100df, 0); // CFG-MSGOUT-NMEA_ID_GBS_UART2   (off)
        setValueSuccess &= configGPS.addCfgValset8(0x209100bc, 60); // CFG-MSGOUT-NMEA_ID_GGA_UART2  (every 60 solutions)
        setValueSuccess &= configGPS.addCfgValset8(0x209100cb, 0); // CFG-MSGOUT-NMEA_ID_GLL_UART2   (off)
        setValueSuccess &= configGPS.addCfgValset8(0x209100b7, 0); // CFG-MSGOUT-NMEA_ID_GNS_UART2   (off)
        setValueSuccess &= configGPS.addCfgValset8(0x209100d0, 0); // CFG-MSGOUT-NMEA_ID_GRS_UART2   (off)
        setValueSuccess &= configGPS.addCfgValset8(0x209100c1, 0); // CFG-MSGOUT-NMEA_ID_GSA_UART2   (off)
        setValueSuccess &= configGPS.addCfgValset8(0x209100d5, 0); // CFG-MSGOUT-NMEA_ID_GST_UART2   (off)
        setValueSuccess &= configGPS.addCfgValset8(0x209100c6, 0); // CFG-MSGOUT-NMEA_ID_GSV_UART2   (off)
        //setValueSuccess &= configGPS.addCfgValset8(0x20910402, 0); // CFG-MSGOUT-NMEA_ID_RLM_UART2    (fails)
        setValueSuccess &= configGPS.addCfgValset8(0x209100ad, 0); // CFG-MSGOUT-NMEA_ID_RMC_UART2   (off)
        setValueSuccess &= configGPS.addCfgValset8(0x209100e9, 0); // CFG-MSGOUT-NMEA_ID_VLW_UART2   (off)
        setValueSuccess &= configGPS.addCfgValset8(0x209100b2, 0); // CFG-MSGOUT-NMEA_ID_VTG_UART2   (off)
        setValueSuccess &= configGPS.addCfgValset8(0x209100da, 0);  // CFG-MSGOUT-NMEA_ID_ZDA_UART2  (off)
        // uart2 protocols (Xbee/NTRIP)
        setValueSuccess &= configGPS.addCfgValset8(0x10750001, 0); // CFG-UART2INPROT-UBX        (off)
        setValueSuccess &= configGPS.addCfgValset8(0x10750002, 0); // CFG-UART2INPROT-NMEA       (off)
        setValueSuccess &= configGPS.addCfgValset8(0x10750004, 1); // CFG-UART2INPROT-RTCM3X     (on)
        setValueSuccess &= configGPS.addCfgValset8(0x10760001, 0); // CFG-UART2OUTPROT-UBX       (off)
        setValueSuccess &= configGPS.addCfgValset8(0x10760002, 1); // CFG-UART2OUTPROT-NMEA      (on) 
        setValueSuccess &= configGPS.addCfgValset8(0x10760004, 0); // CFG-UART2OUTPROT-RTCM3X    (off)
        // uart2 baudrate  (Xbee/NTRIP)
        setValueSuccess &= configGPS.addCfgValset32(0x40530001, 115200); // CFG-UART2-BAUDRATE
      
        // ----- uart1 protocols (Ardumower) --------------- 
        setValueSuccess &= configGPS.addCfgValset8(0x10730001, 1); // CFG-UART1INPROT-UBX     (on)
        setValueSuccess &= configGPS.addCfgValset8(0x10730002, 1); // CFG-UART1INPROT-NMEA    (on)
        setValueSuccess &= configGPS.addCfgValset8(0x10730004, 1); // CFG-UART1INPROT-RTCM3X  (on)
        setValueSuccess &= configGPS.addCfgValset8(0x10740001, 1); // CFG-UART1OUTPROT-UBX    (on)
        setValueSuccess &= configGPS.addCfgValset8(0x10740002, 0); // CFG-UART1OUTPROT-NMEA   (off)
        setValueSuccess &= configGPS.addCfgValset8(0x10740004, 0); // CFG-UART1OUTPROT-RTCM3X (off) 
      
        // ----- USB protocols (Ardumower) ----------------- 
        setValueSuccess &= configGPS.addCfgValset8(0x10770001, 1); // CFG-USBINPROT-UBX     (on)
        setValueSuccess &= configGPS.addCfgValset8(0x10770002, 1); // CFG-USBINPROT-NMEA    (on)
        setValueSuccess &= configGPS.addCfgValset8(0x10770004, 1); // CFG-USBINPROT-RTCM3X  (on)
        setValueSuccess &= configGPS.addCfgValset8(0x10780001, 1); // CFG-USBOUTPROT-UBX    (on)
        setValueSuccess &= configGPS.addCfgValset8(0x10780002, 0); // CFG-USBOUTPROT-NMEA   (off)
        setValueSuccess &= configGPS.addCfgValset8(0x10780004, 0); // CFG-USBOUTPROT-RTCM3X (off) 
      
        // ---- gps fix mode ---------------------------------------------    
        // we contrain altitude here (when receiver is started in docking station it may report a wrong 
        // altitude without correction data and SAPOS will not work with an unplausible reported altitute - the contrains are 
        // ignored once receiver is working in RTK mode)
        setValueSuccess &= configGPS.addCfgValset8(0x20110011, 3); // CFG-NAVSPG-FIXMODE    (1=2d only, 2=3d only, 3=auto)
        setValueSuccess &= configGPS.addCfgValset8(0x10110013, 0); // CFG-NAVSPG-INIFIX3D   (no 3D fix required for initial solution)
        setValueSuccess &= configGPS.addCfgValset32(0x401100c1, 10000); // CFG-NAVSPG-CONSTR_ALT    (100m)
        
        // ----  gps navx5 input filter ----------------------------------
        // minimum input signals the receiver should use
        // https://wiki.ardumower.de/index.php?title=Ardumower_Sunray#RTK_float-to-fix_recovery_and_false-fix_issues  
        //setValueSuccess &= configGPS.addCfgValset8(0x201100a1, 3); // CFG-NAVSPG-INFIL_MINSVS
        //setValueSuccess &= configGPS.addCfgValset8(0x201100a2, 32); // CFG-NAVSPG-INFIL_MAXSVS
        //setValueSuccess &= configGPS.addCfgValset8(0x201100a3, 6); // CFG-NAVSPG-INFIL_MINCNO     
        // ----  gps nav5 input filter ----------------------------------
        // minimum condition when the receiver should try a navigation solution
        // https://wiki.ardumower.de/index.php?title=Ardumower_Sunray#RTK_float-to-fix_recovery_and_false-fix_issues
        if (GPS_CONFIG_FILTER){ // custom filter settings
          setValueSuccess &= configGPS.addCfgValset8(0x201100a4, CPG_CONFIG_FILTER_MINELEV); // CFG-NAVSPG-INFIL_MINELEV  (10 Min SV elevation degree)
          setValueSuccess &= configGPS.addCfgValset8(0x201100aa, CPG_CONFIG_FILTER_NCNOTHRS); // CFG-NAVSPG-INFIL_NCNOTHRS (10 C/N0 Threshold #SVs)
          setValueSuccess &= configGPS.addCfgValset8(0x201100ab, CPG_CONFIG_FILTER_CNOTHRS); // CFG-NAVSPG-INFIL_CNOTHRS  (30 dbHz)
        } else { // ublox default filter settings
          setValueSuccess &= configGPS.addCfgValset8(0x201100a4, 10); // CFG-NAVSPG-INFIL_MINELEV  (10 Min SV elevation degree)
          setValueSuccess &= configGPS.addCfgValset8(0x201100aa, 0);  // CFG-NAVSPG-INFIL_NCNOTHRS (0 C/N0 Threshold #SVs)
          setValueSuccess &= configGPS.addCfgValset8(0x201100ab, 0);  // CFG-NAVSPG-INFIL_CNOTHRS  (0 dbHz)   
        }
        // ----  gps rates ----------------------------------
        setValueSuccess &= configGPS.addCfgValset16(0x30210001, 200); // CFG-RATE-MEAS       (measurement period 200 ms)  
        setValueSuccess &= configGPS.sendCfgValset16(0x30210002, 1,   timeout); //CFG-RATE-NAV  (navigation rate cycles 1)  
      } 
      else if (idx == 2){
        // ----- USB messages (Ardumower) -----------------  
        setValueSuccess &= configGPS.newCfgValset8(0x20910009, 0, VAL_LAYER_RAM); // CFG-MSGOUT-UBX_NAV_PVT_USB    (off)        
        setValueSuccess &= configGPS.addCfgValset8(0x20910090, 1); // CFG-MSGOUT-UBX_NAV_RELPOSNED_USB  (every solution)
        setValueSuccess &= configGPS.addCfgValset8(0x20910036, 1); // CFG-MSGOUT-UBX_NAV_HPPOSLLH_USB   (every solution)
        setValueSuccess &= configGPS.addCfgValset8(0x20910045, 1); // CFG-MSGOUT-UBX_NAV_VELNED_USB     (every solution)
        setValueSuccess &= configGPS.addCfgValset8(0x2091026b, 5); // CFG-MSGOUT-UBX_RXM_RTCM_USB   (every 5 solutions)
        setValueSuccess &= configGPS.addCfgValset8(0x20910348, 20); // CFG-MSGOUT-UBX_NAV_SIG_USB   (every 20 solutions)
        setValueSuccess &= configGPS.addCfgValset8(0x2091005e, 0); // CFG-MSGOUT-UBX_NAV_TIMEUTC_USB   (off)   

        // ----- uart1 messages (Ardumower) -----------------  
        setValueSuccess &= configGPS.addCfgValset8(0x20910007, 0); // CFG-MSGOUT-UBX_NAV_PVT_UART1   (off)
        setValueSuccess &= configGPS.addCfgValset8(0x2091008e, 1); // CFG-MSGOUT-UBX_NAV_RELPOSNED_UART1  (every solution)
        setValueSuccess &= configGPS.addCfgValset8(0x20910034, 1); // CFG-MSGOUT-UBX_NAV_HPPOSLLH_UART1   (every solution)
        setValueSuccess &= configGPS.addCfgValset8(0x20910043, 1); // CFG-MSGOUT-UBX_NAV_VELNED_UART1     (every solution)
        setValueSuccess &= configGPS.addCfgValset8(0x20910269, 5); // CFG-MSGOUT-UBX_RXM_RTCM_UART1   (every 5 solutions)
        setValueSuccess &= configGPS.addCfgValset8(0x20910346, 20); // CFG-MSGOUT-UBX_NAV_SIG_UART1   (every 20 solutions)  
        setValueSuccess &= configGPS.sendCfgValset8(0x2091005c, 0, timeout); // CFG-MSGOUT-UBX_NAV_TIMEUTC_UART1   (off)  
      }
      if (setValueSuccess){
        CONSOLE.println("OK");
        succeeded++;
        break;
      } else {
        CONSOLE.println();
      }
    }    
  }
  if (succeeded == 3){ 
    CONSOLE.println("config sent successfully");
    return true;
  }
  else {
    CONSOLE.println("ERROR: config sending failed");        
    return false;
  }
}

void UBLOX::reboot(){
  CONSOLE.println("rebooting GPS receiver...");
  //configGPS.hardReset();
  configGPS.GNSSRestart();
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
    if (verbose) CONSOLE.println();
    switch (this->msgclass){
      case 0x01:
        switch (this->msgid) {
          case 0x021:
            { // UBX-NAV-TIMEUTC
              iTOW = (unsigned long)this->unpack_int32(0);
              year = (unsigned short)this->unpack_int16(12);
              month = (unsigned char)this->unpack_int8(14);
              day = (unsigned char)this->unpack_int8(15);
              hour = (unsigned char)this->unpack_int8(16);
              mins = (unsigned char)this->unpack_int8(17);
              sec = (unsigned char)this->unpack_int8(18);              
              if (verbose) {
                CONSOLE.print("UBX-NAV-TIMEUTC ");
                CONSOLE.print("year=");
                CONSOLE.print(year);
                CONSOLE.print("  month=");
                CONSOLE.print(month);
                CONSOLE.print("  day=");
                CONSOLE.print(day);
                CONSOLE.print("  hour=");
                CONSOLE.print(hour);
                CONSOLE.print("  min=");
                CONSOLE.print(mins);
                CONSOLE.print("  sec=");
                CONSOLE.println(sec);                
              }
            }
            break;
          case 0x07:
            { // UBX-NAV-PVT
              iTOW = (unsigned long)this->unpack_int32(0);
              //numSV = this->unpack_int8(23);               
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
              if (verbose) {
                CONSOLE.print("UBX-NAV-VELNED ");
                CONSOLE.print("groundSpeed=");
                CONSOLE.print(groundSpeed);
                CONSOLE.print("  heading=");
                CONSOLE.println(heading);                
              }
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
              int healthycnt = 0;              
              for (int i=0; i < numSigs; i++){                
                float prRes = ((float)((short)this->unpack_int16(12+16*i))) * 0.1;
                float cno = ((float)this->unpack_int8(14+16*i));
                int qualityInd = this->unpack_int8(15+16*i);                                                
                int corrSource = this->unpack_int8(16+16*i);                                                
                int sigFlags = (unsigned short)this->unpack_int16(18+16*i);                                                
                bool prUsed = ((sigFlags & 8) != 0);                                    
                bool crUsed = ((sigFlags & 16) != 0);                                    
                bool doUsed = ((sigFlags & 32) != 0);                                    
                bool prCorrUsed = ((sigFlags & 64) != 0);                    
                bool crCorrUsed = ((sigFlags & 128) != 0);                    
                bool doCorrUsed = ((sigFlags & 256) != 0);                    
                bool health = ((sigFlags & 3) == 1);                                                    
                if (health){       // signal is healthy               
                  if (prUsed){     // pseudorange has been used (indicates satellites will be also used for carrier correction)
                  //if (cno > 0){  // signal has some strength (carriar-to-noise)
                    healthycnt++;                   
                    if (crCorrUsed){  // Carrier range corrections have been used
                      /*CONSOLE.print(sigFlags);
                      CONSOLE.print(",");                                
                      CONSOLE.print(qualityInd);
                      CONSOLE.print(",");                                
                      CONSOLE.print(prRes);
                      CONSOLE.print(",");
                      CONSOLE.println(cno); */
                      rsum += fabs(prRes);    // pseudorange residual
                      rmax = max(rmax, fabs(prRes));
                      rmin = min(rmin, fabs(prRes));
                      crcnt++;
                    }                    
                  }
                }                
              }
              ravg = rsum/((float)crcnt);
              numSVdgps = crcnt;
              numSV = healthycnt;                            
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
              solution = (SolType)((this->unpack_int32(60) >> 3) & 3);              
              solutionAvail = true;
              solutionTimeout=millis() + 1000;              
              if (verbose){
                CONSOLE.print("UBX-NAV-RELPOSNED ");
                CONSOLE.print("n=");
                CONSOLE.print(relPosN,2);
                CONSOLE.print("  e=");
                CONSOLE.print(relPosE,2);                       
                CONSOLE.print("  sol=");                       
                CONSOLE.print(solution);                       
                CONSOLE.print(" ");                       
                switch(solution){
                  case 0: 
                    CONSOLE.print("invalid");                       
                    break;
                  case 1: 
                    CONSOLE.print("float");                       
                    break;
                  case 2: 
                    CONSOLE.print("fix");                       
                    break;                  
                  default:
                    CONSOLE.print("unknown");                       
                    break;
                }
                CONSOLE.println();
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
              byte flags = (byte)this->unpack_int8(1);
              if ((flags & 1) != 0) dgpsChecksumErrorCounter++;
              dgpsPacketCounter++;
              dgpsAge = millis();
            }
            break;            
        }
        break;
    }    
    if (verbose) CONSOLE.println();
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
	if (millis() > solutionTimeout){
    //CONSOLE.println("UBLOX::solutionTimeout");
    solution = SOL_INVALID;
    solutionTimeout = millis() + 1000;
    solutionAvail = true;
  }

	// read a byte from the serial port	  
  if (!_bus->available()) return;
  while (_bus->available()) {		
    byte data = _bus->read();        
		parse(data);
#ifdef GPS_DUMP
    if (data == 0xB5) CONSOLE.println("\n");
    CONSOLE.print(data, HEX);
    CONSOLE.print(",");    
#endif
  }
}



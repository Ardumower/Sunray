// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "Arduino.h"
#include "ublox.h"
#include "../../config.h"
#include "../../events.h"
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

  CONSOLE.print("sizeof(short):");
  CONSOLE.println(sizeof(short));
  
  CONSOLE.print("sizeof(int):");
  CONSOLE.println(sizeof(int));

  CONSOLE.print("sizeof(long):");
  CONSOLE.println(sizeof(long));

  CONSOLE.print("sizeof(long long):");
  CONSOLE.println(sizeof(long long));

  CONSOLE.print("sizeof(int16_t):");
  CONSOLE.println(sizeof(int16_t));

  CONSOLE.print("sizeof(int32_t):");
  CONSOLE.println(sizeof(int32_t));

  CONSOLE.print("sizeof(int64_t):");
  CONSOLE.println(sizeof(int64_t));
              
  // ---- UBX-NAV-RELPOSNED decode test (will detect compiler conversion issues) -----------
  payload[8] = 0x55;
  payload[9] = 0x02;
  payload[10] = 0x0;
  payload[11] = 0x0;
  payload[12] = 0x21;
  payload[13] = 0xFD;
  payload[14] = 0xFF;
  payload[15] = 0xFF;   
  float relPosN = ((float)(int32_t)this->unpack_int32(8))/100.0;              
  float relPosE = ((float)(int32_t)this->unpack_int32(12))/100.0;
  CONSOLE.print("ublox UBX-NAV-RELPOSNED decode test: relPosE=");
  CONSOLE.print(relPosE,2);
  CONSOLE.print(" relPosN=");
  CONSOLE.print(relPosN,2);
  if ( (abs(relPosE- -7.35) < 0.01) && (abs(relPosN- 5.97) < 0.01) ) {
    CONSOLE.println(" TEST SUCCEEDED");
  } else {
    CONSOLE.println(" TEST FAILED");
  }

  // ---- UBX-NAV-HPPOSLLH decode test (will detect compiler conversion issues) -----------
  payload[8] = 0x51;
  payload[9] = 0xA5;
  payload[10] = 0x21;
  payload[11] = 0x5;
  payload[24] = 0x1B;
  payload[12] = 0x8F;
  payload[13] = 0x62;
  payload[14] = 0x27;
  payload[15] = 0x1F;
  payload[25] = 0xE1;
  double lon = 1e-7  * (    ((double)((int32_t)this->unpack_int32(8)))   +  ((double)((int8_t)this->unpack_int8(24))) * 1e-2    );
  double lat = 1e-7  *  (   ((double)((int32_t)this->unpack_int32(12)))   +  ((double)((int8_t)this->unpack_int8(25))) * 1e-2   );
  CONSOLE.print("ublox UBX-NAV-HPPOSLLH decode test: lat=");
  CONSOLE.print(lat,8);
  CONSOLE.print(" lon=");
  CONSOLE.print(lon,8);
  if ( (abs(lat- 52.26748307) < 0.00000001) && (abs(lon- 8.60910893) < 0.00000001) ) {
    CONSOLE.println(" TEST SUCCEEDED");
  } else {
    CONSOLE.println(" TEST FAILED");
  }
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
    //Logger.event(EVT_ERROR_GPS_NOT_CONNECTED);
    CONSOLE.println("trying baud 38400");    
    _bus->begin(38400);
    if (configGPS.begin(*_bus)) {
      configGPS.setVal32(0x40520001, _baud, VAL_LAYER_RAM);  // CFG-UART1-BAUDRATE   (Ardumower)
      _bus->begin(_baud);
      break;                
    }
    _bus->begin(_baud);
    CONSOLE.println(F("ERROR: GPS receiver is not responding"));                
    Logger.event(EVT_ERROR_GPS_NOT_CONNECTED);
  }
        
  CONSOLE.println("GPS receiver found!");

  configGPS.getProtocolVersion();
  CONSOLE.print("UBLOX protocol: ");
  CONSOLE.print(configGPS.versionHigh);
  CONSOLE.print(".");
  CONSOLE.println(configGPS.versionLow);
  
  CONSOLE.println("ublox f9p: sending GPS rover configuration...");

  int usbNtripEnabled = 0; 
  #ifdef ENABLE_NTRIP
    usbNtripEnabled = 1; 
  #endif
  int timeout = 2000;
  int idx = 0;
  int succeeded = 0;
  for (int idx=0; idx < 10; idx++){
    for (int i=1; i < 3; i++){
      bool setValueSuccess = true;
      CONSOLE.print("idx=");
      CONSOLE.print(idx);
      CONSOLE.print("...");
      if (idx == 0){        
        // ----- enabled ports -----------------        
        setValueSuccess &= configGPS.newCfgValset8(0x10530005, usbNtripEnabled?0:1, VAL_LAYER_RAM); // CFG-UART2-ENABLED  (off/on)          
        setValueSuccess &= configGPS.addCfgValset8(0x10520005, usbNtripEnabled?0:1); // CFG-UART1-ENABLED (off/on)            
        setValueSuccess &= configGPS.addCfgValset8(0x10510003, 0); // CFG-I2C-ENABLED (off)                    
        setValueSuccess &= configGPS.sendCfgValset8(0x10650001, 1, timeout); // CFG-USB-ENABLED       
      } 
      else if (idx == 1){              
        // ----- USB messages (Ardumower) -----------------        
        setValueSuccess &= configGPS.newCfgValset8(0x20910009, 0, VAL_LAYER_RAM); // CFG-MSGOUT-UBX_NAV_PVT_USB    (off)
        setValueSuccess &= configGPS.addCfgValset8(0x209100bd, 0); // CFG-MSGOUT-NMEA_ID_GGA_USB   (off)
        setValueSuccess &= configGPS.addCfgValset8(0x209100c7, 0); // CFG-MSGOUT-NMEA_ID_GSV_USB   (off)
        setValueSuccess &= configGPS.addCfgValset8(0x209100cc, 0); // CFG-MSGOUT-NMEA_ID_GLL_USB   (off)      
        setValueSuccess &= configGPS.addCfgValset8(0x209100b3, 0); // CFG-MSGOUT-NMEA_ID_VTG_USB   (off)
        setValueSuccess &= configGPS.addCfgValset8(0x209100c2, 0); // CFG-MSGOUT-NMEA_ID_GSA_USB   (off)
        setValueSuccess &= configGPS.addCfgValset8(0x20910090, 0); // CFG-MSGOUT-UBX_NAV_RELPOSNED_USB  (off)
        setValueSuccess &= configGPS.addCfgValset8(0x20910036, 0); // CFG-MSGOUT-UBX_NAV_HPPOSLLH_USB   (off)
        setValueSuccess &= configGPS.addCfgValset8(0x20910045, 0); // CFG-MSGOUT-UBX_NAV_VELNED_USB     (off)
        setValueSuccess &= configGPS.addCfgValset8(0x2091026b, 0); // CFG-MSGOUT-UBX_RXM_RTCM_USB   (off)
        setValueSuccess &= configGPS.addCfgValset8(0x20910348, 0); // CFG-MSGOUT-UBX_NAV_SIG_USB   (off)
        setValueSuccess &= configGPS.addCfgValset8(0x2091005e, 0); // CFG-MSGOUT-UBX_NAV_TIMEUTC_USB   (off)
        setValueSuccess &= configGPS.addCfgValset8(0x20910352, 0); // CFG-MSGOUT-UBX-MON-COMMS_USB   (off)
        setValueSuccess &= configGPS.sendCfgValset8(0x209100ae, 0, timeout); // CFG-MSGOUT-NMEA_ID_RMC_USB   (off)           
      } 
      else if (idx == 2){
        // ----- uart1 messages (Ardumower) -----------------          
        setValueSuccess &= configGPS.newCfgValset8(0x20910007, 0, VAL_LAYER_RAM); // CFG-MSGOUT-UBX_NAV_PVT_UART1   (off)
        setValueSuccess &= configGPS.addCfgValset8(0x2091008e, 0); // CFG-MSGOUT-UBX_NAV_RELPOSNED_UART1  (off)
        setValueSuccess &= configGPS.addCfgValset8(0x20910034, 0); // CFG-MSGOUT-UBX_NAV_HPPOSLLH_UART1   (off)
        setValueSuccess &= configGPS.addCfgValset8(0x20910043, 0); // CFG-MSGOUT-UBX_NAV_VELNED_UART1     (off)
        setValueSuccess &= configGPS.addCfgValset8(0x20910269, 0); // CFG-MSGOUT-UBX_RXM_RTCM_UART1   (off)
        setValueSuccess &= configGPS.addCfgValset8(0x20910346, 0); // CFG-MSGOUT-UBX_NAV_SIG_UART1   (off)
        setValueSuccess &= configGPS.sendCfgValset8(0x2091005c, timeout); // CFG-MSGOUT-UBX_NAV_TIMEUTC_UART1   (off) 
      } 
      else if (idx == 3){        
        // ----- uart2 messages 
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
        setValueSuccess &= configGPS.sendCfgValset8(0x209100da, 0, timeout);  // CFG-MSGOUT-NMEA_ID_ZDA_UART2  (off)        
      }
      else if (idx == 4){
        // uart2 protocols (Xbee/NTRIP)
        setValueSuccess &= configGPS.newCfgValset8(0x10750001, 0, VAL_LAYER_RAM); // CFG-UART2INPROT-UBX        (off)
        setValueSuccess &= configGPS.addCfgValset8(0x10750002, 0); // CFG-UART2INPROT-NMEA       (off)
        setValueSuccess &= configGPS.addCfgValset8(0x10750004, 1); // CFG-UART2INPROT-RTCM3X     (on)
        setValueSuccess &= configGPS.addCfgValset8(0x10760001, 0); // CFG-UART2OUTPROT-UBX       (off)
        setValueSuccess &= configGPS.addCfgValset8(0x10760002, 1); // CFG-UART2OUTPROT-NMEA      (on) 
        setValueSuccess &= configGPS.addCfgValset8(0x10760004, 0); // CFG-UART2OUTPROT-RTCM3X    (off)
        // uart2 baudrate  (Xbee/NTRIP)
        setValueSuccess &= configGPS.addCfgValset32(0x40530001, 115200); // CFG-UART2-BAUDRATE        
      }
      else if (idx == 5){        
        // ----- uart1 protocols (Ardumower) --------------- 
        setValueSuccess &= configGPS.newCfgValset8(0x10730001, 1, VAL_LAYER_RAM); // CFG-UART1INPROT-UBX     (on)
        setValueSuccess &= configGPS.addCfgValset8(0x10730002, 0); // CFG-UART1INPROT-NMEA    (off)
        setValueSuccess &= configGPS.addCfgValset8(0x10730004, 0); // CFG-UART1INPROT-RTCM3X  (off)
        setValueSuccess &= configGPS.addCfgValset8(0x10740001, 1); // CFG-UART1OUTPROT-UBX    (on)
        setValueSuccess &= configGPS.addCfgValset8(0x10740002, 0); // CFG-UART1OUTPROT-NMEA   (off)
        setValueSuccess &= configGPS.sendCfgValset8(0x10740004, 0, timeout); // CFG-UART1OUTPROT-RTCM3X (off)       
      }
      else if (idx == 6){                
        // ----- USB protocols (Ardumower) ----------------- 
        setValueSuccess &= configGPS.newCfgValset8(0x10770001, 1, VAL_LAYER_RAM); // CFG-USBINPROT-UBX     (on)
        setValueSuccess &= configGPS.addCfgValset8(0x10770002, 1); // CFG-USBINPROT-NMEA    (on)
        setValueSuccess &= configGPS.addCfgValset8(0x10770004, usbNtripEnabled?1:0); // CFG-USBINPROT-RTCM3X  (on/off)
        setValueSuccess &= configGPS.addCfgValset8(0x10780001, 1); // CFG-USBOUTPROT-UBX    (on)
        setValueSuccess &= configGPS.addCfgValset8(0x10780002, usbNtripEnabled?1:0); // CFG-USBOUTPROT-NMEA   (on/off)
        setValueSuccess &= configGPS.sendCfgValset8(0x10780004, 0, timeout); // CFG-USBOUTPROT-RTCM3X (off) 
      } 
      else if (idx == 7){                
        // ---- gps fix mode ---------------------------------------------    
        // we contrain altitude here (when receiver is started in docking station it may report a wrong 
        // altitude without correction data and SAPOS will not work with an unplausible reported altitute - the contrains are 
        // ignored once receiver is working in RTK mode)
        setValueSuccess &= configGPS.newCfgValset8(0x20110011, 3, VAL_LAYER_RAM); // CFG-NAVSPG-FIXMODE    (1=2d only, 2=3d only, 3=auto)
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
        setValueSuccess &= configGPS.addCfgValset8(0x201100c4, GPS_CONFIG_DGNSS_TIMEOUT); // CFG-NAVSPG-CONSTR_DGNSSTO  (60s DGNSS timeout)        
        // ----  gps rates ----------------------------------
        setValueSuccess &= configGPS.addCfgValset16(0x30210001, 200); // CFG-RATE-MEAS       (measurement period 200 ms)  
        setValueSuccess &= configGPS.sendCfgValset16(0x30210002, 1,   timeout); //CFG-RATE-NAV  (navigation rate cycles 1)          
      } 
      else if (idx == 8){
        // ----- USB messages (Ardumower) -----------------  
        setValueSuccess &= configGPS.newCfgValset8(0x20910009, 0, VAL_LAYER_RAM); // CFG-MSGOUT-UBX_NAV_PVT_USB    (off)        
        setValueSuccess &= configGPS.addCfgValset8(0x20910090, 1); // CFG-MSGOUT-UBX_NAV_RELPOSNED_USB  (every solution)
        setValueSuccess &= configGPS.addCfgValset8(0x20910036, 1); // CFG-MSGOUT-UBX_NAV_HPPOSLLH_USB   (every solution)
        setValueSuccess &= configGPS.addCfgValset8(0x20910045, 1); // CFG-MSGOUT-UBX_NAV_VELNED_USB     (every solution)
        setValueSuccess &= configGPS.addCfgValset8(0x2091026b, 5); // CFG-MSGOUT-UBX_RXM_RTCM_USB   (every 5 solutions)
        setValueSuccess &= configGPS.addCfgValset8(0x20910348, 20); // CFG-MSGOUT-UBX_NAV_SIG_USB   (every 20 solutions)
        setValueSuccess &= configGPS.addCfgValset8(0x2091005e, 0); // CFG-MSGOUT-UBX_NAV_TIMEUTC_USB   (off)   
        setValueSuccess &= configGPS.addCfgValset8(0x209100bd, 60); // CFG-MSGOUT-NMEA_ID_GGA_USB   (every 60 solutions)
        setValueSuccess &= configGPS.sendCfgValset8(0x20910352, 70, timeout); // CFG-MSGOUT-UBX-MON-COMMS_USB   (every 70 solutions)
      }
      else if (idx == 9){        
        // ----- uart1 messages (Ardumower) -----------------  
        setValueSuccess &= configGPS.newCfgValset8(0x20910007, 0, VAL_LAYER_RAM); // CFG-MSGOUT-UBX_NAV_PVT_UART1   (off)
        setValueSuccess &= configGPS.addCfgValset8(0x2091008e, 1); // CFG-MSGOUT-UBX_NAV_RELPOSNED_UART1  (every solution)
        setValueSuccess &= configGPS.addCfgValset8(0x20910034, 1); // CFG-MSGOUT-UBX_NAV_HPPOSLLH_UART1   (every solution)
        setValueSuccess &= configGPS.addCfgValset8(0x20910043, 1); // CFG-MSGOUT-UBX_NAV_VELNED_UART1     (every solution)
        setValueSuccess &= configGPS.addCfgValset8(0x20910269, 5); // CFG-MSGOUT-UBX_RXM_RTCM_UART1   (every 5 solutions)
        setValueSuccess &= configGPS.addCfgValset8(0x20910346, 20); // CFG-MSGOUT-UBX_NAV_SIG_UART1   (every 20 solutions)  
        setValueSuccess &= configGPS.sendCfgValset8(0x2091005c, 0, timeout); // CFG-MSGOUT-UBX_NAV_TIMEUTC_UART1   (off)          
      }      
      delay(300);
      if (setValueSuccess){
        CONSOLE.println("OK");
        succeeded++;
        break;
      } else {
        CONSOLE.println();
      }
    }    
  }
  if (succeeded == 10){ 
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


// calc UBX checksum (CK_A, CK_B)
void UBLOX::calcUBXChecksum(uint8_t *data, size_t length, uint8_t *ck_a, uint8_t *ck_b) {
  *ck_a = 0;
  *ck_b = 0;
  for (size_t i = 2; i < length; i++) {  // start from Class+ID 
      *ck_a += data[i];
      *ck_b += *ck_a;
  }
}

void UBLOX::send(const uint8_t *buffer, size_t size){
  _bus->write(buffer, size); // send message to ublox receiver
}
    

// send RTCM via UBX
void UBLOX::sendRTCM(const uint8_t *rtcmData, size_t rtcmLength) {
  if (rtcmLength > 1024) {
    CONSOLE.print("UBLOX::sendUbxRtcm error rtcmLength=");
    CONSOLE.println(rtcmLength);
    return; 
  }
  send(rtcmData, rtcmLength);
  return;
  /*
  // RTCM
  #define UBX_CLASS_RXM  0x02
  #define UBX_ID_RTCM    0x32

  byte ubxPacket[1024];
  size_t index = 0;

  // UBX header
  ubxPacket[index++] = UBX_SYNC1;
  ubxPacket[index++] = UBX_SYNC2;
  ubxPacket[index++] = UBX_CLASS_RXM; 
  ubxPacket[index++] = UBX_ID_RTCM;   

  // length (LSB, MSB)
  ubxPacket[index++] = (uint8_t)(rtcmLength & 0xFF);       // LSB
  ubxPacket[index++] = (uint8_t)((rtcmLength >> 8) & 0xFF); // MSB

  // RTCM data as payload
  memcpy(&ubxPacket[index], rtcmData, rtcmLength);
  index += rtcmLength;

  // UBX checksum
  uint8_t ck_a, ck_b;
  calcUBXChecksum(ubxPacket, index, &ck_a, &ck_b);
  ubxPacket[index++] = ck_a;
  ubxPacket[index++] = ck_b;

  send(ubxPacket, index);
  */
}



void UBLOX::parse(int b)
{
  if (debug) CONSOLE.print(b, HEX);
  if (debug) CONSOLE.print(",");
  
  if ( (this->state == GOT_NONE) || (this->state == GOT_SYNC1) ) {
    char ch = char(b);
    if (ch == '$') unparsedMessage = "";    
    if (unparsedMessage.length() < 1000) unparsedMessage += ch;    
    if ((ch == '\r') || (ch == '\n')) {
      //CONSOLE.println(unparsedMessage);
      if (unparsedMessage.startsWith("$GNGGA")) {
        nmeaGGAMessage = unparsedMessage;
        nmeaGGAMessage.trim();
      }
      unparsedMessage = "";
    }
  }
  
  
  if ((b == UBX_SYNC1) && (this->state == GOT_NONE)) {

      if (debug) CONSOLE.println("\n");
      this->state = GOT_SYNC1;
  }

  else if ((b == UBX_SYNC2) && (this->state == GOT_SYNC1)) {

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
              lon = 1e-7  * (    ((double)((int32_t)this->unpack_int32(8)))   +  ((double)((int8_t)this->unpack_int8(24))) * 1e-2    );
              lat = 1e-7  *  (   ((double)((int32_t)this->unpack_int32(12)))   +  ((double)((int8_t)this->unpack_int8(25))) * 1e-2   );
              height = 1e-3 * (  ((double)((int32_t)this->unpack_int32(16))) +  ((double)((int8_t)this->unpack_int8(26))) * 1e-2    ) ; // HAE (WGS84 height)
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
              relPosN = ((float)(int32_t)this->unpack_int32(8))/100.0;              
              relPosE = ((float)(int32_t)this->unpack_int32(12))/100.0;
              relPosD = ((float)(int32_t)this->unpack_int32(16))/100.0;              
              solution = (SolType)((this->unpack_int32(60) >> 3) & 3);              
              solutionAvail = true;
              solutionTimeout=millis() + 3000;              
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

      case 0x0a:
        switch (this->msgid){   
          case 0x36:
            {
              // UBX-MON-COMMS
              byte version = (unsigned char)this->unpack_int8(0); 
              CONSOLE.print("UBX-MON-COMMS v");
              CONSOLE.println(version);
              if (version == 0){
                byte nPorts = (unsigned char)this->unpack_int8(1);
                byte protIds[4];
                for (int i=0; i < 4; i++){
                  protIds[i] = (unsigned char)this->unpack_int8(4 + i);
                }
                for (int i=0; i < nPorts; i++) {
                  int portId = ((short)this->unpack_int16(8 + i*40));                   
                  const uint8_t portBank = portId & 0xff;
                  const uint8_t portNo   = (portId >> 8) & 0xff;                                    
                  unsigned long txBytes = (unsigned long)this->unpack_int32(12 + i*40);
                  byte txPeakUsage = (byte)this->unpack_int8(17 + i*40);
                  unsigned long rxBytes = (unsigned long)this->unpack_int32(20 + i*40);
                  byte rxPeakUsage = (byte)this->unpack_int8(25 + i*40);
                  unsigned long skippedBytes = (unsigned long)this->unpack_int32(44 + i*40); 
                  bool ignore = false;
                  switch (portId){
                    case 0x0100: CONSOLE.print("UART1"); break;
                    case 0x0201: CONSOLE.print("UART2"); break;
                    case 0x0300: CONSOLE.print("USB"); break;
                    //default: CONSOLE.print(portId, HEX);
                    default: ignore = true; break;
                  }
                  if (!ignore){
                    CONSOLE.print(" tx=");
                    CONSOLE.print(txBytes);
                    CONSOLE.print(" (");
                    CONSOLE.print(txPeakUsage);
                    CONSOLE.print("% peak) ");
                    CONSOLE.print(" rx=");
                    CONSOLE.print(rxBytes);
                    CONSOLE.print(" (");
                    CONSOLE.print(rxPeakUsage);
                    CONSOLE.print("% peak)  ");                    
                    CONSOLE.print("skipped=");
                    CONSOLE.print(skippedBytes);
                    CONSOLE.print("    in-msgs: ");                  
                    for (int j=0; j < 4; j++){
                      int protId = protIds[j];                     
                      int msgs = (unsigned short)this->unpack_int16(28 + i*40 + j*2);
                      if ((protId != 0xFF) && (msgs != 0)) {
                        switch (protId){
                          case 0: CONSOLE.print(" UBX="); break;
                          case 1: CONSOLE.print(" NMEA="); break;
                          case 2: CONSOLE.print(" RTCM2="); break;
                          case 5: CONSOLE.print(" RTCM3="); break;
                          case 6: CONSOLE.print(" SPARTN="); break;                        
                          default: CONSOLE.print(" "); CONSOLE.print(protId, HEX); CONSOLE.print("=");
                        }
                        CONSOLE.print(msgs);
                      }
                    }                
                    CONSOLE.println();
                  }  
                }
              } 
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
    // relPosN... PAYLOAD: ofs=8  size=4    2D,6,0,0,        relPosN: 15.81
    // relPosE... PAYLOAD: ofs=12 size=4    95,F9,FF,FF,     relPosE: 42949656.00
    if (verbose){
      CONSOLE.print("UNPACK: ofs=");
      CONSOLE.print(offset);
      CONSOLE.print(" size=");
      CONSOLE.println(size);
      for (int i=0; i < size; i++){        
          CONSOLE.print((byte)this->payload[offset+i], HEX);
          CONSOLE.print(",");
      }
      CONSOLE.println();              
    }
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
    solutionTimeout = millis() + 3000;
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



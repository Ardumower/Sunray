// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "Arduino.h"
#include "skytraq.h"
#include "../../config.h"


//#define GPS_DUMP 1

SKYTRAQ::SKYTRAQ()
{
  debug = false;
  verbose = false;
  useTCP   = false;
  #ifdef GPS_DUMP
    verbose = true;
  #endif
}

void SKYTRAQ::begin(){
  CONSOLE.println("using gps driver: SKYTRAQ");    
  this->state    = GOT_NONE;
  this->msgid    = -1;
  this->msglen   = -1;
  this->chk      = -1;
  this->count    = 0;
  this->dgpsAge  = 0;
  this->solutionAvail = false;
  this->numSV    = 0;
  this->numSVdgps    = 0;
  this->accuracy  =0;
  this->chksumErrorCounter = 0;
  this->dgpsChecksumErrorCounter = 0;
  this->dgpsPacketCounter = 0;
  this->solutionTimeout = 0; 
  gnssUpdateFlag = 0;
  parser.SetNotify(this); 
  
  if (GPS_CONFIG){
    configure();
  }
}

/* starts the serial communication */
void SKYTRAQ::begin(HardwareSerial& bus,uint32_t baud)
{	
  CONSOLE.println("SKYTRAQ::begin serial");
  _bus = &bus;
	_baud = baud;  
  // begin the serial port for skytraq	
  _bus->begin(_baud);  
  // start streaming-in
  begin();
}

/* starts the tcp communication */
void SKYTRAQ::begin(Client &client, char *host, uint16_t port)
{
  CONSOLE.println("SKYTRAQ::begin tcp");
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

bool SKYTRAQ::configure(){  
  CONSOLE.println("using skytraq gps..."); 
  return true;
}

void SKYTRAQ::reboot(){
}

// --------- skytraq binary parser (TODO) -----------

void SKYTRAQ::parseBinary(int b)
{
  if (debug) CONSOLE.print(b, HEX);
  if (debug) CONSOLE.print(",");
  if ((b == 0xA0) && (this->state == GOT_NONE)) {

      if (debug) CONSOLE.println("\n");
      this->state = GOT_SYNC1;
  }

  else if ((b == 0xA1) && (this->state == GOT_SYNC1)) {

      this->state = GOT_SYNC2;
      this->chk = 0;      
  }

  else if (this->state == GOT_SYNC2) {

      this->state = GOT_LENGTH1;
      this->msglen = b;
      //this->addchk(b);
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
      //this->addchk(b);
  }

  else if (this->state == GOT_LENGTH2) {

      this->state = GOT_ID;
      this->msgid = b;
      this->addchk(b);
  }

  else if (this->state == GOT_ID) {

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

      if (b == this->chk){
        this->state = GOT_CHK;
      } else {        
        CONSOLE.print("skytraq chka error, msgid=");
        CONSOLE.print(this->msgid, HEX);
        CONSOLE.print(", msglen=");
        CONSOLE.print(this->msglen, HEX);        
        CONSOLE.print(": ");
        CONSOLE.print(b, HEX);
        CONSOLE.print("!=");
        CONSOLE.println(this->chk, HEX);
        this->state = GOT_NONE;                
        this->chksumErrorCounter++;
      }
  }

  else if (this->state == GOT_CHK) {

      if (b == this->chk) {
          this->dispatchMessage();
          this->state = GOT_NONE;
      }

      else {
          CONSOLE.print("skytraq chkb error, msgid=");
          CONSOLE.print(", msgid=");
          CONSOLE.print(this->msgid, HEX);
          CONSOLE.print(", msglen=");
          CONSOLE.print(this->msglen, HEX);        
          CONSOLE.print(": ");
          CONSOLE.print(b, HEX);
          CONSOLE.print("!=");
          CONSOLE.println(this->chk, HEX);
          this->state = GOT_NONE;
          this->chksumErrorCounter++;
      }
  }
}

void SKYTRAQ::addchk(int b) {

    this->chk = this->chk ^ b;    
}
    

void SKYTRAQ::dispatchMessage() {
    
}

long SKYTRAQ::unpack_int32(int offset) {

    return this->unpack(offset, 4);
}

long SKYTRAQ::unpack_int16(int offset) {

    return this->unpack(offset, 2);
}

long SKYTRAQ::unpack_int8(int offset) {

    return this->unpack(offset, 1);
}

long SKYTRAQ::unpack(int offset, int size) {

    long value = 0; // four bytes on most Arduinos

    for (int k=0; k<size; ++k) {
        value <<= 8;
        value |= (0xFF & this->payload[offset+size-k-1]);
    }

    return value;
}
    
  
/* parse the skytraq data */
void SKYTRAQ::run()
{
	if (millis() > solutionTimeout){
    //CONSOLE.println("SYKTRAQ::solutionTimeout");
    solution = SOL_INVALID;
    solutionTimeout = millis() + 1000;
    solutionAvail = true;
  }
  //CONSOLE.println("SKYTRAQ::run");
  // read a byte from the serial port
  Stream *stream; 
  if (useTCP) stream = _client;
    else stream = _bus;

  if (!stream->available()) return;
  while (!stream->available()) {		
    byte data = stream->read();        		
    parser.Encode(data); // NMEA parser
    //parseBinary(data);  // binary parser
#ifdef GPS_DUMP
    //if (data == 0xA0) CONSOLE.println("\n");
    //CONSOLE.print(data, HEX);
    //CONSOLE.print(",");   
    CONSOLE.print(((char)data)); 
#endif
  }
}

// --------- NMEA parser callback ----------

bool SKYTRAQ::gnssUpdated(U32 f, const char* buf, ParsingType type)
{
  gnssUpdateFlag |= f;
  gdata = parser.GetGnssData();
  processNmea(f, buf, type);
  //return true to clear the flag in SkyTraqNmeaParseryTraq
  return true;
}

bool SKYTRAQ::processNmea(U32 f, const char* buf, ParsingType type)
{
  U32 i = 0;
  const GnssData& gnss = *gdata;

  for(; i < 32; ++i)
  {
    U32 mask = (1 << i);
    switch((mask & f))
    {
    case SkyTraqNmeaParser::NoUpdate:
      //Do nothing
      break;
    case SkyTraqNmeaParser::UpdateDate:
      //CONSOLE.print("Date:");
      //CONSOLE.print(gnss.GetYear());
      //CONSOLE.print('/');
      //CONSOLE.print(gnss.GetMonth());
      //CONSOLE.print('/');
      //CONSOLE.println(gnss.GetDay());
      break;
    case SkyTraqNmeaParser::UpdateTime:
      //CONSOLE.print("Time:");
      //CONSOLE.print(gnss.GetHour());
      //CONSOLE.print(':');
      //CONSOLE.print(gnss.GetMinute());
      //CONSOLE.print(':');
      //CONSOLE.println(gnss.GetSecond());      
      break;
    case SkyTraqNmeaParser::UpdateLatitude:
      //CONSOLE.print("Latitude:");
      //CONSOLE.println(gnss.GetLatitude());
      lat = gnss.GetLatitude();            
      break;
    case SkyTraqNmeaParser::UpdateLongitude:
      lon = gnss.GetLongitude();
      //CONSOLE.print("Longitude:");
      //CONSOLE.print(gnss.GetLongitude(),8);
      //CONSOLE.print(",");
      //CONSOLE.println(lon,8);
      solutionAvail = true;
      solutionTimeout=millis() + 1000;
      break;
    case SkyTraqNmeaParser::UpdateAltitude:
      //CONSOLE.print("Altitude:");
      //CONSOLE.println(gnss.GetAltitudeInMeter());
      height = gnss.GetAltitudeInMeter();
      break;
    case SkyTraqNmeaParser::UpdateCourse:
      //CONSOLE.print("Course:");
      //CONSOLE.println(gnss.GetCourseInDegree());
      break;
    case SkyTraqNmeaParser::UpdateSpeed:
      //CONSOLE.print("Speed:");
      //CONSOLE.println(gnss.GetSpeedInKmHr());
      groundSpeed = gnss.GetSpeedInKmHr() / 1000.0 * 60.0 * 60.0; 
      break;
    case SkyTraqNmeaParser::UpdateQualitMode:      
      //CONSOLE.print("Qualit Mode:");
      //CONSOLE.println(gnss.GetQualitMode());      
      switch (gnss.GetQualitMode()){
        case GnssData::QM_FloatRtk:
          solution = SOL_FLOAT;
          break;
        case GnssData::QM_RealTimeKinematic:
          solution = SOL_FIXED;
          break;
        default:
          solution = SOL_INVALID;
      }   
      break;
    case SkyTraqNmeaParser::UpdateNumberOfSv:
      //CONSOLE.print("Number Of Sv:");
      //CONSOLE.println(gnss.GetNumberOfSv());
      numSV = gnss.GetNumberOfSv();
      break;
    case SkyTraqNmeaParser::UpdateHdop:
      //CONSOLE.print("HDOP:");
      //CONSOLE.println(gnss.GetHdop());
      hAccuracy = gnss.GetHdop();
      break;
    case SkyTraqNmeaParser::UpdatePdop:
      //CONSOLE.print("PDOP:");
      //CONSOLE.println(gnss.GetPdop());
      break;
    case SkyTraqNmeaParser::UpdateVdop:
      //CONSOLE.print("VDOP:");
      //CONSOLE.println(gnss.GetVdop());
      vAccuracy = gnss.GetVdop();
      break;
    case SkyTraqNmeaParser::UpdateNavigationMode:
      //CONSOLE.print("Navigation Mode:");
      //CONSOLE.println(gnss.GetNavigationMode());
      break;
    case SkyTraqNmeaParser::UpdateSatelliteInfo:
#if (_SUPPORT_GPS_SATELLITES_)
      ShowSatellites(gnss.GetGpsSatellites());
#endif
#if (_SUPPORT_GLONASS_SATELLITES_)
      ShowSatellites(gnss.GetGlonassSatellites());
#endif
#if (_SUPPORT_BEIDOU_SATELLITES_)
      ShowSatellites(gnss.GetBeidouSatellites());
#endif
      break;
    case SkyTraqNmeaParser::UpdateEnuVelocity:
      //CONSOLE.print("E-Velocity:");
      //CONSOLE.print(gnss.GetEVelocity());
      //CONSOLE.print("   N-Velocity:");
      //CONSOLE.print(gnss.GetNVelocity());
      //CONSOLE.print("   U-Velocity:");
      //CONSOLE.println(gnss.GetUVelocity());
     break;
    case SkyTraqNmeaParser::UpdateRtkAge:
      //CONSOLE.print("RTK Age:");
      //CONSOLE.println(gnss.GetRtkAge());
      dgpsAge = millis() - gnss.GetRtkAge() * 1000;
      break;
    case SkyTraqNmeaParser::UpdateRtkRatio:
      //CONSOLE.print("RTK Ratio:");
      //CONSOLE.println(gnss.GetRtkRatio());
      numSVdgps = ((float)gnss.GetRtkRatio()) * 100.0 * numSV;  
     break;
    case SkyTraqNmeaParser::UpdateEnuProjection:
      //CONSOLE.print("E-Projection:");
      //CONSOLE.print(gnss.GetEProjection());
      //CONSOLE.print("   N-Projection:");
      //CONSOLE.print(gnss.GetNProjection());
      //CONSOLE.print("   U-Projection:");
      //CONSOLE.println(gnss.GetUProjection());      
      relPosN = gnss.GetNProjection();
      relPosE = gnss.GetEProjection();
      relPosD = gnss.GetUProjection();      
      break;
    case SkyTraqNmeaParser::UpdateBaselineLength:
       //CONSOLE.print("RTK Baseline Length:");
       //CONSOLE.println(gnss.GetBaselineLength());
       break;
    case SkyTraqNmeaParser::UpdateBaselineCourse:
      //CONSOLE.print("RTK Baseline Course:");
      //CONSOLE.println(gnss.GetBaselineCourse());
      break;
    default:
      break;
    }
  }
  return true;
}


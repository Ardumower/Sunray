// SkyTraqNmeaParser Library
// Copyright (C)2016 SkyTraq Technology, Inc. All right reserved
// web: http://www.navspark.com.tw/
//
// This program is a demo of most of the functions
// in the library.
//
// This program requires a NavSpark GPS/GNSS module.
//
//

#include <string.h>
#include <stdlib.h> 
#include "SkyTraqNmeaParser.h"

GnssData::GnssData(void)
{
    Init();
}

GnssData::~GnssData(void)
{
}

void GnssData::Init()
{
  //Date
  year = 0;
  month = 0;
  day = 0;
  //Time
  hour = 0;
  minute = 0;
  second = 0;
  //Geographic coordinates
  latitude = 0;
  longitude = 0;
  altitudeAboutMeanSeaLevel = 0;
  geoidalSeparation = 0;

  //Course over ground
  courseOverGround = 0;
  //Speed over ground
  speedKnot = 0;

  //Accuracy
  hdop = 0;
  pdop = 0;
  vdop = 0;

  //Fix Mode
  qualityMode = QM_NotFix;
  navigationMode = NM_NotFix;

  //Satellite sataus
  numSV = 0;
#if (_SUPPORT_GPS_SATELLITES_)
  ClearInUseGpSatellites();
#endif

#if (_SUPPORT_GLONASS_SATELLITES_)
  ClearInUseGlSatellites();
#endif

#if (_SUPPORT_GLONASS_SATELLITES_)
  ClearInUseBdSatellites();
#endif

  //ENU velocity
  eVelocity = 0; 
  nVelocity = 0; 
  uVelocity = 0; 

  //ENU-Projection of baseline, meters
  eProjection = 0; 
  nProjection = 0; 
  uProjection = 0; 
 
  //RTK Info
  rtkAge = 0; 
  rtkRatio = 0; 
}

#if (_SUPPORT_GPS_SATELLITES_)
void GnssData::ClearInUseGpSatellites() 
{ 
  ClearInUseSatellites(inUseGpSatellites);
}

bool GnssData::AddAnInUseGpPrn(int p)
{
  AddAnInUsePrn(inUseGpSatellites, p);
  return AddAnInUsePrnToSatellites(workingGpSatellites, p);
}

bool GnssData::UpdateGpSatellites(int sv, int elv, int az, int cno)
{
  return UpdateSatellites(workingGpSatellites, sv, elv, az, cno);
}

void GnssData::ClearWorkingGpsSatellites() 
{ 
  ClearSatellites(workingGpSatellites);
}

void GnssData::UpdateGpsInUseToSatellites()
{
  UpdateInUseToSatellites(workingGpSatellites, inUseGpSatellites);
}

void GnssData::CopyWorkingGpsSatellites(bool reversion) 
{ 
  if(reversion)
  {
    CopySatellites(workingGpSatellites, gpSatellites);
  }
  else
  {
    CopySatellites(gpSatellites, workingGpSatellites);
  }
}
#endif  //#if (_SUPPORT_GPS_SATELLITES_)

#if (_SUPPORT_GLONASS_SATELLITES_)
void GnssData::ClearInUseGlSatellites() 
{ 
  ClearInUseSatellites(inUseGlSatellites);
}

bool GnssData::AddAnInUseGlPrn(int p)
{
  AddAnInUsePrn(inUseGlSatellites, p);
  return AddAnInUsePrnToSatellites(workingGlSatellites, p);
}

bool GnssData::UpdateGlSatellites(int sv, int elv, int az, int cno)
{
  return UpdateSatellites(workingGlSatellites, sv, elv, az, cno);
}

void GnssData::ClearWorkingGlonassSatellites() 
{ 
  ClearSatellites(workingGlSatellites);
}

void GnssData::UpdateGlonassInUseToSatellites()
{
  UpdateInUseToSatellites(workingGlSatellites, inUseGlSatellites);
}

void GnssData::CopyWorkingGlonassSatellites(bool reversion) 
{ 
  if(reversion)
  {
    CopySatellites(workingGlSatellites, glSatellites);
  }
  else
  {
    CopySatellites(glSatellites, workingGlSatellites);
  }
}
#endif  //#if (_SUPPORT_GLONASS_SATELLITES_)

#if (_SUPPORT_BEIDOU_SATELLITES_)
void GnssData::ClearInUseBdSatellites() 
{ 
  ClearInUseSatellites(inUseBdSatellites);
}

bool GnssData::AddAnInUseBdPrn(int p)
{
  AddAnInUsePrn(inUseBdSatellites, p);
  return AddAnInUsePrnToSatellites(workingBdSatellites, p);
}

bool GnssData::UpdateBdSatellites(int sv, int elv, int az, int cno)
{
  return UpdateSatellites(workingBdSatellites, sv, elv, az, cno);
}

void GnssData::ClearWorkingBeidouSatellites() 
{ 
  ClearSatellites(workingBdSatellites);
}

void GnssData::UpdateBeidouInUseToSatellites()
{
  UpdateInUseToSatellites(workingBdSatellites, inUseBdSatellites);
}

void GnssData::CopyWorkingBeidouSatellites(bool reversion) 
{ 
  if(reversion)
  {
    CopySatellites(workingBdSatellites, bdSatellites);
 }
  else
  {
    CopySatellites(bdSatellites, workingBdSatellites);
  }
}
#endif  //#if (_SUPPORT_BEIDOU_SATELLITES_)

bool GnssData::SetDate(U16 y, U16 m, U16 d) 
{ 
  if(year == y && month == m && day == d)
  {
    return false;
  }

  year = y; 
  month = m; 
  day = d; 
  return true;
}

bool GnssData::SetTime(U16 h, U16 m, D64 s) 
{ 
  if(hour == h && minute == m && second == s)
  {
    return false;
  }

  hour = h; 
  minute = m; 
  second = s; 
  return true;
}

bool GnssData::SetNmeaLatitude(double lat, char ns)
{
  double tmpLat = 0;  
  int dd = (int)lat / 100;
  tmpLat = dd + (lat - dd * 100) / 60;
  if(ns == 'S')
  {
    tmpLat *= -1;
  }

  if(tmpLat == latitude)
  {
    return false;
  }

  latitude = tmpLat;
  return true;
}

bool GnssData::SetNmeaLongitude(double lon, char ew)
{
  double tmpLon = 0;  
  int dd = (int)lon / 100;
  tmpLon = dd + (lon - dd * 100) / 60;
  if(ew == 'W')
  {
    tmpLon *= -1;
  }

  if(tmpLon == longitude)
  {
    return false;
  }

  longitude = tmpLon;
  return true;
}

bool GnssData::SetQualityMode(QualityMode m)
{ 
  if(qualityMode == m)
  {
    return false;
  }
  qualityMode = m; 
  return true;
}

bool GnssData::SetNavigationMode(NavigationMode n)
{ 
  if(navigationMode == n)
  {
    return false;
  }
  navigationMode = n; 
  return true;
}

bool GnssData::SetAltitudeAboutMeanSeaLevel(double alt) 
{ 
  if(altitudeAboutMeanSeaLevel == alt)
  {
    return false;
  }
  altitudeAboutMeanSeaLevel = alt; 
  return true;
}

bool GnssData::SetGeoidalSeparationInMeter(double gs) 
{ 
  geoidalSeparation = gs; 
  return true;
}

bool GnssData::SetCourse(D64 c) 
{ 
  if(courseOverGround == c)
  {
    return false;
  }
  courseOverGround = c; 
  return true;
}

bool GnssData::SetSpeedInKnots(D64 s) 
{ 
  if(speedKnot == s)
  {
    return false;
  }
  speedKnot = s; 
  return true;
}

bool GnssData::SetNumberOfSv(U16 n)
{ 
  if(numSV == n)
  {
    return false;
  }
  numSV = n; 
  return true;
}

bool GnssData::SetHdop(D64 h)
{
  if(hdop == h)
  {
    return false;
  }
  hdop = h; 
  return true;
}

bool GnssData::SetPdop(D64 p)
{
  if(pdop == p)
  {
    return false;
  }
  pdop = p; 
  return true;
}

bool GnssData::SetVdop(D64 v)
{
  if(vdop == v)
  {
    return false;
  }
  vdop = v; 
  return true;
}

bool GnssData::AddAnInUsePrnToSatellites(SatelliteInfo* s, int p)
{
  int i = 0;
  for(; i < MaxSatelliteNum; ++i)
  {
    if(s[i].sv == 0)
    { //new sv, add to satellites
      s[i].sv = p;
      s[i].isInUse = true;
      return true;
    }
    else if(s[i].sv == p)
    { //exist sv, modify isInUse flag
      bool update = !s[i].isInUse;
      s[i].isInUse = true;
      return update;
    }
  }
  return false;
}

void GnssData::AddAnInUsePrn(U16* s, int p)
{
  for(int i = 0; i < MaxSatelliteNum; ++i)
  {
    if(s[i] == 0)
    {
      s[i] = p;
      break;
    }
  }
}

int GnssData::FindPrnInSatellites(SatelliteInfo* s, int p, bool addNew) const
{
  int i = 0;
  for(; i < MaxSatelliteNum; ++i)
  {
    if(s[i].sv == p)
      return i;
    if(addNew && s[i].sv == 0)
      return i;
  }
  return -1;
}

bool GnssData::UpdateSatellites(SatelliteInfo* s, int sv, int elv, int az, int cno)
{
  int i = FindPrnInSatellites(s, sv, true);
  if(i == -1)
  {
    return false;
  }

  bool update = false;
  if(s[i].sv != sv || s[i].elevation != elv || s[i].azimuth != az || s[i].cno != cno)
  {
    update = true;
  }
  s[i].sv = sv;
  s[i].elevation = elv;
  s[i].azimuth = az;
  s[i].cno = cno;
  
  return update;
}


void GnssData::UpdateInUseToSatellites(SatelliteInfo* s, const U16* inUse)
{
  int i = 0, j = 0;
  for(; i < MaxSatelliteNum; ++i)
  {
    if(inUse[i] == 0)
      break;
    for(j = 0; j < MaxSatelliteNum; ++j)
    {
      if(s[j].sv == inUse[i])
      {
        s[j].isInUse = true;
        break;
      }
    }
  }
}



bool GnssData::SetEnuVelocity(double ev, double nv, double uv) 
{ 
  if(eVelocity == ev && nVelocity == nv && uVelocity == uv)
  {
    return false;
  }
  eVelocity = ev; 
  nVelocity = nv; 
  uVelocity = uv; 
  return true;
}

bool GnssData::SetRtkAge(D64 r)
{
  if(rtkAge == r)
  {
    return false;
  }
  rtkAge = r; 
  return true;
}

bool GnssData::SetRtkRatio(D64 r)
{
  if(rtkRatio == r)
  {
    return false;
  }
  rtkRatio = r; 
  return true;
}

bool GnssData::SetEnuProjection(double ep, double np, double up)
{ 
  if(eProjection == ep && nProjection == np && uProjection == up)
  {
    return false;
  }
  eProjection = ep; 
  nProjection = np; 
  uProjection = up; 
  return true;
}

bool GnssData::SetBaselineLength(D64 b)
{
  if(baselineLength == b)
  {
    return false;
  }
  baselineLength = b; 
  return true;
}

bool GnssData::SetBaselineCourse(D64 b)
{
  if(baselineCourse == b)
  {
    return false;
  }
  baselineCourse = b; 
  return true;
}

void GnssData::CopySatellites(SatelliteInfo* target, const SatelliteInfo* source)
{
  for(int i = 0; i < MaxSatelliteNum; ++i)
  {
    target[i] = source[i];
  }
}

void GnssData::ClearSatellites(SatelliteInfo* s)
{
  for(int i = 0; i < MaxSatelliteNum; ++i)
  {
    s[i].Clear();
  }
}

void GnssData::ClearInUseSatellites(U16* s)
{
  for(int i = 0; i < MaxSatelliteNum; ++i)
  {
    s[i] = 0;
  }
}

SkyTraqNmeaParser::SkyTraqNmeaParser(void)
{
  EmptyBuffer();
  updateFlag = 0;
  notifyFunction = NULL;
}

SkyTraqNmeaParser::~SkyTraqNmeaParser(void)
{
}

ParsingType SkyTraqNmeaParser::Encode(U08 b)
{
  buffer[bufferIndex++] = b;
  //Received one line
  if(bufferIndex > 1 && buffer[bufferIndex - 1] == 0x0a && buffer[bufferIndex - 2] == 0x0d)
  {
    //Remove \r \n
    buffer[bufferIndex - 2] = 0;
    bufferIndex -= 2;
    parsingType = ParsingMessage();
    EmptyBuffer();
    return parsingType;
  }

  if(bufferIndex == LineBufferSize - 1)
  {
    EmptyBuffer();
    return BufferOverflow;
  }
  return None;
}

ParsingType SkyTraqNmeaParser::ParsingMessage()
{
  if(bufferIndex < 10 || buffer[0] != '$' || buffer[bufferIndex - 3] != '*')
  {
    return MessageUnknown;
  }

  parsingType = MessageType(buffer, bufferIndex); 
  if(parsingType != MessageUnknown)
  {
    ScanCommaPos(buffer, bufferIndex);
  }

  switch(parsingType)
  {
  case MessageGGA:
    ProcessingGGA(buffer, bufferIndex);
    break;
  case MessageGNGSA:
    ProcessingGSA(GsUnknown, buffer, bufferIndex);
    break;
  case MessageGPGSA:    
    ProcessingGSA(GsGps, buffer, bufferIndex);
    break;
  case MessageGLGSA:
    ProcessingGSA(GsGlonass, buffer, bufferIndex);
    break;
  case MessageBDGSA:
    ProcessingGSA(GsBeidou, buffer, bufferIndex);
    break;
  case MessageRMC:
    ProcessingRMC(buffer, bufferIndex);   
    break;
  case MessageGLL:
    ProcessingGLL(buffer, bufferIndex);   
    break;
  case MessageZDA:      
    ProcessingZDA(buffer, bufferIndex);           
    break;    
  //case MessageGNS:    
   // ProcessingGNS(buffer, bufferIndex);
   // break; 
  case MessageVTG:        
    ProcessingVTG(buffer, bufferIndex);
    break;
  case MessageGPGSV:    
    ProcessingGSV(GsGps, buffer, bufferIndex);
    break;  
  case MessageGLGSV:
    ProcessingGSV(GsGlonass, buffer, bufferIndex);
    break;
  case MessageBDGSV:
    ProcessingGSV(GsBeidou, buffer, bufferIndex);
    break;
  case MessagePSTI30:
    ProcessingPSTI30(buffer, bufferIndex);
    break;
  case MessagePSTI32:
    ProcessingPSTI32(buffer, bufferIndex);
    break;
  default:
    return parsingType;
  }
  Notify();


  return parsingType;
}

GnssData::QualityMode SkyTraqNmeaParser::GetGgaQualityMode(char q)
{
  switch (q)
  {
  case '0':
    return GnssData::QM_NotFix;
  case '1':
    return GnssData::QM_Autonomous;
  case '2':
    return GnssData::QM_Differential;
  case '3':
    return GnssData::QM_Precise;
  case '4':
    return GnssData::QM_RealTimeKinematic;
  case '5':
    return GnssData::QM_FloatRtk;
  case '6':
    return GnssData::QM_Estimated;
  case '7':
    return GnssData::QM_ManualInput;
  case '8':
    return GnssData::QM_Simulator;
  default:
    return GnssData::QM_NotFix;
  }
}

GnssData::QualityMode SkyTraqNmeaParser::GetRmcQualityMode(char q)
{
  switch (q)
  {
  case 'N':
    return GnssData::QM_NotFix;
  case 'A':
    return GnssData::QM_Autonomous;
  case 'D':
    return GnssData::QM_Differential;
  case 'P':
    return GnssData::QM_Precise;
  case 'R':
    return GnssData::QM_RealTimeKinematic;
  case 'F':
    return GnssData::QM_FloatRtk;
  case 'E':
    return GnssData::QM_Estimated;
  case 'M':
    return GnssData::QM_ManualInput;
  case 'S':
    return GnssData::QM_Simulator;
  default:
    return GnssData::QM_NotFix;
  }
}

GnssData::NavigationMode SkyTraqNmeaParser::GetGsaNavigationMode(char n)
{
  switch (n)
  {
  case '1':
    return GnssData::NM_NotFix;
  case '2':
    return GnssData::NM_2DFix;
  case '3':
    return GnssData::NM_3DFix;
  default:
    return GnssData::NM_NotFix;
  }
}

GnssSystem SkyTraqNmeaParser::GetGNSSSystem(int prn)
{
  if(prn==0)
  {
    return GsUnknown;
  }
  if(prn >= 65 && prn <= 96)
  {
    return GsGlonass;
  }

  if( (prn >= 0 && prn <= 50) || (prn >= 193 && prn <= 197))
  {
    return GsGps;
  }
  return GsBeidou;
}

ParsingType SkyTraqNmeaParser::MessageType(U08* pt, int len)
{
  struct NmeaTypeEntry
  {
    const char* subNmea;
    ParsingType type;
  };

  NmeaTypeEntry nmeaTable[] = {
    { "$GPGGA,", MessageGGA },
    { "$GNGGA,", MessageGGA },
    { "$BDGGA,", MessageGGA },

    { "$GPGSA,", MessageGPGSA },
    { "$GLGSA,", MessageGLGSA },
    { "$BDGSA,", MessageBDGSA },
    { "$GNGSA,", MessageGNGSA },

    { "$GPGSV,", MessageGPGSV },
    { "$GLGSV,", MessageGLGSV },
    { "$BDGSV,", MessageBDGSV },

    { "$GPRMC,", MessageRMC },
    { "$GNRMC,", MessageRMC },
    { "$BDRMC,", MessageRMC },

    { "$GNGNS,", MessageGNS },

    { "$GPVTG,", MessageVTG },
    { "$GNVTG,", MessageVTG },

    { "$GPGLL,", MessageGLL },
    { "$GNGLL,", MessageGLL },

    { "$GPZDA,", MessageZDA },
    { "$GNZDA,", MessageZDA },

    { "$PSTI,030", MessagePSTI30 },
    { "$PSTI,032", MessagePSTI32 },
    { NULL, None }
  };

  if(!VarifyNmeaChecksum(pt, len))
  {
    return MessageUnknown;
  }

  int i = 0;
  parsingType = MessageUnknown;
  while(nmeaTable[i].subNmea != NULL)
  {
    if(0==StrHeaderCompare(pt, (const U08*)nmeaTable[i].subNmea, strlen(nmeaTable[i].subNmea)))
    {
      parsingType = nmeaTable[i].type;
      break;
    }
    ++i;
  }

  return parsingType;
}

int SkyTraqNmeaParser::StrHeaderCompare(const U08* pt, const U08* header, int len) const
{
  const U08* p1 = pt;
  const U08* p2 = header;
  int cl = 0;
  while(*p1 != 0 && *p2 != 0 && cl < len)
  {
    if(*p1 > *p2)
    {
      return 1;
    }
    else if(*p1 < *p2)
    {
      return -1;
    }
    ++cl;
    ++p1;
    ++p2;
  }
  if(*p2==0)
  {
    return 0;
  }
  return -1;
}

bool SkyTraqNmeaParser::VarifyNmeaChecksum(U08* pt, int len) const
{
  char checksum = 0;  
  for(int j = 1; j < len - 3; ++j)
  {   
    checksum ^= pt[j];
  } 
  return checksum == (char)(ConvertChecksum(*(pt + len - 2), *(pt + len - 1)));
}

int SkyTraqNmeaParser::ConvertChecksum(char h, char l) const
{
  return HexChar2Int(l) + HexChar2Int(h) * 16;
}

int SkyTraqNmeaParser::HexChar2Int(char c) const
{
  if(c >= '0' && c <= '9')
  {
    return (c - '0');
  }
  else if(c >= 'A' && c <= 'F')
  {
    return (c - 'A' + 0xA);
  }
  else if(c >= 'a' && c <= 'f')
  {
    return (c - 'a' + 0xA);
  }
  return 0;
}

void SkyTraqNmeaParser::ScanCommaPos(const U08* pt, int len)
{
  int pos = 0;
  EmptyCommaPos();
  for(int i = 5; i < len - 3; ++i)
  {
    if(pt[i] == ',')
    {
      commaPos[pos] = i;
      ++pos;
    }
  }
  commaPos[pos] = len - 3;
  commaNum = pos;
}

static int GetParamInt(const U08* pt, int start, int end, int defaultValue)
{
  const int MaxIntStringSize = 16;
  char buf[MaxIntStringSize] = { 0 };

  if(end - start > MaxIntStringSize)
  {
    return defaultValue;
  }

  memcpy(buf, pt + start, end - start + 1);
  return atoi(buf);
}

static D64 GetParamDouble(const U08* pt, int start, int end, double defaultValue)
{
  const int MaxIntStringSize = 16;
  char buf[MaxIntStringSize] = { 0 };

  if(end - start > MaxIntStringSize)
  {
    return defaultValue;
  }

  memcpy(buf, pt + start, end - start + 1);
  return atof(buf);
}

static char GetParamChar(const U08* pt, int start, int end, char defaultValue)
{
  return (end = start) ? pt[start] : defaultValue;
}

void SkyTraqNmeaParser::ProcessingGGA(const U08* pt, int len)
{
  int hour = GetParamInt(pt, commaPos[0] + 1, commaPos[0] + 2, 0);
  int min = GetParamInt(pt, commaPos[0] + 3, commaPos[0] + 4, 0);
  double sec = GetParamDouble(pt, commaPos[0] + 5, commaPos[1] - 1, 0);
  if(gnssData.SetTime(hour, min, sec))
  {
    updateFlag |= UpdateTime;
  }

  double latitude = 0, lontitude = 0, altAboveMsl = 0, gs = 0;
  latitude = GetParamDouble(pt, commaPos[1] + 1, commaPos[2] - 1, 0);
  if(gnssData.SetNmeaLatitude(latitude, GetParamChar(pt, commaPos[2] + 1, commaPos[3] - 1, ' ')))
  {
    updateFlag |= UpdateLatitude;
  }

  lontitude = GetParamDouble(pt, commaPos[3] + 1, commaPos[4] - 1, 0);
  if(gnssData.SetNmeaLongitude(lontitude, GetParamChar(pt, commaPos[4] + 1, commaPos[5] - 1, ' ')))
  {
    updateFlag |= UpdateLongitude;
  }

  char q = GetParamChar(pt, commaPos[5] + 1, commaPos[6] - 1, 0);
  if(gnssData.SetQualityMode(GetGgaQualityMode(q)))
  {
    updateFlag |= UpdateQualitMode;
  }

  U16 n = GetParamInt(pt, commaPos[6] + 1, commaPos[7] - 1, 0);
  if(gnssData.SetNumberOfSv(n))
  {
    updateFlag |= UpdateNumberOfSv;
  }

  double hdop = GetParamDouble(pt, commaPos[7] + 1, commaPos[8] - 1, 0);
  if(gnssData.SetHdop(hdop))
  {
    updateFlag |= UpdateHdop;
  }

  altAboveMsl = GetParamDouble(pt, commaPos[8] + 1, commaPos[9] - 1, 0);
  bool updated =gnssData.SetAltitudeAboutMeanSeaLevel(altAboveMsl);

  gs = GetParamDouble(pt, commaPos[10] + 1, commaPos[11] - 1, 0);
  updated |= gnssData.SetGeoidalSeparationInMeter(gs);

  if(updated)
  {
    updateFlag |= UpdateAltitude;
  }
}

void SkyTraqNmeaParser::ProcessingGLL(const U08* pt, int len)
{
  double latitude = 0, lontitude = 0, altAboveMsl = 0, gs = 0;
  latitude = GetParamDouble(pt, commaPos[0] + 1, commaPos[1] - 1, 0);
  if(gnssData.SetNmeaLatitude(latitude, GetParamChar(pt, commaPos[1] + 1, commaPos[2] - 1, ' ')))
  {
    updateFlag |= UpdateLatitude;
  }

  lontitude = GetParamDouble(pt, commaPos[2] + 1, commaPos[3] - 1, 0);
  if(gnssData.SetNmeaLongitude(lontitude, GetParamChar(pt, commaPos[3] + 1, commaPos[4] - 1, ' ')))
  {
    updateFlag |= UpdateLongitude;
  }

  int hour = GetParamInt(pt, commaPos[4] + 1, commaPos[4] + 2, 0);
  int min = GetParamInt(pt, commaPos[4] + 3, commaPos[4] + 4, 0);
  double sec = GetParamDouble(pt, commaPos[4] + 5, commaPos[5] - 1, 0);
  if(gnssData.SetTime(hour, min, sec))
  {
    updateFlag |= UpdateTime;
  }
}

void SkyTraqNmeaParser::ProcessingGSA(GnssSystem gs, const U08* pt, int len)
{
  bool gpCleared = false, glCleared = false, bdCleared = false;  
  char navMode = GetParamChar(pt, commaPos[1] + 1, commaPos[2] - 1, 0);
  if(gnssData.SetNavigationMode(GetGsaNavigationMode(navMode)))
  {
    updateFlag |= UpdateNavigationMode;
  }

  int i = 2, p = 0;
  bool updated = false;
  for(; i < 14; ++i)
  {
    if(commaPos[i] == commaPos[i + 1] - 1)
    {
      break;
    }
    p = GetParamInt(pt, commaPos[i] + 1, commaPos[i + 1] - 1, 0);
    if(p == 0)
    {
      break;
    }

    if(GsUnknown == gs)
    {
      gs = GetGNSSSystem(p);
    }
    switch(gs)
    {
#if (_SUPPORT_GPS_SATELLITES_)
    case GsGps:
      if(!gpCleared)
      {
        gnssData.ClearInUseGpSatellites();
        gnssData.CopyWorkingGpsSatellites(true);
        gpCleared = true;
      }
      updated |= gnssData.AddAnInUseGpPrn(p);
      break;
#endif  //#if (_SUPPORT_GPS_SATELLITES_)
#if (_SUPPORT_GLONASS_SATELLITES_)
    case GsGlonass:
      if(!glCleared)
      {
        gnssData.ClearInUseGlSatellites();
        gnssData.CopyWorkingGlonassSatellites(true);
        glCleared = true;
      }
      updated |= gnssData.AddAnInUseGlPrn(p);
      break;
#endif  //#if (_SUPPORT_GLONASS_SATELLITES_)
#if (_SUPPORT_BEIDOU__SATELLITES_)
    case GsBeidou:
      if(!bdCleared)
      {
        gnssData.ClearInUseBdSatellites();
        gnssData.CopyWorkingBeidouSatellites(true);
        bdCleared = true;
      }
      updated |= gnssData.AddAnInUseBdPrn(p);
      break;
#endif  //_SUPPORT_BEIDOU__SATELLITES_
    default:
       break;
    }
  } //for(; i < 14; ++i)  

  if(i == 2)
  {
    return;
  }

  if(updated)
  {
    switch(gs)
    {
#if (_SUPPORT_GPS_SATELLITES_)    
    case GsGps:
      gnssData.CopyWorkingGpsSatellites();
      break;
#endif
#if (_SUPPORT_GLONASS_SATELLITES_)
    case GsGlonass:
      gnssData.CopyWorkingGlonassSatellites();
      break;
#endif
#if (_SUPPORT_BEIDOU__SATELLITES_)
    case GsBeidou:
      gnssData.CopyWorkingBeidouSatellites();
      break;
#endif
    default:
       break;
    }
    updateFlag |= UpdateSatelliteInfo;
  }

  double dop = 0;
  dop = GetParamDouble(pt, commaPos[14] + 1, commaPos[15] - 1, 0.0F);
  if(gnssData.SetPdop(dop))
  {
    updateFlag |= UpdatePdop;
  }

  dop = GetParamDouble(pt, commaPos[15] + 1, commaPos[16] - 1, 0.0F);
  if(gnssData.SetHdop(dop))
  {
    updateFlag |= UpdateHdop;
  }

  dop = GetParamDouble(pt, commaPos[16] + 1, commaPos[17] - 1, 0.0F);
  if(gnssData.SetVdop(dop))
  {
    updateFlag |= UpdateVdop;
  }

}

void SkyTraqNmeaParser::ProcessingGSV(GnssSystem gs, const U08* pt, int len)
{
  const int MaxSateNumInGsv = 4;
  static bool updated = false;

  int numMsg = GetParamInt(pt, commaPos[0] + 1, commaPos[1] - 1, 0);
  int msgNum = GetParamInt(pt, commaPos[1] + 1, commaPos[2] - 1, 0);
  int numSV = GetParamInt(pt, commaPos[2] + 1, commaPos[3] - 1, 0);
  int n = (msgNum < numMsg) ? MaxSateNumInGsv : (numSV - MaxSateNumInGsv * (numMsg - 1));

  if(msgNum == 1)
  {
    updated = false;
    switch(gs)
    {
#if (_SUPPORT_GPS_SATELLITES_)
    case GsGps:
      gnssData.ClearWorkingGpsSatellites();
      break;
#endif
#if (_SUPPORT_GLONASS_SATELLITES_)
    case GsGlonass:
      gnssData.ClearWorkingGlonassSatellites();
      break;
#endif
#if (_SUPPORT_BEIDOU_SATELLITES_)
    case GsBeidou:
      gnssData.ClearWorkingBeidouSatellites();
      break;
#endif
    default:
       break;
    }
  }

  int i = 4;
  for(; i < 4 + n * MaxSateNumInGsv; i += MaxSateNumInGsv)
  {
    if(i + 3 > commaNum)
    {
      break;
    }
    int sv = GetParamInt(pt, commaPos[i - 1] + 1, commaPos[i] - 1, 0);
    int elv = GetParamInt(pt, commaPos[i] + 1, commaPos[i + 1] - 1, 0);
    int az = GetParamInt(pt, commaPos[i + 1] + 1, commaPos[i + 2] - 1, 0);
    int cno = GetParamInt(pt, commaPos[i + 2] + 1, commaPos[i + 3] - 1, 0);
    if(sv == 0)
    {
      continue;
    }

    if(GsUnknown == gs)
    {
      gs = GetGNSSSystem(sv);
    }
    switch(gs)
    {
#if (_SUPPORT_GPS_SATELLITES_)
    case GsGps:
      updated |= gnssData.UpdateGpSatellites(sv, elv, az, cno);
      break;
#endif
#if (_SUPPORT_GLONASS_SATELLITES_)
    case GsGlonass:
      updated |= gnssData.UpdateGlSatellites(sv, elv, az, cno);
      break;
#endif
#if (_SUPPORT_BEIDOU_SATELLITES_)
    case GsBeidou:
      updated |= gnssData.UpdateBdSatellites(sv, elv, az, cno);
      break;
#endif
    default:
       break;
    }
  } //for(; i < 14; ++i)  

  if(msgNum == numMsg)
  {
    switch(gs)
    {
#if (_SUPPORT_GPS_SATELLITES_)
    case GsGps:
      gnssData.UpdateGpsInUseToSatellites();
      gnssData.CopyWorkingGpsSatellites();
      break;
#endif
#if (_SUPPORT_GLONASS_SATELLITES_)
    case GsGlonass:
      gnssData.UpdateGlonassInUseToSatellites();
      gnssData.CopyWorkingGlonassSatellites();
      break;
#endif
#if (_SUPPORT_BEIDOU_SATELLITES_)
    case GsBeidou:
      gnssData.UpdateBeidouInUseToSatellites();
      gnssData.CopyWorkingBeidouSatellites();
      break;
#endif
    default:
       break;
    }
  }

  if(updated && msgNum == numMsg)
  {
    updateFlag |= UpdateSatelliteInfo;
  }
}

void SkyTraqNmeaParser::ProcessingRMC(const U08* pt, int len)
{
  int hour = GetParamInt(pt, commaPos[0] + 1, commaPos[0] + 2, 0);
  int min = GetParamInt(pt, commaPos[0] + 3, commaPos[0] + 4, 0);
  double sec = GetParamDouble(pt, commaPos[0] + 5, commaPos[1] - 1, 0);
  if(gnssData.SetTime(hour, min, sec))
  {
    updateFlag |= UpdateTime;
  }

  double latitude = 0, lontitude = 0;
  latitude = GetParamDouble(pt, commaPos[2] + 1, commaPos[3] - 1, 0);
  if(gnssData.SetNmeaLatitude(latitude, GetParamChar(pt, commaPos[3] + 1, commaPos[4] - 1, ' ')))
  {
    updateFlag |= UpdateLatitude;
  }

  lontitude = GetParamDouble(pt, commaPos[4] + 1, commaPos[5] - 1, 0);
  if(gnssData.SetNmeaLongitude(lontitude, GetParamChar(pt, commaPos[5] + 1, commaPos[6] - 1, ' ')))
  {
    updateFlag |= UpdateLongitude;
  }

  double speedKnots = GetParamDouble(pt, commaPos[6] + 1, commaPos[7] - 1, 0);
  if(gnssData.SetSpeedInKnots(speedKnots))
  {
    updateFlag |= UpdateSpeed;
  }

  double course = GetParamDouble(pt, commaPos[7] + 1, commaPos[8] - 1, 0);
  if(gnssData.SetCourse(course))
  {
    updateFlag |= UpdateCourse;
  }

  int day = GetParamInt(pt, commaPos[8] + 1, commaPos[8] + 2, 0);
  int month = GetParamInt(pt, commaPos[8] + 3, commaPos[8] + 4, 0);
  int year = 2000 + GetParamInt(pt, commaPos[8] + 5, commaPos[9] - 1, 0);
  if(gnssData.SetDate(year, month, day))
  {
    updateFlag |= UpdateDate;
  }

  char q = GetParamChar(pt, commaPos[11] + 1, commaPos[12] - 1, 0);
  if(gnssData.SetQualityMode(GetRmcQualityMode(q)))
  {
    updateFlag |= UpdateQualitMode;
  }
}


void SkyTraqNmeaParser::ProcessingVTG(const U08* pt, int len)
{
  double course = GetParamDouble(pt, commaPos[0] + 1, commaPos[1] - 1, 0);
  if(gnssData.SetCourse(course))
  {
    updateFlag |= UpdateCourse;
  }

  double speedKnots = GetParamDouble(pt, commaPos[4] + 1, commaPos[5] - 1, 0);
  if(gnssData.SetSpeedInKnots(speedKnots))
  {
    updateFlag |= UpdateSpeed;
  }
}

void SkyTraqNmeaParser::ProcessingZDA(const U08* pt, int len)
{
  int hour = GetParamInt(pt, commaPos[0] + 1, commaPos[0] + 2, 0);
  int min = GetParamInt(pt, commaPos[0] + 3, commaPos[0] + 4, 0);
  double sec = GetParamDouble(pt, commaPos[0] + 5, commaPos[1] - 1, 0);
  if(gnssData.SetTime(hour, min, sec))
  {
    updateFlag |= UpdateTime;
  }

  int month = GetParamInt(pt, commaPos[2] + 1, commaPos[3] - 1, 0);
  int day = GetParamInt(pt, commaPos[1] + 1, commaPos[2] - 1, 0);
  int year = GetParamInt(pt, commaPos[3] + 1, commaPos[4] - 1, 0);
  if(gnssData.SetDate(year, month, day))
  {
    updateFlag |= UpdateDate;
  }
}

void SkyTraqNmeaParser::ProcessingPSTI30(const U08* pt, int len)
{
  int hour = GetParamInt(pt, commaPos[1] + 1, commaPos[1] + 2, 0);
  int min = GetParamInt(pt, commaPos[1] + 3, commaPos[1] + 4, 0);
  double sec = GetParamDouble(pt, commaPos[1] + 5, commaPos[2] - 1, 0);
  if(gnssData.SetTime(hour, min, sec))
  {
    updateFlag |= UpdateTime;
  }

  double latitude = 0, lontitude = 0;
  latitude = GetParamDouble(pt, commaPos[3] + 1, commaPos[4] - 1, 0);
  if(gnssData.SetNmeaLatitude(latitude, GetParamChar(pt, commaPos[4] + 1, commaPos[5] - 1, ' ')))
  {
    updateFlag |= UpdateLatitude;
  }

  lontitude = GetParamDouble(pt, commaPos[5] + 1, commaPos[6] - 1, 0);
  if(gnssData.SetNmeaLongitude(lontitude, GetParamChar(pt, commaPos[6] + 1, commaPos[7] - 1, ' ')))
  {
    updateFlag |= UpdateLongitude;
  }

  double altAboveMsl = GetParamDouble(pt, commaPos[7] + 1, commaPos[8] - 1, 0);
  if(gnssData.SetAltitudeAboutMeanSeaLevel(altAboveMsl))
  {
    updateFlag |= UpdateAltitude;
  }

  double eVelocity = GetParamDouble(pt, commaPos[8] + 1, commaPos[9] - 1, 0);
  double nVelocity = GetParamDouble(pt, commaPos[9] + 1, commaPos[10] - 1, 0);
  double uVelocity = GetParamDouble(pt, commaPos[10] + 1, commaPos[11] - 1, 0);
  if(gnssData.SetEnuVelocity(eVelocity, nVelocity, uVelocity))
  {
    updateFlag |= UpdateEnuVelocity;
  }

  int day = GetParamInt(pt, commaPos[11] + 1, commaPos[11] + 2, 0);
  int month = GetParamInt(pt, commaPos[11] + 3, commaPos[11] + 4, 0);
  int year = 2000 + GetParamInt(pt, commaPos[11] + 5, commaPos[12] - 1, 0);
  if(gnssData.SetDate(year, month, day))
  {
    updateFlag |= UpdateDate;
  }

  char q = GetParamChar(pt, commaPos[12] + 1, commaPos[13] - 1, 0);
  if(gnssData.SetQualityMode(GetRmcQualityMode(q)))
  {
    updateFlag |= UpdateQualitMode;
  }

  double rtkAge = GetParamDouble(pt, commaPos[13] + 1, commaPos[14] - 1, 0);
  if(gnssData.SetRtkAge(rtkAge))
  {
    updateFlag |= UpdateRtkAge;
  }

  double rtkRatio = GetParamDouble(pt, commaPos[14] + 1, commaPos[15] - 1, 0);
  if(gnssData.SetRtkRatio(rtkRatio))
  {
    updateFlag |= UpdateRtkRatio;
  }
}

void SkyTraqNmeaParser::ProcessingPSTI32(const U08* pt, int len)
{
  int hour = GetParamInt(pt, commaPos[1] + 1, commaPos[1] + 2, 0);
  int min = GetParamInt(pt, commaPos[1] + 3, commaPos[1] + 4, 0);
  double sec = GetParamDouble(pt, commaPos[1] + 5, commaPos[2] - 1, 0);
  if(gnssData.SetTime(hour, min, sec))
  {
    updateFlag |= UpdateTime;
  }

  int day = GetParamInt(pt, commaPos[2] + 1, commaPos[2] + 2, 0);
  int month = GetParamInt(pt, commaPos[2] + 3, commaPos[2] + 4, 0);
  int year = 2000 + GetParamInt(pt, commaPos[2] + 5, commaPos[3] - 1, 0);
  if(gnssData.SetDate(year, month, day))
  {
    updateFlag |= UpdateDate;
  }

  char status = GetParamChar(pt, commaPos[3] + 1, commaPos[4] - 1, 0);
  if(status == 'V')
  {
    return;
  }

  status = GetParamChar(pt, commaPos[4] + 1, commaPos[5] - 1, 0);
  if(gnssData.SetQualityMode(GetRmcQualityMode(status)))
  {
    updateFlag |= UpdateQualitMode;
  }

  double eProjection = GetParamDouble(pt, commaPos[5] + 1, commaPos[6] - 1, 0);
  double nProjection = GetParamDouble(pt, commaPos[6] + 1, commaPos[7] - 1, 0);
  double uProjection = GetParamDouble(pt, commaPos[7] + 1, commaPos[8] - 1, 0);
  if(gnssData.SetEnuProjection(eProjection, nProjection, uProjection))
  {
    updateFlag |= UpdateEnuProjection;
  }

  double baselineLength = GetParamDouble(pt, commaPos[8] + 1, commaPos[9] - 1, 0);
  if(gnssData.SetBaselineLength(baselineLength))
  {
    updateFlag |= UpdateBaselineLength;
  }

  double baselineCourse = GetParamDouble(pt, commaPos[9] + 1, commaPos[10] - 1, 0);
  if(gnssData.SetBaselineCourse(baselineCourse))
  {
    updateFlag |= UpdateBaselineCourse;
  }
}

void SkyTraqNmeaParser::Notify()
{
  if(notifyFunction != NULL && notifyFunction->gnssUpdated(updateFlag, (const char*)buffer, parsingType))
  {
    updateFlag = 0;
  }
}

void SkyTraqNmeaParser::EmptyBuffer() 
{ 
  memset(buffer, 0, sizeof(buffer)); 
  bufferIndex = 0; 
}

void SkyTraqNmeaParser::EmptyCommaPos() 
{ 
  memset(commaPos, 0, sizeof(commaPos)); 
  commaNum = 0; 
}


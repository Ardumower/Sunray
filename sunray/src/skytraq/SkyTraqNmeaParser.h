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

#pragma once

typedef signed   char             S08;
typedef unsigned char             U08;
typedef signed   short int        S16;
typedef unsigned short int        U16;
typedef signed   long int         S32;
typedef unsigned long int         U32;
typedef float                     F32;
typedef double                    D64;

#define _SUPPORT_GPS_SATELLITES_        0
#define _SUPPORT_GLONASS_SATELLITES_    0
#define _SUPPORT_BEIDOU_SATELLITES_     0


enum ParsingType
  {
    None = 0,
    MessageGGA,
    MessageRMC,
    MessageGLL,
    MessageZDA,
    MessageGNS,
    MessageVTG,

    MessageGNGSA,
    MessageGPGSA,
    MessageGLGSA,
    MessageBDGSA,

    MessageGPGSV,
    MessageGLGSV,
    MessageBDGSV,

    MessagePSTI30,
    MessagePSTI32,

    BufferOverflow,
    MessageUnknown,
  };

enum GnssSystem
{
  GsUnknown,
  GsGps,
  GsGlonass,
  GsBeidou,
  GsGalileo,
};

struct SatelliteInfo
{ 
  SatelliteInfo()
  {
    Clear();
  }
  void Clear()
  {
    sv = 0;
    azimuth = 0;
    elevation = 0;
    cno = 0;
    isInUse = false;
  }
  U16 sv;
  U16 azimuth;
  U16 elevation;
  U16 cno;
  bool isInUse;
};

class GnssData
{
public:
  GnssData();
  ~GnssData();

public:
   friend class SkyTraqNmeaParser;
  enum QualityMode {
    QM_NotFix,           //RMC 'N', GGA '0'
    QM_Autonomous,       //RMC 'A', GGA '1'
    QM_Differential,     //RMC 'D', GGA '2'
    QM_Precise,          //RMC 'P', GGA '3'
    QM_RealTimeKinematic,//RMC 'R', GGA '4'
    QM_FloatRtk,         //RMC 'F', GGA '5'
    QM_Estimated,        //RMC 'E', GGA '6'
    QM_ManualInput,      //RMC 'M', GGA '7'
    QM_Simulator,        //RMC 'S', GGA '8'
  };

  enum NavigationMode {
    NM_NotFix,
    NM_2DFix,
    NM_3DFix,
  };

public:
  static const double KnotToKmHr() { return 1.852; }
  static const double KnotToMph() { return 1.151; }
  void ClearData() { Init(); }

  //Getter functions
  //Date and Time
  U16 GetYear() const { return year; }
  U16 GetMonth() const { return month; }
  U16 GetDay() const { return day; }
  U16 GetHour() const { return hour; }
  U16 GetMinute() const { return minute; }
  D64 GetSecond() const { return second; }

  //Geographic Information
  D64 GetLatitude() const { return latitude; }
  D64 GetLongitude() const { return longitude; }
  D64 GetAltitudeInMeter() const { return altitudeAboutMeanSeaLevel + geoidalSeparation; }
  D64 GetAltitudeAboutMeanSeaLevelInMeter() const { return altitudeAboutMeanSeaLevel; }
  D64 GetGeoidalSeparationInMeter() const { return geoidalSeparation; }

  //Speed and Orientation
  D64 GetCourseInDegree() const { return courseOverGround; }
  D64 GetSpeedInKnots() const { return speedKnot; }
  D64 GetSpeedInKmHr() const { return speedKnot * KnotToKmHr(); }
  D64 GetSpeedInMph() const { return speedKnot * KnotToMph(); }

  //Accuracy
  D64 GetHdop() const { return hdop; }
  D64 GetPdop() const { return pdop; }
  D64 GetVdop() const { return vdop; }

  //Fix Mode
  QualityMode GetQualitMode() const { return qualityMode; }
  NavigationMode GetNavigationMode() const { return navigationMode; }
  bool IsFix() const { return qualityMode != QM_NotFix;  }
  bool Is2DFix() const { return navigationMode == NM_2DFix;  }
  bool Is3DFix() const { return navigationMode == NM_3DFix;  }

  //Satellites information
  static int GetMaxSatelliteNum() { return MaxSatelliteNum; };
  U16 GetNumberOfSv() const { return numSV; };
#if (_SUPPORT_GPS_SATELLITES_)
  const SatelliteInfo* GetGpsSatellites() const { return gpSatellites; };
#endif
#if (_SUPPORT_GLONASS_SATELLITES_)
  const SatelliteInfo* GetGlonassSatellites() const { return glSatellites; };
#endif
#if (_SUPPORT_BEIDOU_SATELLITES_)
  const SatelliteInfo* GetBeidouSatellites() const { return bdSatellites; };
#endif
  //RTK information
  D64 GetEVelocity() const { return eVelocity; };
  D64 GetNVelocity() const { return nVelocity; };
  D64 GetUVelocity() const { return uVelocity; };
  D64 GetRtkAge() const { return rtkAge; };
  D64 GetRtkRatio() const { return rtkRatio; };
  D64 GetEProjection() const { return eProjection; };
  D64 GetNProjection() const { return nProjection; };
  D64 GetUProjection() const { return uProjection; };
  D64 GetBaselineLength() const { return baselineLength; };
  D64 GetBaselineCourse() const { return baselineCourse; };

protected:  //data members
  //Date and Time
  U16 year;
  U16 month;
  U16 day;
  U16 hour;
  U16 minute;
  D64 second;

  //Geographic Information
  D64 latitude;
  D64 longitude;
  D64 altitudeAboutMeanSeaLevel;
  D64 geoidalSeparation;

  //Speed and Orientation
  D64 courseOverGround; //Course over ground
  D64 speedKnot;        //Speed in Knot

  //Accuracy
  D64 hdop;
  D64 pdop;
  D64 vdop;

  //Fix Mode
  QualityMode qualityMode;  //Fix Mode
  NavigationMode navigationMode;

  //Satellites information
  enum { MaxSatelliteNum = 16 };
  U16 numSV;
#if (_SUPPORT_GPS_SATELLITES_)
  U16 inUseGpSatellites[MaxSatelliteNum];
  SatelliteInfo gpSatellites[MaxSatelliteNum];
  SatelliteInfo workingGpSatellites[MaxSatelliteNum];
#endif

#if (_SUPPORT_GLONASS_SATELLITES_)
  U16 inUseGlSatellites[MaxSatelliteNum];
  SatelliteInfo glSatellites[MaxSatelliteNum];
  SatelliteInfo workingGlSatellites[MaxSatelliteNum];
#endif

#if (_SUPPORT_BEIDOU_SATELLITES_)
  U16 inUseBdSatellites[MaxSatelliteNum];
  SatelliteInfo bdSatellites[MaxSatelliteNum];
  SatelliteInfo workingBdSatellites[MaxSatelliteNum];
#endif
  //RTK information
  D64 eVelocity; 
  D64 nVelocity; 
  D64 uVelocity; 

  D64 rtkAge;     //Age of differential
  D64 rtkRatio;   //AR ratio factor for validation

  //ENU-Projection of baseline, meters
  D64 eProjection; 
  D64 nProjection; 
  D64 uProjection; 

  D64 baselineLength;   //Baseline length, meters
  D64 baselineCourse;   //Baseline course (angle between baseline vector and north direction), degrees

protected:  //functions
  void Init();
  void CopySatellites(SatelliteInfo* target, const SatelliteInfo* source);
  void ClearSatellites(SatelliteInfo* s);
  void ClearInUseSatellites(U16* s);

  void AddAnInUsePrn(U16* s, int p);
  bool AddAnInUsePrnToSatellites(SatelliteInfo* s, int p);
  int FindPrnInSatellites(SatelliteInfo* s, int p, bool addNew) const;
  bool UpdateSatellites(SatelliteInfo* s, int sv, int elv, int az, int cno);
  void UpdateInUseToSatellites(SatelliteInfo* s, const U16* inUse);

  //Setter functions
  //Date and Time
  bool SetDate(U16 y, U16 m, U16 d);
  bool SetTime(U16 h, U16 m, D64 s);

  //Geographic Information
  bool SetNmeaLatitude(double lat, char ns);
  bool SetNmeaLongitude(double lon, char ew);
  bool SetAltitudeAboutMeanSeaLevel(double alt);
  bool SetGeoidalSeparationInMeter(double gs);

  //Speed and Orientation
  bool SetCourse(D64 c);
  bool SetSpeedInKnots(D64 s);

  //Accuracy
  bool SetPdop(D64 p);
  bool SetHdop(D64 h);
  bool SetVdop(D64 v);

  //Fix Mode
  bool SetNavigationMode(NavigationMode m);
  bool SetQualityMode(QualityMode m);

  //Satellites information
  bool SetNumberOfSv(U16 n);
#if (_SUPPORT_GPS_SATELLITES_)
  bool AddAnInUseGpPrn(int p);
  void ClearInUseGpSatellites();
  bool UpdateGpSatellites(int sv, int elv, int az, int cno);
  void ClearWorkingGpsSatellites();
  void CopyWorkingGpsSatellites(bool reversion = false);
  void UpdateGpsInUseToSatellites();
#endif

#if (_SUPPORT_GLONASS_SATELLITES_)
  bool AddAnInUseGlPrn(int p);
  void ClearInUseGlSatellites();
  bool UpdateGlSatellites(int sv, int elv, int az, int cno);
  void ClearWorkingGlonassSatellites();
  void CopyWorkingGlonassSatellites(bool reversion = false);
  void UpdateGlonassInUseToSatellites();
#endif

#if (_SUPPORT_BEIDOU_SATELLITES_)
  bool AddAnInUseBdPrn(int p);
  void ClearInUseBdSatellites();
  bool UpdateBdSatellites(int sv, int elv, int az, int cno);
  void ClearWorkingBeidouSatellites();
  void CopyWorkingBeidouSatellites(bool reversion = false);
  void UpdateBeidouInUseToSatellites();
#endif

  //RTK information
  bool SetEnuVelocity(double ev, double nv, double uv);
  bool SetRtkAge(D64 r);
  bool SetRtkRatio(D64 r);
  bool SetEnuProjection(double ep, double np, double up);
  bool SetBaselineLength(double b);
  bool SetBaselineCourse(double b);
};


class SkyTraqNotifyFun 
{
  public:
    virtual bool gnssUpdated(U32 f, const char* buf, ParsingType parsingType) = 0;
};


class SkyTraqNmeaParser
{
public: //constructor and destructor
  SkyTraqNmeaParser(void);
  ~SkyTraqNmeaParser(void);

public: //enum and typedef
  
static const U32 NoUpdate = 0;
static const U32 UpdateDate            = (0x00000001);
static const U32 UpdateTime            = (0x00000002);
static const U32 UpdateLatitude        = (0x00000004);
static const U32 UpdateLongitude       = (0x00000008);
static const U32 UpdateAltitude        = (0x00000010);
static const U32 UpdateCourse          = (0x00000020);
static const U32 UpdateSpeed           = (0x00000040);
static const U32 UpdateQualitMode      = (0x00000080);
static const U32 UpdateNumberOfSv      = (0x00000100);
static const U32 UpdateHdop            = (0x00000200);
static const U32 UpdatePdop            = (0x00000400);
static const U32 UpdateVdop            = (0x00000800);
static const U32 UpdateNavigationMode  = (0x00001000);
static const U32 UpdateSatelliteInfo   = (0x00002000);
static const U32 UpdateEnuVelocity     = (0x00004000);
static const U32 UpdateRtkAge          = (0x00008000);
static const U32 UpdateRtkRatio        = (0x00010000);
static const U32 UpdateEnuProjection   = (0x00020000);
static const U32 UpdateBaselineLength  = (0x00040000);
static const U32 UpdateBaselineCourse  = (0x00080000);
  //Notification Callback function type
  //typedef bool (*NotifyFun)(U32, const char*, ParsingType);

public:   //interface
  ParsingType Encode(U08 b);
  //Provide a pointer to receive notifications
  void SetNotify(SkyTraqNotifyFun *f) { notifyFunction = f; }
  const GnssData* GetGnssData() const { return &gnssData; }
  const U08* GetParsingBuffer() const { return buffer; }

protected:  //protected data member
  //Notification callback
  SkyTraqNotifyFun *notifyFunction; 

  GnssData gnssData;
  U32 updateFlag;
  ParsingType parsingType;

  enum { LineBufferSize = 128 };
  U08 buffer[LineBufferSize];
  int bufferIndex;

  enum { MaxParamNum = 20 };
  int commaPos[MaxParamNum];
  int commaNum;

protected:  //protected functions
  void Notify();

  void EmptyBuffer();
  void EmptyCommaPos();
  ParsingType ParsingMessage();
  void ScanCommaPos(const U08* pt, int len);
  ParsingType MessageType(U08* pt, int size);
  int StrHeaderCompare(const U08* pt, const U08* header, int len) const;
  bool VarifyNmeaChecksum(U08* pt, int len) const;
  int ConvertChecksum(char h, char l) const;
  int HexChar2Int(char c) const;
  GnssData::QualityMode GetGgaQualityMode(char q);
  GnssData::QualityMode GetRmcQualityMode(char q);
  GnssData::NavigationMode GetGsaNavigationMode(char n);

  GnssSystem GetGNSSSystem(int prn);
  ParsingType MessageType(const U08* pt, int len);
  
  ParsingType ParseNMEA(const U08* pt, int len);

  void ProcessingGGA(const U08* pt, int len);
  void ProcessingGSA(GnssSystem gs, const U08* pt, int len);
  void ProcessingGSV(GnssSystem gs, const U08* pt, int len);
  void ProcessingRMC(const U08* pt, int len);
  void ProcessingGLL(const U08* pt, int len);
  void ProcessingZDA(const U08* pt, int len);
  void ProcessingVTG(const U08* pt, int len);
  void ProcessingPSTI30(const U08* pt, int len);
  void ProcessingPSTI32(const U08* pt, int len);
};



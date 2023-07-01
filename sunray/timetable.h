#ifndef TIMETABLE_H
#define TIMETABLE_H

#include <Arduino.h>

/*
  ---timetable example (allowed mowing times), up to 10 frames can be set---

  timeframe: monday    08:00 - 13:00
  timeframe: monday    15:00 - 19:00  
  timeframe: thuesday  08:00 - 19:00
  timeframe: wednesday 12:00 - 19:00
  timeframe: thirsday  08:00 - 19:00
  timeframe: friday    08:00 - 19:00
  timeframe: sunday    08:00 - 19:00
  timeframe: -----     08:00 - 19:00
  timeframe: -----     08:00 - 19:00
  timeframe: -----     08:00 - 19:00

  NOTE: timetable times are UTC times (not local time) 
*/

#define TIME_FRAMES 10

typedef struct daytime_t {
  int hour; // UTC time hour
  int min;  // UTC time minute
} daytime_t;

typedef struct timeframe_t {
  bool enabled;
  int dayOfWeek;  // 0=Sunday, 1=Monday, 2=Tuesday, 3=Wednesday, 4=Thursday, 5=Friday, 6=Saturday
  daytime_t startTime;
  daytime_t endTime;  
} timeframe_t;


typedef struct timetable_t {
  timeframe_t frames[TIME_FRAMES];
} timetable_t;



class TimeTable
{
  public:
    timetable_t timeTable;
    bool enabled;    
    TimeTable(); 
    void dump();
    void clear();
    bool addMowingTimeFrame(timeframe_t timeframe);
    bool mowingAllowed(daytime_t time);   
};


#endif


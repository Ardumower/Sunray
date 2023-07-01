#ifndef TIMETABLE_H
#define TIMETABLE_H

#include <Arduino.h>

/*
  timetable enabled: yes


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

*/

typedef struct daytime_t {
  int hour;
  int min;
} daytime_t;

typedef struct timeframe_t {
  daytime_t day;
  daytime_t startTime;
  daytime_t endTime;  
} timeframe_t;


typedef struct timetable_t {
  timeframe_t timeframes[10];
} timetable_t;



class TimeTable
{
  public:
    timetable_t timeTable;
    bool enabled;
    TimeTable();
};


#endif


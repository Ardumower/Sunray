#ifndef TIMETABLE_H
#define TIMETABLE_H

#include <Arduino.h>

/*
  ---timetable example (allowed mowing times), up to 10 frames can be set---

  timeframe: monday    08:00 - 13:00
  timeframe: monday    15:00 - 19:00  
  timeframe: tuesday   08:00 - 19:00
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


typedef struct datetime_t {
  int year;  // UTC time year
  int month; // UTC time month
  int day;   // UTC time day  
  int hour;  // UTC time hour
  int min;   // UTC time minute 
  int dayOfWeek; // UTC time day of week (0=Monday)
} datetime_t;


typedef struct timeframe_t {
  bool enabled;
  int dayOfWeek;  // 0=Monday, 1=Tuesday, 2=Wednesday, 3=Thursday, 4=Friday, 5=Saturday, 6=Sunday
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
    datetime_t currentTime;
    bool enabled;    
    TimeTable(); 
    // dump timetable
    void dump();
    // clear timetable
    void clear();
    // set current UTC time
    void setCurrentTime(int year, int month, int day, int hour, int min); 
    // add timeframe to timetable 
    bool addMowingTimeFrame(timeframe_t timeframe);
    // mowing allowed for current UTC time?
    bool mowingAllowed();
    // mowing allowed for given UTC time?
    bool mowingAllowed(int dayOfWeek, daytime_t time);  
    // calc dayOfWeek(0=Monday) for given UTC date 
    int calcDayOfWeek(int year, int month, int day); 
};


#endif


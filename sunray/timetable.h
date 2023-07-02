#ifndef TIMETABLE_H
#define TIMETABLE_H

#include <Arduino.h>

/*
  ---timetable example (allowed mowing times)---

  

  NOTE: timetable times are UTC times (not local time) 
*/


// time of day
typedef struct daytime_t {
  int hour; // UTC time hour
  int min;  // UTC time minute
} daytime_t;


// time of week
typedef struct weektime_t {
  int hour;  // UTC time hour
  int min;   // UTC time minute 
  int dayOfWeek; // UTC time day of week (0=Monday)
} weektime_t;


// date and time  
typedef struct datetime_t {
  int year;  // UTC time year
  int month; // UTC time month
  int day;   // UTC time day  
  int hour;  // UTC time hour
  int min;   // UTC time minute 
  int dayOfWeek; // UTC time day of week (0=Monday)
} datetime_t;


// time frame within week
typedef struct timeframe_t {
  bool enabled;
  int dayOfWeek;  // 0=Monday, 1=Tuesday, 2=Wednesday, 3=Thursday, 4=Friday, 5=Saturday, 6=Sunday
  daytime_t startTime;
  daytime_t endTime;  
} timeframe_t;


// day mask 
// 1=mon, 2=tue, 4=wed, 8=thu, 16=fri, 32=sat, 64=sun
typedef byte daymask_t; 

// timetable
typedef struct timetable_t {
  daymask_t hours[24];
} timetable_t;



class TimeTable
{
  public:
    timetable_t timetable;
    weektime_t currentTime;  // current time to compare time table against
    bool enabled;    
    TimeTable(); 

    // dump timetable
    void dump();    
    void dumpWeekTime(weektime_t time);

    // clear timetable
    void clear();

    // set current UTC time
    void setCurrentTime(int hour, int min, int weekOfDay); 

    // set day mask for hour 
    bool setDayMask(int hour, daymask_t mask);

    // mowing allowed for current UTC week time?
    bool mowingAllowed();

    // ------ misc functions -----------------------------------    
    // mowing allowed for given UTC week time?
    bool mowingAllowed(weektime_t time);  

    // calc dayOfWeek(0=Monday) for given UTC date  (untested and not used!)
    int calcDayOfWeek(int year, int month, int day); 
};


#endif


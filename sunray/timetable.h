#ifndef TIMETABLE_H
#define TIMETABLE_H

#include <Arduino.h>

/*
  ---timetable example (allowed mowing times)---
NOTE: timetable times are UTC times (not local time) 

GPS time (UTC): dayOfWeek(0=Monday)=6  hour=19  min=25

timetable (UTC times)    00 01 02 03 04 05 06 07 08 09 10 11 12 13 14 15 16 17 18 19 20 21 22 23 
timetable (UTC times) mon                            *  *  *  *  *  *  *  *  *  *  *             
timetable (UTC times) tue                            *  *  *  *  *  *  *  *  *  *  *             
timetable (UTC times) wed                            *  *  *  *  *  *  *  *  *  *  *             
timetable (UTC times) thu                            *  *  *  *  *  *  *  *  *  *  *             
timetable (UTC times) fri                            *  *  *  *  *  *  *  *  *  *  *             
timetable (UTC times) sat                            *  *  *  *  *  *  *  *  *  *  *             
timetable (UTC times) sun                            *  *  *  *  *  *  *  *  *  *  *             
* means mowing allowed

current GPS UTC weektime: dayOfWeek(0=Monday)=6  hour=19  min=25
mowing allowed (timetable evaluated): 1

*/

#define NOT_SET -1

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


// day mask (if mowing allowed, mask is set for that day) 
// 1=mon, 2=tue, 4=wed, 8=thu, 16=fri, 32=sat, 64=sun
typedef byte daymask_t; 

// timetable
typedef struct timetable_t {
    bool enable;    // use timetable?
    daymask_t hours[24];
} timetable_t;



class TimeTable
{
  public:
    timetable_t timetable;
    weektime_t autostartTime;   // next autostart time
    weektime_t autostopTime;    // next autostop time
    weektime_t currentTime;  // current time to compare time table against
    bool mowingCompletedInCurrentTimeFrame;  // has mowing completed in current time frame?
    TimeTable(); 

    bool shouldAutostartNow();  // robot should autostart now?
    bool shouldAutostopNow();  // robot should autostop now?


    // dump timetable
    void dump();    
    void dumpWeekTime(weektime_t time);

    int crc();

    // clear timetable
    void clear();

    // set current UTC time
    void setCurrentTime(int hour, int min, int weekOfDay); 

    // enable/disable timetable
    void setEnabled(bool flag);

    void setMowingCompletedInCurrentTimeFrame(bool completed);

    // set day mask for hour 
    bool setDayMask(int hour, daymask_t mask);


    // ------ misc functions -----------------------------------    
    bool findAutostartTime(weektime_t &time);    
    bool findAutostopTime(weektime_t &time);    

    // mowing allowed for given UTC week time?
    bool mowingAllowed(weektime_t time);  

    // calc dayOfWeek(0=Monday) for given UTC date  (untested and not used!)
    int calcDayOfWeek(int year, int month, int day); 

    // mowing allowed for current UTC week time?
    bool mowingAllowed();

    // run loop
    void run(); 

  protected:
      unsigned long nextCheckTime; 
      bool lastMowingAllowedState;
      bool autostartTriggered;  // remember transition triggers
      bool autostopTriggered;  // remember transition triggers      
      bool autostartNow;  
      bool autostopNow;   
};


#endif


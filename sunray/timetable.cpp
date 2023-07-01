#include "timetable.h"
#include "config.h"

TimeTable::TimeTable()
{
}


// set current UTC time
void TimeTable::setCurrentTime(int hour, int min, int dayOfWeek){
    currentTime.hour = hour;
    currentTime.min = min;
    currentTime.dayOfWeek = dayOfWeek;
    CONSOLE.print("GPS time (UTC): ");
    dumpWeekTime(currentTime);
}    

void TimeTable::dumpWeekTime(weektime_t time){
    CONSOLE.print("dayOfWeek(0=Monday)=");
    CONSOLE.print(time.dayOfWeek);
    CONSOLE.print("  hour=");
    CONSOLE.print(time.hour);
    CONSOLE.print("  min=");
    CONSOLE.println(time.min);
}


void TimeTable::dump(){
    for (int i=0; i  < TIME_FRAMES; i++){
        CONSOLE.print("timetable (UTC times)");
        CONSOLE.print("  idx=");
        CONSOLE.print(i);
        CONSOLE.print("  en=");
        CONSOLE.print(timeTable.frames[i].enabled);
        CONSOLE.print("  dayOfWeek(0=Monday)=");        
        CONSOLE.print(timeTable.frames[i].dayOfWeek);
        CONSOLE.print("  start=");
        CONSOLE.print(timeTable.frames[i].startTime.hour);
        CONSOLE.print(":");        
        CONSOLE.print(timeTable.frames[i].startTime.min);        
        CONSOLE.print("  end=");
        CONSOLE.print(timeTable.frames[i].endTime.hour);
        CONSOLE.print(":");        
        CONSOLE.println(timeTable.frames[i].endTime.min);       
    }
    CONSOLE.print("current GPS UTC weektime: ");
    dumpWeekTime(currentTime);
    CONSOLE.print("mowing allowed: ");
    CONSOLE.println(mowingAllowed());
}


void TimeTable::clear(){
    for (int i=0; i  < TIME_FRAMES; i++){
        timeTable.frames[i].enabled = false;
    }
}    

bool TimeTable::addMowingTimeFrame(timeframe_t timeframe){
    int idx = 0; 
    while (idx < TIME_FRAMES){
        if (!timeTable.frames[idx].enabled) break;
        idx++;
    }
    if (idx == TIME_FRAMES) {
        CONSOLE.println("TimeTable::addMowingTimeFrame error: timetable overflow");
        return false;
    }
    timeTable.frames[idx] = timeframe;
}
    

bool TimeTable::mowingAllowed(weektime_t time){
    bool mowingAllowed = false; 
    for (int i=0; i  < TIME_FRAMES; i++){
        if (timeTable.frames[i].enabled){
            if ( time.dayOfWeek == timeTable.frames[i].dayOfWeek ){   // dayOfWeek ok
                if ( (time.hour >= timeTable.frames[i].startTime.hour) && (time.hour <= timeTable.frames[i].endTime.hour) ){ // hours ok
                    if  (time.hour == timeTable.frames[i].startTime.hour) {  // starting hour, check minutes
                        if (time.min >= timeTable.frames[i].startTime.min) mowingAllowed = true; // minutes ok
                    }
                    else if  (time.hour == timeTable.frames[i].endTime.hour) { // ending hour, check minutes
                        if (time.min <= timeTable.frames[i].endTime.min) mowingAllowed = true; // minutes ok 
                    }
                    else mowingAllowed = true;                                
                }                  
            }
        }
    }
    return mowingAllowed;
}

 bool TimeTable::mowingAllowed(){
     return mowingAllowed(currentTime);
 }

// calc dayOfWeek(0=Monday) for given UTC date (untested and not used!)
// not needed as GPS signal gives us millis since start of a week
int TimeTable::calcDayOfWeek(int year, int month, int day){
    int a, b, c, d;
    long int e;
    a = day;
    b = month;
    c = year;

    for (d = 1, e = (365 * (c - 1)) + (a - 1) + ((c / 4) - (c / 100) + (c / 400)); d < b; d++)
        e += (d == 2) ? (28) : ((d == 4 || d == 6 || d == 9 || d == 11) ? (30) : (31));
    int dayOfWeek = (e % 7);
    return dayOfWeek;
}



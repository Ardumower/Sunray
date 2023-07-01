#include "timetable.h"
#include "config.h"

TimeTable::TimeTable()
{
}

// set current UTC time
void TimeTable::setCurrentTime(int year, int month, int day, int hour, int min){
    currentTime.year = year;
    currentTime.month = month;
    currentTime.day = day;
    currentTime.hour = hour;
    currentTime.min = min;
    currentTime.dayOfWeek = calcDayOfWeek(year, month, day); 
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
    CONSOLE.print("current GPS UTC datetime: year=");
    CONSOLE.print(currentTime.year);
    CONSOLE.print("  month=");
    CONSOLE.print(currentTime.month);
    CONSOLE.print("  day=");
    CONSOLE.print(currentTime.day);
    CONSOLE.print("  dayOfWeek(0=Monday)=");
    CONSOLE.print(currentTime.dayOfWeek);
    CONSOLE.print("  hour=");
    CONSOLE.print(currentTime.hour);
    CONSOLE.print("  min=");
    CONSOLE.println(currentTime.min);

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
    

bool TimeTable::mowingAllowed(int dayOfWeek, daytime_t time){
    bool mowingAllowed = false; 
    for (int i=0; i  < TIME_FRAMES; i++){
        if (timeTable.frames[i].enabled){
            if ( dayOfWeek == timeTable.frames[i].dayOfWeek ){   // dayOfWeek ok
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
     daytime_t ctime;
     ctime.hour = currentTime.hour;
     ctime.min = currentTime.min;
     return mowingAllowed(currentTime.dayOfWeek, ctime);
 }

// calc dayOfWeek(0=Monday) for given UTC date 
int TimeTable::calcDayOfWeek(int year, int month, int day){
    int a, b, c, d;
    long int e;
    a = day;
    b = month;
    c = year;

    // https://www.indiastudychannel.com/resources/169329-C-Program-to-get-the-Day-of-a-given-Date.aspx
    for (d = 1, e = (365 * (c - 1)) + (a - 1) + ((c / 4) - (c / 100) + (c / 400)); d < b; d++)
        e += (d == 2) ? (28) : ((d == 4 || d == 6 || d == 9 || d == 11) ? (30) : (31));
    int dayOfWeek = (e % 7);
    return dayOfWeek;
}



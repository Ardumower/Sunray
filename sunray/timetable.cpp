#include "timetable.h"
#include "config.h"

TimeTable::TimeTable()
{
}

void TimeTable::dump(){
    for (int i=0; i  < TIME_FRAMES; i++){
        CONSOLE.print("timetable (UTC times)");
        CONSOLE.print("  idx=");
        CONSOLE.print(i);
        CONSOLE.print("  en=");
        CONSOLE.print(timeTable.frames[i].enabled);
        CONSOLE.print("  dayOfWeek(0=Sunday)=");        
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
    

bool TimeTable::mowingAllowed(daytime_t time){
    bool mowingAllowed = false; 
    for (int i=0; i  < TIME_FRAMES; i++){
        if (timeTable.frames[i].enabled){
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
    return mowingAllowed;
}


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
    for (int hour=0; hour < 24; hour++){
        String s;
        if (hour < 10) s += "0";
        s += hour;
        CONSOLE.print(s);
        CONSOLE.print(" ");
    }
    CONSOLE.println();
    for (int day=0; day < 7; day++){
        CONSOLE.print("timetable (UTC times)");
        String s;
        switch (day){
            case 0: s = "mon"; break;
            case 1: s = "tue"; break;
            case 2: s = "wed"; break;
            case 3: s = "thu"; break;
            case 4: s = "fri"; break;
            case 5: s = "sat"; break;
            case 6: s = "sun"; break;         
        }
        CONSOLE.print(s);
        int mask = (1 << day);
        for (int hour=0; hour < 24; hour++){
            String s = "   ";
            if (timetable.hours[hour] & mask != 0) s = " * ";
            CONSOLE.print(s);
        }
        CONSOLE.println();
    }
    CONSOLE.print("current GPS UTC weektime: ");
    dumpWeekTime(currentTime);
    CONSOLE.print("mowing allowed: ");
    CONSOLE.println(mowingAllowed());
}


void TimeTable::clear(){    
    for (int i=0; i  < 24; i++){
        timetable.hours[i] = 0;
    }
}    

// set day mask for hour 
bool TimeTable::setDayMask(int hour, daymask_t mask){
    if ((hour < 0) || (hour > 23)) return false;
    timetable.hours[hour] = mask;
    return true;
}
    

bool TimeTable::mowingAllowed(weektime_t time){
    int hour = time.hour; 
    if ((hour < 0) || (hour > 23)) return false;    
    int mask = (1 << time.dayOfWeek);

    bool allowed = (timetable.hours[hour] & mask != 0);
    return allowed;
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



#include "timetable.h"
#include "config.h"
#include "robot.h"
#include "src/op/op.h"


TimeTable::TimeTable()
{
    autostartNow = false;
    autostopNow = false;
    mowingCompletedInCurrentTimeFrame = false;
    nextCheckTime = 0;
    lastMowingAllowedState = false;
    autostartTriggered = false;
    autostopTriggered = false;    
    autostopTime.dayOfWeek = NOT_SET;
    autostartTime.dayOfWeek = NOT_SET;    
    timetable.enable = false; 

    timetable.hours[0] = 0;
    timetable.hours[1] = 0;
    timetable.hours[2] = 0;
    timetable.hours[3] = 0;
    timetable.hours[4] = 0;
    timetable.hours[5] = 0;
    timetable.hours[6] = 0;
    timetable.hours[7] = 0;
    timetable.hours[8] = 0;
    timetable.hours[9] = 127; // if mowing allowed, mask is set for that day
    timetable.hours[10] = 127;
    timetable.hours[11] = 127;
    timetable.hours[12] = 127;
    timetable.hours[13] = 127;
    timetable.hours[14] = 127;
    timetable.hours[15] = 127;
    timetable.hours[16] = 127;
    timetable.hours[17] = 127;
    timetable.hours[18] = 127;
    timetable.hours[19] = 127;
    timetable.hours[20] = 0;
    timetable.hours[21] = 0;
    timetable.hours[22] = 0;
    timetable.hours[23] = 0;    
}

void TimeTable::setMowingCompletedInCurrentTimeFrame(bool completed){
    mowingCompletedInCurrentTimeFrame = completed;
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
    CONSOLE.print("dayOfWeek=");
    String s;
    switch (time.dayOfWeek){
        case 0: s = "mon"; break;
        case 1: s = "tue"; break;
        case 2: s = "wed"; break;
        case 3: s = "thu"; break;
        case 4: s = "fri"; break;
        case 5: s = "sat"; break;
        case 6: s = "sun"; break;         
    }    
    CONSOLE.print(s);
    CONSOLE.print("  hour=");
    CONSOLE.println(time.hour);
    //CONSOLE.print("  min=");
    //CONSOLE.println(time.min);
}


void TimeTable::dump(){
    CONSOLE.print("timetable (UTC times)    ");                
    for (int hour=0; hour < 24; hour++){
        String s;
        if (hour < 10) s += "0";
        s += hour;
        CONSOLE.print(s);
        CONSOLE.print(" ");
    }
    CONSOLE.println();
    for (int day=0; day < 7; day++){        
        CONSOLE.print("timetable (UTC times) ");
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
            if (timetable.hours[hour] & mask) s = " * ";
            CONSOLE.print(s);
        }
        CONSOLE.println();
    }
    CONSOLE.println("* means mowing allowed");
    CONSOLE.print("current GPS UTC weektime: ");
    dumpWeekTime(currentTime);
    CONSOLE.print("timetable enabled: ");
    CONSOLE.println(timetable.enable);
    CONSOLE.print("mowing allowed: ");    
    CONSOLE.println(mowingAllowed());
}


void TimeTable::clear(){    
    for (int i=0; i  < 24; i++){
        timetable.hours[i] = 0;
    }
}    

int TimeTable::crc(){
    int crc = 0;
    for (int i=0; i  < 24; i++){
        crc += i * timetable.hours[i];
    }
    crc += ((byte)timetable.enable);
    return crc;
}

// set day mask for hour 
bool TimeTable::setDayMask(int hour, daymask_t mask){
    if ((hour < 0) || (hour > 23)) return false;
    //CONSOLE.print("setDayMask hour=");
    //CONSOLE.print(hour);
    //CONSOLE.print("  mask=");
    //CONSOLE.println(mask);
    timetable.hours[hour] = mask;
    return true;
}

void TimeTable::setEnabled(bool flag){
    timetable.enable = flag;
}
    

bool TimeTable::mowingAllowed(weektime_t time){
    if (!timetable.enable) return true; // timetable not enabled => mowing allowed
    int hour = time.hour; 
    if ((hour < 0) || (hour > 23)) return false;    
    int mask = (1 << time.dayOfWeek);

    bool allowed = ( (timetable.hours[hour] & mask) != 0); // if mowing allowed, mask is set for that day
    return allowed;
}

bool TimeTable::mowingAllowed(){
    return mowingAllowed(currentTime);
}


bool TimeTable::findAutostopTime(weektime_t &time){
    time.dayOfWeek = NOT_SET;
    if (!timetable.enable) {
        CONSOLE.println("AUTOSTOP: timetable is disabled");
        return false;
    }
    bool autostop = false;
    bool triggered = autostopTriggered;
    weektime_t checktime = currentTime;
    bool checkstate = true;

    // check timetable
    for (int hour =0; hour < 24 * 7; hour++){
        bool allowed = mowingAllowed(checktime);
        if (allowed != checkstate){   
            if ( (!allowed ) && (!triggered) )   {         
                // timetable status transition                                
                CONSOLE.print("AUTOSTOP: timetable transition ");                                    
                time = checktime;
                dumpWeekTime(time);
                autostop = true;
                break;
            } 
            triggered = false;
            checkstate = allowed;
        }
        // continue to next hour in timetable
        checktime.hour++;
        if (checktime.hour > 23){
            checktime.dayOfWeek++;
            if (checktime.dayOfWeek > 6){
                checktime.dayOfWeek = 0;
            }
            checktime.hour=0;
        }
    }
    if (!autostop) CONSOLE.println("AUTOSTOP: no time found");
    return autostop;
}


bool TimeTable::findAutostartTime(weektime_t &time){    
    time.dayOfWeek = NOT_SET;
    if ( !DOCKING_STATION ){
        CONSOLE.println("AUTOSTART: not defined DOCKING_STATION");
        return false; 
    }
    if (!DOCK_AUTO_START) {// automatic continue mowing allowed?
        CONSOLE.println("AUTOSTART: not defined DOCK_AUTO_START");
        return false;     
    }
    if ( !battery.isDocked() ) { // robot is in dock?
        CONSOLE.println("AUTOSTART: not docked automatically (use DOCK command first)");
        return false;   
    }
    bool autostart = false;    
    weektime_t checktime = currentTime;
    bool checkstate = false;
    bool triggered = autostartTriggered;    
    unsigned long waitmillis = millis(); 
    
    // check timetable and rain timeouts    
    for (int hour =0; hour < 24 * 7; hour++){
        if ( (!dockOp.dockReasonRainTriggered) || (waitmillis > dockOp.dockReasonRainAutoStartTime) ) {  // raining timeout 
            if (!timetable.enable){  // timetable disabled
                if ((!dockOp.initiatedByOperator) && (maps.mowPointsIdx > 0)) { // mowing not completed yet                    
                    CONSOLE.print("AUTOSTART: mowing not completed yet ");
                    time = checktime;
                    dumpWeekTime(time);
                    autostart = true;
                    break;
                }
            } else {   // timetable enabled
                bool allowed = mowingAllowed(checktime);
                if (allowed != checkstate){   
                    if ((allowed) && (!triggered))    {         
                        // timetable status transition
                        CONSOLE.print("AUTOSTART: timetable transition ");                    
                        time = checktime;
                        dumpWeekTime(time);
                        autostart = true;
                        break;
                    }
                    triggered = false;
                    checkstate = allowed;
                }
                if (allowed){   // timetable status
                    if ( (!dockOp.initiatedByOperator) && ((!mowingCompletedInCurrentTimeFrame) || (maps.mowPointsIdx > 0)) )  {
                        CONSOLE.print("AUTOSTART: timetable state ");                    
                        time =checktime;
                        dumpWeekTime(time);
                        autostart = true;
                        break;
                    }
                }
            }
        }
        // continue to next hour in timetable
        checktime.hour++;
        waitmillis += 1000 * 60 * 60; // 1 hour
        if (checktime.hour > 23){
            checktime.dayOfWeek++;
            if (checktime.dayOfWeek > 6){
                checktime.dayOfWeek = 0;
            }
            checktime.hour=0;
        }
    }     
    if (!autostart) CONSOLE.println("AUTOSTART: no time found");
    return autostart;
}

// called from charge operation
bool TimeTable::shouldAutostartNow(){
    if (autostartNow){
        autostartTriggered = true; // remember trigger
        return true;
    }
    return false;
}


// called from mow operation
bool TimeTable::shouldAutostopNow(){
    if (autostopNow){        
        autostopTriggered = true;  // remember trigger
        return true;
    }
    return false;
}

// called every 30s in robot
void TimeTable::run(){    
    //if (millis() < nextCheckTime) return;
    //nextCheckTime = millis() + 30000;    

    // reset triggers on timetable changes
    bool allowed = mowingAllowed(currentTime);
    if (allowed != lastMowingAllowedState){
        lastMowingAllowedState = allowed;
        autostartTriggered = false; // reset trigger
        autostopTriggered = false; // reset trigger    
    }

    autostopNow = false;
    if (findAutostopTime(autostopTime)){ 
        if (autostopTime.dayOfWeek == currentTime.dayOfWeek){
            if (autostopTime.hour == currentTime.hour){                                
                autostopNow = true;
            }
        }
    }

    autostartNow = false;
    if (findAutostartTime(autostartTime)){
        if (autostartTime.dayOfWeek == currentTime.dayOfWeek){
            if (autostartTime.hour == currentTime.hour){
                autostartNow = true;
            }
        } 
    }
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



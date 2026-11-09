#include "SunsetDock.h"
#include <cassert>
#include <cstdio>
#include <limits>
int main(){
 using namespace SunsetDock;
 Date day{2026,11,9};const double lat=52.37695,lon=8.70523;
 auto t=calculate(day,lat,lon);assert(t.valid && t.sunrise<t.sunset);
 // MSC Falke in November: sunrise about 07:34 CET, sunset about 16:44 CET.
 assert(std::abs(t.sunrise-394)<3 && std::abs(t.sunset-944)<3);
 auto check=[&](Date date,int h,int m,int sec,int lead){return evaluate(date,h,m,sec,lat,lon,lead);};
 auto before=check(day,15,13,0,30),after=check(day,15,14,0,30);
 assert(before.valid && !before.night && after.night);
 assert(!check(day,15,14,0,0).night); // configurable advance
 assert(check(day,23,59,59,30).night);
 assert(check({2026,11,10},0,0,0,30).night); // midnight must not release hold
 assert(check({2026,11,10},6,30,0,30).night);
 assert(!check({2026,11,10},7,0,0,30).night);
 assert(!check(day,12,0,0,-1).valid && !check(day,12,0,0,181).valid);
 assert(!check({2026,2,29},12,0,0,30).valid);
 assert(calculate({2028,2,29},lat,lon).valid);
 assert(!calculate(day,std::numeric_limits<double>::quiet_NaN(),lon).valid);
 assert(!calculate(day,lat,200).valid);
 assert(!calculate({2026,6,21},89,0).valid); // polar day: no invented event
 assert(!calculate({2026,12,21},89,0).valid);
 auto summer=calculate({2026,6,21},lat,lon);assert(summer.sunset>1180 && summer.sunset<1200);
 // Date-line calculations select the local solar date and unwrapped UTC times.
 auto west=evaluate({2026,1,1},3,0,0,0,-179,30);
 assert(west.valid && west.date.year==2025 && west.date.month==12 && west.date.day==31 && !west.night);
 auto east=evaluate({2026,12,31},20,0,0,0,179,30);
 assert(east.valid && east.date.year==2027 && east.date.month==1 && east.date.day==1 && !east.night);
 printf("PASS solar dates, sunset advance, night hold, sunrise release, invalid input, leap year, polar cases, date line; MSC sunset UTC %.2f minutes\n",t.sunset);
}

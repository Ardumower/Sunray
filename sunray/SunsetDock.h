#pragma once
#include <cmath>

// NOAA fractional-year solar equations, UTC minutes, east-positive longitude.
// https://gml.noaa.gov/grad/solcalc/solareqns.PDF
// Planning approximation: flat horizon, solar zenith 90.833 degrees.
namespace SunsetDock {
// Night scheduling never overrides an operator-controlled mowing session.
inline bool shouldDock(bool night,bool mowing,bool manual){return night && mowing && !manual;}
struct Date {int year,month,day;};
struct Times {bool valid=false;double sunrise=0,sunset=0;};
struct Decision {bool valid=false,night=false;double sunrise=0,sunset=0,deadline=0,now=0;Date date{};};
inline bool leap(int y){return y%4==0 && (y%100!=0 || y%400==0);}
inline int days(int y,int m){const int n[]={31,28,31,30,31,30,31,31,30,31,30,31};return m==2?n[m-1]+leap(y):n[m-1];}
inline bool valid(Date d){return d.year>=2020 && d.year<=2099 && d.month>=1 && d.month<=12 && d.day>=1 && d.day<=days(d.year,d.month);}
inline Date next(Date d,int offset){
 d.day+=offset;
 if(d.day<1){if(--d.month<1){d.month=12;--d.year;}d.day=days(d.year,d.month);}
 if(d.day>days(d.year,d.month)){d.day=1;if(++d.month>12){d.month=1;++d.year;}}
 return d;
}
inline Times calculate(Date date,double lat,double lon){
 Times out;
 if(!valid(date)||!std::isfinite(lat)||!std::isfinite(lon)||lat<=-90||lat>=90||lon< -180||lon>180)return out;
 const double pi=3.14159265358979323846,r=pi/180;
 int day=date.day;for(int m=1;m<date.month;++m)day+=days(date.year,m);
 double events[2];
 for(int event=0;event<2;++event){
  double minute=720-4*lon+(event?360:-360);
  for(int iteration=0;iteration<3;++iteration){
   double g=2*pi/(leap(date.year)?366:365)*(day-1+(minute/60-12)/24);
   double eq=229.18*(.000075+.001868*std::cos(g)-.032077*std::sin(g)-.014615*std::cos(2*g)-.040849*std::sin(2*g));
   double decl=.006918-.399912*std::cos(g)+.070257*std::sin(g)-.006758*std::cos(2*g)+.000907*std::sin(2*g)-.002697*std::cos(3*g)+.00148*std::sin(3*g);
   double cosine=std::cos(90.833*r)/(std::cos(lat*r)*std::cos(decl))-std::tan(lat*r)*std::tan(decl);
   if(cosine< -1 || cosine>1)return out; // no daily rise/set at polar latitudes
   double angle=std::acos(cosine)/r;
   minute=720-4*lon-eq+(event?4*angle:-4*angle);
  }
  events[event]=minute;
 }
 out.valid=true;out.sunrise=events[0];out.sunset=events[1];return out;
}
inline Decision evaluate(Date utc,int hour,int minute,int second,double lat,double lon,int lead){
 Decision result;
 if(!valid(utc)||hour<0||hour>23||minute<0||minute>59||second<0||second>60||lead<0||lead>180||!std::isfinite(lon)||lon< -180||lon>180)return result;
 double now=hour*60+minute+second/60.0;
 // Select the local solar date, including UTC date-line crossings.
 int shift=int(std::floor((now+4*lon)/1440));
 result.date=next(utc,shift);result.now=now-shift*1440;
 auto times=calculate(result.date,lat,lon);if(!times.valid)return result;
 result.valid=true;result.sunrise=times.sunrise;result.sunset=times.sunset;result.deadline=times.sunset-lead;
 result.night=result.now<result.sunrise || result.now>=result.deadline;
 return result;
}
}

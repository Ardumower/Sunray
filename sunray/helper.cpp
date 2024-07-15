// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)


#include <Arduino.h>
#include "helper.h"
#include "config.h"


// Spannungsteiler Gesamtspannung ermitteln (Reihenschaltung R1-R2, U2 bekannt, U_GES zu ermitteln)
float voltageDividerUges(float R1, float R2, float U2){
	return (U2/R2 * (R1+R2));  // Uges 
}


// ADC-value to voltage
float ADC2voltage(float ADCvalue){
  //return (ADCvalue /1023.0 * IOREF);   // ADC must be configured @ 10 bit (default)
  return (ADCvalue /4095.0 * IOREF);   // ADC must be configured @ 12 bit
}  


// rescale to -PI..+PI
float scalePI(float v)
{
  float d = v;
  while (d < 0) d+=2*PI;
  while (d >= 2*PI) d-=2*PI;
  if (d >= PI) return (-2*PI+d);
  else if (d < -PI) return (2*PI+d);
  else return d;
}

// rescale to -180..+180
float scale180(float v)
{
  float d = v;
  while (d < 0) d+=2*180;
  while (d >= 2*180) d-=2*180;
  if (d >= 180) return (-2*180+d);
  else if (d < -180) return (2*180+d);
  else return d;
}


// computes minimum distance between x radiant (current-value) and w radiant (set-value)
float distancePI(float x, float w)
{
  // cases:
  // w=330 degree, x=350 degree => -20 degree
  // w=350 degree, x=10  degree => -20 degree
  // w=10  degree, x=350 degree =>  20 degree
  // w=0   degree, x=190 degree => 170 degree
  // w=190 degree, x=0   degree => -170 degree
  float d = scalePI(w - x);
  if (d < -PI) d = d + 2*PI;
  else if (d > PI) d = d - 2*PI;
  return d;
}

float distance180(float x, float w)
{
  float d = scale180(w - x);
  if (d < -180) d = d + 2*180;
  else if (d > 180) d = d - 2*180;
  return d;
}

// compute distance to (infinite) line (https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Line_defined_by_two_points)
float distanceLineInfinite(float px, float py, float x1, float y1, float x2, float y2)
{  
  float len = sqrt( sq(y2-y1)+sq(x2-x1) );
  if (abs(len) < 0.01) {
    //CONSOLE.println("distanceLineInfinite: len is zero");
    return 0;
  }
  float distToLine = ((y2-y1)*px-(x2-x1)*py+(x2*y1-y2*x1)) / len;
  if (isnan(distToLine)){
    CONSOLE.print("distanceLineInfinite nan error - px=");
    CONSOLE.print(px);
    CONSOLE.print(" py=");
    CONSOLE.print(py);
    CONSOLE.print(" x1=");    
    CONSOLE.print(x1);
    CONSOLE.print(" x2=");    
    CONSOLE.println(x2);
    return 0;
  }
  return distToLine;
}


// compute distance from point to line (v, w)    
float distanceLine(float px, float py, float x1, float y1, float x2, float y2) {         
    float x3 = px;
    float y3 = py;
    float lx=x2-x1;
    float ly=y2-y1;
    float temp=(lx*lx)+(ly*ly);
    if (abs(temp) < 0.01) {
      //CONSOLE.println("distanceLine: len is zero");
      return 0;
    }
    float u=((x3 - x1) * lx + (y3 - y1) * ly) / (temp);
    if(u>1){
        u=1;
    }
    else if(u<0){
        u=0;
    }
    float x = x1 + u * lx;
    float y = y1 + u * ly;
    float dx = x - x3;
    float dy = y - y3;
    float dist = sqrt(dx*dx + dy*dy);
    if (isnan(dist)){
      CONSOLE.print("distanceLine nan error - px=");
      CONSOLE.print(px);
      CONSOLE.print(" py=");
      CONSOLE.print(py);
      CONSOLE.print(" x1=");    
      CONSOLE.print(x1);
      CONSOLE.print(" x2=");    
      CONSOLE.print(x2);
      CONSOLE.print(" temp=");      
      CONSOLE.println(temp);
      return 0;
    }
    return dist;   
  }      

// weight fusion (w=0..1) of two radiant values (a,b)
float fusionPI(float w, float a, float b)
{
  float c;
  if ((b >= PI/2) && (a <= -PI/2)){
    c = w * a + (1.0-w) * (b-2*PI);
  } else if ((b <= -PI/2) && (a >= PI/2)){
    c = w * (a-2*PI) + (1.0-w) * b;
  } else c = w * a + (1.0-w) * b;
  return scalePI(c);
}


// scale setangle, so that both PI angles have the same sign
float scalePIangles(float setAngle, float currAngle){
  if ((setAngle >= PI/2) && (currAngle <= -PI/2)) return (setAngle-2*PI);
    else if ((setAngle <= -PI/2) && (currAngle >= PI/2)) return (setAngle+2*PI);
    else return setAngle;
}

float distance(float x1, float y1, float x2, float y2){
  return sqrtf( (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) );
}

int sign(double x) { 
  return x<0 ? -1 : 1; 
}

double deg2rad(double deg) {
  return (deg * PI / 180);
}

double rad2deg(double rad) {
  return (rad * 180 / PI);
}


// compute course (angle in rad) between two points
float pointsAngle(float x1, float y1, float x2, float y2){
  float dX = x2 - x1;
  float dY = y2 - y1;
  float angle = scalePI(atan2(dY, dX));

  return angle;
}


double distanceLL(double lat1, double lon1, double lat2, double lon2) 
{ 
  // http://www.movable-type.co.uk/scripts/latlong.html
  // https://forum.sparkfun.com/viewtopic.php?f=17&t=22520
  // http://boulter.com/gps/distance/?from=51.71577+8.74353&to=51.71578+8.74355&units=k#more  
  // returns distance in meters between two positions, both specified 
  // as signed decimal-degrees latitude and longitude. Uses great-circle 
  // distance computation for hypothetical sphere of radius 6372795 meters.
  // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
  // Courtesy of Maarten Lamers
  double delta = radians(lon1-lon2);
  double sdlong = sin(delta);
  double cdlong = cos(delta);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  double slat1 = sin(lat1);
  double clat1 = cos(lat1);
  double slat2 = sin(lat2);
  double clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong); 
  delta = sq(delta); 
  delta += sq(clat2 * sdlong); 
  delta = sqrt(delta); 
  double denom = (slat1 * slat2) + (clat1 * clat2 * cdlong); 
  delta = atan2(delta, denom); 
  return delta * 6372795.0f; 
}

void relativeLL(double lat1, double lon1, double lat2, double lon2, float &n, float &e){  
  // compute relative north/east coordinates (n,e) in meters between (lat,lon1) and (lat2,lon2)  
  int slat = sign(lat2-lat1);
  int slon = sign(lon2-lon1);
  n = slat * distanceLL(lat1, lon1, lat2, lon1);
  e = slon * distanceLL(lat1, lon1, lat1, lon2);
  if (false){
    CONSOLE.print("relativeLL lat1=");
    CONSOLE.print(lat1, 8);
    CONSOLE.print("  lat2=");
    CONSOLE.print(lat2, 8);
    CONSOLE.print("  n=");
    CONSOLE.println(n, 8);
    CONSOLE.print("relativeLL lon1=");
    CONSOLE.print(lon1, 8);
    CONSOLE.print("  lon2=");
    CONSOLE.print(lon2, 8);
    CONSOLE.print("  e=");
    CONSOLE.println(e, 8);
  }
}

void printFloat(float v){
  binaryLongOrFloat b;
  b.floatingPoint = v;  
  CONSOLE.write(b.binary[3]);  
  CONSOLE.write(b.binary[2]);  
  CONSOLE.write(b.binary[1]);  
  CONSOLE.write(b.binary[0]);  
}

void printLong(unsigned long v){
  binaryLongOrFloat b;
  b.ulong = v; 
  CONSOLE.write(b.binary[3]);  
  CONSOLE.write(b.binary[2]);  
  CONSOLE.write(b.binary[1]);  
  CONSOLE.write(b.binary[0]);  
}

void printInt(unsigned int v){
  binaryInt b;
  b.uint = v; 
  CONSOLE.write(b.binary[1]);  
  CONSOLE.write(b.binary[0]);    
}

uint32_t serialToLong(HardwareSerial* serial){
  binaryLongOrFloat b;
  b.binary[3] = serial->read();
  b.binary[2] = serial->read();
  b.binary[1] = serial->read();
  b.binary[0] = serial->read();
  return b.ulong;
}

float serialToFloat(HardwareSerial* serial){
  binaryLongOrFloat b;
  b.binary[3] = serial->read();
  b.binary[2] = serial->read();
  b.binary[1] = serial->read();
  b.binary[0] = serial->read();
  return b.floatingPoint;
}

int freeRam () {
#ifdef __AVR__
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
#else
  return 0;
#endif
}



/*
 * Returns random number in normal distribution centering on 0.
 * ~95% of numbers returned should fall between -2 and 2
 */
float gaussRandom() {
  return 2*((float)rand())/((float)RAND_MAX)-1;
  
  /*static unsigned int rnd = 0;
  rnd += micros(); // seeded with changing number
  rnd ^= rnd << 2; rnd ^= rnd >> 7; rnd ^= rnd << 7;
  return ( 2 * (((float)rnd) / 65535.0) - 1.0 );*/
    

    //printf("random=%3.3f\n", random());
    /*float u = 2*random()-1;
    float v = 2*random()-1;
    float r = u*u + v*v;
    // if outside interval [0,1] start over
    if ((r == 0) || (r > 1)) return gaussRandom();

    float c = sqrt(-2*log(r)/r);
    return u*c;*/

    /* todo: optimize this algorithm by caching (v*c)
     * and returning next time gaussRandom() is called.
     * left out for simplicity */
}

/*
 * Returns member of set with a given mean and standard deviation
 * mean: mean
 * standard deviation: std_dev
 */
float gauss(float mean, float std_dev){
  return mean + (gaussRandom()*std_dev);
}

// calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
float gaussian(float mu, float sigma, float x)
{
  return exp(-   pow(mu - x, 2) / pow(sigma,  2) / 2.0     )
              / sqrt(2.0 * M_PI * pow(sigma, 2));
}


float parseFloatValue(String s, String key){
  int idx = s.indexOf(key+"=");
  if (idx == -1) return 0;
  idx+=key.length()+1;
  String n;
  while ( (s[idx]=='-') || (isdigit(s.charAt(idx))) || (s[idx]=='.') ) {
    n+= s[idx]; idx++;
  }
  return n.toFloat();
}


void toEulerianAngle(float w, float x, float y, float z, float& roll, float& pitch, float& yaw)
{
  double ysqr = y * y;

  // roll (x-axis rotation)
  double t0 = +2.0 * (w * x + y * z);
  double t1 = +1.0 - 2.0 * (x * x + ysqr);
  roll = atan2(t0, t1);

  // pitch (y-axis rotation)
  double t2 = +2.0 * (w * y - z * x);
  t2 = t2 > 1.0 ? 1.0 : t2;
  t2 = t2 < -1.0 ? -1.0 : t2;
  pitch = asin(t2);

  // yaw (z-axis rotation)
  double t3 = +2.0 * (w * z + x * y);
  double t4 = +1.0 - 2.0 * (ysqr + z * z);  
  yaw = atan2(t3, t4);
}


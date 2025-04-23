#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H


#include <Arduino.h>


class LowPassFilter
{
public:
    LowPassFilter(float Tf = 0.001);
    ~LowPassFilter() = default;

    float operator() (float x);
    float Tf; // low pass filter time constant
    void reset();

protected:
    unsigned long timestamp_prev;  // last execution timestamp
    float y_prev; // filtered value in previous execution step 
};

#endif 

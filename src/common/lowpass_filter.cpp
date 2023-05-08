#include "lowpass_filter.h"

LowPassFilter::LowPassFilter(float time_constant)
    : tf(time_constant)
    , y_prev(0.0f)
{
    timestamp_prev = time_us_64();
}


float LowPassFilter::operator() (float x)
{
    unsigned long timestamp = time_us_64();
    float dt = (timestamp - timestamp_prev)*1e-6f;

    if (dt < 0.0f ) dt = 1e-3f;
    else if(dt > 0.3f) {
        y_prev = x;
        timestamp_prev = timestamp;
        return x;
    }

    float alpha = tf/(tf + dt);
    float y = alpha*y_prev + (1.0f - alpha)*x;
    y_prev = y;
    timestamp_prev = timestamp;
    return y;
}

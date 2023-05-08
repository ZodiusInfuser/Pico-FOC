#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H


#include "foc_utils.h"

/**
 * Low pass filter class
 */
class LowPassFilter {
  public:
    /**
     * @param tf - Low pass filter time constant
     */
    LowPassFilter(float tf);
    ~LowPassFilter() = default;

    float operator() (float x);
    float tf; //!< Low pass filter time constant

  protected:
    unsigned long timestamp_prev; //!< Last execution timestamp
    float y_prev; //!< filtered value in previous execution step
};
#endif

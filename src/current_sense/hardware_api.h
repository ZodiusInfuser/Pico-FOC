#ifndef HARDWARE_UTILS_CURRENT_H
#define HARDWARE_UTILS_CURRENT_H

#include "../common/foc_utils.h"

// flag returned if current sense init fails
#define SIMPLEFOC_CURRENT_SENSE_INIT_FAILED ((void*)-1)

// generic implementation of the hardware specific structure
// containing all the necessary current sense parameters
// will be returned as a void pointer from the _configureADCx functions
// will be provided to the _readADCVoltageX() as a void pointer
typedef struct GenericCurrentSenseParams {
  int pins[3];
  float adc_voltage_conv;
} GenericCurrentSenseParams;


/**
 * function reading an ADC value and returning the read voltage
 *
 * @param pin_a - the arduino pin to be read (it has to be ADC pin)
 * @param cs_params -current sense parameter structure - hardware specific
 */
float _readADCVoltageInline(const int pin_a, const void* cs_params);

/**
 * function reading an ADC value and returning the read voltage
 *
 * @param driver_params - driver parameter structure - hardware specific
 * @param pin_a - adc pin A
 * @param pin_b - adc pin B
 * @param pin_c - adc pin C
 */
void* _configureADCInline(const void *driver_params, const int pin_a,const int pin_b,const int pin_c = NOT_SET);

/**
 * function reading an ADC value and returning the read voltage
 *
 * @param driver_params - driver parameter structure - hardware specific
 * @param pin_a - adc pin A
 * @param pin_b - adc pin B
 * @param pin_c - adc pin C
 */
void* _configureADCLowSide(const void *driver_params, const int pin_a,const int pin_b,const int pin_c = NOT_SET);

void _startADC3PinConversionLowSide();

/**
 * function reading an ADC value and returning the read voltage
 *
 * @param pin_a - the arduino pin to be read (it has to be ADC pin)
 * @param cs_params -current sense parameter structure - hardware specific
 */
float _readADCVoltageLowSide(const int pin_a, const void* cs_params);

/**
 * function syncing the Driver with the ADC  for the LowSide Sensing
 * @param driver_params - driver parameter structure - hardware specific
 * @param cs_params - current sense parameter structure - hardware specific
 */
void _driverSyncLowSide(void* driver_params, void* cs_params);

#endif

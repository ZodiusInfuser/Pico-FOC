#include "../hardware_api.h"

// function reading an ADC value and returning the read voltage
__attribute__((weak))  float _readADCVoltageInline(const int pin_a, const void* cs_params){
  uint32_t raw_adc = analogRead(pin_a);
  return raw_adc * ((GenericCurrentSenseParams*)cs_params)->adc_voltage_conv;
}

// function reading an ADC value and returning the read voltage
__attribute__((weak))  void* _configureADCInline(const void* driver_params, const int pin_a,const int pin_b,const int pin_c){
  _UNUSED(driver_params);

  if( _isset(pin_a) ) pinMode(pin_a, INPUT);
  if( _isset(pin_b) ) pinMode(pin_b, INPUT);
  if( _isset(pin_c) ) pinMode(pin_c, INPUT);

  GenericCurrentSenseParams* params = new GenericCurrentSenseParams {
    .pins = { pin_a, pin_b, pin_c },
    .adc_voltage_conv = (5.0f)/(1024.0f)
  };

  return params;
}

// function reading an ADC value and returning the read voltage
__attribute__((weak))  float _readADCVoltageLowSide(const int pin_a, const void* cs_params){
  return _readADCVoltageInline(pin_a, cs_params);
}

// Configure low side for generic mcu
// cannot do much but
__attribute__((weak))  void* _configureADCLowSide(const void* driver_params, const int pin_a,const int pin_b,const int pin_c){
  return _configureADCInline(driver_params, pin_a, pin_b, pin_c);
}

// sync driver and the adc
__attribute__((weak)) void _driverSyncLowSide(void* driver_params, void* cs_params){
  _UNUSED(driver_params);
  _UNUSED(cs_params);
}
__attribute__((weak)) void _startADC3PinConversionLowSide(){ }

#include "../hardware_api.h"

// if the mcu doen't have defiend analogWrite
#if defined(ESP_H) && defined(ARDUINO_ARCH_ESP32)
  __attribute__((weak)) void analogWrite(uint8_t pin, int value){ };
#endif

void analogWrite(int pin, int value) {
  
}

// function setting the high pwm frequency to the supplied pin
// - Stepper motor - 1PWM setting
// - hardware speciffic
// in generic case dont do anything
__attribute__((weak)) void* _configure_1_pwm(long pwm_frequency, const int pin_a) {
  GenericDriverParams* params = new GenericDriverParams {
    .pins = { pin_a },
    .pwm_frequency = pwm_frequency
  };
  return params;
}

// function setting the high pwm frequency to the supplied pins
// - Stepper motor - 2PWM setting
// - hardware speciffic
// in generic case dont do anything
__attribute__((weak)) void* _configure_2_pwm(long pwm_frequency,const int pin_a, const int pin_b) {
  GenericDriverParams* params = new GenericDriverParams {
    .pins = { pin_a, pin_b },
    .pwm_frequency = pwm_frequency
  };
  return params;
}

// function setting the high pwm frequency to the supplied pins
// - BLDC motor - 3PWM setting
// - hardware speciffic
// in generic case dont do anything
__attribute__((weak)) void* _configure_3_pwm(long pwm_frequency,const int pin_a, const int pin_b, const int pin_c) {
  GenericDriverParams* params = new GenericDriverParams {
    .pins = { pin_a, pin_b, pin_c },
    .pwm_frequency = pwm_frequency
  };
  return params;
}


// function setting the high pwm frequency to the supplied pins
// - Stepper motor - 4PWM setting
// - hardware speciffic
// in generic case dont do anything
__attribute__((weak)) void* _configure_4_pwm(long pwm_frequency, const int pin1A, const int pin1B, const int pin2A, const int pin2B) {
  GenericDriverParams* params = new GenericDriverParams {
    .pins = { pin1A, pin1B, pin2A, pin2B },
    .pwm_frequency = pwm_frequency
  };
  return params;
}

// Configuring PWM frequency, resolution and alignment
// - BLDC driver - 6PWM setting
// - hardware specific
__attribute__((weak)) void* _configure_6_pwm(long pwm_frequency, float dead_zone, const int pinA_h, const int pinA_l,  const int pinB_h, const int pinB_l, const int pinC_h, const int pinC_l){
  _UNUSED(pwm_frequency);
  _UNUSED(dead_zone);
  _UNUSED(pinA_h);
  _UNUSED(pinA_l);
  _UNUSED(pinB_h);
  _UNUSED(pinB_l);
  _UNUSED(pinC_h);
  _UNUSED(pinC_l);

  return SIMPLEFOC_DRIVER_INIT_FAILED;
}


// function setting the pwm duty cycle to the hardware
// - Stepper motor - 1PWM setting
// - hardware speciffic
__attribute__((weak)) void _write_duty_cycle_1_pwm(float dc_a, void* params){
  // transform duty cycle from [0,1] to [0,255]
  analogWrite(((GenericDriverParams*)params)->pins[0], 255.0f*dc_a);
}


// function setting the pwm duty cycle to the hardware
// - Stepper motor - 2PWM setting
// - hardware speciffic
__attribute__((weak)) void _write_duty_cycle_2_pwm(float dc_a,  float dc_b, void* params){
  // transform duty cycle from [0,1] to [0,255]
  analogWrite(((GenericDriverParams*)params)->pins[0], 255.0f*dc_a);
  analogWrite(((GenericDriverParams*)params)->pins[1], 255.0f*dc_b);
}

// function setting the pwm duty cycle to the hardware
// - BLDC motor - 3PWM setting
// - hardware speciffic
__attribute__((weak)) void _write_duty_cycle_3_pwm(float dc_a,  float dc_b, float dc_c, void* params){
  // transform duty cycle from [0,1] to [0,255]
  analogWrite(((GenericDriverParams*)params)->pins[0], 255.0f*dc_a);
  analogWrite(((GenericDriverParams*)params)->pins[1], 255.0f*dc_b);
  analogWrite(((GenericDriverParams*)params)->pins[2], 255.0f*dc_c);
}

// function setting the pwm duty cycle to the hardware
// - Stepper motor - 4PWM setting
// - hardware speciffic
__attribute__((weak)) void _write_duty_cycle_4_pwm(float dc_1a,  float dc_1b, float dc_2a, float dc_2b, void* params){
  // transform duty cycle from [0,1] to [0,255]
  
  analogWrite(((GenericDriverParams*)params)->pins[0], 255.0f*dc_1a);
  analogWrite(((GenericDriverParams*)params)->pins[1], 255.0f*dc_1b);
  analogWrite(((GenericDriverParams*)params)->pins[2], 255.0f*dc_2a);
  analogWrite(((GenericDriverParams*)params)->pins[3], 255.0f*dc_2b);
}


// Function setting the duty cycle to the pwm pin (ex. analogWrite())
// - BLDC driver - 6PWM setting
// - hardware specific
__attribute__((weak)) void _write_duty_cycle_6_pwm(float dc_a,  float dc_b, float dc_c, PhaseState *phase_state, void* params){
  _UNUSED(dc_a);
  _UNUSED(dc_b);
  _UNUSED(dc_c);
  _UNUSED(phase_state);
  _UNUSED(params);
}
#include "StepperDriver4PWM.h"
#include "math.h"
#include "stdio.h"

StepperDriver4PWM::StepperDriver4PWM(int ph1A,int ph1B,int ph2A,int ph2B,int en1, int en2){
  // Pin initialization
  pwm1A = ph1A;
  pwm1B = ph1B;
  pwm2A = ph2A;
  pwm2B = ph2B;

  // enable_pin pin
  enable_pin1 = en1;
  enable_pin2 = en2;

  // default power-supply value
  voltage_power_supply = DEF_POWER_SUPPLY;
  voltage_limit = NOT_SET;
  pwm_frequency = NOT_SET;

}

// enable motor driver
void  StepperDriver4PWM::enable(){
    // enable_pin the driver - if enable_pin pin available
    if ( _isset(enable_pin1) ) gpio_put(enable_pin1, true);
    if ( _isset(enable_pin2) ) gpio_put(enable_pin2, true);
    // set zero to PWM
    set_pwm(0, 0);
}

// disable motor driver
void StepperDriver4PWM::disable()
{
  // set zero to PWM
  set_pwm(0, 0);
  // disable the driver - if enable_pin pin available
  if ( _isset(enable_pin1) ) gpio_put(enable_pin1, false);
  if ( _isset(enable_pin2) ) gpio_put(enable_pin2, false);

}

// init hardware pins
int StepperDriver4PWM::init() {

  // PWM pins
  gpio_set_function(pwm1A, GPIO_FUNC_PWM);
  gpio_set_function(pwm1B, GPIO_FUNC_PWM);
  gpio_set_function(pwm2A, GPIO_FUNC_PWM);
  gpio_set_function(pwm2B, GPIO_FUNC_PWM);
  if( _isset(enable_pin1) ) gpio_set_function(enable_pin1, GPIO_FUNC_SIO);
  if( _isset(enable_pin2) ) gpio_set_function(enable_pin2, GPIO_FUNC_SIO);

  // sanity check for the voltage limit configuration
  if( !_isset(voltage_limit) || voltage_limit > voltage_power_supply) voltage_limit =  voltage_power_supply;

  // Set the pwm frequency to the pins
  // hardware specific function - depending on driver and mcu
  params = _configure_4_pwm(pwm_frequency, pwm1A, pwm1B, pwm2A, pwm2B);
  initialized = (params!=SIMPLEFOC_DRIVER_INIT_FAILED);
  return params!=SIMPLEFOC_DRIVER_INIT_FAILED;
}


// Set voltage to the pwm pin
void StepperDriver4PWM::set_pwm(float Ualpha, float Ubeta) {
  float duty_cycle1A(0.0f),duty_cycle1B(0.0f),duty_cycle2A(0.0f),duty_cycle2B(0.0f);
  // limit the voltage in driver
  Ualpha = _constrain(Ualpha, -voltage_limit, voltage_limit);
  Ubeta = _constrain(Ubeta, -voltage_limit, voltage_limit);
  // hardware specific writing
  if( Ualpha > 0 )
    duty_cycle1B = _constrain(abs(Ualpha)/voltage_power_supply,0.0f,1.0f);
  else
    duty_cycle1A = _constrain(abs(Ualpha)/voltage_power_supply,0.0f,1.0f);

  if( Ubeta > 0 )
    duty_cycle2B = _constrain(abs(Ubeta)/voltage_power_supply,0.0f,1.0f);
  else
    duty_cycle2A = _constrain(abs(Ubeta)/voltage_power_supply,0.0f,1.0f);
  printf("c1A = %f, c1B = %f, c2A = %f, c2B = %f\n", duty_cycle1A, duty_cycle1B, duty_cycle2A, duty_cycle2B);
  // write to hardware
  _write_duty_cycle_4_pwm(duty_cycle1A, duty_cycle1B, duty_cycle2A, duty_cycle2B, params);
}
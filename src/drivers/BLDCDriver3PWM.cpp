#include "BLDCDriver3PWM.h"

BLDCDriver3PWM::BLDCDriver3PWM(int phA, int phB, int phC, int en1, int en2, int en3){
  // Pin initialization
  pwmA = phA;
  pwmB = phB;
  pwmC = phC;

  // enable_pin pin
  enableA_pin = en1;
  enableB_pin = en2;
  enableC_pin = en3;

  // default power-supply value
  voltage_power_supply = DEF_POWER_SUPPLY;
  voltage_limit = NOT_SET;
  pwm_frequency = NOT_SET;

}

// enable motor driver
void  BLDCDriver3PWM::enable(){
    // enable_pin the driver - if enable_pin pin available
    if ( _isset(enableA_pin) ) gpio_put(enableA_pin, enable_active_high);
    if ( _isset(enableB_pin) ) gpio_put(enableB_pin, enable_active_high);
    if ( _isset(enableC_pin) ) gpio_put(enableC_pin, enable_active_high);
    // set zero to PWM
    set_pwm(0,0,0);
}

// disable motor driver
void BLDCDriver3PWM::disable()
{
  // set zero to PWM
  set_pwm(0, 0, 0);
  // disable the driver - if enable_pin pin available
  if ( _isset(enableA_pin) ) gpio_put(enableA_pin, !enable_active_high);
  if ( _isset(enableB_pin) ) gpio_put(enableB_pin, !enable_active_high);
  if ( _isset(enableC_pin) ) gpio_put(enableC_pin, !enable_active_high);

}

// init hardware pins
int BLDCDriver3PWM::init() {
  // PWM pins
  gpio_init(pwmA); gpio_set_function(pwmA, GPIO_FUNC_PWM);
  gpio_init(pwmB); gpio_set_function(pwmB, GPIO_FUNC_PWM);
  gpio_init(pwmC); gpio_set_function(pwmC, GPIO_FUNC_PWM);
  if( _isset(enableA_pin)) {
    gpio_init(enableA_pin); gpio_set_function(enableA_pin, GPIO_FUNC_SIO); gpio_set_dir(enableA_pin, true);
  }
  if( _isset(enableB_pin)) {
    gpio_init(enableB_pin); gpio_set_function(enableB_pin, GPIO_FUNC_SIO); gpio_set_dir(enableB_pin, true);
  }
  if( _isset(enableC_pin)) {
    gpio_init(enableC_pin); gpio_set_function(enableC_pin, GPIO_FUNC_SIO); gpio_set_dir(enableC_pin, true);
  }


  // sanity check for the voltage limit configuration
  if(!_isset(voltage_limit) || voltage_limit > voltage_power_supply) voltage_limit =  voltage_power_supply;

  // Set the pwm frequency to the pins
  // hardware specific function - depending on driver and mcu
  params = _configure_3_pwm(pwm_frequency, pwmA, pwmB, pwmC);
  initialized = (params!=SIMPLEFOC_DRIVER_INIT_FAILED);
  return params!=SIMPLEFOC_DRIVER_INIT_FAILED;
}

// Set voltage to the pwm pin
void BLDCDriver3PWM::set_phase_state(PhaseState sa, PhaseState sb, PhaseState sc) {
  // disable if needed
  if( _isset(enableA_pin) &&  _isset(enableB_pin)  && _isset(enableC_pin) ){
    gpio_put(enableA_pin, sa == PhaseState::PHASE_ON ? enable_active_high:!enable_active_high);
    gpio_put(enableB_pin, sb == PhaseState::PHASE_ON ? enable_active_high:!enable_active_high);
    gpio_put(enableC_pin, sc == PhaseState::PHASE_ON ? enable_active_high:!enable_active_high);
  }
}

// Set voltage to the pwm pin
void BLDCDriver3PWM::set_pwm(float Ua, float Ub, float Uc) {

  // limit the voltage in driver
  Ua = _constrain(Ua, 0.0f, voltage_limit);
  Ub = _constrain(Ub, 0.0f, voltage_limit);
  Uc = _constrain(Uc, 0.0f, voltage_limit);
  // calculate duty cycle
  // limited in [0,1]
  dc_a = _constrain(Ua / voltage_power_supply, 0.0f , 1.0f );
  dc_b = _constrain(Ub / voltage_power_supply, 0.0f , 1.0f );
  dc_c = _constrain(Uc / voltage_power_supply, 0.0f , 1.0f );

  // hardware specific writing
  // hardware specific function - depending on driver and mcu
  _write_duty_cycle_3_pwm(dc_a, dc_b, dc_c, params);
}

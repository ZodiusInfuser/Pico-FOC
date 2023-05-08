#include "MagneticSensorPWM.h"
#include "pico/stdlib.h"

/** MagneticSensorPWM(uint8_t _pin_pwm, int _min, int _max)
 * @param _pin_pwm  the pin that is reading the pwm from magnetic sensor
 * @param _min_raw_count  the smallest expected reading
 * @param _max_raw_count  the largest expected reading
 */
MagneticSensorPWM::MagneticSensorPWM(uint8_t _pin_pwm, int _min_raw_count, int _max_raw_count){

    pin_pwm = _pin_pwm;

    cpr = _max_raw_count - _min_raw_count + 1;
    min_raw_count = _min_raw_count;
    max_raw_count = _max_raw_count;

    // define if the sensor uses interrupts
    is_interrupt_based = false;

    // define as not set
    last_call_us = time_us_64();
}


/** MagneticSensorPWM(uint8_t _pin_pwm, int freq_hz, int _total_pwm_clocks, int _min_pwm_clocks, int _max_pwm_clocks)
 *
 * Constructor that computes the min and max raw counts based on the PWM frequency and the number of PWM clocks in one period
 *
 * @param _pin_pwm  the pin that is reading the pwm from magnetic sensor
 * @param freq_hz  the frequency of the PWM signal, in Hz, e.g. 115, 230, 460 or 920 for the AS5600, depending on the PWM frequency setting
 * @param _total_pwm_clocks  the total number of PWM clocks in one period, e.g. 4351 for the AS5600
 * @param _min_pwm_clocks  the 0 value returned by the sensor, in PWM clocks, e.g. 128 for the AS5600
 * @param _max_pwm_clocks  the largest value returned by the sensor, in PWM clocks, e.g. 4223 for the AS5600
 */
MagneticSensorPWM::MagneticSensorPWM(uint8_t _pin_pwm, int freq_hz, int _total_pwm_clocks, int _min_pwm_clocks, int _max_pwm_clocks){

    pin_pwm = _pin_pwm;

    min_raw_count = lroundf(1000000.0f/freq_hz/_total_pwm_clocks*_min_pwm_clocks);
    max_raw_count = lroundf(1000000.0f/freq_hz/_total_pwm_clocks*_max_pwm_clocks);
    cpr = max_raw_count - min_raw_count + 1;

    // define if the sensor uses interrupts
    is_interrupt_based = false;

    min_elapsed_time = 1.0f/freq_hz; // set the minimum time between two readings

    // define as not set
    last_call_us = time_us_64();
}



void MagneticSensorPWM::init(){

    // initial hardware
    pinMode(pin_pwm, INPUT);
    raw_count = get_raw_count();

    this->Sensor::init(); // call base class init
}

// get current angle (rad)
float MagneticSensorPWM::get_sensor_angle(){
    // raw data from sensor
    raw_count = get_raw_count();
    if (raw_count > max_raw_count) raw_count = max_raw_count;
    if (raw_count < min_raw_count) raw_count = min_raw_count;
    return( (float) (raw_count - min_raw_count) / (float)cpr) * _2PI;
}


// read the raw counter of the magnetic sensor
int MagneticSensorPWM::get_raw_count(){
    if (!is_interrupt_based){ // if it's not interrupt based read the value in a blocking way
        pulse_length_us = pulseIn(pin_pwm, HIGH);
    }
    return pulse_length_us;
}


void MagneticSensorPWM::handle_pwm() {
    //  unsigned long now_us = ticks();
    unsigned long now_us = time_us_64();

    // if falling edge, calculate the pulse length
    if (!digitalRead(pin_pwm)) pulse_length_us = now_us - last_call_us;

    // save the currrent timestamp for the next call
    last_call_us = now_us;
    is_interrupt_based = true; // set the flag to true
}

// function enabling hardware interrupts of the for the callback provided
// if callback is not provided then the interrupt is not enabled
void MagneticSensorPWM::enable_interrupt(void (*do_pwm)()){
    // declare it's interrupt based
    is_interrupt_based  = true;

    // enable interrupts on pwm input pin
    attachInterrupt(digitalPinToInterrupt(pin_pwm), do_pwm, CHANGE);
}
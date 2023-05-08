#include "MagneticSensorAnalog.h"

/** MagneticSensorAnalog(uint8_t _pin_analog, int _min, int _max)
 * @param _pin_analog  the pin that is reading the pwm from magnetic sensor
 * @param _min_raw_count  the smallest expected reading.  Whilst you might expect it to be 0 it is often ~15.  Getting this wrong results in a small click once per revolution
 * @param _max_raw_count  the largest value read.  whilst you might expect it to be 2^10 = 1023 it is often ~ 1020. Note: For ESP32 (with 12bit ADC the value will be nearer 4096)
 */
MagneticSensorAnalog::MagneticSensorAnalog(uint8_t _pin_analog, int _min_raw_count, int _max_raw_count){

  pin_analog = _pin_analog;

  cpr = _max_raw_count - _min_raw_count;
  min_raw_count = _min_raw_count;
  max_raw_count = _max_raw_count;

  if(pullup == Pullup::USE_INTERN){
    pinMode(pin_analog, INPUT_PULLUP);
  }else{
    pinMode(pin_analog, INPUT);
  }

}


void MagneticSensorAnalog::init(){
  raw_count = get_raw_count();

  this->Sensor::init(); // call base class init
}

//  Shaft angle calculation
//  angle is in radians [rad]
float MagneticSensorAnalog::get_sensor_angle(){
  // raw data from the sensor
  raw_count = get_raw_count();
  return ( (float) (raw_count) / (float)cpr) * _2PI;
}

// function reading the raw counter of the magnetic sensor
int MagneticSensorAnalog::get_raw_count(){
  return analogRead(pin_analog);
}

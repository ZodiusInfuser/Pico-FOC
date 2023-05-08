#include <SimpleFOC.h>

// software interrupt library
#include <PciManager.h>
#include <PciListenerImp.h>

/**
 * Magnetic sensor reading analog voltage on pin which does not have hardware interrupt support. Such as A0.
 * 
 * MagneticSensorPWM(uint8_t MagneticSensorPWM, int _min, int _max)
 * - pin_pwm         - the pin that is reading the pwm from magnetic sensor
 * - min_raw_count  - the smallest expected reading.  Whilst you might expect it to be 0 it is often ~5.  Getting this wrong results in a small click once per revolution
 * - max_raw_count  - the largest value read.  whilst you might expect it to be 1kHz = 1000 it is often ~910. depending on the exact frequency and saturation
 */
MagneticSensorPWM sensor = MagneticSensorPWM(A0, 4, 904);
void doPWM(){sensor.handle_pwm();}

// encoder interrupt init
PciListenerImp listenerPWM(sensor.pin_pwm, doPWM);

void setup() {
  // monitoring port
  Serial.begin(115200);

  // initialise magnetic sensor hardware
  sensor.init();
  // comment out to use sensor in blocking (non-interrupt) way
  PciManager.registerListener(&listenerPWM);

  printf("Sensor ready");
  sleep_ms(1000);
}

void loop() {
  // iterative function updating the sensor internal variables
  // it is usually called in motor.loop_foc()
  // this function reads the sensor hardware and 
  // has to be called before get_angle nad get_velocity
  sensor.update();
  // display the angle and the angular velocity to the terminal
  Serial.print(sensor.get_angle());
  Serial.print("\t");
  Serial.println(sensor.get_velocity());
}

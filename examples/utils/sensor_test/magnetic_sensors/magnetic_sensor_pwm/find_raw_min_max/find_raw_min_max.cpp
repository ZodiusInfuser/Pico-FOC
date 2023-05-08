#include <SimpleFOC.h>


/**
 * An example to find out the raw max and min count to be provided to the constructor
 * SPin your motor/sensor/magnet to see what is the maximum output of the sensor and what is the minimum value 
 * And replace values 4 and 904 with new values. Once when you replace them make sure there is no jump in the angle reading sensor.get_angle(). 
 * If there is a jump that means you can still find better values. 
 */
MagneticSensorPWM sensor = MagneticSensorPWM(2, 4, 904);
void doPWM(){sensor.handle_pwm();}

void setup() {
  // monitoring port
  Serial.begin(115200);

  // initialise magnetic sensor hardware
  sensor.init();
   // comment out to use sensor in blocking (non-interrupt) way
  sensor.enable_interrupt(doPWM);

  printf("Sensor ready");
  sleep_ms(1000);
}

int max_pulse= 0;
int min_pulse = 10000;

void loop() {
  // iterative function updating the sensor internal variables
  // it is usually called in motor.loop_foc()
  // this function reads the sensor hardware and 
  // has to be called before get_angle nad get_velocity
  sensor.update();

  // keep track of min and max
  if(sensor.pulse_length_us > max_pulse) max_pulse = sensor.pulse_length_us;
  else if(sensor.pulse_length_us < min_pulse) min_pulse = sensor.pulse_length_us;

  // display the raw count, and max and min raw count
  Serial.print("angle:");
  Serial.print(sensor.get_angle());
  Serial.print("\t, raw:");
  Serial.print(sensor.pulse_length_us);
  Serial.print("\t, min:");
  Serial.print(min_pulse);
  Serial.print("\t, max:");
  Serial.println(max_pulse);
}

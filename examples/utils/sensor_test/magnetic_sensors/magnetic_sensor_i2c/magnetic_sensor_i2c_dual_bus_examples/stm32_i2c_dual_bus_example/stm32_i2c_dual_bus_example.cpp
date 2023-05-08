#include <SimpleFOC.h>

/** Annoyingly some i2c sensors (e.g. AS5600) have a fixed chip address.  This means only one of these devices can be addressed on a single bus
 * This example shows how a second i2c bus can be used to communicate with a second sensor.  
 */ 

MagneticSensorI2C sensor0 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);

// example of stm32 defining 2nd bus
TwoWire Wire1(PB11, PB10);


void setup() {

  Serial.begin(115200);
  sleep_ms(750);

  Wire.setClock(400000);
  Wire1.setClock(400000);

  sensor0.init();
  sensor1.init(&Wire1);
}

void loop() {
  // iterative function updating the sensor internal variables
  // it is usually called in motor.loop_foc()
  // this function reads the sensor hardware and 
  // has to be called before get_angle nad get_velocity
  sensor0.update();
  sensor1.update();
  
  sleep_ms(200);
  Serial.print(sensor0.get_angle()); 
  Serial.print(" - "); 
  Serial.print(sensor1.get_angle());
  Serial.println();
}

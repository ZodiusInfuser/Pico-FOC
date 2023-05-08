#include <SimpleFOC.h>

// MagneticSensorSPI(int cs, float _cpr, int _angle_register)
// config           - SPI config
//  cs              - SPI chip select pin 
MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, PA15);

// these are valid pins (mosi, miso, sclk) for 2nd SPI bus on storm32 board (stm32f107rc)
SPIClass SPI_2(PB15, PB14, PB13);

void setup() {
  // monitoring port
  Serial.begin(115200);

  // initialise magnetic sensor hardware
  sensor.init(&SPI_2);

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

/**
 * Simple example intended to help users find the zero offset and natural direction of the sensor. 
 * 
 * These values can further be used to avoid motor and sensor alignment procedure. 
 * 
 * motor.init_foc(zero_offset, sensor_direction);
 * 
 * This will only work for abosolute value sensors - magnetic sensors. 
 * Bypassing the alignment procedure is not possible for the encoders and for the current implementation of the Hall sensors. 
 * library version 1.4.2.
 * 
 */
#include <SimpleFOC.h>

// magnetic sensor instance - SPI
//MagneticSensorSPI sensor = MagneticSensorSPI(10, 14, 0x3FFF);
// magnetic sensor instance - I2C
//MagneticSensorI2C sensor = MagneticSensorI2C(0x36, 12, 0X0C, 4);
// magnetic sensor instance - analog output
MagneticSensorAnalog sensor = MagneticSensorAnalog(A1, 14, 1020);

// BLDC motor instance
BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 5, 6, 8);
// Stepper motor instance
//StepperMotor motor = StepperMotor(50);
//StepperDriver4PWM driver = StepperDriver4PWM(9, 5, 10, 6,  8);

void setup() {

  // power supply voltage
  driver.voltage_power_supply = 12;
  driver.init();
  motor.link_driver(&driver);

  // initialise magnetic sensor hardware
  sensor.init();
  // link the motor to the sensor
  motor.link_sensor(&sensor);

  // aligning voltage 
  motor.voltage_sensor_align = 7;
  
  // set motion control loop to be used
  motor.controller = MotionControlType::torque;

  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.init_foc();

  
  Serial.begin(115200);
  printf("Sensor zero offset is:");
  Serial.println(motor.zero_electric_angle, 4);
  printf("Sensor natural direction is: ");
  Serial.println(motor.sensor_direction == 1 ? "Direction::CW" : "Direction::CCW");

  printf("To use these values provide them to the: motor.init_foc(offset, direction)");
  sleep_ms(1000);
  printf("If motor is not moving the alignment procedure was not successfull!!");
}


void loop() {
    
  // main FOC algorithm function
  motor.loop_foc();

  // Motion control function
  motor.move(2);
}


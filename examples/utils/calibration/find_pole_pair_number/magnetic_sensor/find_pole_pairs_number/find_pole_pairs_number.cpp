/**
 * Utility arduino sketch which finds pole pair number of the motor
 *
 * To run it just set the correct pin numbers for the BLDC driver and sensor CPR value and chip select pin.
 *
 * The program will rotate your motor a specific amount and check how much it moved, and by doing a simple calculation calculate your pole pair number.
 * The pole pair number will be outputted to the serial terminal.
 *
 * If the pole pair number is well estimated your motor will start to spin in voltage mode with 2V target.
 *
 * If the code calculates negative pole pair number please invert your motor connector.
 *
 * Try running this code several times to avoid statistical errors.
 * > But in general if your motor spins, you have a good pole pairs number.
 */
#include <SimpleFOC.h>

// BLDC motor instance
// its important to put pole pairs number as 1!!!
BLDCMotor motor = BLDCMotor(1);
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 5, 6, 8);
// Stepper motor instance
// its important to put pole pairs number as 1!!!
//StepperMotor motor = StepperMotor(1);
//StepperDriver4PWM driver = StepperDriver4PWM(9, 5, 10, 6,  8);

// magnetic sensor instance - SPI
MagneticSensorSPI sensor = MagneticSensorSPI(10, 14, 0x3FFF);
// magnetic sensor instance - I2C
//MagneticSensorI2C sensor = MagneticSensorI2C(0x36, 12, 0X0C, 4);
// magnetic sensor instance - analog output
// MagneticSensorAnalog sensor = MagneticSensorAnalog(A1, 14, 1020);

void setup() {

  // initialise magnetic sensor hardware
  sensor.init();
  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // power supply voltage
  // default 12V
  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);

  // initialize motor hardware
  motor.init();

  // monitoring port
  Serial.begin(115200);

  // pole pairs calculation routine
  printf("Pole pairs (PP) estimator");
  printf("-\n");

  float pp_search_voltage = 4; // maximum power_supply_voltage/2
  float pp_search_angle = 6*_PI; // search electrical angle to turn

  // move motor to the electrical angle 0
  motor.controller = MotionControlType::angle_openloop;
  motor.voltage_limit=pp_search_voltage;
  motor.move(0);
  sleep_ms(1000);
  // read the sensor angle
  sensor.update();
  float angle_begin = sensor.getAngle();
  sleep_ms(50);

  // move the motor slowly to the electrical angle pp_search_angle
  float motor_angle = 0;
  while(motor_angle <= pp_search_angle){
    motor_angle += 0.01f;
    sensor.update(); // keep track of the overflow
    motor.move(motor_angle);
    sleep_ms(1);
  }
  sleep_ms(1000);
  // read the sensor value for 180
  sensor.update(); 
  float angle_end = sensor.getAngle();
  sleep_ms(50);
  // turn off the motor
  motor.move(0);
  sleep_ms(1000);

  // calculate the pole pair number
  int pp = round((pp_search_angle)/(angle_end-angle_begin));

  Serial.print(F("Estimated PP : \n");
  Serial.println(pp);
  printf("PP = Electrical angle / Encoder angle \n");
  Serial.print(pp_search_angle*180/_PI);
  Serial.print(F("/\n");
  Serial.print((angle_end-angle_begin)*180/_PI);
  Serial.print(F(" = \n");
  Serial.println((pp_search_angle)/(angle_end-angle_begin));
  Serial.println();


  // a bit of monitoring the result
  if(pp <= 0 ){
    printf("PP number cannot be negative\n");
    printf(" - Try changing the search_voltage value or motor/sensor configuration.\n");
    return;
  }else if(pp > 30){
    printf("PP number very high, possible error.\n");
  }else{
    printf("If PP is estimated well your motor should turn now!\n");
    printf(" - If it is not moving try to relaunch the program!\n");
    printf(" - You can also try to adjust the target voltage using serial terminal!\n");
  }


  // set motion control loop to be used
  motor.controller = MotionControlType::torque;
  // set the pole pair number to the motor
  motor.pole_pairs = pp;
  //align sensor and start FOC
  motor.initFOC();
  sleep_ms(1000);

  printf("\n Motor ready.\n");
  printf("Set the target voltage using serial terminal:\n");
}

// uq voltage
float target_voltage = 2;

void loop() {

  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz
  motor.loopFOC();

  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  motor.move(target_voltage);

  // communicate with the user
  serialReceiveUserCommand();
}


// utility function enabling serial communication with the user to set the target values
// this function can be implemented in serialEvent function as well
void serialReceiveUserCommand() {

  // a string to hold incoming data
  static String received_chars;

  while (Serial.available()) {
    // get the new uint8_t:
    char inChar = (char)Serial.read();
    // add it to the string buffer:
    received_chars += inChar;
    // end of user input
    if (inChar == '\n') {

      // change the motor target
      target_voltage = received_chars.toFloat();
      Serial.print("Target voltage: ");
      Serial.println(target_voltage);

      // reset the command buffer
      received_chars = "";
    }
  }
}

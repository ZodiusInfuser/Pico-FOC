/**
 *
 * Torque control example using current control loop.
 *
 */
#include <SimpleFOC.h>


// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 5, 6, 8);

// encoder instance
Encoder encoder = Encoder(2, 3, 500);
// channel A and B callbacks
void doA(){encoder.handle_a();}
void doB(){encoder.handle_b();}

// current sensor
InlineCurrentSense current_sense = InlineCurrentSense(0.01f, 50.0f, A0, A2);

// current set point variable
float target_current = 0;
// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_current, cmd); }

void setup() {

  // initialize encoder sensor hardware
  encoder.init();
  encoder.enable_interrupts(doA, doB);
  // link the motor to the sensor
  motor.link_sensor(&encoder);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // link driver
  motor.link_driver(&driver);
  // link current sense and the driver
  current_sense.link_driver(&driver);

  // current sense init hardware
  current_sense.init();
  // link the current sense to the motor
  motor.link_current_sense(&current_sense);

  // set torque mode:
  // TorqueControlType::dc_current
  // TorqueControlType::voltage
  // TorqueControlType::foc_current
  motor.torque_controller = TorqueControlType::foc_current;
  // set motion control loop to be used
  motor.controller = MotionControlType::torque;

  // foc currnet control parameters (Arduino UNO/Mega)
  motor.PID_current_q.P = 5;
  motor.PID_current_q.I= 300;
  motor.PID_current_d.P= 5;
  motor.PID_current_d.I = 300;
  motor.LPF_current_q.tf = 0.01f;
  motor.LPF_current_d.tf = 0.01f;
  // foc currnet control parameters (stm/esp/due/teensy)
  // motor.PID_current_q.P = 5;
  // motor.PID_current_q.I= 1000;
  // motor.PID_current_d.P= 5;
  // motor.PID_current_d.I = 1000;
  // motor.LPF_current_q.tf = 0.002f; // 1ms default
  // motor.LPF_current_d.tf = 0.002f; // 1ms default

  // use monitoring with serial
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);

  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.init_foc();

  // add target command T
  command.add('T', doTarget, "target current");

  printf("Motor ready.\n");
  printf("Set the target current using serial terminal:\n");
  sleep_ms(1000);
}

void loop() {

  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz
  motor.loop_foc();

  // Motion control function
  // velocity, position or torque (defined in motor.controller)
  // this function can be run at much lower frequency than loop_foc() function
  // You can also use motor.move() and set the motor.target in the code
  motor.move(target_current);

  // user communication
  command.run();
}
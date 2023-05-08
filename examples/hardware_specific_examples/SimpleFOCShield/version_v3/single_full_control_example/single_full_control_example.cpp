
#include <SimpleFOC.h>

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(6, 10, 5, 8);

// encoder instance
Encoder encoder = Encoder(2, 3, 500);
// channel A and B callbacks
void doA(){encoder.handle_a();}
void doB(){encoder.handle_b();}

// inline current sensor instance
// ACS712-05B has the resolution of 0.185mV per Amp
InlineCurrentSense current_sense = InlineCurrentSense(1.0f, 0.185f, A0, A2);

// commander communication instance
Commander command = Commander(Serial);
void doMotion(char* cmd){ command.motion(&motor, cmd); }
// void doMotor(char* cmd){ command.motor(&motor, cmd); }

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

  // set control loop type to be used
  motor.controller = MotionControlType::torque;

  // controller configuration based on the control type
  motor.PID_velocity.P = 0.05f;
  motor.PID_velocity.I = 1;
  motor.PID_velocity.D = 0;
  // default voltage_power_supply
  motor.voltage_limit = 12;

  // velocity low pass filtering time constant
  motor.LPF_velocity.tf = 0.01f;

  // angle loop controller
  motor.P_angle.P = 20;
  // angle loop velocity limit
  motor.velocity_limit = 20;

  // use monitoring with serial for motor init
  // monitoring port
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);
  motor.monitor_downsample = 0; // disable intially
  motor.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE; // monitor target velocity and angle

  // current sense init and linking
  current_sense.init();
  motor.link_current_sense(&current_sense);

  // initialise motor
  motor.init();
  // align encoder and start FOC
  motor.init_foc();

  // set the inital target value
  motor.target = 2;

  // subscribe motor to the commander
  command.add('T', doMotion, "motion control");
  // command.add('M', doMotor, "motor");
  
  // Run user commands to configure and the motor (find the full command list in docs.simplefoc.com)
  printf("Motor ready.\n");

  sleep_ms(1000);
}


void loop() {
  // iterative setting FOC phase voltage
  motor.loop_foc();

  // iterative function setting the outter loop target
  motor.move();

  // motor monitoring
  motor.monitor();

  // user communication
  command.run();
}
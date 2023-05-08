/**
 *  Encoder example code using only software interrupts
 * 
 * This is a code intended to test the encoder connections and to 
 * demonstrate the encoder setup fully using software interrupts.
 * - We use PciManager library: https://github.com/prampec/arduino-pcimanager
 * 
 * This code will work on Arduino devices but not on STM32 devices
 * 
 */

#include <SimpleFOC.h>
// software interrupt library
#include <PciManager.h>
#include <PciListenerImp.h>

// encoder instance
Encoder encoder = Encoder(A0, A1, 2048);
// interrupt routine intialisation
void doA(){encoder.handle_a();}
void doB(){encoder.handle_b();}

// encoder interrupt init
PciListenerImp listenerA(encoder.pin_a, doA);
PciListenerImp listenerB(encoder.pin_b, doB);

void setup() {
  // monitoring port
  Serial.begin(115200);

  // enable/disable quadrature mode
  encoder.quadrature = Quadrature::ON;

  // check if you need internal pullups
  encoder.pullup = Pullup::USE_EXTERN;
  
  // initialise encoder hardware
  encoder.init();
  
  // interrupt initialization
  PciManager.registerListener(&listenerA);
  PciManager.registerListener(&listenerB);

  printf("Encoder ready");
  sleep_ms(1000);
}

void loop() {
  // iterative function updating the sensor internal variables
  // it is usually called in motor.loop_foc()
  // not doing much for the encoder though
  encoder.update();
  // display the angle and the angular velocity to the terminal
  Serial.print(encoder.get_angle());
  Serial.print("\t");
  Serial.println(encoder.get_velocity());
}

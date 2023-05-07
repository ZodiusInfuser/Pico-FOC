#include "time_utils.h"

// function buffering delay() 
// arduino uno function doesn't work well with interrupts
void _delay(unsigned long ms){
  // regular micros
  sleep_ms(ms);
}


// function buffering _micros() 
// arduino function doesn't work well with interrupts
unsigned long _micros(){
  // regular micros
  return time_us_64();
}

#include "Encoder.h"


/*
  Encoder(int enc_a, int enc_b , int cpr, int index)
  - enc_a, enc_b    - encoder A and B pins
  - cpr           - counts per rotation number (cpm=ppm*4)
  - index pin     - (optional input)
*/

Encoder::Encoder(int _enc_a, int _enc_b , float _ppr, int _index){

  // Encoder measurement structure init
  // hardware pins
  pin_a = _enc_a;
  pin_b = _enc_b;
  // counter setup
  pulse_counter = 0;
  pulse_timestamp = 0;

  cpr = _ppr;
  A_active = 0;
  B_active = 0;
  I_active = 0;
  // index pin
  index_pin = _index; // its 0 if not used

  // velocity calculation variables
  prev_Th = 0;
  pulse_per_second = 0;
  prev_pulse_counter = 0;
  prev_timestamp_us = time_us_64();

  // extern pullup as default
  pullup = Pullup::USE_EXTERN;
  // enable quadrature encoder by default
  quadrature = Quadrature::ON;
}

//  Encoder interrupt callback functions
// A channel
void Encoder::handle_a() {
  bool A = digitalRead(pin_a);
  switch (quadrature){
    case Quadrature::ON:
      // CPR = 4xPPR
      if ( A != A_active ) {
        pulse_counter += (A_active == B_active) ? 1 : -1;
        pulse_timestamp = time_us_64();
        A_active = A;
      }
      break;
    case Quadrature::OFF:
      // CPR = PPR
      if(A && !digitalRead(pin_b)){
        pulse_counter++;
        pulse_timestamp = time_us_64();
      }
      break;
  }
}
// B channel
void Encoder::handle_b() {
  bool B = digitalRead(pin_b);
  switch (quadrature){
    case Quadrature::ON:
  //     // CPR = 4xPPR
      if ( B != B_active ) {
        pulse_counter += (A_active != B_active) ? 1 : -1;
        pulse_timestamp = time_us_64();
        B_active = B;
      }
      break;
    case Quadrature::OFF:
      // CPR = PPR
      if(B && !digitalRead(pin_a)){
        pulse_counter--;
        pulse_timestamp = time_us_64();
      }
      break;
  }
}

// Index channel
void Encoder::handle_index() {
  if(has_index()){
    bool I = digitalRead(index_pin);
    if(I && !I_active){
      index_found = true;
      // align encoder on each index
      long tmp = pulse_counter;
      // corrent the counter value
      pulse_counter = round((double)pulse_counter/(double)cpr)*cpr;
      // preserve relative speed
      prev_pulse_counter += pulse_counter - tmp;
    }
    I_active = I;
  }
}


void Encoder::update() {
    // do nothing for Encoder
}

/*
	Shaft angle calculation
*/
float Encoder::get_sensor_angle(){
  return get_angle();
}
// TODO: numerical precision issue here if the pulse_counter overflows the angle will be lost
float Encoder::get_mechanical_angle(){
  return  _2PI * ((pulse_counter) % ((int)cpr)) / ((float)cpr);
}

float Encoder::get_angle(){
  return  _2PI * (pulse_counter) / ((float)cpr);
}
double Encoder::get_precise_angle(){
  return  _2PI * (pulse_counter) / ((double)cpr);
}
int32_t Encoder::get_full_rotations(){
  return  pulse_counter / (int)cpr;
}



/*
  Shaft velocity calculation
  function using mixed time and frequency measurement technique
*/
float Encoder::get_velocity(){
  // timestamp
  long timestamp_us = time_us_64();
  // sampling time calculation
  float Ts = (timestamp_us - prev_timestamp_us) * 1e-6f;
  // quick fix for strange cases (micros overflow)
  if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

  // time from last impulse
  float Th = (timestamp_us - pulse_timestamp) * 1e-6f;
  long dN = pulse_counter - prev_pulse_counter;

  // Pulse per second calculation (Eq.3.)
  // dN - impulses received
  // Ts - sampling time - time in between function calls
  // Th - time from last impulse
  // Th_1 - time form last impulse of the previous call
  // only increment if some impulses received
  float dt = Ts + prev_Th - Th;
  pulse_per_second = (dN != 0 && dt > Ts/2) ? dN / dt : pulse_per_second;

  // if more than 0.05f passed in between impulses
  if ( Th > 0.1f) pulse_per_second = 0;

  // velocity calculation
  float velocity = pulse_per_second / ((float)cpr) * (_2PI);

  // save variables for next pass
  prev_timestamp_us = timestamp_us;
  // save velocity calculation variables
  prev_Th = Th;
  prev_pulse_counter = pulse_counter;
  return velocity;
}

// getter for index pin
// return -1 if no index
int Encoder::needs_search(){
  return has_index() && !index_found;
}

// private function used to determine if encoder has index
int Encoder::has_index(){
  return index_pin != 0;
}


// encoder initialisation of the hardware pins
// and calculation variables
void Encoder::init(){

  // Encoder - check if pullup needed for your encoder
  if(pullup == Pullup::USE_INTERN){
    pinMode(pin_a, INPUT_PULLUP);
    pinMode(pin_b, INPUT_PULLUP);
    if(has_index()) pinMode(index_pin,INPUT_PULLUP);
  }else{
    pinMode(pin_a, INPUT);
    pinMode(pin_b, INPUT);
    if(has_index()) pinMode(index_pin,INPUT);
  }

  // counter setup
  pulse_counter = 0;
  pulse_timestamp = time_us_64();
  // velocity calculation variables
  prev_Th = 0;
  pulse_per_second = 0;
  prev_pulse_counter = 0;
  prev_timestamp_us = time_us_64();

  // initial cpr = PPR
  // change it if the mode is quadrature
  if(quadrature == Quadrature::ON) cpr = 4*cpr;

  // we don't call Sensor::init() here because init is handled in Encoder class.
}

// function enabling hardware interrupts of the for the callback provided
// if callback is not provided then the interrupt is not enabled
void Encoder::enable_interrupts(void (*doA)(), void(*doB)(), void(*doIndex)()){
  // attach interrupt if functions provided
  switch(quadrature){
    case Quadrature::ON:
      // A callback and B callback
      if(doA != nullptr) attachInterrupt(digitalPinToInterrupt(pin_a), doA, CHANGE);
      if(doB != nullptr) attachInterrupt(digitalPinToInterrupt(pin_b), doB, CHANGE);
      break;
    case Quadrature::OFF:
      // A callback and B callback
      if(doA != nullptr) attachInterrupt(digitalPinToInterrupt(pin_a), doA, RISING);
      if(doB != nullptr) attachInterrupt(digitalPinToInterrupt(pin_b), doB, RISING);
      break;
  }

  // if index used initialize the index interrupt
  if(has_index() && doIndex != nullptr) attachInterrupt(digitalPinToInterrupt(index_pin), doIndex, CHANGE);
}

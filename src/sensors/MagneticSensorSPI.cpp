
#include "MagneticSensorSPI.h"

/** Typical configuration for the 14bit AMS AS5147 magnetic sensor over SPI interface */
MagneticSensorSPIConfig_s AS5147_SPI = {
  .spi_mode = SPI_MODE1,
  .clock_speed = 1000000,
  .bit_resolution = 14,
  .angle_register = 0x3FFF,
  .data_start_bit = 13,
  .command_rw_bit = 14,
  .command_parity_bit = 15
};
// AS5048 and AS5047 are the same as AS5147
MagneticSensorSPIConfig_s AS5048_SPI = AS5147_SPI;
MagneticSensorSPIConfig_s AS5047_SPI = AS5147_SPI;

/** Typical configuration for the 14bit MonolithicPower MA730 magnetic sensor over SPI interface */
MagneticSensorSPIConfig_s MA730_SPI = {
  .spi_mode = SPI_MODE0,
  .clock_speed = 1000000,
  .bit_resolution = 14,
  .angle_register = 0x0000,
  .data_start_bit = 15,
  .command_rw_bit = 0,  // not required
  .command_parity_bit = 0 // parity not implemented
};


// MagneticSensorSPI(int cs, float _bit_resolution, int _angle_register)
//  cs              - SPI chip select pin
//  _bit_resolution   sensor resolution bit number
// _angle_register  - (optional) angle read register - default 0x3FFF
MagneticSensorSPI::MagneticSensorSPI(int cs, float _bit_resolution, int _angle_register){

  chip_select_pin = cs;
  // angle read register of the magnetic sensor
  angle_register = _angle_register ? _angle_register : DEF_ANGLE_REGISTER;
  // register maximum value (counts per revolution)
  cpr = pow(2,_bit_resolution);
  spi_mode = SPI_MODE1;
  clock_speed = 1000000;
  bit_resolution = _bit_resolution;

  command_parity_bit = 15; // for backwards compatibilty
  command_rw_bit = 14; // for backwards compatibilty
  data_start_bit = 13; // for backwards compatibilty
}

MagneticSensorSPI::MagneticSensorSPI(MagneticSensorSPIConfig_s config, int cs){
  chip_select_pin = cs;
  // angle read register of the magnetic sensor
  angle_register = config.angle_register ? config.angle_register : DEF_ANGLE_REGISTER;
  // register maximum value (counts per revolution)
  cpr = pow(2, config.bit_resolution);
  spi_mode = config.spi_mode;
  clock_speed = config.clock_speed;
  bit_resolution = config.bit_resolution;

  command_parity_bit = config.command_parity_bit; // for backwards compatibilty
  command_rw_bit = config.command_rw_bit; // for backwards compatibilty
  data_start_bit = config.data_start_bit; // for backwards compatibilty
}

void MagneticSensorSPI::init(spi_inst_t* _spi){
  spi = _spi;
  // 1MHz clock (AMS should be able to accept up to 10MHz)
  settings = SPISettings(clock_speed, MSBFIRST, spi_mode);
  //setup pins
  pinMode(chip_select_pin, OUTPUT);
  //SPI has an internal SPI-device counter, it is possible to call "begin()" from different devices
  spi->begin();
  // do any architectures need to set the clock divider for SPI? Why was this in the code?
  //spi->setClockDivider(SPI_CLOCK_DIV8);
  gpio_put(chip_select_pin, HIGH);

  this->Sensor::init(); // call base class init
}

//  Shaft angle calculation
//  angle is in radians [rad]
float MagneticSensorSPI::get_sensor_angle(){
  return (get_raw_count() / (float)cpr) * _2PI;
}

// function reading the raw counter of the magnetic sensor
int MagneticSensorSPI::get_raw_count(){
  return (int)MagneticSensorSPI::read(angle_register);
}

// SPI functions
/**
 * Utility function used to calculate even parity of int16_t
 */
uint8_t MagneticSensorSPI::spi_calc_even_parity(int16_t value){
  uint8_t cnt = 0;
  uint8_t i;

  for (i = 0; i < 16; i++)
  {
    if (value & 0x1) cnt++;
    value >>= 1;
  }
  return cnt & 0x1;
}

  /*
  * Read a register from the sensor
  * Takes the address of the register as a 16 bit int16_t
  * Returns the value of the register
  */
int16_t MagneticSensorSPI::read(int16_t angle_register){

  int16_t command = angle_register;

  if (command_rw_bit > 0) {
    command = angle_register | (1 << command_rw_bit);
  }
  if (command_parity_bit > 0) {
     //Add a parity bit on the the MSB
    command |= ((int16_t)spi_calc_even_parity(command) << command_parity_bit);
  }

  //SPI - begin transaction
  spi->beginTransaction(settings);

  //Send the command
  gpio_put(chip_select_pin, LOW);
  spi->transfer16(command);
  gpio_put(chip_select_pin,HIGH);

  sleep_us(1); // delay 1us, the minimum time possible with sleep functions. 350ns is the required time for AMS sensors, 80ns for MA730, MA702

  //Now read the response
  gpio_put(chip_select_pin, LOW);
  int16_t register_value = spi->transfer16(0x00);
  gpio_put(chip_select_pin, HIGH);

  //SPI - end transaction
  spi->endTransaction();

  register_value = register_value >> (1 + data_start_bit - bit_resolution);  //this should shift data to the rightmost bits of the int16_t

  const static int16_t data_mask = 0xFFFF >> (16 - bit_resolution);

  return register_value & data_mask;  // Return the data, stripping the non data (e.g parity) bits
}

/**
 * Closes the SPI connection
 * SPI has an internal SPI-device counter, for each init()-call the close() function must be called exactly 1 time
 */
void MagneticSensorSPI::close(){
  spi->end();
}

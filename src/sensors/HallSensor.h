#ifndef HALL_SENSOR_LIB_H
#define HALL_SENSOR_LIB_H

#include "pico/stdlib.h"
#include "../common/base_classes/Sensor.h"
#include "../common/foc_utils.h"

// seq 1 > 5 > 4 > 6 > 2 > 3 > 1     000 001 010 011 100 101 110 111
const int8_t ELECTRIC_SECTORS[8] = { -1,  0,  4,  5,  2,  1,  3 , -1 };

class HallSensor: public Sensor{
 public:
    /**
    HallSensor class constructor
    @param enc_a  HallSensor A pin
    @param enc_b  HallSensor B pin
    @param enc_c  HallSensor C pin
    @param pp  pole pairs  (e.g hoverboard motor has 15pp and small gimbals often have 7pp)
    @param index index pin number (optional input)
    */
    HallSensor(int enc_a, int enc_b, int enc_c, int pp);

    /** HallSensor initialise pins */
    void init();
    /**
     *  function enabling hardware interrupts for the HallSensor channels with provided callback functions
     *  if callback is not provided then the interrupt is not enabled
     * 
     * @param do_a pointer to the A channel interrupt handler function
     * @param do_b pointer to the B channel interrupt handler function
     * @param do_c pointer to the C channel interrupt handler function
     * 
     */
    void enable_interrupts(void (*do_a)() = nullptr, void(*do_b)() = nullptr, void(*do_c)() = nullptr);
    
    //  HallSensor interrupt callback functions
    /** A channel callback function */
    void handle_a();
    /** B channel callback function */
    void handle_b();
    /** C channel callback function */
    void handle_c();
    
    
    // pins A and B
    int pin_a; //!< HallSensor hardware pin A
    int pin_b; //!< HallSensor hardware pin B
    int pin_c; //!< HallSensor hardware pin C

    // HallSensor configuration
    Pullup pullup; //!< Configuration parameter internal or external pullups
    int cpr;//!< HallSensor cpr number

    // Abstract functions of the Sensor class implementation
    /** get current angle (rad) */
    float get_sensor_angle() override;
    float get_mechanical_angle() override;
    float get_angle() override;
    /**  get current angular velocity (rad/s) */
    float get_velocity() override;
    double get_precise_angle() override;
    int32_t get_full_rotations() override;

    // whether last step was CW (+1) or CCW (-1).  
    Direction direction;

    void attach_sector_callback(void (*on_sector_change)(int a) = nullptr);

    // the current 3bit state of the hall sensors
    volatile int8_t hall_state;
    // the current sector of the sensor. Each sector is 60deg electrical
    volatile int8_t electric_sector;
    // the number of electric rotations
    volatile long electric_rotations;
    // this is sometimes useful to identify interrupt issues (e.g. weak or no pullup resulting in 1000s of interrupts)
    volatile long total_interrupts; 

    // variable used to filter outliers - rad/s
    float velocity_max = 1000.0f;

  private:
    
    Direction decode_direction(int oldState, int newState);
    void update_state();

    volatile unsigned long pulse_timestamp;//!< last impulse timestamp in us
    volatile int A_active; //!< current active states of A channel
    volatile int B_active; //!< current active states of B channel
    volatile int C_active; //!< current active states of C channel

    // function pointer for on sector change call back
    void (*on_sector_change)(int sector) = nullptr;

    volatile long pulse_diff;
    
};


#endif

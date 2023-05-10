#pragma once

#include "pico/stdlib.h"
#include "../common/foc_utils.h"
#include "../common/base_classes/Sensor.h"

/**
 *  Quadrature mode configuration structure
 */
enum Quadrature : uint8_t {
  ON    = 0x00, //!<  Enable quadrature mode CPR = 4xPPR
  OFF   = 0x01  //!<  Disable quadrature mode / CPR = PPR
};

class Encoder: public Sensor {
  public:
    /**
     * Encoder class constructor
     * @param enc_a  encoder B pin
     * @param enc_b  encoder B pin
     * @param ppr  impulses per rotation  (cpr=ppr*4)
     * @param index index pin number (optional input)
     */
    Encoder(int enc_a, int enc_b , float ppr, int index = 0);

    /** encoder initialise pins */
    void init() override;

    /**
     * function enabling hardware interrupts for the encoder channels with provided callback functions
     * if callback is not provided then the interrupt is not enabled
     *
     * @param do_a pointer to the A channel interrupt handler function
     * @param do_b pointer to the B channel interrupt handler function
     * @param do_index pointer to the Index channel interrupt handler function
     *
     */
    void enable_interrupts(void (*do_a)() = nullptr, void(*do_b)() = nullptr, void(*do_index)() = nullptr);

    //  Encoder interrupt callback functions
    /** A channel callback function */
    void handle_a();
    /** B channel callback function */
    void handle_b();
    /** Index channel callback function */
    void handle_index();

    // pins A and B
    int pin_a; //!< encoder hardware pin A
    int pin_b; //!< encoder hardware pin B
    int index_pin; //!< index pin

    // Encoder configuration
    Pullup pullup; //!< Configuration parameter internal or external pullups
    Quadrature quadrature;//!< Configuration parameter enable or disable quadrature mode
    float cpr;//!< encoder cpr number

    // Abstract functions of the Sensor class implementation
    /** get current angle (rad) */
    float get_sensor_angle() override;
    float get_mechanical_angle() override;
    /**  get current angular velocity (rad/s) */
    float get_velocity() override;
    float get_angle() override;
    double get_precise_angle() override;
    int32_t get_full_rotations() override;
    virtual void update() override;

    /**
     * returns 0 if it does need search for absolute zero
     * 0 - encoder without index
     * 1 - ecoder with index
     */
    int needs_search() override;

  private:
    int has_index(); //!< function returning 1 if encoder has index pin and 0 if not.

    volatile long pulse_counter;//!< current pulse counter
    volatile long pulse_timestamp;//!< last impulse timestamp in us
    volatile int A_active; //!< current active states of A channel
    volatile int B_active; //!< current active states of B channel
    volatile int I_active; //!< current active states of Index channel
    volatile bool index_found = false; //!< flag stating that the index has been found

    // velocity calculation variables
    float prev_Th, pulse_per_second;
    volatile long prev_pulse_counter, prev_timestamp_us;
};

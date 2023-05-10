#pragma once

#include "drivers/hardware_api.h"

class StepperDriver {
  public:
    /** Initialise hardware */
    virtual int init() = 0;
    /** Enable hardware */
    virtual void enable() = 0;
    /** Disable hardware */
    virtual void disable() = 0;

    long pwm_frequency; //!< pwm frequency value in hertz
    float voltage_power_supply; //!< power supply voltage
    float voltage_limit; //!< limiting voltage set to the motor

    bool initialized = false; // true if driver was successfully initialized
    void* params = 0; // pointer to hardware specific parameters of driver

    /**
     * Set phase voltages to the harware
     *
     * @param ua phase A voltage
     * @param ub phase B voltage
    */
    virtual void set_pwm(float ua, float ub) = 0;
};

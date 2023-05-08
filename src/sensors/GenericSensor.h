#ifndef GENERIC_SENSOR_LIB_H
#define GENERIC_SENSOR_LIB_H

#include "pico/stdlib.h"
#include "../common/foc_utils.h"
#include "../common/base_classes/Sensor.h"


class GenericSensor: public Sensor{
 public:
    /**
    GenericSensor class constructor
     * @param read_callback pointer to the function reading the sensor angle
     * @param init_callback pointer to the function initialising the sensor
    */
    GenericSensor(float (*read_callback)() = nullptr, void (*init_callback)() = nullptr);

    float (*read_callback)() = nullptr; //!< function pointer to sensor reading
    void (*init_callback)() = nullptr; //!< function pointer to sensor initialisation

    void init() override;

    // Abstract functions of the Sensor class implementation
    /** get current angle (rad) */
    float get_sensor_angle() override;

};


#endif

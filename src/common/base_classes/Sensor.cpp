#include "Sensor.h"
#include "../foc_utils.h"



void Sensor::update() {
    float val = get_sensor_angle();
    angle_prev_ts = time_us_64();
    float d_angle = val - angle_prev;
    // if overflow happened track it as full rotation
    if(abs(d_angle) > (0.8f*_2PI) ) full_rotations += ( d_angle > 0 ) ? -1 : 1;
    angle_prev = val;
}


 /** get current angular velocity (rad/s) */
float Sensor::get_velocity() {
    // calculate sample time
    float Ts = (angle_prev_ts - vel_angle_prev_ts)*1e-6;
    // TODO handle overflow - we do need to reset vel_angle_prev_ts
    if (Ts < min_elapsed_time) return velocity; // don't update velocity if deltaT is too small

    velocity = ( (float)(full_rotations - vel_full_rotations)*_2PI + (angle_prev - vel_angle_prev) ) / Ts;
    vel_angle_prev = angle_prev;
    vel_full_rotations = full_rotations;
    vel_angle_prev_ts = angle_prev_ts;
    return velocity;
}



void Sensor::init() {
    // initialize all the internal variables of Sensor to ensure a "smooth" startup (without a 'jump' from zero)
    get_sensor_angle(); // call once
    delayMicroseconds(1);
    vel_angle_prev = get_sensor_angle(); // call again
    vel_angle_prev_ts = time_us_64();
    delay(1);
    get_sensor_angle(); // call once
    delayMicroseconds(1);
    angle_prev = get_sensor_angle(); // call again
    angle_prev_ts = time_us_64();
}


float Sensor::get_mechanical_angle() {
    return angle_prev;
}



float Sensor::get_angle(){
    return (float)full_rotations * _2PI + angle_prev;
}



double Sensor::get_precise_angle() {
    return (double)full_rotations * (double)_2PI + (double)angle_prev;
}



int32_t Sensor::get_full_rotations() {
    return full_rotations;
}



int Sensor::needs_search() {
    return 0; // default false
}

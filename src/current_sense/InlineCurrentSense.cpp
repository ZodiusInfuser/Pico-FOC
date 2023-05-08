#include "InlineCurrentSense.h"
// InlineCurrentSensor constructor
//  - shunt_resistor  - shunt resistor value
//  - gain  - current-sense op-amp gain
//  - phA   - A phase adc pin
//  - phB   - B phase adc pin
//  - phC   - C phase adc pin (optional)
InlineCurrentSense::InlineCurrentSense(float _shunt_resistor, float _gain, int _pinA, int _pinB, int _pinC){
    pin_a = _pinA;
    pin_b = _pinB;
    pin_c = _pinC;

    shunt_resistor = _shunt_resistor;
    amp_gain  = _gain;
    volts_to_amps_ratio = 1.0f /_shunt_resistor / _gain; // volts to amps
    // gains for each phase
    gain_a = volts_to_amps_ratio;
    gain_b = volts_to_amps_ratio;
    gain_c = volts_to_amps_ratio;
};


InlineCurrentSense::InlineCurrentSense(float _mVpA, int _pinA, int _pinB, int _pinC){
    pin_a = _pinA;
    pin_b = _pinB;
    pin_c = _pinC;

    volts_to_amps_ratio = 1000.0f / _mVpA; // mV to amps
    // gains for each phase
    gain_a = volts_to_amps_ratio;
    gain_b = volts_to_amps_ratio;
    gain_c = volts_to_amps_ratio;
};



// Inline sensor init function
int InlineCurrentSense::init(){
    // if no linked driver its fine in this case 
    // at least for init()
    void* drv_params = driver ? driver->params : nullptr;
    // configure ADC variables
    params = _configureADCInline(drv_params,pin_a,pin_b,pin_c);
    // if init failed return fail
    if (params == SIMPLEFOC_CURRENT_SENSE_INIT_FAILED) return 0; 
    // calibrate zero offsets
    calibrateOffsets();
    // set the initialized flag
    initialized = (params!=SIMPLEFOC_CURRENT_SENSE_INIT_FAILED);
    // return success
    return 1;
}
// Function finding zero offsets of the ADC
void InlineCurrentSense::calibrateOffsets(){
    const int calibration_rounds = 1000;
    
    // find adc offset = zero current voltage
    offset_ia = 0;
    offset_ib = 0;
    offset_ic = 0;
    // read the adc voltage 1000 times ( arbitrary number )
    for (int i = 0; i < calibration_rounds; i++) {
        if(_isset(pin_a)) offset_ia += _readADCVoltageInline(pin_a, params);
        if(_isset(pin_b)) offset_ib += _readADCVoltageInline(pin_b, params);
        if(_isset(pin_c)) offset_ic += _readADCVoltageInline(pin_c, params);
        sleep_ms(1);
    }
    // calculate the mean offsets
    if(_isset(pin_a)) offset_ia = offset_ia / calibration_rounds;
    if(_isset(pin_b)) offset_ib = offset_ib / calibration_rounds;
    if(_isset(pin_c)) offset_ic = offset_ic / calibration_rounds;
}

// read all three phase currents (if possible 2 or 3)
PhaseCurrent_s InlineCurrentSense::get_phase_currents(){
    PhaseCurrent_s current;
    current.a = (!_isset(pin_a)) ? 0 : (_readADCVoltageInline(pin_a, params) - offset_ia)*gain_a;// amps
    current.b = (!_isset(pin_b)) ? 0 : (_readADCVoltageInline(pin_b, params) - offset_ib)*gain_b;// amps
    current.c = (!_isset(pin_c)) ? 0 : (_readADCVoltageInline(pin_c, params) - offset_ic)*gain_c; // amps
    return current;
}

// Function aligning the current sense with motor driver
// if all pins are connected well none of this is really necessary! - can be avoided
// returns flag
// 0 - fail
// 1 - success and nothing changed
// 2 - success but pins reconfigured
// 3 - success but gains inverted
// 4 - success but pins reconfigured and gains inverted
int InlineCurrentSense::driver_align(float voltage){
    
    int exit_flag = 1;
    if(skip_align) return exit_flag;

    if(_isset(pin_a)){
        // set phase A active and phases B and C down
        driver->set_pwm(voltage, 0, 0);
        sleep_ms(2000);
        PhaseCurrent_s c = get_phase_currents();
        // read the current 100 times ( arbitrary number )
        for (int i = 0; i < 100; i++) {
            PhaseCurrent_s c1 = get_phase_currents();
            c.a = c.a*0.6f + 0.4f*c1.a;
            c.b = c.b*0.6f + 0.4f*c1.b;
            c.c = c.c*0.6f + 0.4f*c1.c;
            sleep_ms(3);
        }
        driver->set_pwm(0, 0, 0);
        // align phase A
        float ab_ratio = c.b ? fabs(c.a / c.b) : 0;
        float ac_ratio = c.c ? fabs(c.a / c.c) : 0;
        if(_isset(pin_b) && ab_ratio > 1.5f ){ // should be ~2
            gain_a *= _sign(c.a);
        }else if(_isset(pin_c) && ac_ratio > 1.5f ){ // should be ~2
            gain_a *= _sign(c.a);
        }else if(_isset(pin_b) && ab_ratio < 0.7f ){ // should be ~0.5
            // switch phase A and B
            int tmp_pinA = pin_a;
            pin_a = pin_b;
            pin_b = tmp_pinA;
            float tmp_offsetA = offset_ia;
            offset_ia = offset_ib;
            offset_ib = tmp_offsetA;
            gain_a *= _sign(c.b);
            exit_flag = 2; // signal that pins have been switched
        }else if(_isset(pin_c) &&  ac_ratio < 0.7f ){ // should be ~0.5
            // switch phase A and C
            int tmp_pinA = pin_a;
            pin_a = pin_c;
            pin_c= tmp_pinA;
            float tmp_offsetA = offset_ia;
            offset_ia = offset_ic;
            offset_ic = tmp_offsetA;
            gain_a *= _sign(c.c);
            exit_flag = 2;// signal that pins have been switched
        }else{
            // error in current sense - phase either not measured or bad connection
            return 0;
        }
    }

    if(_isset(pin_b)){
        // set phase B active and phases A and C down
        driver->set_pwm(0, voltage, 0);
        sleep_ms(200);
        PhaseCurrent_s c = get_phase_currents();
        // read the current 50 times
        for (int i = 0; i < 100; i++) {
            PhaseCurrent_s c1 = get_phase_currents();
            c.a = c.a*0.6 + 0.4f*c1.a;
            c.b = c.b*0.6 + 0.4f*c1.b;
            c.c = c.c*0.6 + 0.4f*c1.c;
            sleep_ms(3);
        }
        driver->set_pwm(0, 0, 0);
        float ba_ratio = c.a ? fabs(c.b / c.a) : 0;
        float bc_ratio = c.c ? fabs(c.b / c.c) : 0;
        if(_isset(pin_a) && ba_ratio > 1.5f ){ // should be ~2
            gain_b *= _sign(c.b);
        }else if(_isset(pin_c) && bc_ratio > 1.5f ){ // should be ~2
            gain_b *= _sign(c.b);
        }else if(_isset(pin_a) && ba_ratio < 0.7f ){ // it should be ~0.5
            // switch phase A and B
            int tmp_pinB = pin_b;
            pin_b = pin_a;
            pin_a = tmp_pinB;
            float tmp_offsetB = offset_ib;
            offset_ib = offset_ia;
            offset_ia = tmp_offsetB;
            gain_b *= _sign(c.a);
            exit_flag = 2; // signal that pins have been switched
        }else if(_isset(pin_c) && bc_ratio < 0.7f ){ // should be ~0.5
            // switch phase A and C
            int tmp_pinB = pin_b;
            pin_b = pin_c;
            pin_c = tmp_pinB;
            float tmp_offsetB = offset_ib;
            offset_ib = offset_ic;
            offset_ic = tmp_offsetB;
            gain_b *= _sign(c.c);
            exit_flag = 2; // signal that pins have been switched
        }else{
            // error in current sense - phase either not measured or bad connection
            return 0;
        }   
    }

    // if phase C measured
    if(_isset(pin_c)){
        // set phase C active and phases A and B down
        driver->set_pwm(0, 0, voltage);
        sleep_ms(200);
        PhaseCurrent_s c = get_phase_currents();
        // read the adc voltage 500 times ( arbitrary number )
        for (int i = 0; i < 100; i++) {
            PhaseCurrent_s c1 = get_phase_currents();
            c.a = c.a*0.6 + 0.4f*c1.a;
            c.b = c.b*0.6 + 0.4f*c1.b;
            c.c = c.c*0.6 + 0.4f*c1.c;
            sleep_ms(3);
        }
        driver->set_pwm(0, 0, 0);
        float ca_ratio = c.a ? fabs(c.c / c.a) : 0;
        float cb_ratio = c.b ? fabs(c.c / c.b) : 0;
        if(_isset(pin_a) && ca_ratio > 1.5f ){ // should be ~2
            gain_c *= _sign(c.c);
        }else if(_isset(pin_b) && cb_ratio > 1.5f ){ // should be ~2
            gain_c *= _sign(c.c);
        }else if(_isset(pin_a) && ca_ratio < 0.7f ){ // it should be ~0.5
            // switch phase A and C
            int tmp_pinC = pin_c;
            pin_c = pin_a;
            pin_a = tmp_pinC;
            float tmp_offsetC = offset_ic;
            offset_ic = offset_ia;
            offset_ia = tmp_offsetC;
            gain_c *= _sign(c.a);
            exit_flag = 2; // signal that pins have been switched
        }else if(_isset(pin_b) && cb_ratio < 0.7f ){ // should be ~0.5
            // switch phase B and C
            int tmp_pinC = pin_c;
            pin_c = pin_b;
            pin_b = tmp_pinC;
            float tmp_offsetC = offset_ic;
            offset_ic = offset_ib;
            offset_ib = tmp_offsetC;
            gain_c *= _sign(c.b);
            exit_flag = 2; // signal that pins have been switched
        }else{
            // error in current sense - phase either not measured or bad connection
            return 0;
        }   
    }

    if(gain_a < 0 || gain_b < 0 || gain_c < 0) exit_flag +=2;
    // exit flag is either
    // 0 - fail
    // 1 - success and nothing changed
    // 2 - success but pins reconfigured
    // 3 - success but gains inverted
    // 4 - success but pins reconfigured and gains inverted

    return exit_flag;
}

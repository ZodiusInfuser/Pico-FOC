#ifndef HARDWARE_UTILS_DRIVER_H
#define HARDWARE_UTILS_DRIVER_H

#include "../common/foc_utils.h"
//#include "../communication/SimpleFOCDebug.h"
#include "../common/base_classes/BLDCDriver.h"


// these defines determine the polarity of the PWM output. Normally, the polarity is active-high,
// i.e. a high-level PWM output is expected to switch on the MOSFET. But should your driver design
// require inverted polarity, you can change the defines below, or set them via your build environment
// or board definition files.

// used for 1-PWM, 2-PWM, 3-PWM, and 4-PWM modes
#ifndef SIMPLEFOC_PWM_ACTIVE_HIGH
#define SIMPLEFOC_PWM_ACTIVE_HIGH true
#endif
// used for 6-PWM mode, high-side
#ifndef SIMPLEFOC_PWM_HIGHSIDE_ACTIVE_HIGH
#define SIMPLEFOC_PWM_HIGHSIDE_ACTIVE_HIGH true
#endif
// used for 6-PWM mode, low-side
#ifndef SIMPLEFOC_PWM_LOWSIDE_ACTIVE_HIGH
#define SIMPLEFOC_PWM_LOWSIDE_ACTIVE_HIGH true
#endif




// flag returned if driver init fails
#define SIMPLEFOC_DRIVER_INIT_FAILED ((void*)-1)

// generic implementation of the hardware specific structure
// containing all the necessary driver parameters
// will be returned as a void pointer from the _configurexPWM functions
// will be provided to the _writeDutyCyclexPWM() as a void pointer
typedef struct GenericDriverParams {
  int pins[6];
  long pwm_frequency;
  float dead_zone;
} GenericDriverParams;


/** 
 * Configuring PWM frequency, resolution and alignment
 * - Stepper driver - 2PWM setting
 * - hardware specific
 * 
 * @param pwm_frequency - frequency in hertz - if applicable
 * @param pin_a pin_a pwm pin
 * 
 * @return -1 if failed, or pointer to internal driver parameters struct if successful
 */
void* _configure_1_pwm(long pwm_frequency, const int pin_a);

/** 
 * Configuring PWM frequency, resolution and alignment
 * - Stepper driver - 2PWM setting
 * - hardware specific
 * 
 * @param pwm_frequency - frequency in hertz - if applicable
 * @param pin_a pin_a bldc driver
 * @param pin_b pin_b bldc driver
 * 
 * @return -1 if failed, or pointer to internal driver parameters struct if successful
 */
void* _configure_2_pwm(long pwm_frequency, const int pin_a, const int pin_b);

/** 
 * Configuring PWM frequency, resolution and alignment
 * - BLDC driver - 3PWM setting
 * - hardware specific
 * 
 * @param pwm_frequency - frequency in hertz - if applicable
 * @param pin_a pin_a bldc driver
 * @param pin_b pin_b bldc driver
 * @param pin_c pin_c bldc driver
 * 
 * @return -1 if failed, or pointer to internal driver parameters struct if successful
 */
void* _configure_3_pwm(long pwm_frequency, const int pin_a, const int pin_b, const int pin_c);

/** 
 * Configuring PWM frequency, resolution and alignment
 * - Stepper driver - 4PWM setting
 * - hardware specific
 * 
 * @param pwm_frequency - frequency in hertz - if applicable
 * @param pin1A pin1A stepper driver
 * @param pin1B pin1B stepper driver
 * @param pin2A pin2A stepper driver
 * @param pin2B pin2B stepper driver
 * 
 * @return -1 if failed, or pointer to internal driver parameters struct if successful
 */
void* _configure_4_pwm(long pwm_frequency, const int pin1A, const int pin1B, const int pin2A, const int pin2B);

/** 
 * Configuring PWM frequency, resolution and alignment
 * - BLDC driver - 6PWM setting
 * - hardware specific
 * 
 * @param pwm_frequency - frequency in hertz - if applicable
 * @param dead_zone  duty cycle protection zone [0, 1] - both low and high side low - if applicable
 * @param pinA_h pin_a high-side bldc driver 
 * @param pinA_l pin_a low-side bldc driver 
 * @param pinB_h pin_a high-side bldc driver 
 * @param pinB_l pin_a low-side bldc driver 
 * @param pinC_h pin_a high-side bldc driver 
 * @param pinC_l pin_a low-side bldc driver 
 * 
 * @return -1 if failed, or pointer to internal driver parameters struct if successful
 */
void* _configure_6_pwm(long pwm_frequency, float dead_zone, const int pinA_h, const int pinA_l,  const int pinB_h, const int pinB_l, const int pinC_h, const int pinC_l);

/** 
 * Function setting the duty cycle to the pwm pin (ex. analogWrite())
 * - Stepper driver - 2PWM setting
 * - hardware specific
 * 
 * @param dc_a  duty cycle phase A [0, 1]
 * @param dc_b  duty cycle phase B [0, 1]
 * @param params  the driver parameters
 */ 
void _write_duty_cycle_1_pwm(float dc_a, void* params);

/** 
 * Function setting the duty cycle to the pwm pin (ex. analogWrite())
 * - Stepper driver - 2PWM setting
 * - hardware specific
 * 
 * @param dc_a  duty cycle phase A [0, 1]
 * @param dc_b  duty cycle phase B [0, 1]
 * @param params  the driver parameters
 */ 
void _write_duty_cycle_2_pwm(float dc_a,  float dc_b, void* params);

/** 
 * Function setting the duty cycle to the pwm pin (ex. analogWrite())
 * - BLDC driver - 3PWM setting
 * - hardware specific
 * 
 * @param dc_a  duty cycle phase A [0, 1]
 * @param dc_b  duty cycle phase B [0, 1]
 * @param dc_c  duty cycle phase C [0, 1]
 * @param params  the driver parameters
 */ 
void _write_duty_cycle_3_pwm(float dc_a,  float dc_b, float dc_c, void* params);

/** 
 * Function setting the duty cycle to the pwm pin (ex. analogWrite())
 * - Stepper driver - 4PWM setting
 * - hardware specific
 * 
 * @param dc_1a  duty cycle phase 1A [0, 1]
 * @param dc_1b  duty cycle phase 1B [0, 1]
 * @param dc_2a  duty cycle phase 2A [0, 1]
 * @param dc_2b  duty cycle phase 2B [0, 1]
 * @param params  the driver parameters
 */ 
void _write_duty_cycle_4_pwm(float dc_1a,  float dc_1b, float dc_2a, float dc_2b, void* params);


/** 
 * Function setting the duty cycle to the pwm pin (ex. analogWrite())
 * - BLDC driver - 6PWM setting
 * - hardware specific
 * 
 * @param dc_a  duty cycle phase A [0, 1]
 * @param dc_b  duty cycle phase B [0, 1]
 * @param dc_c  duty cycle phase C [0, 1]
 * @param phase_state  pointer to PhaseState[3] array
 * @param params  the driver parameters
 * 
 */ 
void _write_duty_cycle_6_pwm(float dc_a,  float dc_b, float dc_c, PhaseState *phase_state, void* params);


#endif
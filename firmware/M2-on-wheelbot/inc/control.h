#ifndef UNIWHEEL_MAEVARM_M2_CONTROL
#define UNIWHEEL_MAEVARM_M2_CONTROL
/**************************************************************************
 *
 *  Header Files to include
 *
 **************************************************************************/
/* subystem header files */
#include "matrix_calculations.h"
#include "checks.h"

/* global program files */
#include "m_general.h"
#include "config.h"

/* External header files */
//#include <math.h>

/**************************************************************************
 *
 *  Function declarations
 *
 **************************************************************************/
void count_start(uint16_t *counter, uint16_t *counter_tmp, uint16_t *counts);
void count(uint16_t *counter, uint16_t *counter_tmp, uint16_t *counts);
(*counter_ptr)(uint16_t *counter, uint16_t *counter_tmp, uint16_t *counts);

float saturate_torque(float torque, float omega, float dCurrent);
float return_current_roll(float q1, float q1_d, float int_q5d, float q5d);
float return_current_pitch(float q2, float q2_d, float int_q4_d, float q4_d);

float test_motor(FlagsStruct *flags, uint16_t counter, float *time_test_start);

void feedforward_signal(float *current, bool* flag_ptr, uint16_t *counter, uint16_t *counter_tmp,
						uint16_t *counts, uint16_t *durations, float *controls,
						uint16_t ramp_length1, uint16_t ramp_length2);
#endif //UNIWHEEL_MAEVARM_M2_CONTROL
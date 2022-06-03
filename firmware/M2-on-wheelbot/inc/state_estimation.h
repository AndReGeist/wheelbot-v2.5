#ifndef UNIWHEEL_MAEVARM_M2_STATE_ESTIMATION
#define UNIWHEEL_MAEVARM_M2_STATE_ESTIMATION
/**************************************************************************
 *
 *  Header Files to include
 *
 **************************************************************************/
/* subystem header files */
#include "state_estimation.h"
#include "matrix_calculations.h"
#include "checks.h"

/* global program files */
#include "config.h"
#include "m_general.h"

/* External header files */
#include <math.h>

/**************************************************************************
 *
 *  Function declarations
 *
 **************************************************************************/
void estimate_pivot_acc(float *acc_pivot_B, float *q, float *dq, float ddq4, FlagsStruct *flags);
void estimate_pivot_acc_linearized(float *acc_pivot_B, float *q, float *dq, float ddq4, FlagsStruct *flags);

void  attitude_estimate_accel(float *q_acc, float *acc_pivot_B, int16_t *imu_data);
void compute_estimator_bias(FlagsStruct *flags, uint16_t counter, uint16_t *counter_tmp, float *roll_bias,
							float *roll_bias_sum, float *pitch_bias, float *pitch_bias_sum, float attitude[2]);

int16_t nope_func(int16_t new_value, int16_t prev_value, bool *imu_check);
int16_t nope_func2(int16_t new_value, int16_t prev_value, bool *imu_check);
int16_t nope_func3(int16_t new_value, int16_t prev_value);

float threshold_func(float value, float prev_value, float boundary);

//void  body_to_euler(float *T, float *body_rate);
float ApproxAtan(float z);
float ApproxAtan2(float y, float x);

#endif

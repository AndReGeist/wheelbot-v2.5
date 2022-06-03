#include "control.h"

float K_roll[4] = {4.5 , 0.25, 0.0003, 0.0018};

float K_pitch[4] = {1.6, 0.14, 0.04, 0.0344}; // great on tilted plane


void count_start(uint16_t *counter, uint16_t *counter_tmp, uint16_t *counts)
{
	*counter_tmp = *counter;
	*counts = *counter - *counter_tmp;
	counter_ptr = count;
}


void count(uint16_t *counter, uint16_t *counter_tmp, uint16_t *counts)
{
	*counts = *counter - *counter_tmp;
}


float saturate_current(float current)
{
	float output = 0;
	if ( current >= MAX_CURRENT_LQR ) {
		output = MAX_CURRENT_LQR;
	} else if ( current <= ( -1.0 * MAX_CURRENT_LQR ) ) {
		output = -1.0 * MAX_CURRENT_LQR;
	} else {
		output = current;
	}
	return output;
}


float return_current_roll(float q1, float q1_d, float int_q5d, float q5d)
{
	return (saturate_current((K_roll[0] * q1 + K_roll[1] * q1_d
			+ K_roll[2] * int_q5d + K_roll[3] * q5d) / MOTOR_TORQUE_CONSTANT));
} // End return_current_roll

float return_current_pitch(float q2, float q2_d, float int_q4_d, float q4_d)
{
	return (saturate_current((K_pitch[0] * q2 + K_pitch[1] * q2_d
		+ K_pitch[2] * int_q4_d + K_pitch[3] * q4_d) / MOTOR_TORQUE_CONSTANT));
} // End return_current_pitch


float test_motor(FlagsStruct *flags, uint16_t counter, float *time_test_start) {
	float time_in_sec = 0.0;
	float max_time = 10.0;
	float motor_current = 0.0;
	float max_current = 6.0;
	float singal_freq = 4.0;

    if ( (*flags).motor_test == FALSE )
    {
        (*flags).motor_test = TRUE;
        *time_test_start = counter / 100.0;
    }

	time_in_sec = ( counter / 100.0 ) - *time_test_start;

    if ( time_in_sec < max_time)
    {
		motor_current = max_current * sin( (PI * time_in_sec) / (2.0 * max_time) )
				* sin( (singal_freq * 10.0 * PI * time_in_sec) / (2.0 * max_time) )
				+ 3.0 * sin( (singal_freq * 100.0 * PI * time_in_sec) / (2.0 * max_time) );
    }
	else
	{
		motor_current = 0.0;
	}

	return motor_current; //motor_current;
} // End test_motor


void feedforward_signal(float *current, bool* flag_ptr,
			 uint16_t *counter, uint16_t *counter_tmp, uint16_t *counts,
			 uint16_t *durations, float *controls, uint16_t ramp_length1, uint16_t ramp_length2)
{

	counter_ptr( counter, counter_tmp, counts );

    if      ( *counts < durations[0] ) { *current = controls[0]; }
    else if ( *counts < durations[1] ) { *current = controls[1]; }
	else if ( *counts < durations[2] ) { *current = controls[2]; }
	//else if ( *counts < durations[3] ) { *current = controls[3]; }
	else if ( *counts <  ( durations[2] + ramp_length1 ) )
	{ *current = controls[3] * sin( (PI_2/ramp_length1) * (*counts - durations[2]) ); }
	else if ( *counts < durations[3] - ramp_length2 ) { *current = controls[3]; }
	else if ( *counts < durations[3] )
	{ *current = controls[3] * cos( (PI_2/ramp_length2) * (*counts - durations[3] + ramp_length2) ); }
	else if ( *counts <  ( durations[3] + 15 ) )
	{ *current = controls[4] * sin( (PI_2/15) * (*counts - durations[3]) ); }
	else if ( *counts < durations[4] ) { *current = controls[4]; }
    else
	{
		*current = 0.0;
		*flag_ptr = TRUE;
	}
}
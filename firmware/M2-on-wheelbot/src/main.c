/**************************************************************************
 *
 *  Header Files to include
 *
 **************************************************************************/
/* subystem header files */
#include "control.h"
#include "icm20948.h"
#include "m_bus.h"
#include "m_rf.h"
#include "m_spi.h"
#include "m_usb.h"
#include "matrix_calculations.h"
#include "state_estimation.h"
#include "udriver.h"
#include "checks.h"

/* global program files */
#include "config.h"
#include "m_general.h"

/* External header files */
#include <avr/wdt.h> // Watchdog
#include <string.h> // Used in check_crc

/**************************************************************************
 *
 *  Function declarations
 *
 **************************************************************************/
void init(); // prototype the initialization function
void setup_timer();
uint16_t select_timer1_prescaler();
uint16_t micros( bool stream );
void init_calibrate_udriver();
bool check_crc( uint8_t *data_in_ptr );
void data_write( uint8_t *packet, int index, float scaling, float data );
void read_usb( uint8_t *buffer );

 /**************************************************************************
 *
 *  Global Variables - these are the only non-stack RAM usage
 *
 **************************************************************************/
volatile bool tick = FALSE; // ISR tick when loop is executed
volatile bool new_data = FALSE; // ISR tick when wifi data_out is being received
uint16_t counter = 0;  // counter used to count control loops
uint16_t counter_lastwifi = 0;  // Last loop that wifi signal has been received
volatile uint8_t cycle_overflow_counter = 0;
Commstruct comm_ud; // Change scope to main and why volatile??

bool stream = TRUE;                                             // Start streaming input
bool transmitOK;

/**************************************************************************
 *
 *  Main function
 *
 **************************************************************************/
int main() {
	/* Error messages are stored in sys_flag
	 * 0: All good
	 * 1: Encoder 1 not properly initialized
	 * 2: Encoder 2 not properly initialized
	 * 3: Loop execution time is too long, check TICK (> 0.01 seconds)
	 * 4: Loop did not fully execute
	 * 5: Loop execution time is almost too long (>0.08 seconds)
	 * 6: Estimator calibration has been triggered
	 */
	uint16_t sys_flag = 0;
	float max_rate_motor2 = 50.0;  // Max rate in rad/s, 400 RPM

	/*
	 * usb_control: If TRUE, USB connection is used instead of RF
	 * use_IMUS: If FALSE, no IMU readings are obtained and control is switched off
	 */
	FlagsStruct flags = { .system = TRUE, .wifi = TRUE, .angle = TRUE, .rate = TRUE, .control = TRUE, .battery = TRUE,
						  .upright = FALSE, .standup1 = FALSE, .standup1_done = FALSE, .standup2 = FALSE,
						  .standup2_done = FALSE, .bias = FALSE , .loop_finished = TRUE, .LQRcontrol1 = FALSE, .LQRcontrol2 = FALSE,
						  .useLQR1 = FALSE, .useLQR2 = FALSE, .do_standup = FALSE, .do_rollup = FALSE,
						  .usb_control = FALSE, .use_IMUs = TRUE, .motor_test = FALSE,
						  .use_pivotAcc_CP = TRUE, .use_pivotAcc_WC_vel = FALSE, .use_pivotAcc_WC_acc = FALSE};

	uint16_t battery_voltage;
	uint16_t tick_time = 0; // Check time between ticks
	uint16_t loop_time = 0; // Check time of control loop

	float time_test_start = 0.0; // Variable used in function "test_motor" to store start time of motor test

	////////////////////////////////////////
	// uDRIVER - MOTOR CONTROL VARIABLES
	////////////////////////////////////////
	Commstruct comm_ud = { .currentLimit1=0.0, .currentTargetMotor1=0.0, .currentLimit2=0.0, .currentTargetMotor2=0.0,
						   .currentMotor1=0.0, .positionMotor1=0.0, .velocityMotor1=0.0, .currentMotor2=0.0,
						   .positionMotor2=0.0, .velocityMotor2=0.0, .status_errcode=0, .status_SE=FALSE, .status_M1E=FALSE,
						   .status_M1R=FALSE, .status_IDX1D=FALSE, .status_IDX1T=FALSE, .status_M2E=FALSE, .status_M2R=FALSE,
						   .status_IDX2D=FALSE, .status_IDX2T=FALSE };

	////////////////////////////////////////
	// IMU VARIABLES
	////////////////////////////////////////
	a_range_t acc_range = ACCEL_2G;
	g_range_t gyro_range = GYRO_500DPS;  //g_range_t gyro_range = GYRO_2000DPS;
	float accel_scaling = 0.000598755; // ACCEL_2G transforms bit in m/s2  //float accel_scaling = 0.00119751; // ACCEL_4G
	float gyro_scaling = 0.0002663161;  // Transform bit to max 500°/s in rad/s

	m2_gpio_t imu_list[NUM_SENSORS] = {IMU_A, IMU_B, IMU_C, IMU_D}; // Chip select pins for each sensor
	float IMUx[NUM_SENSORS] = {-PI/2, -PI/2, PI/2, PI/2};
	float IMUz[NUM_SENSORS] = {(-3*PI)/4, PI/4, -PI/4, (3*PI)/4};
	int32_t R_b_IMU[3][3] = {{0.0}};

	int16_t accel_adjust[NUM_SENSORS][3] = {0}; // bias adjust for accel
	int16_t imu_acc[3] = {0};
	//int16_t imu_acc_C[3] = {0};
	int32_t imu_acc_tmp[3] = {0};

	int16_t imu_gyro[3] = {0};
	int32_t imu_gyro_tmp[3] = {0};

	int16_t imu_data[NUM_SENSORS][NUM_IMU_DATA] = {0}; // data in the body frame

	float body_rate[NUM_DIM] = {0}; // Gyroscope body rate estimate in body frame
	//float euler_rate[NUM_DIM] = {0}; // Gyroscope body rate estimate as Euler rates DELETE

	////////////////////////////////////////
	// VARIABLES FOR ESTIMATOR
	////////////////////////////////////////
	float roll_bias = 0.07; //0.05; //0,0043 // -0.0124; //-0.02; //0.0; //0.00331; //0.00874; //-0.04; //-0.03; //-0.0416; //-0.005; //-0.01; //-0.048; //0.026;
	float pitch_bias = 0.00;//0.017; //-0.0523; //−0,0863//0.0173; //-0.05; //-0.05; //-0.037; //-0.04; //-0.062; //-0.031;

	float roll_bias_sum = 0.0;
	float pitch_bias_sum = 0.0;

	float q[5] = {INIT_ANGLE, 0.0, 0.0, 0.0, 0.0};  // Initial Roll and Pitch angle
	float q_acc[2] = {INIT_ANGLE, 0.0};  // Attitude est. from accelerometer measurements, set to zero before calibration
    float q_acc_nopivot[2] = {INIT_ANGLE, 0.0}; // DEBUG - DELETE
	float dq[5] = {INIT_ANGLE, 0.0, 0.0, 0.0, 0.0};  // Initial Roll and Pitch angle
	float dq4_prev = 0.0;

	float ddq4_est = 0.0;
	float acc_pivot_B[3] = {0.0, 0.0, 0.0};
	float acc_pivot_B0[3] = {0.0, 0.0, 0.0}; // DEBUG - DELETE
	float T[3][3] = {0};                                        // Transformation matrix from body rate to euler rate
	T[0][0] = cos(q[1]);
	T[0][1] = 0.0;
	T[0][2] = sin(q[1]);
	T[1][0] = 0.0;
	T[1][1] = 1;
	T[1][2] = 0;
	T[2][0] = -cos(q[0])*sin(q[1]);
	T[2][1] = sin(q[0]);
	T[2][2] = cos(q[0])*cos(q[1]);

	int16_t acc_ref0[NUM_SENSORS][NUM_DIM] = {0}; // Array to store reference acceleration in IMU frames to get bias in calibration
	uint16_t counter_tmp = 0;
	uint16_t counts = 0;
	counter_ptr = count_start;

	////////////////////////////////////////
	// VARIABLES FOR WIFI
	////////////////////////////////////////
	uint8_t data_in[PACKET_LENGTH_IN] = {0}; // data coming from PC
	uint8_t data_in_prev[PACKET_LENGTH_IN] = {0}; // data coming from PC
	uint8_t data_out[PACKET_LENGTH] = {0}; // data sent to PC
	uint8_t data_in0 = 0;

	uint32_t *crc_out_ptr = (uint32_t *)&data_out[28];
	bool crc_in_OK = TRUE;

	float scaling_attitude = 32767.0 / PI;
	float scaling_current = 32767.0 / 40;
	float scaling_rate = (32767.0 / 3000) * (60 / (2 * PI)); // Motorrate is measured in rad/s, max rate 3000 RPM
	float init_q5_est = 0.0;
	float init_q4_est = 0.0;

	uint16_t durations[5] = {0};
	float controls[5] = {0.0};
	float* motor_ptr;
	bool* flag_ptr;
	uint16_t ramp_length1 = 0;
	uint16_t ramp_length2 = 5;


	// ***************************************
	// ININITILIZATION
	// ***************************************
	init();

	// UDRIVER INIT
	init_calibrate_udriver(&comm_ud); // initialize and calibrate BLDC motor

	if ( flags.use_IMUs == TRUE ){

		start1: // initialize each IMU with same settings
		for ( int i = 0; i < NUM_SENSORS; ++i ) {
			if ( icm20948_init( imu_list[i], acc_range, gyro_range ) ) {
				m_green( ON );
				m_wait( INIT_WAIT );
				m_green( OFF );
			} else {
				m_red( TOGGLE );
				m_wait( 5 * INIT_WAIT );
				goto start1;
			}
		} // end for

		// SPI speed reset, somehow IMU init. takes different SPI_MHZ (?)
		m_spi_speed(SPI_4MHZ);
		//m_spi_speed(SPI_1MHZ);
		//m_spi_speed(SPI_500KHZ);
		//m_spi_speed( SPI_125KHZ );

		// measure accelerometer bias. can be stored in hardware for the future
		acc_ref0[0][0] = 5793;  // x, initial acceleration in IMU frame taken from simulation
		acc_ref0[0][1] = -14189; // y, Note that 1g = 16384
		acc_ref0[0][2] = 5793;  // z

		acc_ref0[1][0] = -5793;   // x
		acc_ref0[1][1] = -14189; // y
		acc_ref0[1][2] = -5793; // z

		acc_ref0[2][0] = 5793; // x
		acc_ref0[2][1] = 14189; // y
		acc_ref0[2][2] = 5793; // z

		acc_ref0[3][0] = -5793;  // x
		acc_ref0[3][1] = 14189; // y
		acc_ref0[3][2] = -5793;  // z

		icm20948_calib_accel(&imu_list[0], &accel_adjust[0][0], acc_ref0[0]);
		icm20948_calib_accel(&imu_list[1], &accel_adjust[1][0], acc_ref0[1]);
		icm20948_calib_accel(&imu_list[2], &accel_adjust[2][0], acc_ref0[2]);
		icm20948_calib_accel(&imu_list[3], &accel_adjust[3][0], acc_ref0[3]);
		 } // end if

	start2: // wait until python interface sends "Start wheelbot" signal
	if (data_in[0] == 'i') {
		m_green( ON );
	} else {

		if (flags.usb_control == TRUE) {
			if( m_usb_rx_available() )
			{
				read_usb( &data_in[0] );
	        }
			data_out[PACKET_LENGTH-2] = 0;
			data_out[PACKET_LENGTH-1] = 0; // If !crc_2PC_OK set 'crc2' to 1
			m_usb_tx_char('X');
			usb_serial_write(data_out, PACKET_LENGTH);
		}
		else
		{
			if ( new_data == TRUE )
			{
				new_data = FALSE;
				m_rf_read( (char *) data_in, PACKET_LENGTH_IN );
				check_crc( data_in );
			}
			*crc_out_ptr = m_crc32( (uint8_t *)data_out, 28 );
			m_rf_send( TXADDRESS, (char *) data_out, PACKET_LENGTH );
		}

		m_green( TOGGLE );
		m_red( TOGGLE );
		m_wait( INIT_WAIT );
		goto start2;
	}

  tick_time = micros(FALSE);
  m_red( OFF ); // phew, we made it
  m_green( OFF );

  // ******************************************************************************
  //
  // WHILE-LOOP
  //
  // ******************************************************************************

  while (1) {

	if (tick) {
	  tick_time = micros(FALSE);
	  ++counter;
	  tick = FALSE;
	  check_tick_time(&flags, &sys_flag, &tick_time);


      // ***************************************
	  // GET MOTOR RATES
	  // ***************************************
	  dq[3] = 0.3 * comm_ud.velocityMotor2 + 0.7 * dq[3]; // LOWER WHEEL
	  dq[4] = 0.3 * comm_ud.velocityMotor1 + 0.7 * dq[4]; // UPPER WHEEL

	  q[3] += (dq[3] / (float)LOOP_FREQ); // Integrate rates
	  q[4] += (dq[4] / (float)LOOP_FREQ);

	  //ddq4_est = 0.9 * (dq[3] - dq4_prev) * LOOP_FREQ + 0.1 * ddq4_est; // LP filter derivative of dq[3]
	  //ddq4_est = (dq[3] - dq4_prev) * LOOP_FREQ; // Derivative of dq[3]
	  //dq4_prev = dq[3];
	  //ddq4_est = 0.8 * (comm_ud.velocityMotor2 - dq4_prev) * LOOP_FREQ + 0.2 * ddq4_est; // LP filter derivative of dq_4
	  ddq4_est = (comm_ud.velocityMotor2 - dq4_prev) * LOOP_FREQ; // Derivative of dq_4
	  dq4_prev = comm_ud.velocityMotor2;

	  if ( flags.use_IMUs == TRUE )
	  {

		// ***************************************
		// GET IMU data, REMOVE ACC BIAS, TRANSFORM TO BODY-FIXED FRAME
		// ***************************************
		for (int i = 0;  i < NUM_SENSORS; ++i) {
			icm20948_get_raw_6dof(imu_list[i], imu_acc, imu_gyro);

			for (int j = 0;  j < 3; ++j) {
				imu_acc[j] -= accel_adjust[i][j];
			}

			rot_b_IMU(R_b_IMU, IMUx[i], IMUz[i]); // Compute body-frame transform scaled by 10000
		    multiply_matrix_vector_long(3, 3, R_b_IMU, imu_acc, imu_acc_tmp);
		    multiply_matrix_vector_long(3, 3, R_b_IMU, imu_gyro, imu_gyro_tmp);

			for (int j = 0;  j < 3; ++j) {
				imu_data[i][j] = (int)( imu_acc_tmp[j] / 10000 ); // the entries of rot_b_IMU are scaled by 10000
				imu_data[i][j+3] = (int)( imu_gyro_tmp[j] / 10000 );
			}
		}

		body_rate[0] =
		    (float)( imu_data[0][3] * 0.25 + imu_data[1][3] * 0.25 + imu_data[2][3] * 0.25 + imu_data[3][3] * 0.25 ) * gyro_scaling; // x
		body_rate[1] =
		    (float)( imu_data[0][4] * 0.25 + imu_data[1][4] * 0.25 + imu_data[2][4] * 0.25 + imu_data[3][4] * 0.25 ) * gyro_scaling; // y
		body_rate[2] =
		    (float)( imu_data[0][5] * 0.25 + imu_data[1][5] * 0.25 + imu_data[2][5] * 0.25 + imu_data[3][5] * 0.25) * gyro_scaling; // z


		// ***************************************
		// TRANSFORM BODY RATE TO EULER RATES
		// ***************************************
		// Transformation matrix from body rate to Euler rate
		// Note: Euler rate defined here as [Roll, Pitch, Yaw]
		T[0][0] = cos(q[1]);
		T[0][1] = 0.0;
		T[0][2] = sin(q[1]);
		T[1][0] = 0.0;
		T[1][1] = 1;
		T[1][2] = 0;
		T[2][0] = -cos(q[0])*sin(q[1]);
		T[2][1] = sin(q[0]);
		T[2][2] = cos(q[0])*cos(q[1]);

		multiply_matrix_vector(3, 3, T, body_rate, dq);


		// ***************************************
		// GET ATTITUDE ESTIMATE FROM GYRO
		// ***************************************
		q[0] += (dq[0] / LOOP_FREQ);
		q[1] += (dq[1] / LOOP_FREQ);


		// ***************************************
		// ESTIMATE STATE
		// ***************************************
		estimate_pivot_acc_linearized(acc_pivot_B, q, dq, ddq4_est, &flags);  // Compute acc_pivot_B
		attitude_estimate_accel(q_acc,  acc_pivot_B, &imu_data[0][0]); // Compute q_acc

		//attitude_estimate_accel(q_acc_nopivot,  acc_pivot_B0, &imu_data[0][0]);

		q[0] = FUSION_TUNING_PARAMETER * q_acc[0] + (1 - FUSION_TUNING_PARAMETER) * q[0] ;
		q[1] = FUSION_TUNING_PARAMETER * q_acc[1] + (1 - FUSION_TUNING_PARAMETER) * q[1] ;

	  } // end if


      // ***************************************
      // EXECUTE COMMAND e.g. compute control
      // ***************************************

	  // LQR CONTROL
	  if ( data_in[0] == 'C' )
	  {
			  flags.useLQR1 = TRUE;
			  flags.useLQR2 = TRUE;
	  }
	  // Two-step standup
	  else if ( data_in[0] == 'S' || data_in[0] == 's')
	  {
        flags.do_standup = TRUE;
		motor_ptr = &comm_ud.currentTargetMotor1;

		if ( data_in[0] == 's' ) { flags.standup1_done = TRUE; }

		if ( !flags.standup1_done && ( fabs(q[0]) > 0.072 ) && (comm_ud.velocityMotor1 < 220) )
		{
				flag_ptr = &flags.standup1_done;
				durations[0] = 1; durations[1] = 100; durations[2] =120; durations[3] = 150; durations[4] = 0;
				controls[0] = 0.0; controls[1] = -5.0; controls[2] = -0.0; controls[3] = 15.56; controls[4] = 0;
		}
		else if ( !flags.standup2_done )
		{
				if (flags.standup2 = FALSE)
				{
					flags.standup2 = TRUE;
					flags.standup1_done = TRUE;
					counter_ptr = count_start; // Resets count function
					counts = 0; // Resets counts just to be sure
				}
				ramp_length2 = 10;
				flag_ptr = &flags.standup2_done;
				durations[0] = 150; durations[1] = 250; durations[2] = 310; durations[3] = 340; durations[4] = 0;
				controls[0] = 0.0; controls[1] = -5.0;  controls[2] = -0.5; controls[3] = 14.53; controls[4] = 0;

				if (q[0] < 0.1 && counts > 250) { flags.standup2_done = TRUE; flags.useLQR1 = TRUE; }
				if (fabs(q[0]) < 0.3 && counts > 250) { flags.useLQR2 = TRUE; }
		}
		else
		{
				flags.do_standup = FALSE;
				flags.standup2_done = TRUE;
				flags.useLQR1 = TRUE;
				flags.useLQR2 = TRUE;
		}
	  }
      // Two-step roll-up
	  else if ( data_in[0] == 'U' || data_in[0] == 'u')
	  {
        flags.do_standup = TRUE;
		motor_ptr = &comm_ud.currentTargetMotor2;

		if ( data_in[0] == 'u' ) { flags.standup1_done = TRUE; }

		if ( !flags.standup1_done || comm_ud.velocityMotor2 > 270 )
		{
				ramp_length2 = 10;
				max_rate_motor2 = MAX_RATE_MOTOR1;
				flag_ptr = &flags.standup1_done;
				durations[0] = 1; durations[1] = 100; durations[2] = 120; durations[3] =155; durations[4] = 195;
				controls[0] = 0.0; controls[1] = -5.0, controls[2] = 0.0; controls[3] = 14.93; controls[4] = -5.6;
		}
		else if ( !flags.standup2_done )
		{
				if (flags.standup2 = FALSE)
				{
					flags.standup2 = TRUE;
					flags.standup1_done = TRUE;
					counter_ptr = count_start; // Resets count function
					counts = 0; // Resets counts just to be sure
					max_rate_motor2 = 50;
				}
				ramp_length2 = 4;
				flag_ptr = &flags.standup2_done;
				durations[0] = 1; durations[1] = 2; durations[2] = 3;  durations[3] = 350;  durations[4] = 360;
				controls[0] = 0.0;  controls[2] = 0.0; controls[2] = 0.0;  controls[3] = 0.0;  durations[4] = 6.0;

				if (fabs(q[1]) < 0.3 && counts > 250) { flags.useLQR1 = TRUE; }
				if (fabs(q[1]) < 0.05 && counts > 250) { flags.standup2_done = TRUE; flags.useLQR2 = TRUE; }
		}
		else
		{
				max_rate_motor2 = 50;
				flags.do_standup = FALSE;
				flags.standup2_done = TRUE;
				flags.useLQR1 = TRUE;
				flags.useLQR2 = TRUE;
		}
	  }
      // CALIBRATE ESTIMATOR
      else if (data_in[0] == 'A')
      {
		  sys_flag = 6;
		  flags.useLQR1 = TRUE;
		  flags.useLQR2 = TRUE;

		  // Estimate bias
		  compute_estimator_bias(&flags, counter, &counter_tmp, &roll_bias, &roll_bias_sum,
								 &pitch_bias, &pitch_bias_sum, q );
	  }
	  // SIMPLE MOTOR COMMANDS
	  else if (data_in[0] == 'M')
	  {
		      comm_ud.currentTargetMotor1 = 1.0;
		      comm_ud.currentTargetMotor2 = 0.0;
	  }
	  else if (data_in[0] == 'm') {
		    comm_ud.currentTargetMotor1 = -1.0;
		    comm_ud.currentTargetMotor2 = 0.0;
	  }
	  else if (data_in[0] == 'T') {
		    comm_ud.currentTargetMotor1 = 0.0;
		    comm_ud.currentTargetMotor2 = 1.0;
	  }
	  else if (data_in[0] == 't') {
		    comm_ud.currentTargetMotor1 = 0.0;
		    comm_ud.currentTargetMotor2 = -1.0;
	  }
	  else if (data_in[0] == 'K') {
			comm_ud.currentTargetMotor1 = 1.5 * test_motor(&flags, counter, &time_test_start); 	// TEST MOTOR CONTROL
	  }

    // JUMP
    if ( flags.do_standup ) { feedforward_signal(motor_ptr, flag_ptr, &counter, &counter_tmp, &counts, durations, controls, ramp_length1, ramp_length2); }

	// ROLL CONTROLLER
	if (flags.useLQR1)
	{
		if (flags.LQRcontrol1 == FALSE) { flags.LQRcontrol1 = TRUE; init_q5_est = q[4]; }
		comm_ud.currentTargetMotor1 = return_current_roll(q[0]-roll_bias, dq[0], q[4] - init_q5_est, dq[4]);
	}

	// PITCH CONTROLLER
	if (flags.useLQR2)
	{
		if (flags.LQRcontrol2 == FALSE) { flags.LQRcontrol2 = TRUE; init_q4_est = q[3]; }
		comm_ud.currentTargetMotor2 = return_current_pitch(q[1]-pitch_bias, dq[1], q[3] - init_q4_est, dq[3]);
	}

	// ***************************************
	// CHECK SAFETY AND SEND CONTROL
	// ***************************************
	//reset_checks(&flags, &comm_ud, &data_in[0]);
	check_control(&flags, &comm_ud);
	check_rates(&flags, &comm_ud, max_rate_motor2);
	//check_battery(&flags, &battery_voltage);
	check_upright(&flags, &comm_ud, q);

	if ( flags.usb_control ){ check_wifi(&flags, counter, counter_lastwifi); }

	flags.system = flags.system && flags.wifi && flags.angle && flags.rate && flags.control && flags.battery;

	if (!flags.system || data_in[0] == 'e')
	{
	  comm_ud.currentTargetMotor1 = 0.0;
	  comm_ud.currentTargetMotor2 = 0.0;
	  send_spi_udriver(&comm_ud);
	  m_red( ON );
	}
	else if (flags.system)
	{
	  send_spi_udriver(&comm_ud); // Send current to motor
	}


	// ***************************************
	// USB / WIFI COMMUNICATION
	// ***************************************
	#if DATA_MODE == 1 // GET DATA FOR CONTROL PLOTS
		data_write(data_out, 0, scaling_attitude, q[0] - roll_bias);  // roll, q[0] - roll_bias, q_acc[0]
		data_write(data_out, 2, scaling_attitude, q[1] - pitch_bias);  // pitch, q[1] - pitch_bias, q_acc[1]
		data_write(data_out, 4, 1/gyro_scaling, dq[0]); // roll_rate
		data_write(data_out, 6, 1/gyro_scaling, dq[1]); // pitch_rate
		data_write(data_out, 8, scaling_current, comm_ud.currentTargetMotor1); // motor 1 current sent
		data_write(data_out, 10, scaling_current, comm_ud.currentTargetMotor2); // motor 2 current sent
		data_write(data_out, 12, scaling_rate, dq[4]); // motor 1 rate - comm_ud.velocityMotor1
		data_write(data_out, 14, scaling_rate, dq[3]); // motor 2 rate - comm_ud.velocityMotor2

		#if FLAG_PLOT == 1 // MISC
		  data_write(data_out, 16, scaling_current, comm_ud.currentTargetMotor2); // DEBUG 1
		  data_write(data_out, 18, 32767/200.0, acc_pivot_B[0]); // DEBUG 2
		  data_write(data_out, 20, scaling_rate, dq4_prev); // DEBUG 3
		#endif

		#if FLAG_PLOT == 2 // SYS FLAG + MOTOR POS
		  data_write(data_out, 16, 1.0, sys_flag); // DEBUG 1
		  data_write(data_out, 18, 32767.0 / (10000*PI), q[4] - init_q5_est); // DEBUG 2
		  data_write(data_out, 20, 32767.0 / (10000*PI), q[3] - init_q4_est); // DEBUG 3
		#endif

		#if FLAG_PLOT == 3
		  data_write(data_out, 16, 1.0, sys_flag); // DEBUG 1
		  data_write(data_out, 18, 0.5, loop_time); // DEBUG 2
		  data_write(data_out, 20, 0.5, tick_time); // DEBUG 3
		#endif

		#if FLAG_PLOT == 6 // PIVOT ACC
		  data_write(data_out, 16, 32767/200.0, acc_pivot_B[0]); // DEBUG 1
		  data_write(data_out, 18, scaling_rate, comm_ud.velocityMotor1); // DEBUG 2
		  data_write(data_out, 20, scaling_current, comm_ud.currentTargetMotor2); // DEBUG 3
		#endif

		#if FLAG_PLOT == 7  // ESTIMATOR BIAS
		  data_write(data_out, 16, 1.0, sys_flag); // DEBUG 1
		  data_write(data_out, 18, 32767/1.8, roll_bias); // DEBUG 2
		  data_write(data_out, 20, 32767/1.8, pitch_bias); // DEBUG 3
		#endif

		#if FLAG_PLOT == 8  // DATA IN
		  data_write(data_out, 16, 1.0, data_in[0]); // DEBUG 1
		  data_write(data_out, 18, 1.0, data_in[1]); // DEBUG 2
		  data_write(data_out, 20, 1.0, data_in[9]); // DEBUG 3
		#endif

		#if FLAG_PLOT == 9 // DATA IN
		  data_write(data_out, 16, 1.0, sys_flag); // DEBUG 1
		  data_write(data_out, 18, scaling_current, comm_ud.currentMotor1); // DEBUG 2
		  data_write(data_out, 20, scaling_current, comm_ud.currentMotor2); // DEBUG 3
		#endif

		#if FLAG_PLOT == 10 // DATA IN
		  data_write(data_out, 16, 1/gyro_scaling, dq[2]); // DEBUG 1
		  data_write(data_out, 18, scaling_attitude, q_acc[0]); // DEBUG 2
		  data_write(data_out, 20, scaling_attitude, q_acc[1]); // DEBUG 3
		#endif

		#if FLAG_PLOT == 11 // DATA IN
		  data_write(data_out, 16, 1/accel_scaling, acc_pivot_B[0]); // DEBUG 1
		  data_write(data_out, 18, 1/accel_scaling, acc_pivot_B[1]); // DEBUG 2
		  data_write(data_out, 20, 1/accel_scaling, acc_pivot_B[2]); // DEBUG 3
		#endif

		#if FLAG_PLOT == 12  // DATA IN
		  data_write(data_out, 16, scaling_attitude, q_acc_nopivot[0]); // DEBUG 1
		  data_write(data_out, 18, scaling_attitude, q_acc_nopivot[1]); // DEBUG 2
		  data_write(data_out, 20, scaling_attitude, q_acc_nopivot[2]); // DEBUG 3
		#endif

	    #if FLAG_PLOT == 13  // DATA IN
		  data_write(data_out, 16, 1,  imu_data[3][0]); // DEBUG 1
		  data_write(data_out, 18, 1/accel_scaling, acc_pivot_B[0]); // DEBUG 2
		  data_write(data_out, 20, scaling_rate, comm_ud.velocityMotor2); // DEBUG 3
		#endif
	#endif //END DATA_MODE == 1

	#if DATA_MODE == 2 // ANALYSE PIVOT ACC ESTIMATOR
		data_write(data_out, 0, scaling_attitude, q_acc[0] );
		data_write(data_out, 2, scaling_attitude, q_acc[1] );
		data_write(data_out, 4, scaling_attitude, q_acc_nopivot[0] );
		data_write(data_out, 6, scaling_attitude, q_acc_nopivot[1] );
		data_write(data_out, 8, scaling_rate, dq[3]);
		data_write(data_out, 10, scaling_rate, comm_ud.velocityMotor2);
		data_write(data_out, 12, 1, imu_data[2][0]);
		data_write(data_out, 14, 1, imu_data[3][0]);
		data_write(data_out, 16, 1, imu_data[2][1]);
		data_write(data_out, 18, 1/accel_scaling, acc_pivot_B[0]);
		data_write(data_out, 20, 1/accel_scaling, acc_pivot_B[1]);
	#endif

	#if DATA_MODE == 3 // ANALYSE IMU ACC VALUES
		data_write(data_out, 0, 1, imu_data[0][0]);
		data_write(data_out, 2, 1, imu_data[0][1]);
		data_write(data_out, 4, 1, imu_data[0][2]);
		data_write(data_out, 6, 1, imu_data[1][0]);
		data_write(data_out, 8, 1, imu_data[1][1]);
		data_write(data_out, 10, 1, imu_data[1][2]);
		data_write(data_out, 12, 1, imu_data[2][0]);
		data_write(data_out, 14, 1, imu_data[2][1]);
		data_write(data_out, 16, 1, imu_data[2][2]);
		data_write(data_out, 18, 1, imu_data[3][0]);
		data_write(data_out, 20, 1, imu_data[3][1]);
	#endif

	#if DATA_MODE == 4 // ANALYSE IMU ACC VALUES
		data_write(data_out, 0, 1, imu_data[0][3]);
		data_write(data_out, 2, 1, imu_data[0][4]);
		data_write(data_out, 4, 1, imu_data[0][5]);
		data_write(data_out, 6, 1, imu_data[1][3]);
		data_write(data_out, 8, 1, imu_data[1][4]);
		data_write(data_out, 10, 1, imu_data[1][5]);
		data_write(data_out, 12, 1, imu_data[2][3]);
		data_write(data_out, 14, 1, imu_data[2][4]);
		data_write(data_out, 16, 1, imu_data[2][5]);
		data_write(data_out, 18, 1, imu_data[3][3]);
		data_write(data_out, 20, 1, imu_data[3][4]);
	#endif

	data_out[22] = ( comm_ud.status_SE <<0 | comm_ud.status_M1R <<1 | comm_ud.status_M2R <<2 |\
	            comm_ud.status_M1E <<3 | comm_ud.status_M2E <<4 | ( comm_ud.status_IDX1D & comm_ud.status_IDX2D ) <<5|\
	            ((comm_ud.status_errcode >> 0) & 1) <<6 | ((comm_ud.status_errcode >> 1) & 1) <<7); // Safe system info bools in char
	data_out[23] = (flags.system <<0 | flags.angle <<1 | flags.rate <<2 |\
	            flags.control <<3 | flags.battery <<4 | flags.wifi <<5|\
	            flags.standup1_done <<6 | flags.standup2_done <<7); // Safe system info bools in char


	data_out[24] = *(char *)&battery_voltage;
	data_out[25] = *(((char *)&battery_voltage) + 1);
	data_out[26] = *(char *)&counter;
	data_out[27] = *(((char *)&counter) + 1);

    /* SEND DATA */
    if ( flags.usb_control )
	{
		if( m_usb_rx_available() )
		{
			read_usb( &data_in[0] );
	    }
		// COMPUTE CRC AND SEND DATA TO PC
		*crc_out_ptr = m_crc32( (uint8_t *)data_out, PACKET_LENGTH-4 );
		data_out[PACKET_LENGTH-2] = 0;
		data_out[PACKET_LENGTH-1] = 0; // If !crc_2PC_OK set 'crc2' to 1
		m_usb_tx_char('X');
		usb_serial_write( data_out, PACKET_LENGTH );
	}
	else
	{
		if ( new_data )
		{
			// READ INCOMING WIFI MESSAGE
			new_data = FALSE;
			counter_lastwifi = counter;
			m_green( TOGGLE );

			data_in0 =  data_in[0];
			m_rf_read( (char *) data_in, PACKET_LENGTH_IN );

			//if ( (counter % 100) == 0 ){ data_in[4] = 'J'; } // DEBUG - Test CRC
			crc_in_OK = check_crc( data_in );
			//if( !crc_in_OK ){ memset( data_in, 0, PACKET_LENGTH_IN ); }
			if( !crc_in_OK ){ data_in[0] = data_in0; } // Set
		}

		// COMPUTE CRC AND SEND DATA TO PC
		*crc_out_ptr = m_crc32( (uint8_t *)data_out, PACKET_LENGTH-4 );
		m_rf_send( TXADDRESS, (char *) data_out, PACKET_LENGTH );
	}

	  // CHECK LOOP TIMES
	  sys_flag = 0;

	  loop_time = micros(FALSE);
	  flags.loop_finished = TRUE;
	  check_loop_time( &sys_flag, loop_time );

	  } // end if(tick)
  } // end while()
} // end main


/**************************************************************************
 *
 *  Interrupt Service routines
 *
 **************************************************************************/
ISR(INT2_vect) // a command telegram coming in
{
	new_data = TRUE;
}

ISR(TIMER1_COMPA_vect) // fires approximately every 1/LOOP_FREQ [s]
{
  tick = TRUE;
}

ISR(TIMER3_OVF_vect) {
	cycle_overflow_counter++;
}


/**************************************************************************
 *
 *  Functions
 *
 **************************************************************************/

//char checksum(int *ptr, uint16_t  sz) {
//	char chk = 0;
//	while (sz-- != 0)
//		chk -= *ptr++;
//	return chk;
//}

bool check_crc( uint8_t *data_in_ptr )
{ // Check crc of incoming message
	uint8_t crc_in[4] = {0};
	uint32_t *crc_in_ptr = (uint32_t *)&crc_in[0];

	*crc_in_ptr = m_crc32( data_in_ptr, PACKET_LENGTH_IN-4 );

	bool crc_OK;
	crc_OK = (( crc_in[0] == data_in_ptr[PACKET_LENGTH_IN-4] )
			  && ( crc_in[1] == data_in_ptr[PACKET_LENGTH_IN-3] )
			  && ( crc_in[2] == data_in_ptr[PACKET_LENGTH_IN-2] )
			  && ( crc_in[3] == data_in_ptr[PACKET_LENGTH_IN-1] ));

	return crc_OK;
}

void read_usb(uint8_t *buffer)
{ // Reads in chars if the first char is an 'X'
		if(m_usb_rx_char() == 'X')	// new packet incoming!
		{
			int i;
			for(i=0; i < PACKET_LENGTH_IN; i++)
			{
				while(!m_usb_rx_available());	// wait for byte
				buffer[i] = m_usb_rx_char();
			}
		}
}

// Write data into wifi package using big-endian
void data_write(uint8_t *packet, int index, float scaling, float data) {

	int temp = 0;

	temp = (int)(scaling * data);
	packet[index] = *(((char *)&temp) + 1);
	packet[index+1] = *(char *)&temp;
}

//! \brief setup timers for compare match interrupt and cycle clock timer
void setup_timer() {
  cli(); // disable interrupts

  TCCR1A = 0; // Reset timer registers and timer itself
  TCCR1B = 0; // ^
  TCNT1 = 0;  // ^

  // Compare match register, 16MHz / PRESCALER / DES. IMU FREQ.
  uint16_t prescaler = select_timer1_prescaler(); // select correct prescaler for timer
  OCR1A = F_CPU / prescaler / LOOP_FREQ;

  // PWM (Fast PWM, top = OCR1A, update OCR1 at TOP, flag set on TOP)
  set(TCCR1B, WGM13); // ^
  set(TCCR1B, WGM12); // ^
  set(TCCR1A, WGM11); // ^
  set(TCCR1A, WGM10); // ^

  set(TIMSK1, OCIE1A); // Enable interrupts on timers

  // clock used to calculate cycle time, for micros function
  set(TCCR3B, CS30);  // prescale 1
  set(TIMSK3, TOIE3); // interrupt on overflow.

  sei(); // enable interrupts
} //! \end of setup_timer function

//! \brief Selects correct CPU prescaler for timer needed. also sets registers on 32u4
//! \return 16 bit unsigned value of prescaler. has to be used in OCRxx for correct freq.
uint16_t select_timer1_prescaler() {
  long int fcpu = F_CPU;
  long int loop_freq = LOOP_FREQ;
  if ((65534) > (fcpu / loop_freq)) {
    set(TCCR1B, CS10);
    return 1;
  } else if (((65534 * 8) > (fcpu / loop_freq)) && ((65534) <= (fcpu / loop_freq))) {
    set(TCCR1B, CS11);
    return 8;
  } else if (((65534 * 64) > (fcpu / loop_freq)) && ((65334 * 8) <= (fcpu / loop_freq))) {
    set(TCCR1B, CS11);
    set(TCCR1B, CS10);
    return 64;
  } else if (((65334 * 256) > (fcpu / loop_freq)) && ((65534 * 64) <= (fcpu / loop_freq))) {
    set(TCCR1B, CS12);
    return 256;
  } else if (((65534 * 1024) > (fcpu / loop_freq)) && ((65534 * 256) <= (fcpu / loop_freq))) {
    set(TCCR1B, CS12);
    set(TCCR1B, CS11);
    set(TCCR1B, CS10);
    return 1024;
  } else {
    return 0;
  }
} //! \end of select_timer1_prescaler function

//! \brief function to check loop duration in micro seconds
//! \param[in] stream, bool, if true, this will print the cycle time, otherwise not
//! \return amount of micro seconds the last time the function is called has lapsed
uint16_t micros(bool stream) {
  uint16_t val = 0;
  val = (float)(((long)cycle_overflow_counter * 65535) + (((long)TCNT3H) << 8) + ((long)TCNT3L)) /
        16; // this must be done twice somehow
  val = (float)(((long)cycle_overflow_counter * 65535) + (((long)TCNT3H) << 8) + ((long)TCNT3L)) / 16;
  cycle_overflow_counter = 0;
  TCNT3H = 0x00;
  TCNT3L = 0x00;
  /* if (stream) {
    m_usb_tx_string("\t");
    m_usb_tx_uint(val);
  } */
  return val;
} //! \end of micros function

//! \brief function to enable, calibrate and wait for motor 1 to be ready.
void init_calibrate_udriver(Commstruct *comm_ud) {
  volatile bool udriver_calibrated;
  udriver_calibrated = FALSE;
  m_spi_cs_setup(MD); // setup udriver pin for SPI
  m_wait(10 * INIT_WAIT);
  setMode(SYSTEM_ENABLE);
  setMode(MOTOR_1_ENABLE);
  setMode(MOTOR_2_ENABLE);
  (*comm_ud).currentLimit1 = MAX_CURRENT_MOTOR;
  (*comm_ud).currentLimit2 = MAX_CURRENT_MOTOR;
  m_green(OFF);

  while (!udriver_calibrated) {
	if (tick) {
	  send_spi_udriver(comm_ud); // send stuff
	  tick = FALSE;

		if ( (*comm_ud).status_M1R && (*comm_ud).status_M2R) {
		  udriver_calibrated = TRUE;
		  //m_green(ON);
		  }
    }
  }
} //! \end of init_calibrate_udriver function

void init(){
	cli(); // disable interrupts
	m_green( ON );

	m_clockdivide( 0 );     // 16 MHz operation

	// SETUP ANALOG PIN FOR VOLTAGE MEASUREMENT
	// ADC INIT, see: http://medesign.seas.upenn.edu/index.php/Guides/MaEvArM-adc
	set( ADMUX  , REFS0 );  // voltage reference to Vcc
	set( ADCSRA , ADPS2 );  // set ADC prescaler to /64
	set( ADCSRA , ADPS1 );  // ^

	if ( PCB_VERSION == 0 )
	{
		// Set F0 or F1 to read voltage measurements
		set( DIDR0  , ADC0D );  // disable digital on F0 to read current measurements
		// set( ADMUX  , MUX0  );  // Connect F1
	}
	else if ( PCB_VERSION == 3 )
	{
		m_disableJTAG();		// let me use F4-7 as GPIO

		// Set B4 to read voltage measurements
		set( DIDR2  , ADC11D );  // disable digital on B4 to read current measurements
		set( ADCSRB , MUX5 );  // Start new sample on channel set in MUXn
		set( ADMUX  , MUX1  );  // ^
		set( ADMUX  , MUX0  );  // ^

		set( ADCSRA , ADATE );  // free-running mode
		set( ADCSRA , ADEN  );  // enable ADC
		set( ADCSRA , ADSC  );  // start first conversion

		// ENABLE POWER TO UDRIVER
		//#ifdef MP
		set( DDRD , 5 );           // Init pin that controls power to udriver
		set( PORTD , 5 );			// Set pin high to power udriver
		//#endif
	}


	// COMMUNICATION INIT
	m_bus_init();                                 // enable mBUS
	m_usb_init();     // Init. USB streaming library
	m_rf_open( CHANNEL, RXADDRESS, PACKET_LENGTH ); // configure mRF module

	setup_timer(); // setup timer for desired frequency to poll IMUs
	m_spi_init(); // initialize SPI library
	m_spi_speed( SPI_125KHZ ); // slower SPI speed for initialization
	sei(); // enable interrupts
}
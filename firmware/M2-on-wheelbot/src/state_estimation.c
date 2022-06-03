#include "state_estimation.h"

void estimate_pivot_acc(float *acc_pivot_B, float *q, float *dq, float ddq4, FlagsStruct *flags) {
	/* STATES
   * q[0]: roll, q1
   * q[1]: pitch, q2
   * q[2]: yaw, q3
   * dq[3] :rate wheel 2, dq4
   */

	float acc_pivot_I[3] = {0.0, 0.0, 0.0};

	if ( (*flags).use_pivotAcc_CP )
	{
	  acc_pivot_I[0] += Rw*ddq4;
	  //acc_pivot_I[1] += Rw*dq[3]*dq[2];
	}

	if ( (*flags).use_pivotAcc_WC_vel )
	{
	  acc_pivot_I[0] += 2*Rw*cos(q[0])*dq[0]*dq[2];
	  acc_pivot_I[1] += ( Rw*sin(q[0])*(dq[0]*dq[0] + dq[2]*dq[2]) + Rw*dq[3]*dq[2] );
	  acc_pivot_I[2] += -Rw*cos(q[0])*dq[0]*dq[0];
	}

	// Transform acc_pivot into body-fixed frame

	acc_pivot_B[0] = cos(q[1]) * acc_pivot_I[0] + sin(q[1]) * ( sin(q[0]) * acc_pivot_I[1] - cos(q[0]) * acc_pivot_I[2] );
	acc_pivot_B[1] = cos(q[0]) * acc_pivot_I[1] + sin(q[0]) * acc_pivot_I[2];
	acc_pivot_B[2] = sin(q[1]) * acc_pivot_I[0] - cos(q[1]) * ( sin(q[0]) * acc_pivot_I[1] - cos(q[0]) * acc_pivot_I[2] );

	  /*
	if ( (*flags).compPivotAcc_WC_acc )
	{
	  acc_pivot[0] += Rw*sin(q[0])*ddq3;
	  acc_pivot[1] += -Rw*cos(q[0])*ddq1;
	  acc_pivot[2] += -Rw*sin(q[0])*ddq1;
	}
	*/
}

void estimate_pivot_acc_linearized(float *acc_pivot_B, float *q, float *dq, float ddq4, FlagsStruct *flags) {
	/* STATES
   * q[0]: roll, q1
   * q[1]: pitch, q2
   * q[2]: yaw, q3
   * dq[3] :rate wheel 2, dq4
   */

	acc_pivot_B[0] = 0.0;
	acc_pivot_B[1] = 0.0;
	acc_pivot_B[2] = 0.0;

	if ( (*flags).use_pivotAcc_CP )
	{
	  acc_pivot_B[0] += cos(q[1]) *Rw*ddq4; // Note that cos(q[1]) shouuld be 1
	  acc_pivot_B[2] += sin(q[1]) *Rw*ddq4;
	  //acc_pivot_B[0] += Rw*ddq4; // Note that cos(q[1]) shouuld be 1
	}

	if ( (*flags).use_pivotAcc_WC_vel )
	{
	  acc_pivot_B[0] += 2*Rw*dq[0]*dq[2];
	  acc_pivot_B[1] +=  Rw*dq[3]*dq[2];
	  acc_pivot_B[2] += (-1*Rw*q[0]*dq[0]);
	}
}

void attitude_estimate_accel(float *q_acc, float *acc_pivot_B, int16_t *imu_data) {
  float M[NUM_DIM][NUM_SENSORS]; // IMU measurements in matrix form
  float g_B[NUM_DIM];  // Gravitation vector estimate in body frame

  float x1_star[NUM_SENSORS] =   {0.0163, 0.0163, 0.4837, 0.4837}; // Estimator gain
  //float x1_star[NUM_SENSORS] =   {0.0, 0.0, 0.5, 0.5}; // Estimator gain

  /*
  for (int i = 0; i < NUM_SENSORS; i++) {
    for (int j = 0; j < NUM_DIM; j++) {
      M[j][i] = (float)(*(imu_data + i * NUM_IMU_DATA + j)) * 0.000598755 - acc_pivot_B[j];
    }
  }
   */


  M[0][0] = (float)(*(imu_data + 0 * NUM_IMU_DATA + 0)) * 0.000598755 - acc_pivot_B[0];
  M[1][0] = (float)(*(imu_data + 0 * NUM_IMU_DATA + 1)) * 0.000598755 - acc_pivot_B[1];
  M[2][0] = (float)(*(imu_data + 0 * NUM_IMU_DATA + 2)) * 0.000598755 - acc_pivot_B[2];

  M[0][1] = (float)(*(imu_data + 1 * NUM_IMU_DATA + 0)) * 0.000598755 - acc_pivot_B[0];
  M[1][1] = (float)(*(imu_data + 1 * NUM_IMU_DATA + 1)) * 0.000598755 - acc_pivot_B[1];
  M[2][1] = (float)(*(imu_data + 1 * NUM_IMU_DATA + 2)) * 0.000598755 - acc_pivot_B[2];

  M[0][2] = (float)(*(imu_data + 2 * NUM_IMU_DATA + 0)) * 0.000598755 - acc_pivot_B[0];
  M[1][2] = (float)(*(imu_data + 2 * NUM_IMU_DATA + 1)) * 0.000598755 - acc_pivot_B[1];
  M[2][2] = (float)(*(imu_data + 2 * NUM_IMU_DATA + 2)) * 0.000598755 - acc_pivot_B[2];

  M[0][3] = (float)(*(imu_data + 3 * NUM_IMU_DATA + 0)) * 0.000598755 - acc_pivot_B[0];
  M[1][3] = (float)(*(imu_data + 3 * NUM_IMU_DATA + 1)) * 0.000598755 - acc_pivot_B[1];
  M[2][3] = (float)(*(imu_data + 3 * NUM_IMU_DATA + 2)) * 0.000598755 - acc_pivot_B[2];


  multiply_matrix_vector(NUM_DIM, NUM_SENSORS, M, x1_star, g_B);

  q_acc[0] = atan(g_B[1]/sqrt(g_B[0]*g_B[0] + g_B[2]*g_B[2]));
  q_acc[1] = atan(-g_B[0]/g_B[2]);

} // end of theta_estimate_accel function

void compute_estimator_bias(FlagsStruct *flags, uint16_t counter, uint16_t *counter_standup, float *roll_bias,
							float *roll_bias_sum, float *pitch_bias, float *pitch_bias_sum, float attitude[2]) {
	// If angle is bigger than 20 degree (0.35 rad) switch off
      	if ( (*flags).bias == FALSE )
      	{
		    m_red( ON );
      		(*flags).bias = TRUE;
      		*counter_standup = counter;
	    }
	    if ( ( counter - *counter_standup ) == 1500)
	    {
			m_red( OFF );
		    *roll_bias = *roll_bias_sum /(float)1000;
		    *pitch_bias = *pitch_bias_sum /(float)1000;
		    // Ensure that control bias is smaller 6 degrees, to prevent control mayhem
		    if ( (fabs(*roll_bias) > 0.09) || (fabs(*pitch_bias) > 0.09) )
		    {
			    (*flags).control = FALSE;
		    }
	    }
	    if ( ( ( counter - *counter_standup )  >= 500) && ( (counter - *counter_standup) < 1500) )
	    {
		    *roll_bias_sum += attitude[0];
		    *pitch_bias_sum += attitude[1];
	    }
}
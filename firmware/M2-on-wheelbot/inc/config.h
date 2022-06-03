#ifndef IMUTEST_CONFIG
#define IMUTEST_CONFIG
/**************************************************************************
 *
 *  SETTINGS
 *
 **************************************************************************/
#define INIT_WAIT 100  // ms to wait after init. each IMU
#define LOOP_FREQ 100  // desired polling freq [Hz] of IMUs
#define TIME_STEP (1/LOOP_FREQ)  // desired polling freq [Hz] of IMUs
#define CALIB_NUM_SAMPLES 20 // amount of samples used for calibrating gyro + accelerometer.
#define NUM_SENSORS 4           // number of sensors
#define NUM_DIM 3
#define NUM_IMU_DATA 6

/* MOTHERBOARD PIN ASSIGNMENT
 * 0: Motherboard as used on Wheelbot v2
 * 3: New motherboard as designed by Naomi and detailed in Confluence
 */

#define PCB_VERSION 0

#if PCB_VERSION == 3
		#define IMU_A PIN_D6         // pin of IMU 1 (closest to hinge)
		#define IMU_B PIN_D7         // pin of IMU 2 (furthest to hinge)
		#define IMU_C PIN_F6
		#define IMU_D PIN_F7
		#define BATTERY_VOLTAGE PIN_B4 // 34V max, 5V ADC max, 1bit=23.5mV
		#define MIN_BATTERY_VOLTAGE 22.4 * 30.09
		#define MD PIN_D4  // uDriver CS pin
		//#define MP PIN_D5  // Power switch to udriver
#endif

#if PCB_VERSION == 0
		#define IMU_A PIN_D6         // pin of IMU 1 (closest to hinge)
		#define IMU_B PIN_D7         // pin of IMU 2 (furthest to hinge)
		#define IMU_C PIN_D3
		#define IMU_D PIN_D4
		#define BATTERY_VOLTAGE PIN_FO // 34V max, 5V ADC max, 1bit=23.5mV
		#define MIN_BATTERY_VOLTAGE 22.4 * 28.0
		#define MD PIN_D5  // uDriver CS pin
#endif

#define MOTOR_KV 160.0 // kv of motor
#define MOTOR_V 24.5 // nominal voltage of motor
#define MOTOR_TORQUE_CONSTANT 0.075 //0.06//0.075 //0.09375//(0.050259456) // kt of motor
#define FUSION_TUNING_PARAMETER 0.02 // tuning parameter for complementary filter


#define NOPE 3000 // Treshold at which gyro values are discarded
#define NOPE2 52 // Treshold at which gyro values are discarded

// ranges for gyro and accel
//#define accel_r ACCEL_2G
//#define gyro_r GYRO_500DPS

// filters for gyro and accel
//#define accel_f ACC_LPF_5HZ
//#define gyro_f GY_LPF_41HZ

#define MAX_CURRENT_MOTOR 20.0 // Max current in Amps, the MN6007 KV160 has a max. peak current of 18A
#define MAX_CURRENT_LQR 15.0
#define MAX_RATE_MOTOR1 330.0  // Max rate in rad/s, 3150 RPM
// The max. motor rate of the MN6007 Kv160 is 3800 RPM
// 398 rad/s = 3800 RPM
// 314 rad/s = 3000 RPM
// 210 rad/s = 2000 RPM
// 10 rad/s = 95 RPM
#define INIT_ANGLE -0.524 //-0.524
#define Rw 0.049 // Wheel radius used for pivot point acc. computation

#define DATA_MODE 1

/* DATA_MODE
 * 1: CONTROL PLOTS -> FLAG_CONTROL = ?
 * 2: PIVOT ACC PLOTS
 * 3: IMU acc
 * 4: IMU rate
 * */

/* FLAG_PLOT chooses which additional data to send to PC
 *  1: MISC
 *  2: sys_flag, q[4] - init_q5_est, q[3] - init_q4_est
 *  3: sys_flag, loop_time,
 *  6: acc_pivot_B[0], comm_ud.velocityMotor2, comm_ud.currentTargetMotor2
 *  7: sys_flag, roll_bias, pitch_bias
 *  8: data_in[0], data
 *  9: sys_flag, comm_ud.currentMotor1, comm_ud.currentMotor2
 *  10: dq[2], q_acc[0], q_acc[1]
 *  11: acc_pivot_B[0], acc_pivot_B[1], acc_pivot_B[2]
 *  12:
 *  13: imu_data[3][0], acc_pivot_B[0], comm_ud.velocityMotor2
 */

#define FLAG_PLOT 9 // 9

/**************************************************************************
 *
 *  Constants
 *
 **************************************************************************/
#define F_CPU 16000000 // freq of CPU
#define GRAV 9.81
#define PI 3.14159265358979
#define PI_2 1.57079632679 // pi/2
#define STATIC_INIT_VALUE 70000
#define MD_PACKET_SIZE 34

// Setup wifi comm
#define CHANNEL 1
#define RXADDRESS 0xF4
#define TXADDRESS 0x54
#define PACKET_LENGTH 32
#define PACKET_LENGTH_IN 32 // Note that m_rf_open() requires PACKET_LENGTH = PACKET_LENGTH_IN

#endif // IMUTEST_CONFIG_H

#ifndef MAEVARM_M2_CHECKS
#define MAEVARM_M2_CHECKS

/* global program files */
#include "config.h"
#include "m_general.h"
#include "udriver.h"

typedef struct flags_struct {
		bool system; // System safe?
		bool wifi; // WiFi still connected?
		bool angle; // Angle small during stabilization control?
		bool rate; // Rate below MAX_RATE_MOTOR?
		bool control; // Motor current below MAX_CURRENT_MOTOR?
		bool battery; // Battery voltage larger than MIN_BATTERY_VOLTAGE?
		bool upright; // Set when Wheelbot reaches upright position
		bool standup1; // Set when standup1 started
		bool standup1_done; // Set when standup1 done
		bool standup2; // Set when standup2 started
		bool standup2_done; // Set when standup2 done
		bool bias; // Set when estimator bias computation started
		bool loop_finished; // Set when MAIN WHILE loop has een fully executed
		bool LQRcontrol1; // Set when roll control starts
		bool LQRcontrol2; // Set when pitch control starts
		bool useLQR1; // Set to start roll control
		bool useLQR2; // Set to start pitch control
		bool do_standup; // Set to start standup
		bool do_rollup; // Set to start rollup
		bool usb_control; // User setting that causes communication over USB instead of WiFi
		bool use_IMUs; // User setting that disables all computation routines that use IMUs
		bool motor_test; // Set when motor control test is started
		bool use_pivotAcc_CP; // User setting that adds pivot acc. of ddq4 to pose estimate
		bool use_pivotAcc_WC_vel; // User setting that adds pivot acc. of dq1, dq2, dq3 to pose estimate
		bool use_pivotAcc_WC_acc; // User setting that adds pivot acc. of ddq1, ddq2 to pose estimate
	} FlagsStruct;

void check_wifi(FlagsStruct *flags, uint16_t counter, uint16_t counter_lastwifi);
void check_control(FlagsStruct *flags, Commstruct *comm_ud);
void check_rates(FlagsStruct *flags, Commstruct *comm_ud, float max_rate_motor2);
void check_battery(FlagsStruct *flags, uint16_t *battery_voltage);
void check_upright(FlagsStruct *flags, Commstruct *comm_ud, float attitude[2]);
//void reset_checks(FlagsStruct *flags, Commstruct *comm_ud, uint8_t *buffer_wifi_in);
void check_tick_time(FlagsStruct *flags, uint16_t *sys_flag, uint16_t *tick_time);
void check_loop_time(uint16_t *sys_flag, uint16_t loop_time);

#endif

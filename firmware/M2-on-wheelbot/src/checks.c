#include "checks.h"

void check_wifi(FlagsStruct *flags, uint16_t counter, uint16_t counter_lastwifi){
	// Check Wifi connection
	if ( ( (counter - counter_lastwifi) > 150 ) && ( counter_lastwifi != 0 ) ) {
		(*flags).wifi = FALSE;
	}
}

void check_control(FlagsStruct *flags, Commstruct *comm_ud){
	// Check motor currents
	if ( fabs( (*comm_ud).currentMotor1 ) > MAX_CURRENT_MOTOR ||
        fabs( (*comm_ud).currentMotor2 ) > MAX_CURRENT_MOTOR ) {
	        (*flags).control = FALSE;
      }
}

void check_rates(FlagsStruct *flags, Commstruct *comm_ud, float max_rate_motor2) {
      // Check motor rates
	  if (fabs( (*comm_ud).velocityMotor1 ) > MAX_RATE_MOTOR1 ||
	      fabs( (*comm_ud).velocityMotor2 ) > max_rate_motor2 ) {
	        (*flags).rate = FALSE;
	  }
}

void check_battery(FlagsStruct *flags, uint16_t *battery_voltage){
	  // Check battery voltage
	  uint16_t battery_voltage_new;

      if( check( ADCSRA , ADIF ) ) { // when the conversion finishes
	    set( ADCSRA , ADIF ); // reset the flag
	    battery_voltage_new = ADCL;
	    battery_voltage_new |= (unsigned int)ADCH << 8;
		*battery_voltage = 0.02 * battery_voltage_new + 0.98 * (*battery_voltage); // low pass filter filtering acceleration estimation
	    if ( (*battery_voltage) < MIN_BATTERY_VOLTAGE ) {
	    	(*flags).battery = FALSE;
	    }
      }
}

void check_upright(FlagsStruct *flags, Commstruct *comm_ud, float attitude[2]){
	  // Check Wheelbot reached upright, 0.35 DEBUG DEBUG
	  if ( ( (*flags).upright == FALSE ) && fabs(attitude[0]) < 0.2) {
		  (*flags).upright = TRUE;
	  }

	  // Check roll angle, ( 20 degree = 0.35 rad )
	  //if ( ( (*flags).upright == TRUE ) && ( fabs(attitude[0]) > 0.52 ) ) {
	  //	(*flags).angle = FALSE;
	  //}
	  // Check pitch angle
	  //if ( fabs(attitude[1]) > 0.52 ) {
	  //	(*flags).angle = FALSE;
	  //}
}

/*
void reset_checks(FlagsStruct *flags, Commstruct *comm_ud, uint8_t *buffer_wifi_in){
      // Reset command
      // Note: Control and current safety-flags should not be resetable
      if (buffer_wifi_in[0] == 'x')
      {
	      (*flags).upright = FALSE;
	      (*flags).system = TRUE;
	      (*flags).wifi = TRUE;
	      (*flags).angle = TRUE;

	      if ( fabs( (*comm_ud).velocityMotor1) < MAX_RATE_MOTOR1 ||
	          fabs( (*comm_ud).velocityMotor2) < MAX_RATE_MOTOR2 ) {
			      (*flags).rate = TRUE;
			      m_red( OFF );
	      }
      }
}
*/

void check_tick_time(FlagsStruct *flags, uint16_t *sys_flag, uint16_t *tick_time){
	  *tick_time = micros(FALSE);

	  if ( *tick_time > 10050 ) { // USE LOOP_FREQ in future
		 *sys_flag = 3;
		 m_red( ON );
		 }
	  else if ( (*flags).loop_finished == FALSE ) {
		 *sys_flag = 4;
		 m_red( ON );
		 }

	  (*flags).loop_finished = FALSE;
}

void check_loop_time(uint16_t *sys_flag, uint16_t loop_time){

	  if ( loop_time > 10000 ) { // USE LOOP_FREQ in future
		 *sys_flag = 5;
		 m_red( ON );
		 }
}



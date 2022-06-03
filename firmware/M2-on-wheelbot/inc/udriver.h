#ifndef UNIWHEEL_MAEVARM_M2_UDRIVER
#define UNIWHEEL_MAEVARM_M2_UDRIVER
/**************************************************************************
 *
 *  Header Files to include
 *
 **************************************************************************/
/* subystem header files */
#include "m_crc32.h"
#include "m_spi.h"

/* global program files */
#include "m_general.h"
//#include "m_usb.h"
#include "config.h"

/* External header files */
#include <math.h>

/**************************************************************************
 *
 *  typedef and enumerates
 *
 **************************************************************************/
enum
{
    SYSTEM_ENABLE = 0x8000,
    MOTOR_1_ENABLE = 0x4000,
    MOTOR_2_ENABLE = 0x2000,
    ROLLOVER_ENABLE = 0x1000,
    MOTOR_1_INDEX_COMPENSATION_ENABLE = 0x0800,
    MOTOR_2_INDEX_COMPENSATION_ENABLE = 0x0400,
    SYSTEM_DISABLE = ~0x8000,
    MOTOR_1_DISABLE = ~0x4000,
    MOTOR_2_DISABLE = ~0x2000,
    ROLLOVER_DISABLE = ~0x1000,
    MOTOR_1_INDEX_COMPENSATION_DISABLE = ~0x0800,
    MOTOR_2_INDEX_COMPENSATION_DISABLE = ~0x0400,
};

typedef struct {
	// MOSI
	float currentLimit1;
	float currentTargetMotor1;

	float currentLimit2;
	float currentTargetMotor2;

	// MISO
	float currentMotor1;
	float positionMotor1;
	float velocityMotor1;

    float currentMotor2;
	float positionMotor2;
	float velocityMotor2;

	//bool motor1Ready;
	//bool motor2Ready;
	//bool systemEnabled;

	uint8_t status_errcode; // uDriver error messages
	// 0: No error, 1: Encoder error, 2: CAN recieve timeout, 3: Critical temperature
	bool status_SE; // systemeEnabled: udriver ON
	bool status_M1E; // motor1Enabled: Motor 1 enabled
	bool status_M1R; // Motor 1 is ready
	bool status_IDX1D; // motor1IndexDetected: At least one encoder has been detected
	bool status_IDX1T; // motor1IndexToggle: toggles for every encoder index

	bool status_M2E;
	bool status_M2R;
	bool status_IDX2D;
	bool status_IDX2T;
} Commstruct;

/**************************************************************************
 *
 *  Function declarations
 *
 **************************************************************************/
void setMode(uint16_t setting);
bool send_spi_udriver(Commstruct *comm_ud);
void check_motor_signs(Commstruct *pcomm, uint16_t *p_sys_flag, float *sign_motor1, float *sign_motor2,
					   bool (*send_spi_udriver)(Commstruct *));

void printState();

uint16_t flipu16(uint16_t val);
int16_t flip16(int16_t val);
uint32_t flipu32(uint32_t val);
int32_t flip32(int32_t val);

#endif
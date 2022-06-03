#include "udriver.h"
/*
 * Script: udriver.c
 * -----------------
 *  variable initialization and functions for SPI communication with uDriver
 *
 *  Info:
 *
 */

/**************************************************************************
 *
 *  Variables
 *
 **************************************************************************/

uint8_t mosi[MD_PACKET_SIZE] = {0};
uint8_t miso[MD_PACKET_SIZE] = {0};

uint16_t desiredMode = 0;
uint8_t crc_check[4] = {0};
uint32_t *crc_check_ptr = (uint32_t *)&crc_check[0];
bool crcOK;
uint16_t boardStatus;
uint16_t communicationInputIndex;
//uint16_t timestamp;

// MOSI POINTERS
uint16_t *mode = (uint16_t *)&mosi[0];
//int32_t *refPosition1 = (int32_t *)&mosi[2];
//int32_t* refPosition2 = (int32_t*)&mosi[6];
// int16_t *refVelocity1 = (int16_t *)&mosi[10]; // DEBUG: Commented out as not needed for current control
// int16_t* refVelocity2 = (int16_t*)&mosi[12]; // DEBUG: Commented out as not needed for current control
int16_t *refCurrent1 = (int16_t *)&mosi[14];
int16_t* refCurrent2 = (int16_t*)&mosi[16];
//uint16_t* kp1 = (uint16_t*)&mosi[18];
//uint16_t* kp2 = (uint16_t*)&mosi[20];
//uint16_t* kd1 = (uint16_t*)&mosi[22];
//uint16_t* kd2 = (uint16_t*)&mosi[24];
uint8_t *currentSaturation1 = (uint8_t *)&mosi[26];
uint8_t *currentSaturation2 = (uint8_t*)&mosi[27];
uint16_t *sendIdx = (uint16_t *)&mosi[28];
uint32_t *sendCrc = (uint32_t *)&mosi[30];

// MISO POINTERS
uint16_t *status = (uint16_t *)&miso[0];
uint16_t *ts = (uint16_t *)&miso[2];
int32_t *mPositionMotor1 = (int32_t *)&miso[4];
int32_t *mPositionMotor2 = (int32_t*)&miso[8];
int16_t *mVelocityMotor1 = (int16_t *)&miso[12];
int16_t *mVelocityMotor2 = (int16_t*)&miso[14];
int16_t *mcurrentMotor1 = (int16_t *)&miso[16];
int16_t *mcurrentMotor2 = (int16_t*)&miso[18];
uint16_t *r_a = (uint16_t *)&miso[20];
uint16_t *r_b = (uint16_t *)&miso[22];
//uint16_t *mADCMotor1 = (uint16_t *)&miso[24];
//uint16_t *mADCMotor2 = (uint16_t*)&miso[26];
uint16_t *receiveIndex = (uint16_t *)&miso[28];
uint32_t *receiveCrc = (uint32_t *)&miso[30];

/**************************************************************************
 *
 *
 *  Functions
 *
 **************************************************************************/

//! \brief Set mode of uDriver board, this is cumulative, except for system/motor disable. check
//! https://atlas.is.localnet/confluence/pages/viewpage.action?pageId=64260519
//! \param[in] setting, setting you want to send to the uDriver board
void setMode(uint16_t setting) {
  // set mode bits in human readable commands
  switch (setting) {
  case SYSTEM_ENABLE:
    desiredMode |= (uint16_t)SYSTEM_ENABLE;
    break;
  case SYSTEM_DISABLE:
    desiredMode &= (uint16_t)SYSTEM_DISABLE;
    break;
  case MOTOR_1_ENABLE:
    desiredMode |= (uint16_t)MOTOR_1_ENABLE;
    break;
  case MOTOR_1_DISABLE:
    desiredMode &= (uint16_t)MOTOR_1_DISABLE;
    break;
  case MOTOR_2_ENABLE:
    desiredMode |= (uint16_t)MOTOR_2_ENABLE;
    break;
  case MOTOR_2_DISABLE:
    desiredMode &= (uint16_t)MOTOR_2_DISABLE;
    break;
  case ROLLOVER_ENABLE:
    desiredMode |= (uint16_t)ROLLOVER_ENABLE;
    break;
  case ROLLOVER_DISABLE:
    desiredMode &= (uint16_t)ROLLOVER_DISABLE;
    break;
  case MOTOR_1_INDEX_COMPENSATION_ENABLE:
    desiredMode |= (uint16_t)MOTOR_1_INDEX_COMPENSATION_ENABLE;
    break;
  case MOTOR_1_INDEX_COMPENSATION_DISABLE:
    desiredMode &= (uint16_t)MOTOR_1_INDEX_COMPENSATION_DISABLE;
    break;
  case MOTOR_2_INDEX_COMPENSATION_ENABLE:
    desiredMode |= (uint16_t)MOTOR_2_INDEX_COMPENSATION_ENABLE;
    break;
  case MOTOR_2_INDEX_COMPENSATION_DISABLE:
    desiredMode &= (uint16_t)MOTOR_2_INDEX_COMPENSATION_DISABLE;
    break;
  default:                               // set communication timeout in ms
    if ((setting > 0) & (setting < 255)) // delay in ms
      desiredMode |= setting;
  }
} //! \end of setMode function

//! \brief exchange data/commands with uDriver board
//! \param[in] incoming_comm_struct, a struct of type "Commstruct". see udriver.h
//! \return bool if the transfer/CRC is successful
bool send_spi_udriver(Commstruct *comm_ud) {
	// reset the outbound packet
	for (int i = 0; i < MD_PACKET_SIZE; i++) {
		mosi[i] = 0;
	}

	// load mosi buffer
	*mode = flipu16(desiredMode);

	// set current in desired format, with correct resolution etc.
	*refCurrent1 = flip16((int16_t)(( (*comm_ud).currentTargetMotor1 ) * (1 << 10)));
	*currentSaturation1 = (uint8_t)(( (*comm_ud).currentLimit1 ) * (1 << 3));
	*refCurrent2 = flip16((int16_t)(( (*comm_ud).currentTargetMotor2 ) * (1 << 10)));
	*currentSaturation2 = (uint8_t)(( (*comm_ud).currentLimit2 ) * (1 << 3));

	// send SPI stuff and check incoming CRC
	*sendCrc = flipu32(m_crc32((uint8_t *)mosi, 30));
	m_spi_shift_MD(MD, (uint8_t *)mosi, (uint8_t *)miso, MD_PACKET_SIZE);
	*crc_check_ptr = flipu32(m_crc32((uint8_t *)miso, 30));
	crcOK = ((crc_check[0] == miso[32]) && (crc_check[1] == miso[33]) && (crc_check[2] == miso[30]) &&
	       (crc_check[3] == miso[31]));


	if (crcOK) // write values to public variables
	{

		(*comm_ud).positionMotor1 = (2.0 * PI * ((float)(flip32(*mPositionMotor1)) / (float)((int32_t)1 << 24)));
		(*comm_ud).positionMotor2 = (2.0 * PI * ((float)(flip32(*mPositionMotor2)) / (float)((int32_t)1 << 24)));
		(*comm_ud).velocityMotor1 =
			  ((float)(2.0 * PI * 1000.0 / 60.0 * ((double)(flip16(*mVelocityMotor1)) / (double)((int32_t)1 << 11))));
		(*comm_ud).velocityMotor2 =
			  ((float)(2.0 * PI * 1000.0 / 60.0 * ((double)(flip16(*mVelocityMotor2)) / (double)((int32_t)1 << 11))));
		(*comm_ud).currentMotor1 = (((float)(flip16(*mcurrentMotor1)) / (float)((int32_t)1 << 10)));
		(*comm_ud).currentMotor2 = (((float)(flip16(*mcurrentMotor2)) / (float)((int32_t)1 << 10)));

		////    ADCMotor2 = (int16_t)(3.3 * ((float)(flip16(*mADCMotor2)) / (float)((int32_t)1 << 16)));
		////
		////    communicationInputIndex = flipu16(*receiveIndex);
		////
		boardStatus = flipu16(*status);
		//    timestamp = flipu16(*ts);

		(*comm_ud).status_SE = (boardStatus >> 15) & 1;
		(*comm_ud).status_M1E = (boardStatus >> 14) & 1;
		(*comm_ud).status_M1R = (boardStatus >> 13) & 1;
		(*comm_ud).status_M2E = (boardStatus >> 12) & 1;
		(*comm_ud).status_M2R = (boardStatus >> 11) & 1;
		(*comm_ud).status_IDX1D = (boardStatus >> 10) & 1;
		(*comm_ud).status_IDX2D = (boardStatus >> 9) & 1;
		(*comm_ud).status_IDX1T = (boardStatus >> 8) & 1;
		(*comm_ud).status_IDX2T = (boardStatus >> 7) & 1;
		(*comm_ud).status_errcode = boardStatus & 0x07;
	}
	return crcOK;
}

void check_motor_signs(Commstruct *pcomm, uint16_t *p_sys_flag, float *sign_motor1, float *sign_motor2, bool (*send_spi_udriver)(Commstruct *)) {
		// Is a positive current creating a positive torque?

		float tmp1 = 0.0;
		float tmp2 = 0.0;
		uint16_t count;

		(*send_spi_udriver)( pcomm );
		tmp1 = (*pcomm).positionMotor1;
		tmp2 = (*pcomm).positionMotor2;

		count = 0;
		while(true) {
			(*pcomm).currentTargetMotor1 += 0.01;
			(*send_spi_udriver)( pcomm );
			fabs( (*pcomm).positionMotor1 - tmp1 );
			count += 1;
			m_wait(10);

		    if ( fabs( (*pcomm).positionMotor1 - tmp1 ) > 0.1 ) {
		        break;
		    }
			if ( count > 300 ){
				*p_sys_flag = 1;
				break;
			}
		}
		(*pcomm).currentTargetMotor1 = 0.0;
		(*send_spi_udriver)( pcomm );

		count = 0;
		while(true) {
			(*pcomm).currentTargetMotor2 += 0.01;
			(*send_spi_udriver)( pcomm );
			fabs( (*pcomm).positionMotor2 - tmp2 );
			count += 1;
			m_wait(10);

		    if ( fabs( (*pcomm).positionMotor2 - tmp2 ) > 0.1 ) {
		        break;
		    }
			if ( count > 300 ){
				*p_sys_flag = 2;
				break;
			}
		}
		(*pcomm).currentTargetMotor2 = 0.0;
		(*send_spi_udriver)( pcomm );

		if ( (*pcomm).positionMotor1 > tmp1) {
			*sign_motor1 = 1.0;
		}
		else if ((*pcomm).positionMotor1 < tmp1){
			*sign_motor1 = -1.0;
		}
		else {
			*sign_motor1 = 1.0;
		}

		if ( (*pcomm).positionMotor2 > tmp2) {
			*sign_motor2 = 1.0;
		}
		else if ( (*pcomm).positionMotor2 < tmp2 ){
			*sign_motor2 = -1.0;
		}
		else {
			*sign_motor2 = 1.0;
		}
  }

// switch bytes to low byte first
uint16_t flipu16(uint16_t val) { return (val << 8) | (val >> 8); }

// switch bytes to low byte first
int16_t flip16(int16_t val) { return (val << 8) | ((val >> 8) & 0xFF); }

// switch bytes to low byte first
uint32_t flipu32(uint32_t val) {
  val = ((val << 8) & 0xFF00FF00) | ((val >> 8) & 0xFF00FF);
  return (val << 16) | (val >> 16);
}

// switch bytes to low byte first
int32_t flip32(int32_t val) {
  val = ((val << 8) & 0xFF00FF00) | ((val >> 8) & 0xFF00FF);
  return (val << 16) | ((val >> 16) & 0xFFFF);
}


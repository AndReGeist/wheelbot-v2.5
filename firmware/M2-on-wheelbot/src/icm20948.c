#include "icm20948.h"

bool icm20948_init(m2_gpio_t cs_pin, a_range_t accel_range, g_range_t gyro_range){
    
    // INITIALIZE THE SPI SUBSYSTEM (IT IS OKAY TO CALL MULTIPLE TIMES)
    m_spi_init();
    
    // ASSOCIATE THE CHIP-SELECT PIN
    m_spi_cs_setup(cs_pin);
    
    // SET THE CLOCK SOURCE
    m_spi_write_register(cs_pin, REG_BANK_SEL, BANK_0);       // BANK 0
    m_wait(100);
    m_spi_write_register(cs_pin, PWR_MGMT_1, 0b10000000);       // RESET (7) + AUTO CLK (1)
    m_wait(100);
    m_spi_write_register(cs_pin, USER_CTRL, 0b00110000);        //(1<<5)|(1<<4));
    m_wait(100);
    m_spi_write_register(cs_pin, PWR_MGMT_1, 0x01);             // AUTO CLK (1)
    m_wait(100);
    if(m_spi_read_register(cs_pin, WHO_AM_I) != WHOAMI_VALUE) {
        return false;
    }

    // CONFIGURE ACCEL & GYRO
    m_spi_write_register(cs_pin, PWR_MGMT_2, 0x00);             // ENABLE ACCEL & GYRO
    icm20948_set_accel_range(cs_pin, accel_range);              // SET ACCEL RANGE
    icm20948_set_gyro_range(cs_pin, gyro_range);                // SET GYRO RANGE
    // low-pass filter
    // sample-rate divider
    // CONFIGURE MAGNETOMETER

    return true;
}

void icm20948_set_accel_range(m2_gpio_t cs_pin, a_range_t accel_range){
    spi_freq_t current_freq = _spi_freq;  // Slow down SPI as needed
    if ((uint8_t)current_freq > (uint8_t)SPI_250KHZ)  { m_spi_speed(SPI_250KHZ); }
    m_spi_write_register(cs_pin, REG_BANK_SEL, BANK_2); // switch to bank 2
    m_spi_write_register(cs_pin, ACCEL_CONFIG, accel_range);
    m_spi_write_register(cs_pin, REG_BANK_SEL, BANK_0); // switch to bank 1
    if ((uint8_t)current_freq > (uint8_t)SPI_250KHZ)  { m_spi_speed(current_freq); }
}

void icm20948_set_gyro_range(m2_gpio_t cs_pin, g_range_t gyro_range)
{
    
    spi_freq_t current_freq = _spi_freq;  // Slow down SPI as needed
    if ((uint8_t)current_freq > (uint8_t)SPI_1MHZ)  { m_spi_speed(SPI_1MHZ); }
    m_spi_write_register(cs_pin, REG_BANK_SEL, BANK_2); // switch to bank 2
    m_spi_write_register(cs_pin, GYRO_CONFIG, gyro_range);
    m_spi_write_register(cs_pin, REG_BANK_SEL, BANK_0); // switch to bank 1
    if ((uint8_t)current_freq > (uint8_t)SPI_1MHZ)  { m_spi_speed(current_freq); }
}

void icm20948_get_raw_3dof(m2_gpio_t cs_pin, int16_t *buffer) {
      buffer[0] = (256 * m_spi_read_register(cs_pin, ACCEL_XOUT_H) +
             m_spi_read_register(cs_pin, ACCEL_XOUT_L));
      buffer[1] = (256 * m_spi_read_register(cs_pin, ACCEL_YOUT_H) +
             m_spi_read_register(cs_pin, ACCEL_YOUT_L));
      buffer[2] = (256 * m_spi_read_register(cs_pin, GYRO_ZOUT_H) +
             m_spi_read_register(cs_pin, GYRO_ZOUT_L));
}


void icm20948_get_raw_6dof(m2_gpio_t cs_pin, int16_t *buffer_acc,  int16_t *buffer_gyro) {
	  buffer_acc[0] = (256 * m_spi_read_register(cs_pin, ACCEL_XOUT_H) + m_spi_read_register(cs_pin, ACCEL_XOUT_L));
	  buffer_acc[1] = (256 * m_spi_read_register(cs_pin, ACCEL_YOUT_H) + m_spi_read_register(cs_pin, ACCEL_YOUT_L));
	  buffer_acc[2] = (256 * m_spi_read_register(cs_pin, ACCEL_ZOUT_H) + m_spi_read_register(cs_pin, ACCEL_ZOUT_L));
	  buffer_gyro[0] = (256 * m_spi_read_register(cs_pin, GYRO_XOUT_H) + m_spi_read_register(cs_pin, GYRO_XOUT_L));
	  buffer_gyro[1] = (256 * m_spi_read_register(cs_pin, GYRO_YOUT_H) + m_spi_read_register(cs_pin, GYRO_YOUT_L));
	  buffer_gyro[2] = (256 * m_spi_read_register(cs_pin, GYRO_ZOUT_H) + m_spi_read_register(cs_pin, GYRO_ZOUT_L));
}


void icm20948_calib_accel(m2_gpio_t *cs_pin, volatile int16_t *accel_adjust, int16_t *acc_ref0) {
  for (int k = 0; k < 1; k++) { // loop over one IMU
    for (int j = 0; j < NUM_DIM; j++) {         // loop over all directions (x,y)
      float accel_calib = 0;
      for (int i = 0; i < CALIB_NUM_SAMPLES; i++) {
        accel_calib += ((float)((m_spi_read_register(*(cs_pin + k), ACCEL_XOUT_H + j * 2) << 8) |
                                (m_spi_read_register(*(cs_pin + k), ACCEL_XOUT_L + j * 2))) /
                        (float)CALIB_NUM_SAMPLES);
        // IMUS send out 16bit messages split in 2 bytes, ACCEL_XOUt_H: Adress to bits 8-15, ACCEL_XOUt_L: Address to bits 0-7
      }
      (*(accel_adjust + k * 2 + j)) = (uint16_t)accel_calib - acc_ref0[j];
    }
  }
} //! \end of mpu9250_calib_accel function
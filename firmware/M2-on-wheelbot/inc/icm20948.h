#ifndef icm20948__
#define icm20948__

#include <stdlib.h>
#include "m_general.h"
#include "m_spi.h"
//#include "m_usb.h"
#include "config.h"

typedef enum
{
  ACCEL_2G  = (0b00 << 1),
  ACCEL_4G  = (0b01 << 1),
  ACCEL_8G  = (0b10 << 1),
  ACCEL_16G = (0b11 << 1)
} a_range_t;

typedef enum
{
  GYRO_250DPS  = (0b00 << 1),
  GYRO_500DPS  = (0b01 << 1),
  GYRO_1000DPS = (0b10 << 1),
  GYRO_2000DPS = (0b11 << 1)
} g_range_t;

typedef enum
{
  ACC_LPF_1046HZ = 8,
  ACC_LPF_420HZ = 7,
  ACC_LPF_218HZ_B = 0,
  ACC_LPF_218HZ = 1,
  ACC_LPF_99HZ = 2,
  ACC_LPF_45HZ = 3,
  ACC_LPF_21HZ = 4,
  ACC_LPF_10HZ = 5,
  ACC_LPF_5HZ = 6
} lpf_accel_bw_t;

typedef enum
{
  GY_LPF_8800HZ = 8,
  GY_LPF_3600HZ_HISPD = 16,
  GY_LPF_3600HZ = 7,
  GY_LPF_250HZ = 0,
  GY_LPF_184HZ = 1,
  GY_LPF_92HZ = 2,
  GY_LPF_41HZ = 3,
  GY_LPF_20HZ = 4,
  GY_LPF_10HZ = 5,
  GY_LPF_5HZ = 6
} lpf_gyro_bw_t;

// PUBLIC FUNCTIONS
bool icm20948_init(m2_gpio_t cs_pin, a_range_t accel_range, g_range_t gyro_range);
void icm20948_set_accel_range(m2_gpio_t cs_pin, a_range_t accel_range);
void icm20948_set_gyro_range(m2_gpio_t cs_pin, g_range_t gyro_range);
void icm20948_get_raw_3dof(m2_gpio_t cs_pin, int16_t *buffer);
void icm20948_get_raw_6dof(m2_gpio_t cs_pin, int16_t *buffer_acc, int16_t *buffer_gyro);
void icm20948_calib_accel(m2_gpio_t *cs_pin, volatile int16_t *accel_adjust, int16_t *acc_ref0);

// BANK 0 REGISTERS
#define WHO_AM_I            0x00
#define USER_CTRL           0x03
#define PWR_MGMT_1          0x06
#define PWR_MGMT_2          0x07
#define ACCEL_XOUT_H        0x2D
#define ACCEL_XOUT_L        0x2E
#define ACCEL_YOUT_H        0x2F
#define ACCEL_YOUT_L        0x30
#define ACCEL_ZOUT_H        0x31
#define ACCEL_ZOUT_L        0x32
#define GYRO_XOUT_H         0x33
#define GYRO_XOUT_L         0x34
#define GYRO_YOUT_H         0x35
#define GYRO_YOUT_L         0x36
#define GYRO_ZOUT_H         0x37
#define GYRO_ZOUT_L         0x38
#define REG_BANK_SEL        0x7F

// BANK 2 REGISTERS
#define ACCEL_CONFIG        0x14
#define GYRO_CONFIG         0x01

// VALUES
#define WHOAMI_VALUE        0xEA

#define BANK_0              0x00
#define BANK_1              0x10
#define BANK_2              0x20
#define BANK_3              0x30


/*
 // MPU9250
#define  ACCEL_OUT         0x3B
#define  TEMP_OUT          0x41
#define  GYRO_OUT          0x43
#define  EXT_SENS_DATA_00  0x49

#define  SMPDIV            0x19
#define  CONFIG            0x1A
#define  ACCEL_CONFIG2     0x1D
#define  INT_PIN_CFG       0x37
#define  INT_ENABLE        0x38
#define  INT_DISABLE       0x00
#define  INT_PULSE_50US    0x00
#define  INT_WOM_EN        0x40
#define  INT_RAW_RDY_EN    0x01
#define  PWR_CYCLE         0x20
#define  DIS_GYRO          0x07

#define  I2C_MST_EN        0x20
#define  I2C_MST_CLK       0x0D
#define  I2C_MST_CTRL      0x24
#define  I2C_SLV0_ADDR     0x25
#define  I2C_SLV0_REG      0x26
#define  I2C_SLV0_DO       0x63
#define  I2C_SLV0_CTRL     0x27
#define  I2C_SLV0_EN       0x80
#define  I2C_READ_FLAG     0x80
#define  MOT_DETECT_CTRL   0x69
#define  ACCEL_INTEL_EN    0x80
#define  ACCEL_INTEL_MODE  0x40
#define  LP_ACCEL_ODR      0x1E
#define  WOM_THR           0x1F
#define  FIFO_EN           0x23
#define  FIFO_TEMP         0x80
#define  FIFO_GYRO         0x70
#define  FIFO_ACCEL        0x08
#define  FIFO_MAG          0x01
#define  FIFO_COUNT        0x72
#define  FIFO_READ         0x74

#define  AK8963_I2C_ADDR   0x0C
#define  AK8963_HXL        0x03
#define  AK8963_CNTL1      0x0A
#define  AK8963_PWR_DOWN   0x00
#define  AK8963_CNT_MEAS1  0x12
#define  AK8963_CNT_MEAS2  0x16
#define  AK8963_FUSE_ROM   0x0F
#define  AK8963_CNTL2      0x0B
#define  AK8963_RESET      0x01
#define  AK8963_ASA        0x10
#define  AK8963_WHO_AM_I   0x00
*/

/*
 // VARIABLES
 extern uint8_t _buffer[];
 
 float _accel_scale[NUM_IMU];
 float _gyro_scale[NUM_IMU];
 float _mag_scale_x[NUM_IMU];
 float _mag_scale_y[NUM_IMU];
 float _mag_scale_z[NUM_IMU];
 
 a_range_t _accel_range[NUM_IMU];
 g_range_t _gyro_range[NUM_IMU];
 
 uint8_t _fchoice_accel[NUM_IMU];
 uint8_t _fchoice_gyro[NUM_IMU];
 
 lpf_accel_bw_t _accel_lpf_bandwidth[NUM_IMU];
 lpf_gyro_bw_t _gyro_lpf_bandwidth[NUM_IMU];
 
 uint8_t _srd[NUM_IMU];
 
 int32_t _gx_bias[NUM_IMU];
 int32_t _gy_bias[NUM_IMU];
 int32_t _gz_bias[NUM_IMU];
 
 
 // FUNCTIONS
 void m_icm20948_init();
 void m_mpu9250_set_accel(uint8_t, a_range_t);
 void m_mpu9250_set_gyro(uint8_t, g_range_t);
 void m_mpu9250_set_accel_lpf(uint8_t, lpf_accel_bw_t);
 void m_mpu9250_set_gyro_lpf(uint8_t, lpf_gyro_bw_t);
 void m_mpu9250_fast_mode(uint8_t);
 void m_mpu9250_fifo_on(uint8_t);
 void m_read_spi_mag_registers(m2_gpio_t, uint8_t, uint8_t, uint8_t*);
 void m_write_spi_mag_register(m2_gpio_t, uint8_t, uint8_t);
 void m_read_spi_registers(m2_gpio_t, uint8_t, uint8_t, uint8_t*);
 
 
 // PRIVATE FUNCTIONS
 void _m_ak8963_init();
 void _m_ak8963_init_1(uint8_t);
 void _m_ak8963_init_2(uint8_t);
 void _m_ak8963_init_3(uint8_t);
 void _m_ak8963_init_4(uint8_t);
 void _m_mpu9250_calibrate_gyro(uint8_t);
 void _blink_yes_or_no(bool);
 */

#endif

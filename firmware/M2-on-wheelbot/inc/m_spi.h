// -----------------------------------------------------------------------------
// M2 SPI communication subsystem
// version: 1.0
// date: March 5, 2019
// authors: J. Fiene & B. Wright
// -----------------------------------------------------------------------------

#ifndef m_spi__
#define m_spi__

#include "m_general.h"
#include <stdlib.h>

#define WRITE_FLAG  0x00
#define READ_FLAG   0x80

#define  CS_B0()  set(DDRB, 0)
#define  SELECT_B0()  clear(PORTB, 0)
#define  DESELECT_B0()  set(PORTB, 0)

#define  CS_B4()  set(DDRB, 4)
#define  SELECT_B4()  clear(PORTB, 4)
#define  DESELECT_B4()  set(PORTB, 4)

#define  CS_B5()  set(DDRB, 5)
#define  SELECT_B5()  clear(PORTB, 5)
#define  DESELECT_B5()  set(PORTB, 5)

#define  CS_B6()  set(DDRB, 6)
#define  SELECT_B6()  clear(PORTB, 6)
#define  DESELECT_B6()  set(PORTB, 6)

#define  CS_B7()  set(DDRB, 7)
#define  SELECT_B7()  clear(PORTB, 7)
#define  DESELECT_B7()  set(PORTB, 7)

#define  CS_C6()  set(DDRC, 6)
#define  SELECT_C6()  clear(PORTC, 6)
#define  DESELECT_C6()  set(PORTC, 6)

#define  CS_C7()  set(DDRC, 7)
#define  SELECT_C7()  clear(PORTC, 7)
#define  DESELECT_C7()  set(PORTC, 7)

#define  CS_D0()  set(DDRD, 0)
#define  SELECT_D0()  clear(PORTD, 0)
#define  DESELECT_D0()  set(PORTD, 0)

#define  CS_D1()  set(DDRD, 1)
#define  SELECT_D1()  clear(PORTD, 1)
#define  DESELECT_D1()  set(PORTD, 1)

#define  CS_D2()  set(DDRD, 2)
#define  SELECT_D2()  clear(PORTD, 2)
#define  DESELECT_D2()  set(PORTD, 2)

#define  CS_D3()  set(DDRD, 3)
#define  SELECT_D3()  clear(PORTD, 3)
#define  DESELECT_D3()  set(PORTD, 3)

#define  CS_D4()  set(DDRD, 4)
#define  SELECT_D4()  clear(PORTD, 4)
#define  DESELECT_D4()  set(PORTD, 4)

#define  CS_D5()  set(DDRD, 5)
#define  SELECT_D5()  clear(PORTD, 5)
#define  DESELECT_D5()  set(PORTD, 5)

#define  CS_D6()  set(DDRD, 6)
#define  SELECT_D6()  clear(PORTD, 6)
#define  DESELECT_D6()  set(PORTD, 6)

#define  CS_D7()  set(DDRD, 7)
#define  SELECT_D7()  clear(PORTD, 7)
#define  DESELECT_D7()  set(PORTD, 7)

#define  CS_E6()  set(DDRE, 6)
#define  SELECT_E6()  clear(PORTE, 6)
#define  DESELECT_E6()  set(PORTE, 6)

#define  CS_F0()  set(DDRF, 0)
#define  SELECT_F0()  clear(PORTF, 0)
#define  DESELECT_F0()  set(PORTF, 0)

#define  CS_F1()  set(DDRF, 1)
#define  SELECT_F1()  clear(PORTF, 1)
#define  DESELECT_F1()  set(PORTF, 1)

#define  CS_F4()  set(DDRF, 4)
#define  SELECT_F4()  clear(PORTF, 4)
#define  DESELECT_F4()  set(PORTF, 4)

#define  CS_F5()  set(DDRF, 5)
#define  SELECT_F5()  clear(PORTF, 5)
#define  DESELECT_F5()  set(PORTF, 5)

#define  CS_F6()  set(DDRF, 6)
#define  SELECT_F6()  clear(PORTF, 6)
#define  DESELECT_F6()  set(PORTF, 6)

#define  CS_F7()  set(DDRF, 7)
#define  SELECT_F7()  clear(PORTF, 7)
#define  DESELECT_F7()  set(PORTF, 7)

#ifndef CS_SETUP
#define  CS_SETUP(cs) \
do { \
switch (cs) \
{ \
case PIN_B0: \
DESELECT_B0(); \
CS_B0(); \
break; \
case PIN_B4: \
DESELECT_B4(); \
CS_B4(); \
break; \
case PIN_B5: \
DESELECT_B5(); \
CS_B5(); \
break; \
case PIN_B6: \
DESELECT_B6(); \
CS_B6(); \
break; \
case PIN_B7: \
DESELECT_B7(); \
CS_B7(); \
break; \
case PIN_C6: \
DESELECT_C6(); \
CS_C6(); \
break; \
case PIN_C7: \
DESELECT_C7(); \
CS_C7(); \
break; \
case PIN_D0: \
DESELECT_D0(); \
CS_D0(); \
break; \
case PIN_D1: \
DESELECT_D1(); \
CS_D1(); \
break; \
case PIN_D2: \
DESELECT_D2(); \
CS_D2(); \
break; \
case PIN_D3: \
DESELECT_D3(); \
CS_D3(); \
break; \
case PIN_D4: \
DESELECT_D4(); \
CS_D4(); \
break; \
case PIN_D5: \
DESELECT_D5(); \
CS_D5(); \
break; \
case PIN_D6: \
DESELECT_D6(); \
CS_D6(); \
break; \
case PIN_D7: \
DESELECT_D7(); \
CS_D7(); \
break; \
case PIN_E6: \
DESELECT_E6(); \
CS_E6(); \
break; \
case PIN_F0: \
DESELECT_F0(); \
CS_F0(); \
break; \
case PIN_F1: \
DESELECT_F1(); \
CS_F1(); \
break; \
case PIN_F4: \
DESELECT_F4(); \
CS_F4(); \
break; \
case PIN_F5: \
DESELECT_F5(); \
CS_F5(); \
break; \
case PIN_F6: \
DESELECT_F6(); \
CS_F6(); \
break; \
case PIN_F7: \
DESELECT_F7(); \
CS_F7(); \
break; \
} \
} while (0)
#endif  // CHIP_SELECT


#ifndef CHIP_SELECT
#define  CHIP_SELECT(cs) \
do { \
switch (cs) \
{ \
case PIN_B0: \
SELECT_B4(); \
break; \
case PIN_B4: \
SELECT_B4(); \
break; \
case PIN_B5: \
SELECT_B5(); \
break; \
case PIN_B6: \
SELECT_B6(); \
break; \
case PIN_B7: \
SELECT_B7(); \
break; \
case PIN_C6: \
SELECT_C6(); \
break; \
case PIN_C7: \
SELECT_C7(); \
break; \
case PIN_D0: \
SELECT_D0(); \
break; \
case PIN_D1: \
SELECT_D1(); \
break; \
case PIN_D2: \
SELECT_D2(); \
break; \
case PIN_D3: \
SELECT_D3(); \
break; \
case PIN_D4: \
SELECT_D4(); \
break; \
case PIN_D5: \
SELECT_D5(); \
break; \
case PIN_D6: \
SELECT_D6(); \
break; \
case PIN_D7: \
SELECT_D7(); \
break; \
case PIN_E6: \
SELECT_E6(); \
break; \
case PIN_F0: \
SELECT_F0(); \
break; \
case PIN_F1: \
SELECT_F1(); \
break; \
case PIN_F4: \
SELECT_F4(); \
break; \
case PIN_F5: \
SELECT_F5(); \
break; \
case PIN_F6: \
SELECT_F6(); \
break; \
case PIN_F7: \
SELECT_F7(); \
break; \
} \
} while (0)
#endif  // CHIP_SELECT

#ifndef CHIP_DESELECT
#define  CHIP_DESELECT(cs) \
do { \
switch (cs) \
{ \
case PIN_B0: \
DESELECT_B0(); \
break; \
case PIN_B4: \
DESELECT_B4(); \
break; \
case PIN_B5: \
DESELECT_B5(); \
break; \
case PIN_B6: \
DESELECT_B6(); \
break; \
case PIN_B7: \
DESELECT_B7(); \
break; \
case PIN_C6: \
DESELECT_C6(); \
break; \
case PIN_C7: \
DESELECT_C7(); \
break; \
case PIN_D0: \
DESELECT_D0(); \
break; \
case PIN_D1: \
DESELECT_D1(); \
break; \
case PIN_D2: \
DESELECT_D2(); \
break; \
case PIN_D3: \
DESELECT_D3(); \
break; \
case PIN_D4: \
DESELECT_D4(); \
break; \
case PIN_D5: \
DESELECT_D5(); \
break; \
case PIN_D6: \
DESELECT_D6(); \
break; \
case PIN_D7: \
DESELECT_D7(); \
break; \
case PIN_E6: \
DESELECT_E6(); \
break; \
case PIN_F0: \
DESELECT_F0(); \
break; \
case PIN_F1: \
DESELECT_F1(); \
break; \
case PIN_F4: \
DESELECT_F4(); \
break; \
case PIN_F5: \
DESELECT_F5(); \
break; \
case PIN_F6: \
DESELECT_F6(); \
break; \
case PIN_F7: \
DESELECT_F7(); \
break; \
} \
} while (0)
#endif  // CHIP_DESELECT


typedef enum
{
  SPI_125KHZ = 0,
  SPI_250KHZ,
  SPI_500KHZ,
  SPI_1MHZ,
  SPI_2MHZ,
  SPI_4MHZ,
  SPI_8MHZ
} spi_freq_t;

typedef enum
{
    PIN_B0,
    PIN_B4,
    PIN_B5,
    PIN_B6,
    PIN_B7,
    PIN_C6,
    PIN_C7,
    PIN_D0,
    PIN_D1,
    PIN_D2,
    PIN_D3,
    PIN_D4,
    PIN_D5,
    PIN_D6,
    PIN_D7,
    PIN_E6,
    PIN_F0,
    PIN_F1,
    PIN_F4,
    PIN_F5,
    PIN_F6,
    PIN_F7
} m2_gpio_t;

spi_freq_t _spi_freq;

// EXTERNAL FUNCTIONS
void m_spi_init();
void m_spi_cs_setup(m2_gpio_t cs_pin);
void m_spi_speed(spi_freq_t);
void m_spi_write_register(m2_gpio_t cs_pin, uint8_t reg, uint8_t val);
uint8_t m_spi_read_register(m2_gpio_t cs_pin, uint8_t reg);
void m_spi_read_registers(m2_gpio_t cs_pin, uint8_t start_reg, uint8_t count, uint8_t *dest);
void m_spi_shift_buffers(m2_gpio_t cs_pin, uint8_t *tx, uint8_t *rx, uint16_t bytes);
void m_spi_shift_MD(m2_gpio_t cs_pin, uint8_t *tx, uint8_t *rx, uint8_t bytes);


// INTERNAL FUNCTIONS
uint8_t read_spi_byte();
void write_spi_byte(uint8_t);
uint8_t exchange_spi_byte(uint8_t);

#endif

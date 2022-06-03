// -----------------------------------------------------------------------------
// M2 SPI communication subsystem
// version: 1.0
// date: March 5, 2019
// authors: J. Fiene & B. Wright
// -----------------------------------------------------------------------------

#include "config.h"
#include "m_spi.h"
//#include "m_usb.h"

void m_spi_init()
{
    // DISABLE SPI POWER REDUCTION
    clear(PRR0, PRSPI);

	if ( PCB_VERSION == 3 )
	{
		// IMUs
		set(DDRD, 6);  // CS output
		set(PORTD, 6); // CS high
		set(DDRD, 7);  // CS output
		set(PORTD, 7); // CS high
		set(DDRF, 6);  // CS output
		set(PORTF, 6); // CS high
		set(DDRF, 7);  // CS output
		set(PORTF, 7); // CS high

		// udriver
		set(DDRD, 4);  // CS output
		set(PORTD, 4); // CS high
	}
	else if ( PCB_VERSION == 0 )
	{
		// IMUs
		set(DDRD, 6);  // CS output
		set(PORTD, 6); // CS high
		set(DDRD, 7);  // CS output
		set(PORTD, 7); // CS high
		set(DDRD, 3);  // CS output
		set(PORTD, 3); // CS high
		set(DDRD, 4);  // CS output
		set(PORTD, 4); // CS high

		 // udriver
		set(DDRD, 5);  // CS output
		set(PORTD, 5); // CS high
	}

    // default SS need to be output and low too, otherwise can change into slave mode
    set(DDRB, 0);    // CS output
    set(PORTB, 0);   // CS high

    set(DDRB, 1);       // SCLK output
    clear(PORTB, 1);    // SCLK starts low

    set(DDRB, 2);       // MOSI output
    set(PORTB, 2);      // MOSI starts high

    clear(DDRB, 3);     // MISO input
	set(PORTB, 3);    // MISO pull-up enabled?

    // ENABLE IN MASTER MODE
    set(SPCR, SPE);
    set(SPCR, MSTR);

    clear(SPCR,CPOL); // clock is low when idle (clear = default
    clear(SPCR,CPHA); // cpol=0,cpha=0 is defualt but done anyway
}

void m_spi_cs_setup(m2_gpio_t cs_pin){
    CS_SETUP(cs_pin);
}

void m_spi_speed(spi_freq_t spi_frequency)
{
  switch (spi_frequency)
  {
    case SPI_125KHZ:
      set(SPCR, SPR0);
      set(SPCR, SPR1);
      clear(SPSR, SPI2X);
      break;
    case SPI_250KHZ:
      clear(SPCR, SPR0);
      set(SPCR, SPR1);
      clear(SPSR, SPI2X);
      break;
    case SPI_500KHZ:
      clear(SPCR, SPR0);
      set(SPCR, SPR1);
      set(SPSR, SPI2X);
      break;
    case SPI_1MHZ:
      set(SPCR, SPR0);
      clear(SPCR, SPR1);
      clear(SPSR, SPI2X);
      break;
    case SPI_2MHZ:
      set(SPCR, SPR0);
      clear(SPCR, SPR1);
      set(SPSR, SPI2X);
      break;
    case SPI_4MHZ:
      clear(SPCR, SPR0);
      clear(SPCR, SPR1);
      clear(SPSR, SPI2X);
      break;
    case SPI_8MHZ:
      clear(SPCR, SPR0);
      clear(SPCR, SPR1);
      set(SPSR, SPI2X);
      break;
  }
  _spi_freq = spi_frequency;
}

void m_spi_write_register(m2_gpio_t cs_pin, uint8_t reg, uint8_t val)
{
    CHIP_SELECT(cs_pin);
    write_spi_byte(reg | WRITE_FLAG);
    write_spi_byte(val);
    CHIP_DESELECT(cs_pin);
    //read_spi_byte();  // TODO
}

uint8_t m_spi_read_register(m2_gpio_t cs_pin, uint8_t reg)
{
    uint8_t response = 0xFF;
    CHIP_SELECT(cs_pin);
    write_spi_byte(reg | READ_FLAG);
    response = read_spi_byte(); // deadlock
    CHIP_DESELECT(cs_pin);
    //read_spi_byte();  // TODO
    return response;
}

void m_spi_read_registers(m2_gpio_t cs_pin, uint8_t start_reg, uint8_t count, uint8_t *dest)
{
    uint8_t i;
    CHIP_SELECT(cs_pin);
    write_spi_byte(start_reg | READ_FLAG);
    for (i = 0; i < count; i++)
    {
        dest[i] = read_spi_byte();
    }
    CHIP_DESELECT(cs_pin);
    //read_spi_byte();  // TODO
}

void m_spi_shift_buffers(m2_gpio_t cs_pin, uint8_t *tx, uint8_t *rx, uint16_t bytes){
//    m_usb_tx_string("ssb");
    uint16_t i;
    CHIP_SELECT(cs_pin);
    for( i = 0 ; i < bytes ; i++ ){
//        m_usb_tx_int(i);
        rx[i] = exchange_spi_byte(tx[i]);
    }
    CHIP_DESELECT(cs_pin);
//    m_usb_tx_string("\n");
}

uint8_t read_spi_byte()
{
    SPDR = 0xFF;
    while(!(SPSR & (1<<SPIF)));
    clear(SPSR, SPIF);
    return SPDR;
}

void write_spi_byte(uint8_t byte)
{
    SPDR = byte;
    while(!check(SPSR, SPIF));
    clear(SPSR, SPIF);
}

uint8_t exchange_spi_byte(uint8_t byte)
{
    SPDR = byte;
    while(!check(SPSR, SPIF));
    clear(SPSR, SPIF);
    return SPDR;
}

void m_spi_shift_MD(m2_gpio_t cs_pin, uint8_t *tx, uint8_t *rx, uint8_t bytes)
{
    int i;
    CHIP_SELECT(cs_pin);

    for(i=0; i<bytes; i++){
        SPDR = (char)(tx[i]); // low byte
        while (!check(SPSR, SPIF));
        rx[i] = SPDR;
    }

    CHIP_DESELECT(cs_pin);
}

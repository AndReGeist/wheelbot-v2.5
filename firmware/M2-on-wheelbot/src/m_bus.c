// -----------------------------------------------------------------------------
// M2 data bus subsystem
// version: 2.0
// date: May 30, 2011
// author: J. Fiene
// -----------------------------------------------------------------------------

#include "m_bus.h"

// private function prototypes:
unsigned char twi_start(unsigned char address, unsigned char readwrite);
unsigned char twi_send_byte(unsigned char byte);
unsigned char twi_read_byte(void);
void twi_stop(void);
unsigned char twi_wait_for_ack(void);


// PUBLIC FUNCTIONS
void m_bus_init(void)
{
	// ENABLE PULLUPS
	set(PORTD,0);
	set(PORTD,1);
	set(PORTD,2);
	
	// CONFIGURE THE CLOCK
	TWBR = 12;	// CLK freq = CPU clock / (16 + 2*TWBR*(4^TWPS)), 16MHz clock, TWBR=12, TWPS=00 -> 400kHz

	// ENABLE interrupts in INT2 (D2)
	set(EICRA,ISC21); clear(EICRA,ISC20); // trigger on falling edge
	set(EIMSK,INT2); // demask the interrupt
	sei(); // enable global interrupts
}

unsigned char m_read_register(unsigned char addr, unsigned char reg)
{
    if(!twi_start(addr,WRITE)) return 0; // START + W
	if(!twi_send_byte(reg)) return 0;    // register to read
    twi_stop();                          // STOP
    if(!twi_start(addr,READ)) return 0;  // START + R
	return(twi_read_byte());             // return register value
}

unsigned char m_write_register(unsigned char addr, unsigned char reg, unsigned char value)
{
    if(!twi_start(addr,WRITE)) return 0; // START + W
	if(!twi_send_byte(reg)) return 0;    // register to write to
    if(!twi_send_byte(value)) return 0;  // value
    twi_stop();                          // STOP
    return(1);
}

// PRIVATE FUNCTIONS:

// TWI: send START condition, wait for ACK, send ADDRESS with R/W flag
// readwrite = 1 for read, 0 for write
unsigned char twi_start(unsigned char address, unsigned char readwrite)
{
    unsigned char status;
    
    // START packet:
    TWCR = (1<<TWEN)|(1<<TWSTA)|(1<<TWINT);
    if(!twi_wait_for_ack()){
        twi_stop();
        return 0; // COMM failure
    }
    
    // ADDRESS packet:
    if(readwrite)
    { // READ
        status = twi_send_byte(((address<<1) + 1));
        if(status== 0x48){ // ACK was not received - may not be connected/listening
            twi_stop();    
            return 0;	// failure
        }	
    } else { // WRITE
        status = twi_send_byte(address<<1);
        if(status== 0x20){ // ACK was not received - may not be connected/listening
            twi_stop();
            return 0;	// failure
        }	        
    }
    return 1;	// success
    
}

// TWI: send BYTE, wait for ACK
unsigned char twi_send_byte(unsigned char byte)
{
	TWDR = byte;					// load the byte
	TWCR = (1<<TWINT) | (1<<TWEN);	// send the byte
	return (twi_wait_for_ack()) ? (TWSR & 0xF8) : 0 ;
}

// TWI: read BYTE, NACK, STOP
unsigned char twi_read_byte(void)
{
    TWCR = (1<<TWINT) | (1<<TWEN);	// clear the flag, NACK, and wait for another byte
    while(!(TWCR & (1<<TWINT))){};  // wait for an interrupt to signal that a new byte is available
    return TWDR;
    twi_stop();
}

// TWI: send STOP condition
void twi_stop(void)
{
	TWCR = (1<<TWINT)|(1<<TWEN)| (1<<TWSTO);
}

// TWI:  wait for ACK packet (0=fail, 1=success)
unsigned char twi_wait_for_ack(void)
{
	unsigned int wait=0;
	while((!(TWCR & (1<<TWINT))) && (wait++<MAX_WAIT)){};	// wait for acknowledgement that the byte was sent
    return (wait==MAX_WAIT? 0 : 1);
}
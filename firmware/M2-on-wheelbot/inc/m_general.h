// -----------------------------------------------------------------------------
// MAEVARM general macros
// version: 2.1
// date: Sept 29, 2011
// author: J. Fiene
// -----------------------------------------------------------------------------
#ifndef m_general__
#define m_general__

// -----------------------------------------------------------------------------
// Useful pre-compile constants
// -----------------------------------------------------------------------------

#define TRUE	1
#define FALSE	0

#define OFF		0
#define ON		1
#define TOGGLE	2

// -----------------------------------------------------------------------------
// General AVR libraries that we'll need at times:
// -----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>


// -----------------------------------------------------------------------------
// Bit manipulation and validation:
// -----------------------------------------------------------------------------

#define set(reg,bit)		reg |= (1<<(bit))
#define clear(reg,bit)		reg &= ~(1<<(bit))
#define toggle(reg,bit)		reg ^= (1<<(bit))

#define check(reg,bit)		(bool)(reg & (1<<(bit)))
// As of version 2.0, this will return either 1 (TRUE) or 0 (FALSE)


// -----------------------------------------------------------------------------
// Disable JTAG to access F4-F7:
// -----------------------------------------------------------------------------

#define m_disableJTAG()		MCUCR = (1 << JTD); MCUCR = (1 << JTD)
// Setting the JTD bit in MCUCR twice within four clock cycles will allow user
// access to F4-F7 as normal port pins. Note that using |= is too slow for this
// operation to work correctly, so we are setting the entire register 
// (forutnately, all other bits in MCUCR are 0 anyway).


// -----------------------------------------------------------------------------
// Set the system clock:
// -----------------------------------------------------------------------------

#define m_clockdivide(val)	CLKPR = (1<<CLKPCE); CLKPR=val
// "val" must be an integer from 0 to 8
// this will divide the 16MHz system clock by 2^val:
// 0 = 16 MHz
// 1 = 8 MHz
// 2 = 4 MHz
// 3 = 2 MHz
// 4 = 1 MHz
// 5 = 500 kHz
// 6 = 250 kHz
// 7 = 125 kHz
// 8 = 62.5 kHz


// -----------------------------------------------------------------------------
// Wait for a specified number of milliseconds:
// -----------------------------------------------------------------------------

#define m_wait(val)			_delay_ms(val)
// "val" must be an integer from 1 to 65535
// this function assumes a 16MHz clock


// -----------------------------------------------------------------------------
// Change the state of the green on-board LED:
// -----------------------------------------------------------------------------

#define m_green(val)		set(DDRE,2); if(val==OFF){set(PORTE,2);}else if(val==ON){clear(PORTE,2);}else if(val==TOGGLE){toggle(PORTE,2);}
// "val" must be either OFF, ON, or TOGGLE, as defined above


// -----------------------------------------------------------------------------
// Change the state of the red on-board LED:
// -----------------------------------------------------------------------------

#define m_red(val)			set(DDRE,6); if(val==OFF){set(PORTE,6);}else if(val==ON){clear(PORTE,6);}else if(val==TOGGLE){toggle(PORTE,6);}
// "val" must be either OFF, ON, or TOGGLE, as defined above
// note that this takes over control of pin E6 and sets it to an output

#endif
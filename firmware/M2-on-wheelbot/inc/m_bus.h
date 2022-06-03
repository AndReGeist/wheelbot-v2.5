// -----------------------------------------------------------------------------
// M2 data bus subsystem
// version: 2.0
// date: June 30, 2011
// author: J. Fiene
// -----------------------------------------------------------------------------

#ifndef m_bus__
#define m_bus__

#include "m_general.h"
#include <util/twi.h>

#define MAX_WAIT        1000
#define INTERPACKET     10
#define READ            1
#define WRITE           0

// -----------------------------------------------------------------------------
// Public functions:
// -----------------------------------------------------------------------------

void m_bus_init(void);
// FUNCTIONALITY:
// initialize the M2 data bus, which uses pins D0-D2 and is available through the
// 5-pin end header.  When new data is available from a slave, the INT2_vect interrupt
// will be triggered, and you must act accordingly!
//
// TAKES:
// nothing
//
// RETURNS:
// nothing

unsigned char m_read_register(unsigned char addr, unsigned char reg);
// FUNCTIONALITY:
// sends [START + W] [register address] [STOP] [START + R]
//
// TAKES:
// addr - I2C slave address
// reg - register address
//
// RETURNS:
// register value

unsigned char m_write_register(unsigned char addr, unsigned char reg, unsigned char value);
// FUNCTIONALITY:
// sends [START + W] [register address] [value] [STOP]
//
// TAKES:
// addr - I2C slave address
// reg - register address
// value - value to place in register
//
// RETURNS:
// 1 - success
// 0 - communication error

#endif
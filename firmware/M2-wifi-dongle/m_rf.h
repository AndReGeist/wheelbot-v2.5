// -----------------------------------------------------------------------------
// M2 RF communication subsystem
// version: 2.0
// date: June 30, 2011
// author: J. Fiene
// -----------------------------------------------------------------------------

#ifndef m_rf__
#define m_rf__

#include "m_general.h"
#include "m_bus.h"

// -----------------------------------------------------------------------------
// Public functions:
// -----------------------------------------------------------------------------

char m_rf_open(char channel, char RXaddress, char packet_length);
// FUNCTIONALITY:
// configure the RF communications channel and place in RX mode, 
// which will take INT2 low when data is available
//
// TAKES:
// channel : 1 to 32 (must match between sender/receiver)
// RXaddress : (the module's unique RX address)
// packet_length : 1 to 32 (must match between sender/receiver)
// 
// RETURNS:
// 1 : module acknowledged setup
// 0 : something went wrong


char m_rf_read(char* buffer, char packet_length);
// FUNCTIONALITY
// get the message from the module's receive buffer
//
// TAKES:
// buffer : (pointer to the first element of a buffer that is packet_length long)
// packet_length : 1 to 32 (must match how it was set up with m_rf_open!)
//
// RETURNS:
// 1 : something was read
// 0 : not connected, nothing to read, or buffer length mismatch


char m_rf_send(char TXaddress, char* buffer, char packet_length);
// FUNCTIONALITY:
// take the transmitter out of receive mode
// send a message to a specified receiver
// wait for an ACK packet
// resend up to three times
// return the transmitter to receive mode

// TAKES:
// buffer : (pointer to the first element of a buffer that is packet_length long)
// TXaddress : (the receiving module's unique RX address)
//
// RETURNS:
// 1 : indicates successful transmission to the mRF module (does NOT indicate receipt by receiver)
// 0 : something went wrong

#endif
// -----------------------------------------------------------------------------
// M2 RF communication subsystem
// version: 2.0
// date: June 30, 2011
// author: J. Fiene
// -----------------------------------------------------------------------------

#include "m_rf.h"

#define MRFTWIADDR		0x28
#define MRFINIT			0x01
#define	MRFREAD			0x02
#define MRFSEND			0x03

char m_rf_open(char channel, char RXaddress, char packet_length)
{	
	// START | MRFTWIADDR | MRFINIT | channel | RXaddress | packet_length | STOP
	
	m_bus_init();
	
	// START
	TWCR = (1<<TWEN)|(1<<TWSTA)|(1<<TWINT);
	while(!(TWCR & (1<<TWINT))){};
	
	// ADDRESS
	TWDR = MRFTWIADDR<<1;
	TWCR = (1<<TWINT) | (1<<TWEN);
	while(!(TWCR & (1<<TWINT))){};
	if((TWSR & 0xF8)== 0x20){ // ACK was not received - may not be connected/listening
		TWCR = (1<<TWINT)|(1<<TWEN)| (1<<TWSTO); // let go of the line (STOP)
		return 0;
	}
	
	// SEND THE DESIRED MRF MODE
	TWDR = MRFINIT;
	TWCR = (1<<TWINT) | (1<<TWEN);
	while(!(TWCR & (1<<TWINT))){};
	
	// SEND DATA
	TWDR = channel;
	TWCR = (1<<TWINT) | (1<<TWEN);
	while(!(TWCR & (1<<TWINT))){};
	
	TWDR = RXaddress;
	TWCR = (1<<TWINT) | (1<<TWEN);
	while(!(TWCR & (1<<TWINT))){};
	
	TWDR = packet_length;
	TWCR = (1<<TWINT) | (1<<TWEN);
	while(!(TWCR & (1<<TWINT))){};
	
	// STOP
	TWCR = (1<<TWINT)|(1<<TWEN)| (1<<TWSTO);
	
	return 1;
}

char m_rf_read(char* buffer, char packet_length)
{
	// START | MRFTWIADDR (in MR mode) | (BYTES) | (DATA_0) | ... | (DATA_N) | STOP

	char bytes;
	int i;

	// START
	TWCR = (1<<TWEN)|(1<<TWSTA)|(1<<TWINT);
	while(!(TWCR & (1<<TWINT))){};
	
	// ADDRESS (in Master-Receiver Mode)
	TWDR = ((MRFTWIADDR<<1)|1);
	TWCR = (1<<TWINT) | (1<<TWEN);
	while(!(TWCR & (1<<TWINT))){}; // wait until finished sending address
	if((TWSR & 0xF8)== 0x48){ // ACK was not received
		TWCR = (1<<TWINT)|(1<<TWEN)| (1<<TWSTO); // let go of the line (STOP)
		return 0; // not connected/listening
	}
	

	// (BYTES)
	TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN);	// clear the flag, enable ACKs, and wait for another byte
	while(!(TWCR & (1<<TWINT))){}; // wait for an interrupt to signal that a new byte is available
	bytes = TWDR;
	if(bytes != packet_length){
		TWCR = (1<<TWINT)|(1<<TWEN)| (1<<TWSTO); // let go of the line (STOP)
		return 0; // indicate length mismatch
	}		
	
	// (DATA_0 ... DATA_N)
	for(i=0;i<(bytes-1);i++)
	{
		TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN);	// clear the flag, enable ACKs, and wait for another byte
		while(!(TWCR & (1<<TWINT))){}; // wait for an interrupt to signal that a new byte is available
		buffer[i] = TWDR;
	}
	TWCR = (1<<TWINT) | (1<<TWEN);	// clear the flag, no ACK, and wait for another byte
	while(!(TWCR & (1<<TWINT))){}; // wait for an interrupt to signal that a new byte is available
	buffer[i++] = TWDR;
	
	// STOP
	TWCR = (1<<TWINT)|(1<<TWEN)| (1<<TWSTO);

	return 1;
}

char m_rf_send(char TXaddress, char* buffer, char packet_length)
{
	// START | MRFTWIADDR | MRFSEND | TXaddress | DATA_0 | ... | DATA_N | STOP
	
	int i;
	
	// DISABLE INTERRUPTS
	cli();
	
	// START
	TWCR = (1<<TWEN)|(1<<TWSTA)|(1<<TWINT);
	while(!(TWCR & (1<<TWINT))){};

	// ADDRESS
	TWDR = MRFTWIADDR<<1;
	TWCR = (1<<TWINT) | (1<<TWEN);
	while(!(TWCR & (1<<TWINT))){};
	if((TWSR & 0xF8)== 0x20){ // ACK was not received - may not be connected/listening
		TWCR = (1<<TWINT)|(1<<TWEN)| (1<<TWSTO); // let go of the line (STOP)
		sei();	// re-enable interrupts
		return 0;
	}
	
	// SEND THE DESIRED MRF MODE (SEND)
	TWDR = MRFSEND;
	TWCR = (1<<TWINT) | (1<<TWEN);
	while(!(TWCR & (1<<TWINT))){};
	
	// SEND THE DESIRED MRF MODE (SEND)
	TWDR = TXaddress;
	TWCR = (1<<TWINT) | (1<<TWEN);
	while(!(TWCR & (1<<TWINT))){};

	// SEND DATA
	for(i=0;i<packet_length;i++){
		TWDR = buffer[i];
		TWCR = (1<<TWINT) | (1<<TWEN);
		while(!(TWCR & (1<<TWINT))){};
	}
	
	// STOP
	TWCR = (1<<TWINT)|(1<<TWEN)| (1<<TWSTO);
	
	// RE-ENABLE INTERRUPTS
	sei();
	
	return 1;
}


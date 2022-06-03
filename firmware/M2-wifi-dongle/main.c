#include "m_general.h"
#include "m_usb.h"
#include "m_bus.h"
#include "m_rf.h"
#include "m_crc32.h"

#define CHANNEL 1
#define RXADDRESS 0x54
#define TXADDRESS 0xF4
#define PACKET_LENGTH_2PC 32
#define PACKET_LENGTH_2WB 32

bool check_crc( uint8_t *data_in_ptr );
void read_usb( uint8_t *buffer );

volatile bool new_data;
uint8_t data_2PC[PACKET_LENGTH_2PC] = {0}; // Wifi data coming FROM Wheelbot
uint8_t data_2WB[PACKET_LENGTH_2WB] = {0}; // Wifi data send TO wheelbot
uint32_t *crc_2WB_ptr  = (uint32_t *)&data_2WB[PACKET_LENGTH_2WB-4];
bool crc_2PC_OK;

uint16_t counter = 0;  // counter used to count control loops


int main(void)
{
    // INITIALIZE
	m_red(ON);                                  // use the red LED to verify that we complete initialization
	m_clockdivide(0);                           // 16 Mhz
	m_disableJTAG();                            // access to F port pins
	m_rf_open(CHANNEL, RXADDRESS, PACKET_LENGTH_2PC); // enable mRF
	m_usb_init();                               // enable USB
	while(!m_usb_isconnected());                // wait for USB enumeration
    
    //for(i=0;i<PACKET_LENGTH_2PC;i++){               // send an all-zero data_2WB to PC
    //    m_usb_tx_hex8(data_2PC[i]);
    //}
	m_usb_tx_char('X');
	usb_serial_write( data_2PC, PACKET_LENGTH_2PC );
    m_red(OFF);
    
	while(1){
		++counter;

		if( m_usb_rx_available() )
		{
			read_usb( &data_2WB[0] );
		}

        if( new_data ){
			new_data = FALSE;

			// RECEIVE DATA FROM WHEELBOT
			m_rf_read( (char *) data_2PC, PACKET_LENGTH_2PC );
			//if ( (counter % 100)==0 ){ data_2PC[4] = 'J'; } // DEBUG - Test CRC
			crc_2PC_OK = check_crc( data_2PC );

			/* CODE FOR DEBUGING CRC
			data_2PC[16] = crc_2PC[0]; data_2PC[17] = crc_2PC[1];
			data_2PC[18] = crc_2PC[2]; data_2PC[19] = crc_2PC[3];
			*/
			//data_2PC[21] = crc_2PC_OK; // DEBUG 3
			//data_2PC[20] = 0;
			//crc_2PC_OK = TRUE;

			if( crc_2PC_OK )
			{
				data_2PC[PACKET_LENGTH_2PC-2] = 0;
				data_2PC[PACKET_LENGTH_2PC-1] = 0;
			}
			else if( !crc_2PC_OK ) // If !crc_2PC_OK set 'crc2' to 1
			{
				data_2PC[PACKET_LENGTH_2PC-2] = 1;
				data_2PC[PACKET_LENGTH_2PC-1] = 0;
			}
			m_usb_tx_char('X');
			usb_serial_write( data_2PC, PACKET_LENGTH_2PC );
			m_green( TOGGLE ); // Flash green if wifi data is coming in


			// SEND DATA TO WHEELBOT
			m_wait(0.1);
			*crc_2WB_ptr  = m_crc32((uint8_t *)data_2WB, PACKET_LENGTH_2WB-4);
			m_rf_send(TXADDRESS, (char *) data_2WB, PACKET_LENGTH_2WB);
			m_red( TOGGLE ); // Flash red if wifi data is going out
        } // endif newdata
	} // close while
}  // close main

ISR(INT2_vect){                                 // thrown on new-data_2WB receipt
    new_data = TRUE;
}

bool check_crc( uint8_t *data_2PC_ptr )
{ // Check crc of incoming message
	uint8_t crc_2PC[4] = {0};
	uint32_t *crc_2PC_ptr = (uint32_t *)&crc_2PC[0];

	*crc_2PC_ptr = m_crc32( data_2PC_ptr, PACKET_LENGTH_2PC-4 );

	bool crc_OK;
	crc_OK = (( crc_2PC[0] == data_2PC_ptr[PACKET_LENGTH_2PC-4] )
			  && ( crc_2PC[1] == data_2PC_ptr[PACKET_LENGTH_2PC-3] )
			  && ( crc_2PC[2] == data_2PC_ptr[PACKET_LENGTH_2PC-2] )
			  && ( crc_2PC[3] == data_2PC_ptr[PACKET_LENGTH_2PC-1] ));

	return crc_OK;
}

void read_usb(uint8_t *buffer) { // Reads in chars if the first char is an 'X'
		if(m_usb_rx_char() == 'X')	// new packet incoming!
		{
			int i;
			for(i=0; i < PACKET_LENGTH_2WB; i++)
			{
				while(!m_usb_rx_available());	// wait for byte
				buffer[i] = m_usb_rx_char();
			}
		}
}

void data_write(char *packet, int index, float scaling, float data) {

	int temp = 0;

	temp = (int)(scaling * data);
	packet[index] = *(((char *)&temp) + 1);
	packet[index+1] = *(char *)&temp;
}
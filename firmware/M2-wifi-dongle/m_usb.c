#define USB_SERIAL_PRIVATE_INCLUDE
#include "m_usb.h"

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

// ---- OVERLOADS FOR M1 BACK COMPATIBILITY ----
#define usb_init()			m_usb_init()
#define usb_configured()	m_usb_isconnected()

#define usb_rx_available()	m_usb_rx_available()		   		   
#define usb_rx_flush()		m_usb_rx_flush()		   			   
#define usb_rx_char()		m_usb_rx_char()

#define usb_tx_char(val)	m_usb_tx_char(val)
#define usb_tx_hex(val)		m_usb_tx_hex(val)
#define usb_tx_decimal(val)	m_usb_tx_uint(val)
#define usb_tx_string(val)	m_usb_tx_string(val)
#define usb_tx_push()		m_usb_tx_push()

#define m_usb_rx_ascii()	m_usb_rx_char()
#define m_usb_tx_ascii(val)	m_usb_tx_char(val)


// EVERYTHING ELSE *****************************************************************

// setup

int8_t usb_serial_putchar(uint8_t c);	// transmit a character
int8_t usb_serial_putchar_nowait(uint8_t c);  // transmit a character, do not wait
int8_t usb_serial_write(const uint8_t *buffer, uint16_t size); // transmit a buffer
void print_P(const char *s);
void phex(unsigned char c);
void phex16(unsigned int i);
void m_usb_tx_hex8(unsigned char i);
void m_usb_tx_push(void);	       			   


// serial parameters
uint32_t usb_serial_get_baud(void);	// get the baud rate
uint8_t usb_serial_get_stopbits(void);	// get the number of stop bits
uint8_t usb_serial_get_paritytype(void);// get the parity type
uint8_t usb_serial_get_numbits(void);	// get the number of data bits
uint8_t usb_serial_get_control(void);	// get the RTS and DTR signal state
int8_t usb_serial_set_control(uint8_t signals); // set DSR, DCD, RI, etc

// constants corresponding to the various serial parameters
#define USB_SERIAL_DTR			0x01
#define USB_SERIAL_RTS			0x02
#define USB_SERIAL_1_STOP		0
#define USB_SERIAL_1_5_STOP		1
#define USB_SERIAL_2_STOP		2
#define USB_SERIAL_PARITY_NONE		0
#define USB_SERIAL_PARITY_ODD		1
#define USB_SERIAL_PARITY_EVEN		2
#define USB_SERIAL_PARITY_MARK		3
#define USB_SERIAL_PARITY_SPACE		4
#define USB_SERIAL_DCD			0x01
#define USB_SERIAL_DSR			0x02
#define USB_SERIAL_BREAK		0x04
#define USB_SERIAL_RI			0x08
#define USB_SERIAL_FRAME_ERR		0x10
#define USB_SERIAL_PARITY_ERR		0x20
#define USB_SERIAL_OVERRUN_ERR		0x40

// This file does not include the HID debug functions, so these empty
// macros replace them with nothing, so users can compile code that
// has calls to these functions.
#define usb_debug_putchar(c)
#define usb_debug_flush_output()

#define EP_TYPE_CONTROL			0x00
#define EP_TYPE_BULK_IN			0x81
#define EP_TYPE_BULK_OUT		0x80
#define EP_TYPE_INTERRUPT_IN		0xC1
#define EP_TYPE_INTERRUPT_OUT		0xC0
#define EP_TYPE_ISOCHRONOUS_IN		0x41
#define EP_TYPE_ISOCHRONOUS_OUT		0x40
#define EP_SINGLE_BUFFER		0x02
#define EP_DOUBLE_BUFFER		0x06
#define EP_SIZE(s)	((s) == 64 ? 0x30 :	\
((s) == 32 ? 0x20 :	\
((s) == 16 ? 0x10 :	\
0x00)))

#define MAX_ENDPOINT		4

#define LSB(n) (n & 255)
#define MSB(n) ((n >> 8) & 255)

#define HW_CONFIG() (UHWCON = 0x01)

#ifdef M1
#define PLL_CONFIG() (PLLCSR = 0x02) // fixed to 8MHz clock
#else
#define PLL_CONFIG() (PLLCSR = 0x12) // 0001 0010 For a 16MHz clock
#endif

#define USB_CONFIG() (USBCON = ((1<<USBE)|(1<<OTGPADE)))
#define USB_FREEZE() (USBCON = ((1<<USBE)|(1<<FRZCLK)))

// standard control endpoint request types
#define GET_STATUS			0
#define CLEAR_FEATURE			1
#define SET_FEATURE			3
#define SET_ADDRESS			5
#define GET_DESCRIPTOR			6
#define GET_CONFIGURATION		8
#define SET_CONFIGURATION		9
#define GET_INTERFACE			10
#define SET_INTERFACE			11
// HID (human interface device)
#define HID_GET_REPORT			1
#define HID_GET_PROTOCOL		3
#define HID_SET_REPORT			9
#define HID_SET_IDLE			10
#define HID_SET_PROTOCOL		11
// CDC (communication class device)
#define CDC_SET_LINE_CODING		0x20
#define CDC_GET_LINE_CODING		0x21
#define CDC_SET_CONTROL_LINE_STATE	0x22


/**************************************************************************
 *
 *  Configurable Options
 *
 **************************************************************************/

#define STR_MANUFACTURER	L"J. Fiene"
#define STR_PRODUCT		L"M2"
#define STR_SERIAL_NUMBER	L"410"
#define VENDOR_ID		0x16C0 // must match INF file in Windows
#define PRODUCT_ID		0x047A // must match INF file in Windows
#define TRANSMIT_FLUSH_TIMEOUT	5   /* in milliseconds */
#define TRANSMIT_TIMEOUT	25   /* in milliseconds */
#define SUPPORT_ENDPOINT_HALT // can save 116 bytes by removing, makes fully USB compliant

/**************************************************************************
 *
 *  Endpoint Buffer Configuration
 *
 **************************************************************************/

#define ENDPOINT0_SIZE		16
#define CDC_ACM_ENDPOINT	2
#define CDC_RX_ENDPOINT		3
#define CDC_TX_ENDPOINT		4
#define CDC_ACM_SIZE		16
#define CDC_ACM_BUFFER		EP_SINGLE_BUFFER
#define CDC_RX_SIZE		64
#define CDC_RX_BUFFER 		EP_DOUBLE_BUFFER
#define CDC_TX_SIZE		64
#define CDC_TX_BUFFER		EP_DOUBLE_BUFFER

static const uint8_t PROGMEM endpoint_config_table[] = {
	0,
	1, EP_TYPE_INTERRUPT_IN,  EP_SIZE(CDC_ACM_SIZE) | CDC_ACM_BUFFER,
	1, EP_TYPE_BULK_OUT,      EP_SIZE(CDC_RX_SIZE) | CDC_RX_BUFFER,
	1, EP_TYPE_BULK_IN,       EP_SIZE(CDC_TX_SIZE) | CDC_TX_BUFFER
};


/**************************************************************************
 *
 *  Descriptor Data
 *
 **************************************************************************/

static const uint8_t PROGMEM device_descriptor[] = {
	18,					// bLength
	1,					// bDescriptorType
	0x00, 0x02,				// bcdUSB
	2,					// bDeviceClass
	0,					// bDeviceSubClass
	0,					// bDeviceProtocol
	ENDPOINT0_SIZE,				// bMaxPacketSize0
	LSB(VENDOR_ID), MSB(VENDOR_ID),		// idVendor
	LSB(PRODUCT_ID), MSB(PRODUCT_ID),	// idProduct
	0x00, 0x01,				// bcdDevice
	1,					// iManufacturer
	2,					// iProduct
	3,					// iSerialNumber
	1					// bNumConfigurations
};

#define CONFIG1_DESC_SIZE (9+9+5+5+4+5+7+9+7+7)
static const uint8_t PROGMEM config1_descriptor[CONFIG1_DESC_SIZE] = {
	// configuration descriptor, USB spec 9.6.3, page 264-266, Table 9-10
	9, 					// bLength;
	2,					// bDescriptorType;
	LSB(CONFIG1_DESC_SIZE),			// wTotalLength
	MSB(CONFIG1_DESC_SIZE),
	2,					// bNumInterfaces
	1,					// bConfigurationValue
	0,					// iConfiguration
	0xC0,					// bmAttributes
	50,					// bMaxPower
	// interface descriptor, USB spec 9.6.5, page 267-269, Table 9-12
	9,					// bLength
	4,					// bDescriptorType
	0,					// bInterfaceNumber
	0,					// bAlternateSetting
	1,					// bNumEndpoints
	0x02,					// bInterfaceClass
	0x02,					// bInterfaceSubClass
	0x01,					// bInterfaceProtocol
	0,					// iInterface
	// CDC Header Functional Descriptor, CDC Spec 5.2.3.1, Table 26
	5,					// bFunctionLength
	0x24,					// bDescriptorType
	0x00,					// bDescriptorSubtype
	0x10, 0x01,				// bcdCDC
	// Call Management Functional Descriptor, CDC Spec 5.2.3.2, Table 27
	5,					// bFunctionLength
	0x24,					// bDescriptorType
	0x01,					// bDescriptorSubtype
	0x01,					// bmCapabilities
	1,					// bDataInterface
	// Abstract Control Management Functional Descriptor, CDC Spec 5.2.3.3, Table 28
	4,					// bFunctionLength
	0x24,					// bDescriptorType
	0x02,					// bDescriptorSubtype
	0x06,					// bmCapabilities
	// Union Functional Descriptor, CDC Spec 5.2.3.8, Table 33
	5,					// bFunctionLength
	0x24,					// bDescriptorType
	0x06,					// bDescriptorSubtype
	0,					// bMasterInterface
	1,					// bSlaveInterface0
	// endpoint descriptor, USB spec 9.6.6, page 269-271, Table 9-13
	7,					// bLength
	5,					// bDescriptorType
	CDC_ACM_ENDPOINT | 0x80,		// bEndpointAddress
	0x03,					// bmAttributes (0x03=intr)
	CDC_ACM_SIZE, 0,			// wMaxPacketSize
	64,					// bInterval
	// interface descriptor, USB spec 9.6.5, page 267-269, Table 9-12
	9,					// bLength
	4,					// bDescriptorType
	1,					// bInterfaceNumber
	0,					// bAlternateSetting
	2,					// bNumEndpoints
	0x0A,					// bInterfaceClass
	0x00,					// bInterfaceSubClass
	0x00,					// bInterfaceProtocol
	0,					// iInterface
	// endpoint descriptor, USB spec 9.6.6, page 269-271, Table 9-13
	7,					// bLength
	5,					// bDescriptorType
	CDC_RX_ENDPOINT,			// bEndpointAddress
	0x02,					// bmAttributes (0x02=bulk)
	CDC_RX_SIZE, 0,				// wMaxPacketSize
	0,					// bInterval
	// endpoint descriptor, USB spec 9.6.6, page 269-271, Table 9-13
	7,					// bLength
	5,					// bDescriptorType
	CDC_TX_ENDPOINT | 0x80,			// bEndpointAddress
	0x02,					// bmAttributes (0x02=bulk)
	CDC_TX_SIZE, 0,				// wMaxPacketSize
	0					// bInterval
};

// If you're desperate for a little extra code memory, these strings
// can be completely removed if iManufacturer, iProduct, iSerialNumber
// in the device desciptor are changed to zeros.
struct usb_string_descriptor_struct {
	uint8_t bLength;
	uint8_t bDescriptorType;
	int16_t wString[];
};
static const struct usb_string_descriptor_struct PROGMEM string0 = {
	4,
	3,
	{0x0409}
};
static const struct usb_string_descriptor_struct PROGMEM string1 = {
	sizeof(STR_MANUFACTURER),
	3,
	STR_MANUFACTURER
};
static const struct usb_string_descriptor_struct PROGMEM string2 = {
	sizeof(STR_PRODUCT),
	3,
	STR_PRODUCT
};
static const struct usb_string_descriptor_struct PROGMEM string3 = {
	sizeof(STR_SERIAL_NUMBER),
	3,
	STR_SERIAL_NUMBER
};

// This table defines which descriptor data is sent for each specific
// request from the host (in wValue and wIndex).
static const struct descriptor_list_struct {
	uint16_t	wValue;
	uint16_t	wIndex;
	const uint8_t	*addr;
	uint8_t		length;
} PROGMEM descriptor_list[] = {
	{0x0100, 0x0000, device_descriptor, sizeof(device_descriptor)},
	{0x0200, 0x0000, config1_descriptor, sizeof(config1_descriptor)},
	{0x0300, 0x0000, (const uint8_t *)&string0, 4},
	{0x0301, 0x0409, (const uint8_t *)&string1, sizeof(STR_MANUFACTURER)},
	{0x0302, 0x0409, (const uint8_t *)&string2, sizeof(STR_PRODUCT)},
	{0x0303, 0x0409, (const uint8_t *)&string3, sizeof(STR_SERIAL_NUMBER)}
};
#define NUM_DESC_LIST (sizeof(descriptor_list)/sizeof(struct descriptor_list_struct))


/**************************************************************************
 *
 *  Variables - these are the only non-stack RAM usage
 *
 **************************************************************************/

// zero when we are not configured, non-zero when enumerated
static volatile uint8_t usb_configuration=0;

// the time remaining before we transmit any partially full
// packet, or send a zero length packet.
static volatile uint8_t transmit_flush_timer=0;
static uint8_t transmit_previous_timeout=0;

// serial port settings (baud rate, control signals, etc) set
// by the PC.  These are ignored, but kept in RAM.
static uint8_t cdc_line_coding[7]={0x00, 0xE1, 0x00, 0x00, 0x00, 0x00, 0x08};
static uint8_t cdc_line_rtsdtr=0;


/**************************************************************************
 *
 *  Public Functions - these are the API intended for the user
 *
 **************************************************************************/

// initialize USB serial
void m_usb_init(void)
{
	HW_CONFIG();
        USB_FREEZE();				// enable USB
        PLL_CONFIG();				// config PLL, 16 MHz xtal
        while (!(PLLCSR & (1<<PLOCK))) ;	// wait for PLL lock
        USB_CONFIG();				// start USB clock
        UDCON = 0;				// enable attach resistor
	usb_configuration = 0;
	cdc_line_rtsdtr = 0;
        UDIEN = (1<<EORSTE)|(1<<SOFE);
	sei();
}

// return 0 if the USB is not configured, or the configuration
// number selected by the HOST
char m_usb_isconnected(void)
{
	return (char)usb_configuration;
}

// get the next character, or -1 if nothing received
char m_usb_rx_char(void)
{
	uint8_t c, intr_state;

	// interrupts are disabled so these functions can be
	// used from the main program or interrupt context,
	// even both in the same program!
	intr_state = SREG;
	cli();
	if (!usb_configuration) {
		SREG = intr_state;
		return -1;
	}
	UENUM = CDC_RX_ENDPOINT;
	if (!(UEINTX & (1<<RWAL))) {
		// no data in buffer
		SREG = intr_state;
		return -1;
	}
	// take one byte out of the buffer
	c = UEDATX;
	// if buffer completely used, release it
	if (!(UEINTX & (1<<RWAL))) UEINTX = 0x6B;
	SREG = intr_state;
	return (char)c;
}

// number of bytes available in the receive buffer
unsigned char m_usb_rx_available(void)
{
	uint8_t n=0, intr_state;

	intr_state = SREG;
	cli();
	if (usb_configuration) {
		UENUM = CDC_RX_ENDPOINT;
		n = UEBCLX;
	}
	SREG = intr_state;
	return (unsigned char)n;
}

// discard any buffered input
void m_usb_rx_flush(void)
{
	uint8_t intr_state;

	if (usb_configuration) {
		intr_state = SREG;
		cli();
		UENUM = CDC_RX_ENDPOINT;
		while ((UEINTX & (1<<RWAL))) {
			UEINTX = 0x6B; 
		}
		SREG = intr_state;
	}
}

// transmit a character.  0 returned on success, -1 on error
char m_usb_tx_char(unsigned char c)
{
	uint8_t timeout, intr_state;

	// if we're not online (enumerated and configured), error
	if (!usb_configuration) return -1;
	// interrupts are disabled so these functions can be
	// used from the main program or interrupt context,
	// even both in the same program!
	intr_state = SREG;
	cli();
	UENUM = CDC_TX_ENDPOINT;
	// if we gave up due to timeout before, don't wait again
	if (transmit_previous_timeout) {
		if (!(UEINTX & (1<<RWAL))) {
			SREG = intr_state;
			return -1;
		}
		transmit_previous_timeout = 0;
	}
	// wait for the FIFO to be ready to accept data
	timeout = UDFNUML + TRANSMIT_TIMEOUT;
	while (1) {
		// are we ready to transmit?
		if (UEINTX & (1<<RWAL)) break;
		SREG = intr_state;
		// have we waited too long?  This happens if the user
		// is not running an application that is listening
		if (UDFNUML == timeout) {
			transmit_previous_timeout = 1;
			return -1;
		}
		// has the USB gone offline?
		if (!usb_configuration) return -1;
		// get ready to try checking again
		intr_state = SREG;
		cli();
		UENUM = CDC_TX_ENDPOINT;
	}
	// actually write the byte into the FIFO
	UEDATX = (uint8_t)c;
	// if this completed a packet, transmit it now!
	if (!(UEINTX & (1<<RWAL))) UEINTX = 0x3A;
	transmit_flush_timer = TRANSMIT_FLUSH_TIMEOUT;
	SREG = intr_state;
	return 0;
}


// transmit a character, but do not wait if the buffer is full,
//   0 returned on success, -1 on buffer full or error 
int8_t usb_serial_putchar_nowait(uint8_t c)
{
	uint8_t intr_state;

	if (!usb_configuration) return -1;
	intr_state = SREG;
	cli();
	UENUM = CDC_TX_ENDPOINT;
	if (!(UEINTX & (1<<RWAL))) {
		// buffer is full
		SREG = intr_state;
		return -1;
	}
	// actually write the byte into the FIFO
	UEDATX = c;
		// if this completed a packet, transmit it now!
	if (!(UEINTX & (1<<RWAL))) UEINTX = 0x3A;
	transmit_flush_timer = TRANSMIT_FLUSH_TIMEOUT;
	SREG = intr_state;
	return 0;
}

// transmit a buffer.
//  0 returned on success, -1 on error
// This function is optimized for speed!  Each call takes approx 6.1 us overhead
// plus 0.25 us per byte.  12 Mbit/sec USB has 8.67 us per-packet overhead and
// takes 0.67 us per byte.  If called with 64 byte packet-size blocks, this function
// can transmit at full USB speed using 43% CPU time.  The maximum theoretical speed
// is 19 packets per USB frame, or 1216 kbytes/sec.  However, bulk endpoints have the
// lowest priority, so any other USB devices will likely reduce the speed.  Speed
// can also be limited by how quickly the PC-based software reads data, as the host
// controller in the PC will not allocate bandwitdh without a pending read request.
// (thanks to Victor Suarez for testing and feedback and initial code)

int8_t usb_serial_write(const uint8_t *buffer, uint16_t size)
{
	uint8_t timeout, intr_state, write_size;

	// if we're not online (enumerated and configured), error
	if (!usb_configuration) return -1;
	// interrupts are disabled so these functions can be
	// used from the main program or interrupt context,
	// even both in the same program!
	intr_state = SREG;
	cli();
	UENUM = CDC_TX_ENDPOINT;
	// if we gave up due to timeout before, don't wait again
	if (transmit_previous_timeout) {
		if (!(UEINTX & (1<<RWAL))) {
			SREG = intr_state;
			return -1;
		}
		transmit_previous_timeout = 0;
	}
	// each iteration of this loop transmits a packet
	while (size) {
		// wait for the FIFO to be ready to accept data
		timeout = UDFNUML + TRANSMIT_TIMEOUT;
		while (1) {
			// are we ready to transmit?
			if (UEINTX & (1<<RWAL)) break;
			SREG = intr_state;
			// have we waited too long?  This happens if the user
			// is not running an application that is listening
			if (UDFNUML == timeout) {
				transmit_previous_timeout = 1;
				return -1;
			}
			// has the USB gone offline?
			if (!usb_configuration) return -1;
			// get ready to try checking again
			intr_state = SREG;
			cli();
			UENUM = CDC_TX_ENDPOINT;
		}

		// compute how many bytes will fit into the next packet
		write_size = CDC_TX_SIZE - UEBCLX;
		if (write_size > size) write_size = size;
		size -= write_size;

		// write the packet
		switch (write_size) {
			#if (CDC_TX_SIZE == 64)
			case 64: UEDATX = *buffer++;
			case 63: UEDATX = *buffer++;
			case 62: UEDATX = *buffer++;
			case 61: UEDATX = *buffer++;
			case 60: UEDATX = *buffer++;
			case 59: UEDATX = *buffer++;
			case 58: UEDATX = *buffer++;
			case 57: UEDATX = *buffer++;
			case 56: UEDATX = *buffer++;
			case 55: UEDATX = *buffer++;
			case 54: UEDATX = *buffer++;
			case 53: UEDATX = *buffer++;
			case 52: UEDATX = *buffer++;
			case 51: UEDATX = *buffer++;
			case 50: UEDATX = *buffer++;
			case 49: UEDATX = *buffer++;
			case 48: UEDATX = *buffer++;
			case 47: UEDATX = *buffer++;
			case 46: UEDATX = *buffer++;
			case 45: UEDATX = *buffer++;
			case 44: UEDATX = *buffer++;
			case 43: UEDATX = *buffer++;
			case 42: UEDATX = *buffer++;
			case 41: UEDATX = *buffer++;
			case 40: UEDATX = *buffer++;
			case 39: UEDATX = *buffer++;
			case 38: UEDATX = *buffer++;
			case 37: UEDATX = *buffer++;
			case 36: UEDATX = *buffer++;
			case 35: UEDATX = *buffer++;
			case 34: UEDATX = *buffer++;
			case 33: UEDATX = *buffer++;
			#endif
			#if (CDC_TX_SIZE >= 32)
			case 32: UEDATX = *buffer++;
			case 31: UEDATX = *buffer++;
			case 30: UEDATX = *buffer++;
			case 29: UEDATX = *buffer++;
			case 28: UEDATX = *buffer++;
			case 27: UEDATX = *buffer++;
			case 26: UEDATX = *buffer++;
			case 25: UEDATX = *buffer++;
			case 24: UEDATX = *buffer++;
			case 23: UEDATX = *buffer++;
			case 22: UEDATX = *buffer++;
			case 21: UEDATX = *buffer++;
			case 20: UEDATX = *buffer++;
			case 19: UEDATX = *buffer++;
			case 18: UEDATX = *buffer++;
			case 17: UEDATX = *buffer++;
			#endif
			#if (CDC_TX_SIZE >= 16)
			case 16: UEDATX = *buffer++;
			case 15: UEDATX = *buffer++;
			case 14: UEDATX = *buffer++;
			case 13: UEDATX = *buffer++;
			case 12: UEDATX = *buffer++;
			case 11: UEDATX = *buffer++;
			case 10: UEDATX = *buffer++;
			case  9: UEDATX = *buffer++;
			#endif
			case  8: UEDATX = *buffer++;
			case  7: UEDATX = *buffer++;
			case  6: UEDATX = *buffer++;
			case  5: UEDATX = *buffer++;
			case  4: UEDATX = *buffer++;
			case  3: UEDATX = *buffer++;
			case  2: UEDATX = *buffer++;
			default:
			case  1: UEDATX = *buffer++;
			case  0: break;
		}
		// if this completed a packet, transmit it now!
		if (!(UEINTX & (1<<RWAL))) UEINTX = 0x3A;
		transmit_flush_timer = TRANSMIT_FLUSH_TIMEOUT;
	}
	SREG = intr_state;
	return 0;
}


// immediately transmit any buffered output.
// This doesn't actually transmit the data - that is impossible!
// USB devices only transmit when the host allows, so the best
// we can do is release the FIFO buffer for when the host wants it
void m_usb_tx_push(void)
{
	uint8_t intr_state;

	intr_state = SREG;
	cli();
	if (transmit_flush_timer) {
		UENUM = CDC_TX_ENDPOINT;
		UEINTX = 0x3A;
		transmit_flush_timer = 0;
	}
	SREG = intr_state;
}

// functions to read the various async serial settings.  These
// aren't actually used by USB at all (communication is always
// at full USB speed), but they are set by the host so we can
// set them properly if we're converting the USB to a real serial
// communication
uint32_t usb_serial_get_baud(void)
{
	return *(uint32_t *)cdc_line_coding;
}
uint8_t usb_serial_get_stopbits(void)
{
	return cdc_line_coding[4];
}
uint8_t usb_serial_get_paritytype(void)
{
	return cdc_line_coding[5];
}
uint8_t usb_serial_get_numbits(void)
{
	return cdc_line_coding[6];
}
uint8_t usb_serial_get_control(void)
{
	return cdc_line_rtsdtr;
}
// write the control signals, DCD, DSR, RI, etc
// There is no CTS signal.  If software on the host has transmitted
// data to you but you haven't been calling the getchar function,
// it remains buffered (either here or on the host) and can not be
// lost because you weren't listening at the right time, like it
// would in real serial communication.
// TODO: this function is untested.  Does it work?  Please email
// paul@pjrc.com if you have tried it....
int8_t usb_serial_set_control(uint8_t signals)
{
	uint8_t intr_state;

	intr_state = SREG;
	cli();
	if (!usb_configuration) {
		// we're not enumerated/configured
		SREG = intr_state;
		return -1;
	}

	UENUM = CDC_ACM_ENDPOINT;
	if (!(UEINTX & (1<<RWAL))) {
		// unable to write
		// TODO; should this try to abort the previously
		// buffered message??
		SREG = intr_state;
		return -1;
	}
	UEDATX = 0xA1;
	UEDATX = 0x20;
	UEDATX = 0;
	UEDATX = 0;
	UEDATX = 0; // TODO: should this be 1 or 0 ???
	UEDATX = 0;
	UEDATX = 2;
	UEDATX = 0;
	UEDATX = signals;
	UEDATX = 0;
	UEINTX = 0x3A;
	SREG = intr_state;
	return 0;
}



/**************************************************************************
 *
 *  Private Functions - not intended for general user consumption....
 *
 **************************************************************************/


// USB Device Interrupt - handle all device-level events
// the transmit buffer flushing is triggered by the start of frame
//
ISR(USB_GEN_vect)
{
	uint8_t intbits, t;

        intbits = UDINT;
        UDINT = 0;
        if (intbits & (1<<EORSTI)) {
		UENUM = 0;
		UECONX = 1;
		UECFG0X = EP_TYPE_CONTROL;
		UECFG1X = EP_SIZE(ENDPOINT0_SIZE) | EP_SINGLE_BUFFER;
		UEIENX = (1<<RXSTPE);
		usb_configuration = 0;
		cdc_line_rtsdtr = 0;
        }
	if (intbits & (1<<SOFI)) {
		if (usb_configuration) {
			t = transmit_flush_timer;
			if (t) {
				transmit_flush_timer = --t;
				if (!t) {
					UENUM = CDC_TX_ENDPOINT;
					UEINTX = 0x3A;
				}
			}
		}
	}
}


// Misc functions to wait for ready and send/receive packets
static inline void usb_wait_in_ready(void)
{
	while (!(UEINTX & (1<<TXINI))) ;
}
static inline void usb_send_in(void)
{
	UEINTX = ~(1<<TXINI);
}
static inline void usb_wait_receive_out(void)
{
	while (!(UEINTX & (1<<RXOUTI))) ;
}
static inline void usb_ack_out(void)
{
	UEINTX = ~(1<<RXOUTI);
}



// USB Endpoint Interrupt - endpoint 0 is handled here.  The
// other endpoints are manipulated by the user-callable
// functions, and the start-of-frame interrupt.
//
ISR(USB_COM_vect)
{
        uint8_t intbits;
	const uint8_t *list;
        const uint8_t *cfg;
	uint8_t i, n, len, en;
	uint8_t *p;
	uint8_t bmRequestType;
	uint8_t bRequest;
	uint16_t wValue;
	uint16_t wIndex;
	uint16_t wLength;
	uint16_t desc_val;
	const uint8_t *desc_addr;
	uint8_t	desc_length;

        UENUM = 0;
        intbits = UEINTX;
        if (intbits & (1<<RXSTPI)) {
                bmRequestType = UEDATX;
                bRequest = UEDATX;
                wValue = UEDATX;
                wValue |= (UEDATX << 8);
                wIndex = UEDATX;
                wIndex |= (UEDATX << 8);
                wLength = UEDATX;
                wLength |= (UEDATX << 8);
                UEINTX = ~((1<<RXSTPI) | (1<<RXOUTI) | (1<<TXINI));
                if (bRequest == GET_DESCRIPTOR) {
			list = (const uint8_t *)descriptor_list;
			for (i=0; ; i++) {
				if (i >= NUM_DESC_LIST) {
					UECONX = (1<<STALLRQ)|(1<<EPEN);  //stall
					return;
				}
				desc_val = pgm_read_word(list);
				if (desc_val != wValue) {
					list += sizeof(struct descriptor_list_struct);
					continue;
				}
				list += 2;
				desc_val = pgm_read_word(list);
				if (desc_val != wIndex) {
					list += sizeof(struct descriptor_list_struct)-2;
					continue;
				}
				list += 2;
				desc_addr = (const uint8_t *)pgm_read_word(list);
				list += 2;
				desc_length = pgm_read_byte(list);
				break;
			}
			len = (wLength < 256) ? wLength : 255;
			if (len > desc_length) len = desc_length;
			do {
				// wait for host ready for IN packet
				do {
					i = UEINTX;
				} while (!(i & ((1<<TXINI)|(1<<RXOUTI))));
				if (i & (1<<RXOUTI)) return;	// abort
				// send IN packet
				n = len < ENDPOINT0_SIZE ? len : ENDPOINT0_SIZE;
				for (i = n; i; i--) {
					UEDATX = pgm_read_byte(desc_addr++);
				}
				len -= n;
				usb_send_in();
			} while (len || n == ENDPOINT0_SIZE);
			return;
                }
		if (bRequest == SET_ADDRESS) {
			usb_send_in();
			usb_wait_in_ready();
			UDADDR = wValue | (1<<ADDEN);
			return;
		}
		if (bRequest == SET_CONFIGURATION && bmRequestType == 0) {
			usb_configuration = wValue;
			cdc_line_rtsdtr = 0;
			transmit_flush_timer = 0;
			usb_send_in();
			cfg = endpoint_config_table;
			for (i=1; i<5; i++) {
				UENUM = i;
				en = pgm_read_byte(cfg++);
				UECONX = en;
				if (en) {
					UECFG0X = pgm_read_byte(cfg++);
					UECFG1X = pgm_read_byte(cfg++);
				}
			}
        		UERST = 0x1E;
        		UERST = 0;
			return;
		}
		if (bRequest == GET_CONFIGURATION && bmRequestType == 0x80) {
			usb_wait_in_ready();
			UEDATX = usb_configuration;
			usb_send_in();
			return;
		}
		if (bRequest == CDC_GET_LINE_CODING && bmRequestType == 0xA1) {
			usb_wait_in_ready();
			p = cdc_line_coding;
			for (i=0; i<7; i++) {
				UEDATX = *p++;
			}
			usb_send_in();
			return;
		}
		if (bRequest == CDC_SET_LINE_CODING && bmRequestType == 0x21) {
			usb_wait_receive_out();
			p = cdc_line_coding;
			for (i=0; i<7; i++) {
				*p++ = UEDATX;
			}
			usb_ack_out();
			usb_send_in();
			return;
		}
		if (bRequest == CDC_SET_CONTROL_LINE_STATE && bmRequestType == 0x21) {
			cdc_line_rtsdtr = wValue;
			usb_wait_in_ready();
			usb_send_in();
			return;
		}
		if (bRequest == GET_STATUS) {
			usb_wait_in_ready();
			i = 0;
			#ifdef SUPPORT_ENDPOINT_HALT
			if (bmRequestType == 0x82) {
				UENUM = wIndex;
				if (UECONX & (1<<STALLRQ)) i = 1;
				UENUM = 0;
			}
			#endif
			UEDATX = i;
			UEDATX = 0;
			usb_send_in();
			return;
		}
		#ifdef SUPPORT_ENDPOINT_HALT
		if ((bRequest == CLEAR_FEATURE || bRequest == SET_FEATURE)
		  && bmRequestType == 0x02 && wValue == 0) {
			i = wIndex & 0x7F;
			if (i >= 1 && i <= MAX_ENDPOINT) {
				usb_send_in();
				UENUM = i;
				if (bRequest == SET_FEATURE) {
					UECONX = (1<<STALLRQ)|(1<<EPEN);
				} else {
					UECONX = (1<<STALLRQC)|(1<<RSTDT)|(1<<EPEN);
					UERST = (1 << i);
					UERST = 0;
				}
				return;
			}
		}
		#endif
        }
	UECONX = (1<<STALLRQ) | (1<<EPEN);	// stall
}


// BELOW FROM PRINT.C

void print_P(const char *s)
{
	char c;

	while (1) {
		c = pgm_read_byte(s++);
		if (!c) break;
		if (c == '\n') usb_tx_char('\r');
		usb_tx_char(c);
	}
}

void phex1(unsigned char c)
{
	usb_tx_char(c + ((c < 10) ? '0' : 'A' - 10));
}

void phex(unsigned char c)
{
	phex1(c >> 4);
	phex1(c & 15);
}

void m_usb_tx_hex(unsigned int i)
{
	phex(i >> 8);
	phex(i);
}

void m_usb_tx_hex8(unsigned char i)
{
	phex(i);
}

void m_usb_tx_int(int i)
{
	char string[6] = {0,0,0,0,0,0};
	itoa(i,string,10);
	for(i=0;i<6;i++){
		m_usb_tx_char(string[i]);
	}
}

void m_usb_tx_uint(unsigned int i)
{
	char string[5] = {0,0,0,0,0};
	utoa(i,string,10);
	for(i=0;i<5;i++){
		m_usb_tx_char(string[i]);
	}
}

void m_usb_tx_long(long i)
{
	char string[11] = {0,0,0,0,0,0,0,0,0,0,0};
	ltoa(i,string,10);
	for(i=0;i<11;i++){
		m_usb_tx_char(string[i]);
	}
}

void m_usb_tx_ulong(unsigned long i)
{
	char string[10] = {0,0,0,0,0,0,0,0,0,0};
	ultoa(i,string,10);
	for(i=0;i<10;i++){
		m_usb_tx_char(string[i]);
	}
}


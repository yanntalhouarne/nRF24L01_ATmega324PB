#include "spi.h"
#include "nRF24L01.h"
#include "mirf.h"
#include "print.h"
#include <avr/io.h>
#include <avr/interrupt.h>

// Defines for setting the MiRF registers for transmitting or receiving mode
#define TX_POWERUP mirf_config_register(CONFIG, mirf_CONFIG | ((1 << PWR_UP) | (0 << PRIM_RX)))
#define RX_POWERUP mirf_config_register(CONFIG, mirf_CONFIG | ((1 << PWR_UP) | (1 << PRIM_RX)))

// Flag which denotes transmitting mode
volatile char PTX;

void mirf_init()
// Initializes pins ans interrupt to communicate with the MiRF module
// Should be called in the early initializing phase at startup.
{
	// Define CSN and CE as Output and set them to default
	DDRD |= (1 << CE); // PD4
	DDRD |= (1 << SS_1); // PD6
	
	mirf_CE_lo;
	mirf_CSN_hi;

	// Initialize external interrupt 0 (PD2)
	EIMSK &= ~(1 << INT0);
	EICRA |= (1 << ISC01); // the falling edge of INT0 generates asynchronously an interrupt request.
	EIMSK |= (1 << INT0);  // enable interrupts on INT0

	// Initialize spi module
	spi1_master_initialize();
}

void mirf_config()
// Sets the important registers in the MiRF module and powers the module
// in receiving mode
{
	// Set RF channel
	mirf_config_register(RF_CH, mirf_CH);
	// Set length of incoming payload
	mirf_config_register(RX_PW_P0, mirf_PAYLOAD);
	
	
	// Start receiver
	PTX = 0;    // Start in receiving mode
	RX_POWERUP; // Power up in receiving mode
	mirf_CE_hi; // Listening for packets
}

void mirf_set_RADDR(char *adr)
// Sets the receiving address
{
	mirf_CE_lo;
	mirf_write_register(RX_ADDR_P0, adr, 5);
	mirf_CE_hi;
}

void mirf_set_TADDR(char *adr)
// Sets the transmitting address
{
	mirf_write_register(TX_ADDR, adr, 5);
}

extern char mirf_data_ready()
// Checks if data is available for reading
{
	//println_0("in mirf_data_ready();");
	if (PTX)
		return 0;
	uint8_t status;
	// Read MiRF status
	mirf_CSN_lo;       // Pull down chip select
	status = spi1_exchange_char(NOP); // Read status register
	mirf_CSN_hi;                     // Pull up chip select
	return status & (1 << RX_DR);
}

extern void mirf_get_data(char *data)
// Reads mirf_PAYLOAD bytes into data array
{
	mirf_CSN_lo;                                  // Pull down chip select
	spi1_send_char(R_RX_PAYLOAD);              // Send cmd to read rx payload
	spi1_exchange_bytes(data, data, mirf_PAYLOAD); // Read payload
	mirf_CSN_hi;                                  // Pull up chip select
	mirf_config_register(STATUS, (1 << RX_DR));   // Reset status register
}

void mirf_config_register(char reg, char value)
// Clocks only one byte into the given MiRF register
{
	mirf_CSN_lo;
	spi1_send_char(W_REGISTER | (REGISTER_MASK & reg));
	//mirf_CSN_hi;
	//_delay_us(50);
	
	//mirf_CSN_lo;
	spi1_send_char(value);
	mirf_CSN_hi;	
}

void mirf_read_register(char reg, char *value, char len)
// Reads an array of bytes from the given start position in the MiRF registers.
{
	mirf_CSN_lo;
	spi1_send_char(R_REGISTER | (REGISTER_MASK & reg));
	spi1_exchange_bytes(value, value, len);
	mirf_CSN_hi;
}

void mirf_write_register(char reg, char *value, char len)
// Writes an array of bytes into the the MiRF registers.
{
	mirf_CSN_lo;
	spi1_send_char(W_REGISTER | (REGISTER_MASK & reg));
	spi1_send_bytes(value, len);
	mirf_CSN_hi;
}

void mirf_send(char *value, char len)
// Sends a data package to the default address. Be sure to send the correct
// amount of bytes as configured as payload on the receiver.
{
	println_0("In mirf_send();");

	while (PTX)
	{
				
	} // Wait until last packet is send

	mirf_CE_lo;

	PTX = 1;    // Set to transmitter mode
	//print("in mirf_send, PTX set to ;");
	//println_int(PTX);
	//print_char_0(NL);
	TX_POWERUP; // Power up

	mirf_CSN_lo;                 // Pull down chip select
	spi1_send_char(FLUSH_TX); // Write cmd to flush tx fifo
	mirf_CSN_hi;                 // Pull up chip select
	
	mirf_CSN_lo;                     // Pull down chip select
	spi1_send_char(W_TX_PAYLOAD); // Write cmd to write payload
	spi1_send_bytes(value, len);      // Write payload
	mirf_CSN_hi;                     // Pull up chip select

	mirf_CE_hi; // Start transmission
}

ISR(INT0_vect) // Interrupt handler
{
	char status;
	// If still in transmitting mode then finish transmission
	if (PTX)
	{

		// Read MiRF status
		mirf_CSN_lo;                     // Pull down chip select
		status = spi1_exchange_char(NOP); // Read status register
		mirf_CSN_hi;                     // Pull up chip select

		mirf_CE_lo; // Deactivate transreceiver
		RX_POWERUP; // Power up in receiving mode
		mirf_CE_hi; // Listening for pakets
		PTX = 0;    // Set to receiving mode

		// Reset status register for further interaction
		mirf_config_register(STATUS, (1 << TX_DS) | (1 << MAX_RT)); // Reset status register
	}
}
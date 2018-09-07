#include "spi.h"
#include "nRF24L01.h"
#include "mirf.h"
#include "print.h"
#include <avr/io.h>
#include <avr/interrupt.h>

// Defines for setting the MiRF registers for transmitting or receiving mode
#define TX_POWERUP mirf_config_register(CONFIG, mirf_CONFIG | ((1 << PWR_UP) | (0 << PRIM_RX)))
#define RX_POWERUP mirf_config_register(CONFIG, mirf_CONFIG | ((1 << PWR_UP) | (1 << PRIM_RX)))

void set_RX_MODE()
{
	// Start receiver
	mirf_config_register(STATUS,(1<<RX_DR)|(1<<TX_DS)|(1<<MAX_RT)); // clear flags
	
	RX_POWERUP; // Power up in receiving mode
	
	mirf_CE_hi; // enter RX mode
}

void mirf_init()
// Initializes pins ans interrupt to communicate with the MiRF module
// Should be called in the early initializing phase at startup.
{
	// Define CSN and CE as Output and set them to default
	DDRC |= (1 << CE); // CE is output
	DDRD |= (1 << SS_1); // SS is output
	
	mirf_CE_lo;    // standby-I mode
	mirf_CSN_hi;   // SPI comm OFF
	
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
	mirf_config_register(RX_PW_P0, mirf_PAYLOAD); // Auto-ACK pipe ...
	mirf_config_register(RX_PW_P1, 0x00); // Data payload pipe
	mirf_config_register(RX_PW_P2, 0x00);
	mirf_config_register(RX_PW_P3, 0x00);
	mirf_config_register(RX_PW_P4, 0x00);
	mirf_config_register(RX_PW_P5, 0x00);
   
    // 250 kbps, TX gain: 0dbm
   	mirf_config_register(RF_SETUP, (0x03<<RF_PWR)); // 1Mbps, 0dBm
	   
    // Auto Acknowledgment
    mirf_config_register(EN_AA,(1<<ENAA_P0)|(0<<ENAA_P1)|(0<<ENAA_P2)|(0<<ENAA_P3)|(0<<ENAA_P4)|(0<<ENAA_P5)); // AA enable on Pipe 0
	
	// Enable RX addresses
	 mirf_config_register(EN_RXADDR,(1<<ERX_P0)|(0<<ERX_P1)|(0<<ERX_P2)|(0<<ERX_P3)|(0<<ERX_P4)|(0<<ERX_P5)); // Enable Pipe 1
	
	// Auto retransmit delay: 1000 us and Up to 15 retransmit trials
	mirf_config_register(SETUP_RETR,(0x0F<<ARD)|(0x0F<<ARC)); // 4000us delay, 15 retransmissions
	
	// clear flags
	mirf_config_register(STATUS, (1<<RX_DR)|(1<<TX_DS)|(1<<MAX_RT)); 
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
	/* RX_ADDR_P0 must be set to the sending addr for auto ack to work. */
	mirf_write_register(RX_ADDR_P0, adr, 5);
	mirf_write_register(TX_ADDR, adr, 5);
}

extern char mirf_data_ready()
// Checks if data is available for reading
{
	uint8_t status;
	// Read MiRF status
	mirf_CSN_lo;       // Pull down chip select
	status = spi1_exchange_char(NOP); // Read status register
	mirf_CSN_hi;                     // Pull up chip select
	return status & (1 << RX_DR);
}

extern char mirf_data_sent()
// Checks if data is available for reading
{
	int8_t status;
	// Read MiRF status
	mirf_CSN_lo;       // Pull down chip select
	status = spi1_exchange_char(NOP); // Read status register
	mirf_CSN_hi;                     // Pull up chip select

	_delay_ms(1);
	return status & (1 << TX_DS);
}

extern unsigned char mirf_read_MAX_RT()
{
	unsigned char status = 0;
	mirf_CSN_lo;
	status = spi1_exchange_char(NOP); // Read status register
	mirf_CSN_hi;                     // Pull up chip select
	
	_delay_ms(1);
	
	if ((status>>MAX_RT)&0x01)
			status = 1;
		else 
			status = 0;
		
	return status;
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
	_delay_us(25);
	spi1_send_char(value);
	mirf_CSN_hi;
	_delay_us(25);
}

void mirf_read_register(char reg, char *value, char len)
// Reads an array of bytes from the given start position in the MiRF registers.
{
	mirf_CSN_lo;
	spi1_send_char(R_REGISTER | (REGISTER_MASK & reg));
	spi1_exchange_bytes(value, value, len);
	mirf_CSN_hi;
	_delay_us(25);
}

void mirf_write_register(char reg, char *value, char len)
// Writes an array of bytes into the the MiRF registers.
{
	mirf_CSN_lo;
	spi1_send_char(W_REGISTER | (REGISTER_MASK & reg));
	spi1_send_bytes(value, len);
	mirf_CSN_hi;
	_delay_us(25);
	
}

void mirf_send(char *value, char len)
// Sends a data package to the default address. Be sure to send the correct
// amount of bytes as configured as payload on the receiver.
{
	mirf_CE_lo; // Enter Standby-I mode
	
	TX_POWERUP; // Enter TX mode (PRIM_RX = 0, PWR_UP = 1)
	
	_delay_us(25);
	
	mirf_config_register(STATUS,(1<<RX_DR)|(1<<TX_DS)|(1<<MAX_RT)); // clear flags 
	
	mirf_CSN_lo;                     // Pull down chip select
	spi1_send_char(W_TX_PAYLOAD); // Write cmd to write payload
	spi1_send_bytes(value, len);      // Write payload
	mirf_CSN_hi;                     // Pull up chip select

	mirf_CE_hi; // Start transmission
	_delay_us(15); // pulse CE for at least 10us
	mirf_CE_lo; // data packet sent, ShockBurst automatically enters RX mode
}


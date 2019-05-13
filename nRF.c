/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*
 * based on mirf library written by  Tinkerer (http://141.70.125.224/AVRLib/nRF24L01/)   *
 *																					     *
 *&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/

#include <avr/io.h>
#include <avr/interrupt.h>

#include "spi.h"
#include "nRF24L01.h"
#include "nRF.h"


// Defines for setting the MiRF registers for transmitting or receiving mode
#define nRF_TX_POWERUP nRF_configReg(CONFIG, mirf_CONFIG | ((1 << PWR_UP) | (0 << PRIM_RX)))
#define nRF_RX_POWERUP nRF_configReg(CONFIG, mirf_CONFIG | ((1 << PWR_UP) | (1 << PRIM_RX)))

/********************************************************************************************
 *	- nRF_SetRxMode :																	    *
 *																							*
 ********************************************************************************************/
void nRF_SetRxMode()
{
	// Start receiver
	nRF_configReg(STATUS,(1<<RX_DR)|(1<<TX_DS)|(1<<MAX_RT)); // clear flags
	
	nRF_RX_POWERUP; // Power up in receiving mode
	
	nRF_SET_CE; // enter RX mode
}


/********************************************************************************************
 * nRF_init : - Initializes pins ans interrupt to communicate with the MiRF module,		    *
 *			  - Should be called in the early initializing phase at startup.				*
 *																							*
 ********************************************************************************************/
void nRF_init()
{
	// Define CSN and CE as Output and set them to default
	DDRC |= (1 << CE); // CE is output
	DDRD |= (1 << SS_1); // SS is output
	
	nRF_CLEAR_CE;    // standby-I mode
	nRF_SET_CSN;   // SPI comm OFF
	
	// Initialize spi module
	SPI1_masterInit();
}


/********************************************************************************************
 *	nRF_config : - Sets the important registers in the MiRF module and powers the module in *
 *		           receiving mode.															*
 *																							*
 ********************************************************************************************/
void nRF_config()
{
	// Set RF channel
	nRF_configReg(RF_CH, mirf_CH);
	
	 // enable dynamic payload and ACK with payload 
	 nRF_configReg(FEATURE, (1<<EN_DPL) | (1<<EN_ACK_PAY));
	
	// enable dynamic payload on Pipe 0 
	nRF_configReg(DYNPD, (1<<DPL_P1));
   
    // 250 kbps, TX gain: 0dbm
   	nRF_configReg(RF_SETUP, (0x03<<RF_PWR) | (1<<RF_DR_LOW) | (0<<RF_DR_HIGH));
	   
    // Auto Acknowledgment
    nRF_configReg(EN_AA,(1<<ENAA_P0)|(1<<ENAA_P1)|(0<<ENAA_P2)|(0<<ENAA_P3)|(0<<ENAA_P4)|(0<<ENAA_P5)); // use pipe 0 for RX
	
	// Enable RX addresses
	nRF_configReg(EN_RXADDR,(1<<ERX_P0)|(1<<ERX_P1)|(0<<ERX_P2)|(0<<ERX_P3)|(0<<ERX_P4)|(0<<ERX_P5)); // enable Pipe 0 and1
	
	// Auto retransmit delay: 1000 us and Up to 15 retransmit trials
	nRF_configReg(SETUP_RETR,(0x0F<<ARD)|(0x0F<<ARC)); // 4000us delay, 15 retransmissions
	
	// clear flags
	nRF_configReg(STATUS, (1<<RX_DR)|(1<<TX_DS)|(1<<MAX_RT)); 
}


/********************************************************************************************
 *	nRF_RXsetAddr : - Sets the receiving address.											*
 *																							*
 ********************************************************************************************/
void nRF_RXsetAddr(char *adr)
{
	nRF_CLEAR_CE;
	nRF_writeReg(RX_ADDR_P1, adr, 5);
	nRF_SET_CE;
}

/********************************************************************************************
 *	nRF_TXsetAddr : - Sets the transmitting address.										*
 *																							*
 ********************************************************************************************/
void nRF_TXsetAddr(char *adr)
{
	/* RX_ADDR_P0 must be set to the sending addr for auto ack to work. */
	nRF_writeReg(RX_ADDR_P0, adr, 5);
	nRF_writeReg(TX_ADDR, adr, 5);
}

/********************************************************************************************
 *	nRF_dataReady : - Checks if data is available for reading.								*
 *																							*
 ********************************************************************************************/
char nRF_dataReady()
{
	uint8_t status;
	// Read MiRF status
	nRF_CLEAR_CSN;       // Pull down chip select
	status = SPI1_exchangeChar(NOP); // Read status register
	nRF_SET_CSN;                     // Pull up chip select
	return status & (1 << RX_DR);
}


/********************************************************************************************
 *	nRF_dataSent : - Checks if data is was sent successfully.								*
 *																							*
 ********************************************************************************************/
char nRF_dataSent()
{
	int8_t status;
	// Read MiRF status
	nRF_CLEAR_CSN;       // Pull down chip select
	status = SPI1_exchangeChar(NOP); // Read status register
	nRF_SET_CSN;                     // Pull up chip select

	_delay_ms(1);
	return status & (1 << TX_DS);
}

/********************************************************************************************
 *	nRF_getMaxRT : - Checks the maximum transmission flag.									*
 *																							*
 ********************************************************************************************/
unsigned char nRF_getMaxRT()
{
	unsigned char status = 0;
	nRF_CLEAR_CSN;
	status = SPI1_exchangeChar(NOP); // Read status register
	nRF_SET_CSN;                     // Pull up chip select
	
	_delay_ms(1);
	
	if ((status>>MAX_RT)&0x01)
			status = 1;
		else 
			status = 0;
		
	return status;
}


/********************************************************************************************
 *	nRF_getData : - Reads mirf_PAYLOAD bytes into data array.								*
 *																							*
 ********************************************************************************************/
void nRF_getData(char *data, char payload_length)
{
	nRF_CLEAR_CSN;                                  // Pull down chip select
	SPI1_sendChar(R_RX_PAYLOAD);              // Send cmd to read rx payload
	SPI1_exchangeBytes(data, data, payload_length); // Read payload
	nRF_SET_CSN;                                  // Pull up chip select
	nRF_configReg(STATUS, (1 << RX_DR));   // Reset status register
}


/********************************************************************************************
 *	nRF_configReg : - Clocks only one byte into the given MiRF register.					*
 *																							*
 ********************************************************************************************/
void nRF_configReg(char reg, char value)
{
	nRF_CLEAR_CSN;
	SPI1_sendChar(W_REGISTER | (REGISTER_MASK & reg));
	_delay_us(25);
	SPI1_sendChar(value);
	nRF_SET_CSN;
	_delay_us(25);
}



/********************************************************************************************
 *	nRF_readReg : - Reads an array of bytes from the given start position in the MiRF		*
 *					registers.																*
 *																							*
 ********************************************************************************************/
void nRF_readReg(char reg, char *value, char len)
{
	nRF_CLEAR_CSN;
	SPI1_sendChar(R_REGISTER | (REGISTER_MASK & reg));
	SPI1_exchangeBytes(value, value, len);
	nRF_SET_CSN;
	_delay_us(25);
}


/********************************************************************************************
 *	nRF_writeReg : - Writes an array of bytes into the the MiRF registers.					*
 *																							*
 ********************************************************************************************/
void nRF_writeReg(char reg, char *value, char len)
{
	nRF_CLEAR_CSN;
	SPI1_sendChar(W_REGISTER | (REGISTER_MASK & reg));
	SPI1_sendBytes(value, len);
	nRF_SET_CSN;
	_delay_us(25);
	
}


/********************************************************************************************
 *	nRF_sendData : - Sends a data package to the default address. Be sure to send the		*
 *					 correct																*
 *				   - Amount of bytes as configured as payload on the receiver.				*
 *																							*
 ********************************************************************************************/
void nRF_sendData(char *value, char len)
{	
	nRF_TX_POWERUP; // Enter TX mode (PRIM_RX = 0, PWR_UP = 1)
	
	_delay_us(25);
	
	nRF_configReg(STATUS,(1<<RX_DR)|(1<<TX_DS)|(1<<MAX_RT)); // clear flags 
	
	nRF_CLEAR_CSN;                     // Pull down chip select
	SPI1_sendChar(W_TX_PAYLOAD); // Write cmd to write payload
	SPI1_sendBytes(value, len);      // Write payload
	nRF_SET_CSN;                     // Pull up chip select

	nRF_SET_CE; // Start transmission
	_delay_us(15); // pulse CE for at least 10us
	nRF_CLEAR_CE; // data packet sent, ShockBurst automatically enters RX mode
}


void nRF24_uploadACKpay(char pipe, char *value, char len)
{
	nRF_CLEAR_CSN;                     // Pull down chip select
	SPI1_sendChar(W_ACK_PAYLOAD | pipe);	 // Write cmd to write payload
	SPI1_sendBytes(value, len);      // Write payload
	nRF_SET_CSN;                     // Pull up chip select
}

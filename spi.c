#include "spi.h"

void spi0_master_initialize()
{
	DDRB |= (1 << MOSI_0);  // MOSI_0 is output
	DDRB |= (1 << SCK_0);   // SCK_0 is output
	DDRB |= (1 << SS_0);   // SS_0 is output
	
	SPCR0 = (1 << SPE) | (1 << MSTR) | (1 << SPR1) | (1 << CPHA); // Enable SPI, Master, set clock rate fck/16, SPI_MODE1, MSB first
}
void spi0_slave_initialize()
{
	DDRB |= (1 << MISO_0);	// MSIO_0 is output
	
	SPCR0 = (1 << SPE); // Enable SPI, Slave
}

void spi1_master_initialize()
{
	DDRE |= (1 << MOSI_1);  // MOSI_1 is output
	DDRD |= (1 << SCK_1);   // SCK_1 is output  
	DDRD |= (1 << SS_1);   // SS_1 is output 
												 
	SPCR1 = (1 << SPE) | (1 << MSTR) | (1 << SPR1) | (1 << CPHA); // Enable SPI, Master, set clock rate fck/16, SPI_MODE1, MSB first
}
void spi1_slave_initialize()
{
	DDRE |= (1 << MISO_1);	// MSIO_1 is output 
										
	SPCR1 = (1 << SPE); // Enable SPI, Slave
}


void spi0_send_char(char data)
{
	SPDR0 = data;
	while (!(SPSR0 & (1 << SPIF)))
	; // wait for transmission to complete
}
void spi1_send_char(char data)
{
	SPDR1 = data;
	while (!(SPSR1 & (1 << SPIF)))
	; // wait for transmission to complete
}

char spi0_exchange_char(char data)
{
	SPDR0 = data; // start transmission
	
	while (!(SPSR0 & (1 << SPIF)))
	; // wait for transmission complete
	
	return SPDR0;
}
char spi1_exchange_char(char data)
{
	SPDR1 = data; // start transmission
	
	while (!(SPSR1 & (1 << SPIF)))
	; // wait for transmission complete
	
	return SPDR1;
}

void spi0_send_bytes(char *pdata, char bytes)
{
	int i;

	for (i = 0; i < bytes; i++)
	{
		SPDR0 = pdata[i]; // start transmission
		while (!(SPSR0 & (1 << SPIF)))
		; // wait for transmission complete
	}
}
void spi1_send_bytes(char *pdata, char bytes)
{
	int i;

	for (i = 0; i < bytes; i++)
	{
		SPDR1 = pdata[i]; // start transmission
		while (!(SPSR1 & (1 << SPIF)))
		; // wait for transmission complete
	}
}

void spi0_exchange_bytes(char *mosi, char *miso, char bytes)
{
	int i;

	for (i = 0; i < bytes; i++)
	{
		SPDR0 = mosi[i]; // start transmission

		while (!(SPSR0 & (1 << SPIF)))
		; // wait for transmission complete
		miso[i] = SPDR0;
	}
}
void spi1_exchange_bytes(char *mosi, char *miso, char bytes)
{
	int i;

	for (i = 0; i < bytes; i++)
	{
		SPDR1 = mosi[i]; // start transmission

		while (!(SPSR1 & (1 << SPIF)))
		; // wait for transmission complete
		miso[i] = SPDR1;
	}
}

unsigned int spi0_exchange_int(unsigned int mosi) // not tested
{
	unsigned int miso = 0;
	
	// first send MSB
	//while (!(SPSR0 & (1 << SPIF)));
	SPDR0 = (char)(mosi >> 8); // start transmission by sending MSB of MOSI

	while (!(SPSR0 & (1 << SPIF))); // wait for transmission complete
	miso = (0xFF00) & (SPDR0 << 8); // store MSB of MISO

	//while (!(SPSR0 & (1 << SPIF)));
	SPDR0 = (char)mosi; // send LSB of MOSI

	while (!(SPSR0 & (1 << SPIF)));// wait for transmission complete
	miso |= (0x00FF)&(SPDR0); // store LSB of MISO

	return miso;
}
unsigned int spi1_exchange_int(unsigned int mosi) // not tested
{
	unsigned int miso = 0;
	
	// first send MSB
	//while (!(SPSR1 & (1 << SPIF)));
	SPDR1 = (char)(mosi >> 8); // start transmission by sending MSB of MOSI

	while (!(SPSR1 & (1 << SPIF))); // wait for transmission complete
	miso = (0xFF00) & (SPDR1 << 8); // store MSB of MISO

	//while (!(SPSR1 & (1 << SPIF)));
	SPDR1 = (char)mosi; // send LSB of MOSI

	while (!(SPSR1 & (1 << SPIF)));// wait for transmission complete
	miso |= (0x00FF)&(SPDR1); // store LSB of MISO

	return miso;
}

void spi0_send_int(unsigned int mosi) // not tested
{
	while (!(SPSR0 & (1 << SPIF)));
	SPDR0 = (char)(mosi >> 8); // start transmission by sending MSB of MOSI

	while (!(SPSR0 & (1 << SPIF)));
	SPDR0 = (char)mosi; // send LSB of MOSI
}
void spi1_send_int(unsigned int mosi) // not tested
{
	while (!(SPSR1 & (1 << SPIF)));
	SPDR1 = (char)(mosi >> 8); // start transmission by sending MSB of MOSI

	while (!(SPSR1 & (1 << SPIF)));
	SPDR1 = (char)mosi; // send LSB of MOSI
}


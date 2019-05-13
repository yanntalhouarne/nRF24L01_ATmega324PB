#include "spi.h"

void SPI0_masterInit()
{
	DDRB |= (1 << MOSI_0);  // MOSI_0 is output
	DDRB |= (1 << SCK_0);   // SCK_0 is output
	DDRB |= (1 << SS_0);   // SS_0 is output
	
	SPCR0 = (1 << SPE) | (1 << MSTR) | (1 << SPR0) | (0 << CPHA); // Enable SPI, Master, set clock rate fck/16, SPI_MODE1, MSB first
}
void SPI0_slaveInit()
{
	DDRB |= (1 << MISO_0);	// MSIO_0 is output
	
	SPCR0 = (1 << SPE); // Enable SPI, Slave
}

void SPI1_masterInit()
{
	DDRE |= (1 << MOSI_1);  // MOSI_1 is output
	DDRD |= (1 << SCK_1);   // SCK_1 is output
	DDRD |= (1 << SS_1);   // SS_1 is output
	
	SPCR1 = (1 << SPE) | (1 << MSTR) | (1 << SPR1); // Enable SPI, Master, set clock rate fck/16, SPI_MODE0, MSB first
}
void SPI1_slaveInit()
{
	DDRE |= (1 << MISO_1);	// MSIO_1 is output
	
	SPCR1 = (1 << SPE); // Enable SPI, Slave
}


void SPI0_sendChar(char data)
{
	SPDR0 = data;
	while (!(SPSR0 & (1 << SPIF)))
	; // wait for transmission to complete
}
void SPI1_sendChar(char data)
{
	SPDR1 = data;
	while (!(SPSR1 & (1 << SPIF)))
	; // wait for transmission to complete
}

char SPI0_exchangeChar(char data)
{
	SPDR0 = data; // start transmission
	
	while (!(SPSR0 & (1 << SPIF)))
	; // wait for transmission complete
	
	return SPDR0;
}
char SPI1_exchangeChar(char data)
{
	SPDR1 = data; // start transmission
	
	while (!(SPSR1 & (1 << SPIF)))
	; // wait for transmission complete
	
	return SPDR1;
}

void SPI0_sendBytes(char *pdata, char bytes)
{
	int i;

	for (i = 0; i < bytes; i++)
	{
		SPDR0 = pdata[i]; // start transmission
		while (!(SPSR0 & (1 << SPIF)))
		; // wait for transmission complete
	}
}
void SPI1_sendBytes(char *pdata, char bytes)
{
	int i;

	for (i = 0; i < bytes; i++)
	{
		SPDR1 = pdata[i]; // start transmission
		while (!(SPSR1 & (1 << SPIF)))
		; // wait for transmission complete
	}
}

void SPI0_exchangeBytes(char *mosi, char *miso, char bytes)
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
void SPI1_exchangeBytes(char *mosi, char *miso, char bytes)
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

unsigned int SPI0_exchangeInt(unsigned int mosi) // not tested
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
unsigned int SPI1_exchangeInt(unsigned int mosi) // not tested
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

void SPI0_sendInt(unsigned int mosi) // not tested
{
	while (!(SPSR0 & (1 << SPIF)));
	SPDR0 = (char)(mosi >> 8); // start transmission by sending MSB of MOSI

	while (!(SPSR0 & (1 << SPIF)));
	SPDR0 = (char)mosi; // send LSB of MOSI
}
void SPI1_sendInt(unsigned int mosi) // not tested
{
	while (!(SPSR1 & (1 << SPIF)));
	SPDR1 = (char)(mosi >> 8); // start transmission by sending MSB of MOSI

	while (!(SPSR1 & (1 << SPIF)));
	SPDR1 = (char)mosi; // send LSB of MOSI
}

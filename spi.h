/*
 * spi.h
 *
 * Created: 10/16/2017 7:12:34 PM
 *  Author: talho_000
 */

#include <avr/io.h>

#ifndef SPI_H_
#define SPI_H_

//SPI_0 pins
#define MISO_0 6 // PB6
#define MOSI_0 5 // PB5
#define SCK_0 7  // PB7
#define SS_0 4   // PB4

//SPI_1 pins
#define MISO_1 2 // PE2
#define MOSI_1 3 // PE3
#define SCK_1 7  // PD7
#define SS_1 6   // PD6	

// setup
	void SPI0_masterInit();
	void SPI0_slaveInit();
	
	void SPI1_masterInit();
	void SPI1_slaveInit();
	
	
// char (one byte)
	void SPI0_sendChar(char data); // send a single char (one byte)
	void SPI1_sendChar(char data);
	
	char SPI0_exchangeChar(char data); 
	char SPI1_exchangeChar(char data); // exchange single char (one byte)
	
	
// array of char (multiple bytes)
	void SPI0_sendBytes(char *pdata,char bytes); // send array of char of size "bytes"
	void SPI1_sendBytes(char *pdata,char bytes);
	
	void SPI0_exchangeBytes(char *mosi, char *miso, char bytes); // exchange array of char of size "bytes"
	void SPI1_exchangeBytes(char *mosi, char *miso, char bytes);
	
	
// int (single int)
	void SPI0_sendInt(unsigned int mosi); // send a int value LSB first
	void SPI1_sendInt(unsigned int mosi);
	
	unsigned int SPI0_exchangeInt(unsigned int mosi); // exchange int value
	unsigned int SPI1_exchangeInt(unsigned int mosi);

#endif /* SPI_H_ */
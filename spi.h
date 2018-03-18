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
	void spi0_master_initialize();
	void spi0_slave_initialize();
	
	void spi1_master_initialize();
	void spi1_slave_initialize();
	
	
// char (one byte)
	void spi0_send_char(char data); // send a single char (one byte)
	void spi1_send_char(char data);
	
	char spi0_exchange_char(char data); 
	char spi1_exchange_char(char data); // exchange single char (one byte)
	
	
// array of char (multiple bytes)
	void spi0_send_bytes(char *pdata,char bytes); // send array of char of size "bytes"
	void spi1_send_bytes(char *pdata,char bytes);
	
	void spi0_exchange_bytes(char *mosi, char *miso, char bytes); // exchange array of char of size "bytes"
	void spi1_exchange_bytes(char *mosi, char *miso, char bytes);
	
	
// int (single int)
	void spi0_send_int(unsigned int mosi); // send a int value LSB first
	void spi1_send_int(unsigned int mosi);
	
	unsigned int spi0_exchange_int(unsigned int mosi); // exchange int value
	unsigned int spi1_exchange_int(unsigned int mosi);

#endif /* SPI_H_ */
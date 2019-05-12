#ifndef _MIRF_H_
#define _MIRF_H_

#define F_CPU 16000000

#include <avr/io.h>
#include "nRF24L01.h"
#include "spi.h"
#include <util/delay.h>

// Mirf settings
#define mirf_CH 108
#define mirf_PAYLOAD 6
#define mirf_CONFIG (1 << MASK_RX_DR) | (1 << MAX_RT) | (1 << EN_CRC) | (0 << CRCO)

// Pin definitions for chip select and chip enabled of the MiRF module
#define CE 7 // PC7

// Definitions for selecting and enabling MiRF module
#define nRF_SET_CSN PORTD |=  (1 << SS_1);  // PD6 is Chip Select for SPI
#define nRF_CLEAR_CSN PORTD &= ~(1 << SS_1);
#define nRF_SET_CE  PORTC |=  (1 << CE); // PC7 is Chip Enable
#define nRF_CLEAR_CE  PORTC &= ~(1 << CE);


// Public standard functions
extern void nRF_init();
extern void nRF_config();
extern void nRF_sendData(char *value, char len);
extern void nRF_RXsetAddr(char *adr);
extern void nRF_TXsetAddr(char *adr);
extern unsigned char nRF_getMaxRT();
extern char nRF_dataReady();
extern char nRF_dataSent();
extern void nRF_getData(char *data);
void nRF_SetRxMode();

// Public extended functions
extern void nRF_configReg(char reg, char value);
extern void nRF_readReg(char reg, char *value, char len);
extern void nRF_writeReg(char reg, char *value, char len);

#endif /* _MIRF_H_ */
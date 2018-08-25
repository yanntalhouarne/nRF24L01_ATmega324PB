#ifndef _MIRF_H_
#define _MIRF_H_

#define F_CPU 16000000

#include <avr/io.h>
#include "nRF24L01.h"
#include "spi.h"
#include <util/delay.h>

// Mirf settings
#define mirf_CH 108
#define mirf_PAYLOAD 3
#define mirf_CONFIG (1 << MASK_RX_DR) | (1 << MAX_RT) | (1 << EN_CRC) | (0 << CRCO)

// Pin definitions for chip select and chip enabled of the MiRF module
#define CE 7 // PC7

// Definitions for selecting and enabling MiRF module
#define mirf_CSN_hi PORTD |=  (1 << SS_1);  // PD6 is Chip Select for SPI
#define mirf_CSN_lo PORTD &= ~(1 << SS_1);
#define mirf_CE_hi  PORTC |=  (1 << CE); // PC7 is Chip Enable
#define mirf_CE_lo  PORTC &= ~(1 << CE);


// Public standard functions
extern void mirf_init();
extern void mirf_config();
extern void mirf_send(char *value, char len);
extern void mirf_set_RADDR(char *adr);
extern void mirf_set_TADDR(char *adr);
extern unsigned char mirf_read_MAX_RT();
extern char mirf_data_ready();
extern char mirf_data_sent();
extern void mirf_get_data(char *data);
void set_RX_MODE();

// Public extended functions
extern void mirf_config_register(char reg, char value);
extern void mirf_read_register(char reg, char *value, char len);
extern void mirf_write_register(char reg, char *value, char len);

#endif /* _MIRF_H_ */
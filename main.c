/*
 * nRF24L01_Atmega324PB.c
 *
 * Created: 3/17/2018 11:41:05 PM
 * Author : Yann
 */ 
#define F_CPU 8000000

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "print.h"
#include "usart.h"
#include "spi.h"
#include "mirf.h"


int main(void)
{
	setup_usart0(BR_9600); // for FTDI debugging (terminal)
	spi1_master_initialize(); // setup device as master for SPI com with nRF24L01
	
	println_0("System initialized;");
	
	sei(); // enable global interrupts
	
    while (1) 
    {
		_delay_ms(500);
			
    }
}

	

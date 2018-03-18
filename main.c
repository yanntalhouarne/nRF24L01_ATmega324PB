/*
 * nRF24L01_Atmega324PB.c
 *
 * Created: 3/17/2018 11:41:05 PM
 * Author : Yann
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "print.h"
#include "usart.h"
#include "spi.h"

// function declarations
void system_initialize();
void setup_gpio();

// macros
#define BR 51 // baud rate for USART (51 for 9600 at 8MHz)
#define SS1 0 // chip select for AS5048a is PB0

// AS5048a commands
#define READ_ANGLE 0x3FFF
#define NOP 0x0000



int main(void)
{
	println_0("System initialized;");
	
	
	unsigned int miso1 = 0; // MISO 2-byte variable for SPI1 (AS5048a)
	
    while (1) 
    {
		PORTB &= ~(1<<SS1);
			miso1 = spi1_exchange_int(READ_ANGLE);
		PORTB |= (1<<SS1);
		
		_delay_us(10);
		
		PORTB &= ~(1<<SS1);
			miso1 = spi1_exchange_int(READ_ANGLE);
		PORTB |= (1<<SS1);
		
		println_int(miso1);
		
		_delay_ms(1000);
			
    }
}

void system_initialize()
{
	setup_gpio();
	// communication setup
	setup_usart0(BR); // for FTDI debugging (terminal)
	spi1_master_initialize(); // setup device as master for SPI com with nRF24L01
	
	sei(); // enable global interrupts
}

void setup_gpio()
{
	DDRB |= (1<<SS1); // PB0 is chip select for AS5048
	PORTB |= (1<<SS1); // pull SS0 high
}
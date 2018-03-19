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

#define LED_DDR DDRB
#define LED_PORT PORTB 
#define LED_PIN 0
#define LED_ON LED_PORT |= (1<<LED_PIN);
#define LED_OFF LED_PORT &= ~(1<<LED_PIN);
#define TOGGLE_LED LED_PORT ^= (1<<LED_PIN);

#define BUFFER_SIZE 2

char buffer[mirf_PAYLOAD];

void setup_gpios();

uint8_t status = 0;

int main(void)
{
	setup_gpios();
	setup_usart0(BR_9600); // for FTDI debugging (terminal)
	spi1_master_initialize(); // setup device as master for SPI com with nRF24L01
	mirf_init();
	_delay_ms(50);	
	
	TOGGLE_LED;
	_delay_ms(100);
	TOGGLE_LED;
	_delay_ms(100);
	TOGGLE_LED;
	_delay_ms(100);
	TOGGLE_LED;
	_delay_ms(100);
	TOGGLE_LED;
	_delay_ms(100);
	TOGGLE_LED;
	_delay_ms(100);
	TOGGLE_LED;
	_delay_ms(100);
	TOGGLE_LED
	
	sei(); // enable global interrupts
	
	mirf_config();

	println_0("nRF24L01 initialized...;");
	_delay_ms(1);

	println_0("Testing comunication...;");
	_delay_ms(1);
	mirf_send(buffer, mirf_PAYLOAD);

	println_0("Waiting for echo...;");
	while(!mirf_data_ready())
	{
		/*
		mirf_CSN_lo;       // Pull down chip select
		status = spi1_exchange_char(NOP); // Read status register
		mirf_CSN_hi;                     // Pull up chip select
		print_0("status: ;");
		println_int_0(status);
		*/
	}
	mirf_get_data(buffer);

	print_0("Echo received: ;");
	print_char_0(buffer[0]);
	print_char_0(',');
	print_char_0(' ');
	print_char_0(buffer[1]);
	print_char_0(NL);
	

	uint8_t status = 0;

    while (1) 
    {
		TOGGLE_LED;
		
		//mirf_send(buffer, BUFFER_SIZE);
		
		/*
		mirf_CSN_lo;
		status = spi1_exchange_char(NOP);
		mirf_CSN_hi;
		
		_delay_us(10);
		print_0("status: ;");
		print_int_0(status)
		
		
		print_char_0(NL);
		*/
		
		_delay_ms(500);
			
    }
}

void setup_gpios()
{
	LED_DDR |= (1<<LED_PIN); // set LED gpio as output 
}
	

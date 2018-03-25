/*
 * nRF24L01_Atmega324PB.c
 *
 * Created: 3/17/2018 11:41:05 PM
 * Author : Yann
 */ 
#define F_CPU 8000000
#define LOOP_DELAY 100

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

char buffer[mirf_PAYLOAD] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
char rx_buffer[mirf_PAYLOAD] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

void setup_gpios();

uint8_t status = 0;

int main(void)
{
	setup_gpios();
	setup_usart0(BR_500000); // for FTDI debugging (terminal)
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
	
	
	_delay_ms(1000);
	
	sei(); // enable global interrupts
	
	mirf_config();

	println_0("nRF24L01 initialized...;");
	_delay_ms(10);

    while (1) 
    {
		buffer[0]++;
		//buffer[1] = 2;
		TOGGLE_LED;
		//println_0("Sending data...;");
		//_delay_ms(1);
		mirf_send(buffer, mirf_PAYLOAD);
		_delay_us(10);
		while (!mirf_data_sent());
		mirf_config_register(STATUS, (1 << TX_DS) | (1 << MAX_RT)); // Reset status register

		//println_0("Waiting for echo...;");
		_delay_us(10);
		while(!mirf_data_ready()); // wait for the receiver to echo the data sent

		//mirf_config_register(STATUS, (1 << RX_DR) | (1 << MAX_RT)); // Reset status register
		//LED_ON; // turn on LED if echo has been received
		mirf_get_data(rx_buffer); // read the data from the nRF24L01
		//TOGGLE_LED;	// toggle LED while waiting
		
		//print_0("Response received: ;"); // send data to uart_0 (terminal)
		//println_int_0(rx_buffer[0]);
		//print_char_0(',');
		//print_char_0(' ');
		//print_int_0(buffer[1]);
		//print_char_0(NL);
		
		//while(1);
		
		_delay_ms(LOOP_DELAY);
			
    }
}

void setup_gpios()
{
	LED_DDR |= (1<<LED_PIN); // set LED gpio as output 
}
	
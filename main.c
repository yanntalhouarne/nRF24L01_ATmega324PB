 
#define F_CPU 8000000

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "usart.h"
#include "print.h"
#include "spi.h"
#include "mirf.h"

#define LOOP_DELAY 1000


 //&&&&&&&&&&&&&&&&& MACROS &&&&&&&&&&&&&&&&&&&&&&
 //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
/* LED */
#define LED_DDR DDRE
#define LED_PORT PORTE
#define LED_PIN 4
#define LED_ON LED_PORT |= (1<<LED_PIN);
#define LED_OFF LED_PORT &= ~(1<<LED_PIN);
#define TOGGLE_LED LED_PORT ^= (1<<LED_PIN);

/* miscellaneous */
void setup_gpios();
void flash_LED(uint8_t count, uint16_t ms);
void delay_ms(uint16_t ms);




 //&&&&&&&&&&&&&&&&& GLOABLS &&&&&&&&&&&&&&&&&&&&&
 //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
 /* nRF24L01 variables */
char buffer[mirf_PAYLOAD] = {0,0,0}; // for the nRF24L01 receive and transmit data
char tx_address[5] = {0xD7,0xD7,0xD7,0xD7,0xD7};


int main(void)
{
	setup_gpios(); 
	
	/* USART setup */
	setup_usart0(BR_9600); // for NEO6 GPS
	
	/* nRF24L01 setup */
	spi1_master_initialize(); // setup device as master for SPI com with nRF24L01
	mirf_init(); // initialize nRF24L01
	mirf_config(); // configure nRF24L01
	mirf_set_TADDR(tx_address);
	
	buffer[0] = '1';
	buffer[1] = '2';
	buffer[2] = '3';
			
	sei(); // enable global interrupts
	
	/* setup complete notification */
	flash_LED(10, 50); // flash LED 10 times at intervals of 50ms
	_delay_ms(1000);
	
    while (1) 
    {
		TOGGLE_LED;
		 
		 
		mirf_send(buffer, mirf_PAYLOAD);
		
		while (!mirf_data_sent())
		{
			if (mirf_read_MAX_RT())
			{
				println_0("MAX_RT;");
				break;
			}
		}
		
		_delay_ms(LOOP_DELAY);
    }
}




//&&&&&&&&&&&&&&&&&&&&&& MISC &&&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void setup_gpios()
{
	LED_DDR |= (1<<LED_PIN); // set LED gpio as output
	DDRC |= (1<<4);
	
}
void flash_LED(uint8_t count, uint16_t ms)
{
	for (uint8_t i = 0; i < count; i++)
	{
		TOGGLE_LED;
		delay_ms(ms);
	}
}
void delay_ms(uint16_t ms)
{
	for (uint16_t i  = 0; i < ms; i++)
	{
		_delay_ms(1);
	}
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&



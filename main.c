 
#define F_CPU 16000000

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "usart.h"
#include "print.h"
#include "spi.h"
#include "mirf.h"

#define LOOP_DELAY 10


 //&&&&&&&&&&&&&&&&& MACROS &&&&&&&&&&&&&&&&&&&&&&
 //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
/* LED */
#define LED1_DDR DDRE
#define LED1_PORT PORTE
#define LED1_PIN 4
#define LED1_ON LED1_PORT |= (1<<LED1_PIN);
#define LED1_OFF LED1_PORT &= ~(1<<LED1_PIN);
#define TOGGLE_LED1 LED1_PORT ^= (1<<LED1_PIN);

#define LED2_DDR DDRB
#define LED2_PORT PORTB
#define LED2_PIN 5
#define LED2_ON LED2_PORT |= (1<<LED2_PIN);
#define LED2_OFF LED2_PORT &= ~(1<<LED2_PIN);
#define TOGGLE_LED2 LED2_PORT ^= (1<<LED2_PIN);



/* functions */
void setup_gpios();
void flash_LED(uint8_t count, uint16_t ms);
void delay_ms(uint16_t ms);
void setup_timer_1();


 //&&&&&&&&&&&&&&&&& GLOBALS &&&&&&&&&&&&&&&&&&&&&
 //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
 /* nRF24L01 variables */
char buffer[mirf_PAYLOAD] = {0,0,2}; // for the nRF24L01 receive and transmit data
char tx_address[5] = {0xD7,0xD7,0xD7,0xD7,0xD7};
	
int16_t mtr_cmd = 1000;	
uint16_t mtr_cmd_to_send = 1000;
int8_t sign = 1;


	
volatile uint16_t max_rt_count = 0;
volatile uint16_t transmission_count = 0;
volatile float signal_strength = 0;

volatile uint8_t temp = 0;

volatile uint8_t TMR1_flag = 0;

int main(void)
{
	setup_gpios(); 
	
	/* USART setup */
	setup_usart0(BR_38400); // for NEO6 GPS
	
	/* nRF24L01 setup */
	spi1_master_initialize(); // setup device as master for SPI com with nRF24L01
	mirf_init(); // initialize nRF24L01
	mirf_config(); // configure nRF24L01
	mirf_set_TADDR(tx_address);
	setup_timer_1();
	void print_signal_strength();
	
	sei(); // enable global interrupts
	
	/* setup complete notification */
	flash_LED(10, 50); // flash LED 10 times at intervals of 50ms
	println_0("System initialized...;");
	_delay_ms(10);
	
    while (1) 
    {
		TOGGLE_LED2;
		
  		if ( (mtr_cmd > 999) || (mtr_cmd < -999) )
			sign *= -1;
			
		mtr_cmd += sign;

		// data sent as a sigend 16-bit
		buffer[0] = mtr_cmd;	  // LSB
		buffer[1] = (mtr_cmd>>8); // MSB 
		  
		mirf_send(buffer, mirf_PAYLOAD);
		
		transmission_count++;
		
		
		while (!mirf_data_sent())
		{	
			if (mirf_read_MAX_RT())
			{
				max_rt_count++;	
				break;
			}
		}
		
 		if (TMR1_flag)
 		{
 			TMR1_flag = 0;
			TOGGLE_LED1;
 		}
		
		//_delay_ms(LOOP_DELAY);
    }
}




//&&&&&&&&&&&&&&&&&&&&&& FUNCTIONS &&&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void setup_gpios()
{
	LED1_DDR |= (1<<LED1_PIN); // set LED gpio as output
	LED2_DDR |= (1<<LED2_PIN);
	DDRC |= (1<<4);
	
}
void flash_LED(uint8_t count, uint16_t ms)
{
	for (uint8_t i = 0; i < count; i++)
	{
		TOGGLE_LED2;
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
void setup_timer_1()
{
	
	TCCR1A |= (1<<COM1A1) | (1<<COM1A0); // clear OCRA on compare
	TCCR1B |= (1<<CS12) | (1<<CS10) | (1<<WGM12); // 1024 prescaler, CTC mode
	OCR1A = 7812; // 1 second period
	TIMSK1 |= (1<<OCIE1A); // set interrupt on OCA compare

}

void print_signal_strength()
{
// 	print_int_0(transmission_count);
// 	print_char_0('/');
// 	println_int_0(max_rt_count);
	
	signal_strength = (float)(max_rt_count)/(max_rt_count);
	if (signal_strength>.99)
	signal_strength = .99;
	
	signal_strength = 1-signal_strength;
	
	signal_strength *= 100;
	
	max_rt_count = (uint16_t)signal_strength;
	
	print_int_0(max_rt_count);
	print_char_0('%');
	print_char_0(NL);
	
	transmission_count = 0;
	signal_strength = 0;
	max_rt_count = 0;
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&



//&&&&&&&&&&&&&&&&&&&&&& ISRs &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
ISR(TIMER1_COMPA_vect)
 {
	 TMR1_flag = 1;
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&


/*
 * nRF24L01_Atmega324PB.c
 *
 * Created: 3/17/2018 11:41:05 PM
 * Author : Yann
 */ 
#define F_CPU 8000000
#define LOOP_DELAY 30

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "print.h"
#include "usart.h"
#include "spi.h"
#include "mirf.h"

// LED
#define LED_DDR DDRE
#define LED_PORT PORTE
#define LED_PIN 4
#define LED_ON LED_PORT |= (1<<LED_PIN);
#define LED_OFF LED_PORT &= ~(1<<LED_PIN);
#define TOGGLE_LED LED_PORT ^= (1<<LED_PIN);

// IN1
#define IN1 1
#define IN1_DDR DDRB
#define IN1_PORT PORTB
#define IN1_PIN 1

// IN2
#define IN2 2
#define IN2_DDR DDRB
#define IN2_PORT PORTB
#define IN2_PIN 2

// ENB 
#define ENA_PORT PORTD
#define ENA_DDR DDRD
#define ENA 5



#define BUFFER_SIZE 2

char buffer[mirf_PAYLOAD] = {0,0};
	
int8_t tx_address[5] = {0xD7,0xD7,0xD7,0xD7,0xD7};
int8_t rx_address[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};

void setup_gpios();
void setup_gpios();
void setup_pwm();
void set_duty_cycle(int duty_cycle);
void move_motor_forward();
void move_motor_backward();
void motor_off();

uint8_t status = 0;


int main(void)
{
	setup_gpios();
	setup_usart0(BR_500000); // for FTDI debugging (terminal)
	spi1_master_initialize(); // setup device as master for SPI com with nRF24L01
	mirf_init();
	setup_pwm();
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
	
	//mirf_config_register(EN_AA, (0<<0)); // disable auto ACK for pipe 0
	//mirf_config_register(EN_AA, (0<<1));

	mirf_config();
	
	mirf_set_TADDR(tx_address);
	mirf_set_RADDR(rx_address);

	println_0("nRF24L01 initialized...;");
	_delay_ms(10);

    while (1) 
    {
		TOGGLE_LED;
		//print_char_0(',');
		//buffer[0]++;
		//buffer[1] = 2;
		//println_0("Sending data...;");
		//_delay_ms(1);
		//mirf_send(buffer, mirf_PAYLOAD);
		while(!mirf_data_ready());
		mirf_get_data(buffer);
		//_delay_ms(30);
		//while (!mirf_data_sent());
		//mirf_config_register(STATUS, (1 << TX_DS) | (1 << MAX_RT)); // Reset status register

		//println_0("Waiting for echo...;");
	//	_delay_us(10);
	//	while(!mirf_data_ready()); // wait for the receiver to echo the data sent

		//mirf_config_register(STATUS, (1 << RX_DR) | (1 << MAX_RT)); // Reset status register
		//LED_ON; // turn on LED if echo has been received
		//mirf_get_data(rx_buffer); // read the data from the nRF24L01
		//TOGGLE_LED;	// toggle LED while waiting
		
		//print_0("Response received: ;"); // send data to uart_0 (terminal)
		//println_int_0(rx_buffer[0]);
		//print_char_0(',');
		//print_char_0(' ');
		//print_int_0(buffer[1]);
		//print_char_0(NL);
		
		//while(1);
		
		//println_int_0(buffer[0]);
		
		if ((buffer[0] < -100) || (buffer[0] > 100))
		{
			motor_off();
		}
		else if ( (buffer[0]<6)) // deadband
		{
			motor_off();
		}
		else if (buffer[0]>6)
		{
			set_duty_cycle(buffer[0]);
			move_motor_forward();
		}
		else if (buffer[0]<0)
		{
			set_duty_cycle(buffer[0]);
			move_motor_backward();
		}
		
		//_delay_ms(LOOP_DELAY);
			
    }
}

void setup_gpios()
{
	LED_DDR |= (1<<LED_PIN); // set LED gpio as output 
	IN1_DDR |= (1<<IN1);
	IN2_DDR |= (1<<IN2);
	ENA_DDR |= (1<<ENA);
	
}
	
void setup_pwm()
{
	TCCR1A |= (1 << WGM10) | (1 << COM1A1)| (1 << COM1A0); // fast PWM
	TCCR1B |= (1 << WGM12) | (1 << CS10);                                 // no prescaler with f_osc (so 62.5KHz PWM)
}

void set_duty_cycle(int duty_cycle)
{
	duty_cycle = 2.56 * duty_cycle - 1;
	OCR1A = (char)((0x00FF) & duty_cycle);
}

void move_motor_forward()
{
	IN1_PORT |= (1<<IN1);
	IN2_PORT &= ~(1<<IN2);
}

void move_motor_backward()
{
	IN1_PORT &= ~(1<<IN1);
	IN2_PORT |= (1<<IN2);
}

void motor_off()
{
	IN1_PORT |= (1<<IN1);
	IN2_PORT |= (1<<IN2);
}

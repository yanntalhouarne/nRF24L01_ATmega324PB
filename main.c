/*
 * nRF24L01_Atmega324PB.c
 *
 * Created: 3/17/2018 11:41:05 PM
 * Author : Yann
 */ 
#define F_CPU 8000000
#define LOOP_DELAY 10

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "print.h"
#include "usart.h"
#include "spi.h"
#include "mirf.h"

// LED
#define LED_DDR DDRB
#define LED_PORT PORTB 
#define LED_PIN 0
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

#define BUFFER_SIZE 2

char buffer[mirf_PAYLOAD] = {0,0};
//char rx_buffer[mirf_PAYLOAD] = {0,0};

void setup_gpios();
void setup_pwm();
void set_duty_cycle(int duty_cycle);\
void move_motor_forward();
void move_motor_backward();
void motor_off();

uint8_t status = 0;

int main(void)
{
	setup_gpios();
	setup_usart0(BR_500000); // for FTDI debugging (terminal)
	spi1_master_initialize(); // setup device as master for SPI com with nRF24L01
	setup_pwm();
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
		TOGGLE_LED;
		
		while(!mirf_data_ready())
		{
			mirf_config_register(STATUS, (1 << MAX_RT)); // Reset status register
		}
		
		mirf_get_data(buffer);
		
		set_duty_cycle(buffer[0]);	
		
		if ((buffer[0] < -100) || (buffer[0] > 100) || (buffer[0] == 0))
		{
			motor_off();
		}
		else if (buffer[0]>0)
		{
			set_duty_cycle(buffer[0]);
			move_motor_forward();	
		}
		else if (buffer[0]<0)
		{
			set_duty_cycle(buffer[0]);
			move_motor_backward();
		}
    }
}

void setup_gpios()
{
	LED_DDR |= (1<<LED_PIN); // set LED gpio as output 
}
	
void setup_pwm()
{
	TCCR0A |= (1 << WGM00) | (1 << WGM01) | (1 << COM0A1); // fast PWM, non-inverted
	TCCR0B |= (1 << CS00);                                 // no prescaler with f_osc (so 62.5KHz PWM)
}

void set_duty_cycle(int duty_cycle)
{
	duty_cycle = 2.56 * duty_cycle - 1;
	OCR0A = (char)((0x00FF) & duty_cycle);
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
	PORTD |= (1<<IN1);
	PORTC |= (1<<IN2);
}


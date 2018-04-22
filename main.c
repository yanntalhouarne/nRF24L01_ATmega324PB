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
#include "adc.h"

// thermistor
#define SERIESRESISTOR 10000 // for temp sensor
#define THERMISTORNOMINAL 10000  // resistance at 25C
#define BCOEFFICIENT 3400 // beta coeff. of thermistor
#define TEMPERATURENOMINAL 25
#define TEMP_PIN 0 // AN0
#define CURRENT_PIN 1 // AN1


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

// servo PWM PIN
#define SERVO_PWM_DDR DDRB
#define SERVO_PWM 3  // PB3



#define BUFFER_SIZE 3

int8_t buffer[mirf_PAYLOAD] = {0,0,0};
	
int8_t tx_address[5] = {0xD7,0xD7,0xD7,0xD7,0xD7};
int8_t rx_address[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
	
int16_t mtr_cmd = 0;
int8_t srv_cmd = 0;

float raw_temp = 0;
float temp = 0;

float raw_current = 0;
float current = 0;

void setup_gpios();
void setup_TMR1_pwm();
void setup_TMR0_pwm();
void set_TMR1_duty_cycle(int duty_cycle);
void set_TMR3_duty_cycle(int duty_cycle);
void move_motor_forward();
void move_motor_backward();
void motor_off();
void move_servo(float angle);
int temp_scaling(float raw_value);
int current_scaling(float raw_value);

uint8_t status = 0;


int main(void)
{
	setup_gpios();
	setup_usart0(BR_500000); // for FTDI debugging (terminal)
	spi1_master_initialize(); // setup device as master for SPI com with nRF24L01
	mirf_init();
	setup_adc();
	setup_TMR1_pwm();
	setup_TMR0_pwm();
			
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
	
	mirf_set_TADDR(tx_address);
	mirf_set_RADDR(rx_address);

	println_0("nRF24L01 initialized...;");
	_delay_ms(10);

    while (1) 
    {
		TOGGLE_LED;
		
		while(!mirf_data_ready());
		
		mirf_get_data(buffer);
		
		mtr_cmd = ((0xFF00)&(buffer[0]<<8)) | ((0x00FF)&(buffer[1]));
		srv_cmd = buffer[2];
// 		
// 		println_int_0(mtr_cmd);
// 		println_int_0(srv_cmd);
// 		
// 		print_char_0(' ');
// 		print_char_0(NL);
		
		if (mtr_cmd > 0 )
		{
			set_TMR1_duty_cycle(mtr_cmd);
			move_motor_forward();
		}
		else if (mtr_cmd < 0)
		{
			set_TMR1_duty_cycle(abs(mtr_cmd));
			move_motor_backward();
		}
		else if (abs(mtr_cmd) < 100)
		{
			set_TMR1_duty_cycle(1);
			motor_off();
		}
		
		move_servo((float)srv_cmd);
		
		// get temp
		raw_temp = analog_get_average(TEMP_PIN,5);	
		temp = temp_scaling(raw_temp);
		
// 		// get 
// 		raw_current = analog_get_average(CURRENT_PIN,5);
// 		//current = current_scaling(raw_current)
// 		current = raw_current / 204.6;
// 		current -= 2.5;
// 		 current *= 10;
		
		println_int_0(temp);
		//println_int_0(current);
		
		_delay_ms(LOOP_DELAY);

    }
}

void setup_gpios()
{
	LED_DDR |= (1<<LED_PIN); // set LED gpio as output 
	IN1_DDR |= (1<<IN1);
	IN2_DDR |= (1<<IN2);
	ENA_DDR |= (1<<ENA);
	SERVO_PWM_DDR |= (1<<SERVO_PWM); 
	
}
	
void setup_TMR1_pwm()
{
	TCCR1A |= (1 << WGM10) | (1 << COM1A1); // fast PWM
	TCCR1B |= (1 << WGM12) | (1 << CS10); // no prescaler with f_osc (so 62.5KHz PWM)
}

void setup_TMR0_pwm()
{
	TCCR0A |= (1 << COM0A1) | (1 << WGM01) | (1 << WGM00); // fast PWM, Clear OC3A/OC3B on Compare Match, set OC3A/OC3B at BOTTOM (non-inverting mode)
	TCCR0B |=  (1 << CS02); // prescaler of 256 with f_osc (so 62.5KHz PWM)
	 move_servo(45);
}

void set_TMR1_duty_cycle(int duty_cycle)
{
	duty_cycle = .256 * duty_cycle - 1;
	if (duty_cycle > 255)
	duty_cycle = 255; 
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

void move_servo(float angle)
{ 
	angle = 47 + angle*.355;
	OCR0A = (uint8_t)angle;
}

int temp_scaling(float raw_value)
{
	float steinhart;
	raw_value = 1023 / raw_value - 1;
	raw_value = SERIESRESISTOR / raw_value;

	steinhart = raw_value / THERMISTORNOMINAL;     // (R/Ro)
	steinhart = log(steinhart);                  // ln(R/Ro)
	steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
	steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
	steinhart = 1.0 / steinhart;                 // Invert
	steinhart -= 273.15;                         // convert to C
	
	return steinhart;
}

int current_scaling(float raw_value)
{
		float  value = 0;
		value = raw_value / 204.6; // get voltage
 		//value -= 2.5; // get rid of offset
 		//value *= 1000; // scale to current
		
		return value;
}
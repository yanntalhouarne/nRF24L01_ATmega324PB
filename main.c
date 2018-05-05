 
#define F_CPU 16000000
#define LOOP_DELAY 10

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "print.h"
#include "usart.h"
#include "spi.h"
#include "mirf.h"
#include "adc.h"

 //&&&&&&&&&&&&&&&&& MACROS &&&&&&&&&&&&&&&&&&&&&&
 //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
/* thermistor */
#define SERIESRESISTOR 10000 // for temp sensor
#define THERMISTORNOMINAL 10000  // resistance at 25C
#define BCOEFFICIENT 3400 // beta coeff. of thermistor
#define TEMPERATURENOMINAL 25
#define TEMP_PIN 0 // AN0
#define CURRENT_PIN 1 // AN1
/* LED */
#define LED_DDR DDRE
#define LED_PORT PORTE
#define LED_PIN 4
#define LED_ON LED_PORT |= (1<<LED_PIN);
#define LED_OFF LED_PORT &= ~(1<<LED_PIN);
#define TOGGLE_LED LED_PORT ^= (1<<LED_PIN);
/* IN1 */
#define IN1 5
#define IN1_DDR DDRD
#define IN1_PORT PORTD
#define IN1_PIN 5
/* IN2 */
#define IN2 4
#define IN2_DDR DDRC
#define IN2_PORT PORTC
#define IN2_PIN 4
/* ENA */
#define EN1_PORT PORTB
#define EN1_DDR DDRB
#define EN1 3
/* ENB */ 
#define EN2_PORT PORTB
#define EN2_DDR DDRB
#define EN2 2
/* servo PWM PIN */
#define SERVO_PWM_DDR DDRB
#define SERVO_PWM 3  // PB3
/* nRF24L01 */
#define BUFFER_SIZE 3
#define GET_TEMP 'T'
#define GET_LAT 'A'
#define GET_LON 'O'



 //&&&&&&&&&&&& FUNCTION PROTOTYPES &&&&&&&&&&&&&&
 //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
 /* DC motor */
void setup_TMR1_pwm();
void set_TMR1A_duty_cycle(int duty_cycle);
void set_TMR4A_duty_cycle(int duty_cycle);
void move_motor_forward();
void move_motor_backward();
void motor_off();
/* Servo */
void setup_TMR0_pwm();
void move_servo(float angle);
/* current sensor */
int16_t current_scaling(float raw_value);
int16_t get_current();
/* temp sensor */
int16_t temp_scaling(float raw_value);
int16_t get_temp();
/* miscellanous */
void setup_gpios();
void flash_LED(uint8_t count, uint16_t ms);
void delay_ms(uint16_t ms);
void setup_TMR3();
void reset_TMR3();
void parse_GPMRC();
void setup_TMR4A_pwm();



 //&&&&&&&&&&&&&&&&& GLOABLS &&&&&&&&&&&&&&&&&&&&&
 //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
int8_t buffer[mirf_PAYLOAD] = {0,0,0};
int8_t tx_address[5] = {0xD7,0xD7,0xD7,0xD7,0xD7};
int8_t rx_address[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
int16_t mtr_cmd = 0;
int8_t srv_cmd = 0;
int8_t temperature = 23;
uint8_t comm_lost = 0;
uint8_t comm_lost_count = 0;
volatile uint8_t k_RX = 0;
volatile uint8_t HEADER = 0;
volatile uint8_t GPRMC_SENTENCE = 0;
volatile uint8_t lat_buf[8];
volatile uint8_t lon_buf[8];
uint8_t lat_deg = 0;
uint8_t lat_min = 0;
uint8_t lat_sec = 0;

uint8_t lon_deg = 0;
uint8_t lon_min = 0;
uint8_t lon_sec = 0;





int main(void)
{
	setup_gpios(); 
	setup_usart0(BR_1000000); // for FTDI debugging (terminal)
	setup_usart1(BR_9600); // for NEO6 GPS
	spi1_master_initialize(); // setup device as master for SPI com with nRF24L01
	mirf_init(); // initialize nRF24L01
	mirf_config(); // configure nRF24L01
	setup_adc();
	setup_TMR1_pwm(); // setup TMR1 PWM for DC motor
	setup_TMR4A_pwm();
	setup_TMR0_pwm(); // setup TMR0 PWM for servo
	setup_TMR3();
		
	flash_LED(10, 50); // flash LED 10 times at intervals of 50ms
	_delay_ms(1000);
	sei(); // enable global interrupts
	
 	mirf_set_TADDR(tx_address);
 	mirf_set_RADDR(rx_address);

	println_0("nRF24L01 initialized...;");
	_delay_ms(10);

    while (1) 
    {
		
		TOGGLE_LED;
		
		if (comm_lost_count > 50)
		{
			comm_lost_count = 0;
			mirf_config();
		}
		
		reset_TMR3();
		while(!mirf_data_ready())  // wait to receive command from controller
		{
			if (TCNT3 > 1500) // timeout of one second
			{
				comm_lost_count++;
				comm_lost = 1;
				break;
			}
		}
		if (comm_lost == 0)
		{
			mirf_get_data(buffer); // get the data, put it in buffer
		
			if (buffer[0] == GET_LAT) // if the command is temperature request
			{
				buffer[0] = lat_deg;
				buffer[1] = lat_min;
				buffer[2] = lat_sec;
				reset_TMR3();
				mirf_send(buffer, mirf_PAYLOAD);
				while (!mirf_data_sent())
				{
					if (TCNT3 > 1500) // timeout of one second
					{
						comm_lost_count++;
						comm_lost = 1;
						break;
					}
				}
				
				set_RX_MODE();
				
			}
			else if (buffer[0] == GET_LON) // if the command is temperature request
			{
				buffer[0] = lon_deg;
				buffer[1] = lon_min;
				buffer[2] = lon_sec;
				reset_TMR3();
				mirf_send(buffer, mirf_PAYLOAD);
				while (!mirf_data_sent())
				{
					if (TCNT3 > 1500) // timeout of one second
					{
						comm_lost_count++;
						comm_lost = 1;
						break;
					}
				}
				
				
				lat_deg = 0;
				lat_min = 0;
				lat_sec = 0;
				lon_deg = 0;
				lon_min = 0;
				lon_sec = 0;
				
				
				set_RX_MODE();

				
			}
			else // otherwise, the command is for motor control
			{
				mtr_cmd = ((0xFF00)&(buffer[0]<<8)) | ((0x00FF)&(buffer[1])); // get the motor duty cycle
				srv_cmd = buffer[2];

				if (mtr_cmd > 0 ) // forward direction
				{
					set_TMR1A_duty_cycle(mtr_cmd);
					move_motor_forward();
				}
				else if (mtr_cmd < 0)  // backward direction
				{
					set_TMR4A_duty_cycle(abs(mtr_cmd));
					move_motor_backward();
				}
				else if (abs(mtr_cmd) < 100) // deadband (mtr_cmd is from -1000 to 1000)
				{
					set_TMR1A_duty_cycle(1);
					set_TMR4A_duty_cycle(1);
					motor_off();
				}
		
				move_servo((float)srv_cmd);
			}
					
// 			print_int_0(mtr_cmd);
// 			print_char_0(',');
// 			println_int_0(srv_cmd);
		}
		else
		comm_lost = 0;
		cli();
		parse_GPMRC();
		sei();

		_delay_ms(LOOP_DELAY);

    }
}



//&&&&&&&&&&&&&&&&&&& DC MOTOR &&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void setup_TMR1_pwm()
{
	TCCR1A |= (1 << WGM10) | (1 << COM1A1); // fast PWM
	TCCR1B |= (1 << WGM12) | (1 << CS10); // no prescaler with f_osc (so 62.5KHz PWM)
}
void set_TMR1A_duty_cycle(int duty_cycle)
{
	duty_cycle = .256 * duty_cycle - 1;
	if (duty_cycle > 255)
	duty_cycle = 255;
	OCR1A = (char)((0x00FF) & duty_cycle);
}
void setup_TMR4A_pwm()
{
	TCCR4A |= (1 << WGM40) | (1 << COM4A1); // fast PWM
	TCCR4B |= (1 << WGM42) | (1 << CS40); // no prescaler with f_osc (so 62.5KHz PWM)
}
void set_TMR4A_duty_cycle(int duty_cycle)
{
	duty_cycle = .256 * duty_cycle - 1;
	if (duty_cycle > 255)
	duty_cycle = 255;
	OCR4A = (char)((0x00FF) & duty_cycle);
}
void move_motor_forward()
{
 	EN1_PORT |= (1<<EN1);
 	EN2_PORT &= ~(1<<EN2);
}
void move_motor_backward()
{
	EN1_PORT &= ~(1<<EN1);
	EN2_PORT |= (1<<EN2);
}
void motor_off()
{
	EN1_PORT |= (1<<EN1);
	EN2_PORT |= (1<<EN2);
}
void setup_TMR0_pwm()
{
	TCCR0A |= (1 << COM0A1) | (1 << WGM01) | (1 << WGM00); // fast PWM, Clear OC3A/OC3B on Compare Match, set OC3A/OC3B at BOTTOM (non-inverting mode)
	TCCR0B |=  (1 << CS02); // prescaler of 256 with f_osc (so 62.5KHz PWM)
	move_servo(45);
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
}
void move_servo(float angle)
{ 
	angle = 47 + angle*.355;
	OCR0A = (uint8_t)angle;
}



//&&&&&&&&&&&&&&&&&& CURRENT SENSE &&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
int current_scaling(float raw_value)
{
		float  value = 0;
		value = raw_value / 204.6; // get voltage
 		//value -= 2.5; // get rid of offset
 		//value *= 1000; // scale to current
		
		return value;
}
int get_current()
{
	int current = 0;
	current = analog_get_average(CURRENT_PIN,5);
	current = current_scaling(current);
	
	return current;
}



//&&&&&&&&&&&&&&&&&& TEMPERATURE SENSE &&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
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
int get_temp()
{
	int temp = 0;
	temp = analog_get_average(TEMP_PIN,5);
	temp = temp_scaling(temp);
	return temp;
}


// TIMER3
void setup_TMR3()
{
	TCCR3B |= (1<<CS32); // 256 prescaler, CTC mode

}
void reset_TMR3()
{
	TCNT3 = 0;
}


//&&&&&&&&&&&&&&&&&&&&&& MISC &&&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void setup_gpios()
{
	LED_DDR |= (1<<LED_PIN); // set LED gpio as output
	IN1_DDR |= (1<<IN1);
	IN2_DDR |= (1<<IN2);
	EN1_DDR |= (1<<EN1);
	EN2_DDR |= (1<<EN2);
	//SERVO_PWM_DDR |= (1<<SERVO_PWM);
	
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

void parse_GPMRC()
{
	uint8_t temp_buf[2];
	
	temp_buf[0] = lat_buf[0];
	temp_buf[1] = lat_buf[1];
	lat_deg = atoi(temp_buf);
	
	temp_buf[0] = lat_buf[2];
	temp_buf[1] = lat_buf[3];
	lat_min = atoi(temp_buf);
	
	temp_buf[0] = lat_buf[5];
	temp_buf[1] = lat_buf[6];
	lat_sec = atoi(temp_buf);
	
	temp_buf[0] = lon_buf[0];
	temp_buf[1] = lon_buf[1];
	lon_deg = atoi(temp_buf);
	
	temp_buf[0] = lon_buf[2];
	temp_buf[1] = lon_buf[3];
	lon_min = atoi(temp_buf);
	
	temp_buf[0] = lon_buf[5];
	temp_buf[1] = lon_buf[6];
	lon_sec = atoi(temp_buf);
	
}

// $GPRMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,ddmmyy,x.x,a*hh
ISR(USART1_RX_vect)
{
	rcv_string[k_RX] = UDR1;
	
	if (rcv_string[k_RX] == '$')
	HEADER = 1;
	
	if (HEADER)
	{
		if (rcv_string[k_RX] == ',')
		{
			if ( (rcv_string[3] == 'R') && (rcv_string[4] == 'M') && (rcv_string[5] == 'C'))
			{
				GPRMC_SENTENCE = 1;
			}
			else
			k_RX = 0;
			HEADER = 0;
		}
		else
		k_RX++;
	}
	
	if (GPRMC_SENTENCE)
	{
		if (rcv_string[k_RX] == '*')
		{
			int j = 0;
			for (int i = 19; i < 26; i++)
			{
				lat_buf[j] = rcv_string[i];
				j++;
			}
			lat_buf[j] = NL;
			
			
			j = 0;
			for (int i = 33; i <40; i++)
			{
				lon_buf[j] = rcv_string[i];
				j++;
			}
			lon_buf[j] = NL;
			
			GPRMC_SENTENCE = 0;
			k_RX = 0;
		}
		else
		k_RX++;
	}
}

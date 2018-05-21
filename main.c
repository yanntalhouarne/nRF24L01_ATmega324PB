 
#define F_CPU 8000000
#define LOOP_DELAY 20

//#define DIRECT_JOYSTICK
#define GPS_ON

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>



#ifdef GPS_ON
#include "usart.h"
#include "print.h"
#endif

#ifndef DIRECT_JOYSTICK
#include "spi.h"
#include "mirf.h"
#endif

#include "adc.h"


 //&&&&&&&&&&&&&&&&& MACROS &&&&&&&&&&&&&&&&&&&&&&
 //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
 
 /* joystick reading and scaling */
 #define CMD_SCALE 1
 #define NEUTRAL_CMD 500
 #define DEADBAND_MIN 460
 #define DEADBAND_MAX 540
 #define OFFSET 0
 
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
#define IN2_DDR DDRD
#define IN2_PORT PORTD
#define IN2_PIN PIND
/* ENA */
#define EN1_PORT PORTB
#define EN1_DDR DDRB
#define EN1 1
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
void stop_TMR1_pwm();
void start_TMR1A_pwm();
void start_TMR1B_pwm();
void stop_TMR1A_pwm();
void stop_TMR1B_pwm();
void motor_off();
void motor_on();
void set_TMR1A_duty_cycle(int duty_cycle);
void set_TMR1B_duty_cycle(int duty_cycle);
/* Communication time-out */
void setup_TMR3();
void reset_TMR3();
#ifdef GPS_ON
/* GPS parsing */
void parse_GPMRC();
#endif
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
#ifdef DIRECT_JOYSTICK
	/* direct joystick input */
	int js_mtr_scaling(int value);
	int js_srv_scaling(float value);
#endif



 //&&&&&&&&&&&&&&&&& GLOABLS &&&&&&&&&&&&&&&&&&&&&
 //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
 /* nRF24L01 variables */
int8_t buffer[mirf_PAYLOAD] = {0,0,0}; // for the nRF24L01 receive and trasnmit data
int8_t tx_address[5] = {0xD7,0xD7,0xD7,0xD7,0xD7};
int8_t rx_address[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
/* motor command variables */
int16_t mtr_cmd = 0; 
int16_t  old_mtr_cmd = 0;
/* servo command variables */
int16_t srv_cmd = 0;
int8_t old_srv_cmd = 0;
/* temperature */
int8_t temperature = 23;
/* communication status */
uint8_t comm_lost = 0;
uint8_t comm_lost_count = 0;
#ifdef GPS_ON
/* GPS variables */
volatile uint8_t k_RX = 0;
volatile uint8_t HEADER = 0;
volatile uint8_t GPRMC_SENTENCE = 0;
volatile uint8_t lat_buf[8];
volatile uint8_t lon_buf[8];
volatile uint8_t gps_string_ready = 0; // flag set when RX1 interrupt has received a full GPS sentence
uint8_t lat_deg = 0;
uint8_t lat_min = 0;
uint8_t lat_sec = 0;
uint8_t lon_deg = 0;
uint8_t lon_min = 0;
uint8_t lon_sec = 0;
#endif






int main(void)
{
	setup_gpios(); 
	#ifdef GPS_ON
	/* USART setup */
	setup_usart0(BR_9600); // for NEO6 GPS
	#endif
	
	/* nRF24L01 setup */
	#ifndef DIRECT_JOYSTICK
	spi1_master_initialize(); // setup device as master for SPI com with nRF24L01
	mirf_init(); // initialize nRF24L01
	mirf_config(); // configure nRF24L01
	mirf_set_TADDR(tx_address);
	mirf_set_RADDR(rx_address);
	#endif
	
	/* ADC for current and temperature sensor (and joystick i ndirect joystick mode) */
	setup_adc();
	
	/* Timers setup */
	setup_TMR1_pwm(); // setup TMR1 PWM for DC motor
	setup_TMR0_pwm(); // setup TMR0 PWM for servo
	setup_TMR3(); // for communication timeout with controller
	
	sei(); // enable global interrupts
	
	/* setup complete notification */
	flash_LED(10, 50); // flash LED 10 times at intervals of 50ms
	_delay_ms(1000);
	

	_delay_ms(10);

    while (1) 
    {
		
		TOGGLE_LED;
		#ifndef DIRECT_JOYSTICK
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
		#endif
		
		if (comm_lost == 0) // if data has been received (will be 0 by default so in direct joystick mode, we will always enter this if statement
		{
			#ifndef DIRECT_JOYSTICK
			mirf_get_data(buffer); // get the data, put it in buffer
		
			if (buffer[0] == GET_LAT) // if the command is latitude request
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
				
				set_RX_MODE(); // listen for new data
				
			}
			else if (buffer[0] == GET_LON) // if the command is longitude request
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
				
				set_RX_MODE(); // listen for new data
				
			}
			else // otherwise, the command is for motor control (default command)
			{
				mtr_cmd = ((0xFF00)&(buffer[0]<<8)) | ((0x00FF)&(buffer[1])); // get the motor duty cycle
				srv_cmd = buffer[2];
			#endif		
						
			#ifdef DIRECT_JOYSTICK
			mtr_cmd = analog_read(1);
			mtr_cmd = js_mtr_scaling(mtr_cmd);
			mtr_cmd = 0.25*mtr_cmd + .75*old_mtr_cmd;
			old_mtr_cmd = mtr_cmd;
			
			srv_cmd = analog_read(2);
			srv_cmd = js_srv_scaling(srv_cmd);
 			if ((srv_cmd < 5) && (srv_cmd > -5))
 			srv_cmd = 0;
			#endif
			
			/* scaling and deadband */
			 if (srv_cmd > 20)
				srv_cmd = 20;
			else if (srv_cmd < -20)
				srv_cmd = -20;
			srv_cmd = 0.75*srv_cmd + .25*old_srv_cmd;
			old_srv_cmd = srv_cmd;
			
			if (abs(mtr_cmd) < 100) // deadband (mtr_cmd is from -1000 to 1000)
			{
				stop_TMR1A_pwm();
				stop_TMR1B_pwm();
				motor_off();
			}
			else  
			{
				motor_on();
				if (mtr_cmd > 0 ) // forward direction
				{
					stop_TMR1B_pwm();
					start_TMR1A_pwm();
					set_TMR1A_duty_cycle(mtr_cmd);
				}
				else if (mtr_cmd < 0)  // backward direction
				{
					stop_TMR1A_pwm();
					start_TMR1B_pwm();
					set_TMR1B_duty_cycle(abs(mtr_cmd));
				}
			}
			
				move_servo((float)srv_cmd);
			#ifndef DIRECT_JOYSTICK
			}
			#endif
 		}
		#ifndef DIRECT_JOYSTICK
 		else
 			comm_lost = 0;
		#endif
		
		#ifdef GPS_ON	 
		if (gps_string_ready) // if a full gps sentence has been received, parse it
		{
			gps_string_ready = 0;
 			cli();
 				parse_GPMRC(); // parse GPS string received by UASRT1 RX interrupt
			sei();
		}
		#endif
		_delay_ms(LOOP_DELAY);

    }
}



//&&&&&&&&&&&&&&&&&&& DC MOTOR &&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void setup_TMR1_pwm()
{
	TCCR1A |= (1 << WGM10) ; // fast PWM
	TCCR1B |= (1 << WGM12) | (1<<CS10); // no prescaler with f_osc (so 62.5KHz PWM)
}
void set_TMR1A_duty_cycle(int duty_cycle)
{
	duty_cycle = .256 * duty_cycle - 1;
	if (duty_cycle > 200)
	duty_cycle = 200;
	OCR1A = (char)((0x00FF) & duty_cycle);
}
void set_TMR1B_duty_cycle(int duty_cycle)
{
	duty_cycle = .256 * duty_cycle - 1;
	if (duty_cycle > 200)
	duty_cycle = 200;
	OCR1B = (char)((0x00FF) & duty_cycle);
}
void stop_TMR1A_pwm()
{
	TCCR1A &= ~(1 << COM1A1); // no prescaler with f_osc (so 62.5KHz PWM)
}
void stop_TMR1B_pwm()
{
	TCCR1A &= ~(1 << COM1B1); // no prescaler with f_osc (so 62.5KHz PWM)
}
void start_TMR1A_pwm()
{
	TCCR1A |= (1 << COM1A1); // no prescaler with f_osc (so 62.5KHz PWM)
}
void start_TMR1B_pwm()
{
	TCCR1A |= (1 << COM1B1); // no prescaler with f_osc (so 62.5KHz PWM)
}
void motor_off()
{
	EN1_PORT &= ~(1<<EN1);
	EN2_PORT &= ~(1<<EN2);
}
void motor_on()
{
	EN1_PORT |= (1<<EN1);
	EN2_PORT |= (1<<EN2);
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

//&&&&&&&&&&&&&&&&&&& SERVO MOTOR &&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void setup_TMR0_pwm()
{ 
	TCCR0A |= (1 << COM0A1) | (1 << WGM01) | (1 << WGM00); // fast PWM, Clear OC3A/OC3B on Compare Match, set OC3A/OC3B at BOTTOM (non-inverting mode)
	TCCR0B |=  (1 << CS02); // prescaler of 1024
	move_servo(45);

}
void move_servo(float angle)
{ 
	angle = 46 + angle*.355;
	OCR0A = (uint8_t)angle;
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&


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
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&


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
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

//&&&&&&&&&&&&&&&&&& TIMER3 TIMOUT &&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void setup_TMR3()
{
	TCCR3B |= (1<<CS32); // 256 prescaler, CTC mode

}
void reset_TMR3()
{
	TCNT3 = 0;
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

//&&&&&&&&&&&&&&&&&&&&&& MISC &&&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void setup_gpios()
{
	LED_DDR |= (1<<LED_PIN); // set LED gpio as output
	IN1_DDR |= (1<<IN1);
	IN2_DDR |= (1<<IN2);
	EN1_DDR |= (1<<EN1);
	EN2_DDR |= (1<<EN2);
	SERVO_PWM_DDR |= (1<<SERVO_PWM);
	
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

//&&&&&&&&&&&&&&&&&&&&&&& GPS &&&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void parse_GPMRC()
{
	uint8_t temp_buf[2];
	
	temp_buf[0] = lat_buf[0];
	temp_buf[1] = lat_buf[1];
	lat_deg = atoi((const char *)temp_buf);
	
	temp_buf[0] = lat_buf[2];
	temp_buf[1] = lat_buf[3];
	lat_min = atoi((const char *)temp_buf);
	
	temp_buf[0] = lat_buf[5];
	temp_buf[1] = lat_buf[6];
	lat_sec = atoi((const char *)temp_buf);
	
	temp_buf[0] = lon_buf[0];
	temp_buf[1] = lon_buf[1];
	lon_deg = atoi((const char *)temp_buf);
	
	temp_buf[0] = lon_buf[2];
	temp_buf[1] = lon_buf[3];
	lon_min = atoi((const char *)temp_buf);
	
	temp_buf[0] = lon_buf[5];
	temp_buf[1] = lon_buf[6];
	lon_sec = atoi((const char *)temp_buf);
	
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

#ifdef DIRECT_JOYSTICK
//&&&&&&&&&&&&&&&&&&& JOYSTICK &&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
int js_mtr_scaling(int value) // scales the result to commands from -1000 to 1000.
{
	value = value * CMD_SCALE - OFFSET; // scale to 0 -> 1000

	if ((value < DEADBAND_MAX) && (value > DEADBAND_MIN)) // if within the dead band, send neutral command (0)
	value = 0;
	else if (value > DEADBAND_MAX)            // if joystick is higher than deadband
	value = 2 * (value - NEUTRAL_CMD);    // compute the change from the neutral position multiply by 2 to scale to 1000
	else if (value < DEADBAND_MIN)            // if joystick is lower than deadband
	value = (-2) * (NEUTRAL_CMD - value); // compute the change from the neutral position and invert (multiply by 2 to scale to -1000
	if (value > 1000)                         // do not send any value bigger than 1000 or smaller than -1000
	value = 1000;

	return value;
} // end of joystick_scaling

int js_srv_scaling(float value) // scales the result to commands from -1000 to 1000.
{
	value = value * CMD_SCALE - OFFSET; // scale to 0 -> 1000
	if ((value < DEADBAND_MAX) && (value > DEADBAND_MIN)) // if within the dead band, send neutral command (0)
	value = 0;
	else if (value > DEADBAND_MAX)            // if joystick is higher than deadband
	{
		value = 2 * (value - NEUTRAL_CMD);    // compute the change from the neutral position multiply by 2 to scale to 1000
		value = value  / 22.2; // scale to -45 45
	}
	else if (value < DEADBAND_MIN)            // if joystick is lower than deadband
	{
		value = (-2) * (NEUTRAL_CMD - value); // compute the change from the neutral position and invert (multiply by 2 to scale to -1000
		value = value  / 22.2; // scale to -45 45
	}
	if (value > 1000)                         // do not send any value bigger than 1000
	value = 1000;
	if (value < -1000)                        // do not send any value smaller than -1000
	value = -1000;

	return value;
} // end of joystick_scaling
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
#endif

#ifdef GPS_ON
//&&&&&&&&&&&&&&&&&&&&&&& USART1 ISR for GPS &&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
// $GPRMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,ddmmyy,x.x,a*hh
ISR(USART0_RX_vect)
{
	rcv_string[k_RX] = UDR0;
	
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
			gps_string_ready = 1;
		}
		else
		k_RX++;
	}
}
#endif
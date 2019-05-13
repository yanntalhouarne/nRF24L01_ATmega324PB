 
#define F_CPU 16000000

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <math.h>

#include "usart.h"
#include "adc.h"
#include "print.h"
#include "spi.h"
#include "nRF.h" 





 //&&&&&&&&&&&&&&&&& MACROS &&&&&&&&&&&&&&&&&&&&&&
 //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
 
 /*&&& Main loop delay ****/
		#define LOOP_DELAY 10
		
 
/*&&& LEDs &&&*/
	/* Blue LED on main board */
		#define LED_DDR    DDRC
		#define LED_PORT   PORTC
		#define LED_PIN    0
		#define LED_ON     LED_PORT  |= (1<<LED_PIN);
		#define LED_OFF    LED_PORT &= ~(1<<LED_PIN);
		#define TOGGLE_LED LED_PORT  ^= (1<<LED_PIN);
	/* Blue LED on top board */
		#define BLUE_LED_DDR    DDRC
		#define BLUE_LED_PORT 	PORTC
		#define BLUE_LED_PIN 		3
		#define BLUE_LED_ON 		BLUE_LED_PORT  |= (1<<BLUE_LED_PIN);
		#define BLUE_LED_OFF 		BLUE_LED_PORT &= ~(1<<BLUE_LED_PIN);
		#define TOGGLE_BLUE_LED BLUE_LED_PORT  ^= (1<<BLUE_LED_PIN);
	/* Red LED on top board */
		#define RED_LED_DDR    DDRC
		#define RED_LED_PORT   PORTC
		#define RED_LED_PIN    1
		#define RED_LED_ON     RED_LED_PORT  |= (1<<RED_LED_PIN);
		#define RED_LED_OFF    RED_LED_PORT &= ~(1<<RED_LED_PIN);
		#define TOGGLE_RED_LED RED_LED_PORT  ^= (1<<RED_LED_PIN);
	/* Orange LED on top board */
		#define ORANGE_LED_DDR    DDRC
		#define ORANGE_LED_PORT   PORTC
		#define ORANGE_LED_PIN    5
		#define ORANGE_LED_ON     ORANGE_LED_PORT  |= (1<<ORANGE_LED_PIN);
		#define ORANGE_LED_OFF    ORANGE_LED_PORT &= ~(1<<ORANGE_LED_PIN);
		#define TOGGLE_ORANGE_LED ORANGE_LED_PORT  ^= (1<<ORANGE_LED_PIN);

/*&&& DC motor &&&*/
	/* IN1 */
		#define IN1 		 5
		#define IN1_DDR  DDRD
		#define IN1_PORT PORTD
		#define IN1_PIN  5
	/* IN2 */
		#define IN2			 4
		#define IN2_DDR  DDRD
		#define IN2_PORT PORTD
		#define IN2_PIN  PIND
	/* ENA */
		#define EN1_PORT PORTB
		#define EN1_DDR  DDRB
		#define EN1 	   1
	/* ENB */
		#define EN2_PORT PORTB
		#define EN2_DDR  DDRB
		#define EN2      2
	/* RATE LIMIT */
		#define RATE_LIMIT_THRESHOLD 25
		#define RATE_LIMIT 10
		
/*&&& Motor command macros &&&*/
		#define MTR_CMD 				  0
		#define ANALOG 					  1

/*&&& ADC macros &&&*/
		#define VREF 						 	 5.08
		#define CURRENT_SENSOR     0
		#define VOLTAGE_SENSOR     1
		#define TEMP_SENSOR        2
		#define SERIESRESISTOR     10000 
		#define THERMISTORNOMINAL  10000
		#define TEMPERATURENOMINAL 25
		#define BCOEFFICIENT 		   4250
		
/*&&& Servo macros &&&*/
		#define SERVO_PWM_DDR DDRB
		#define SERVO_PWM_PIN 3  // PB3
		
/*&&& nRF24L01 macros &&&*/
		#define ACK_PAYLOAD_LENGTH 6
		#define TX_PAYLOAD_LENGTH 3



 //&&&&&&&&&&&&& Function protorypes &&&&&&&&&&&&&&
 //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
 
/*&&& DC motor functions &&&*/
		void setup_TMR1_pwm();
		void start_TMR1A_pwm();
		void start_TMR1B_pwm();
		void stop_TMR1A_pwm();
		void stop_TMR1B_pwm();
		void motor_off();
		void motor_on();
		void set_TMR1A_duty_cycle(int duty_cycle);
		void set_TMR1B_duty_cycle(int duty_cycle);
		void process_mtr_cmd();
		
/*&&& Servo functions &&&*/		
		void setup_TMR0_pwm();
		void move_servo(float angle);
		
/*&&& TMR3 for communication timeout &&&*/
		void setup_TMR3();
		void start_TMR3();
		void stop_TMR3();
	
/*&&& Analog reading functions &&&*/
		float get_voltage();
		float get_current();
		float get_temperature();

	/*&&& Misc &&&*/
		 void setup_gpios();
		 void flash_LED(uint8_t count, uint16_t ms);
		 void delay_ms(uint16_t ms);

 //&&&&&&&&&&&&&&&&& GLOBALS &&&&&&&&&&&&&&&&&&&&&
 //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
 
/*&&& nRF24L01 data buffers &&&*/
	char buffer[TX_PAYLOAD_LENGTH] = {0,0,0}; // for the nRF24L01 receive and transmit data
	char buffer_analog[ACK_PAYLOAD_LENGTH] = {0,0,0,0,0,0};
	char rx_address[5] = {0xD7,0xD7,0xD7,0xD7,0xD7};
	//char tx_address[5] = {0xAA,0xAA,0xAA,0xAA,0xAA};	
	
/*&&& motor command &&&*/
	volatile int16_t mtr_cmd = 0;	
	volatile int16_t old_mtr_cmd = 0;
	volatile uint16_t unsigned_motor_cmd = 1000;
	float alpha = 0.7;
	
/*&&& servo command &&&*/	
	int8_t srv_cmd = 0;
	
/*&&& communication status &&&*/
	volatile uint16_t max_rt_count = 0;
	volatile uint16_t transmission_count = 0;
	volatile float signal_strength = 0;

/*&&& TMR1 flag &&&*/
	volatile uint8_t TMR1_flag = 0;
	
/*&&& communication timeout variables &&&*/
	volatile uint8_t timeout = 0;
	volatile int8_t false_cmd = 0;
	uint8_t nbr_rcv_cmd = 0;
	
/*&&& ADC variables &&&*/
	volatile int16_t current = 0;
	volatile int16_t old_current = 0;
	volatile int16_t voltage = 0;
	volatile int16_t old_voltage = 0;
	volatile int16_t temperature = 0;
	volatile int16_t old_temperature = 0;
	

	
	
	
 //&&&&&&&&&&&&&&&&& MAIN &&&&&&&&&&&&&&&&&&&&&&&&
 //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
int main(void)
{
	/* setup pins */
	setup_gpios(); 
	
	/* USART setup */
	USART0_setup(BR_38400); // for NEO6 GPS
	
	/* nRF24L01 setup */
	SPI1_masterInit();  // setup device as master for SPI com with nRF24L01
	nRF_init(); 							 // initialize nRF24L01
	nRF_config(); 						 // configure nRF24L01
	nRF_RXsetAddr(rx_address);
	//nRF_TXsetAddr(tx_address);
	
	/* setup TMR3 for communication timeout */
	setup_TMR3();
	stop_TMR3();

	/* ADC for current and temperature sensor (and joystick indirect joystick mode) */
	setup_adc();

	/* DC motor setup */
	setup_TMR1_pwm(); // setup TMR1 PWM for DC motor
	motor_off();
	
	/* Servo motor setup */
	setup_TMR0_pwm();
	
	sei(); // enable global interrupts
	
	/* setup complete notification */
	flash_LED(10, 50); // flash LED 10 times at intervals of 50ms
	println_0("System initialized...;");
	_delay_ms(10);
	
	
    while (1) // main loop
    {	
		/* enter RX mode */
		nRF_SetRxMode();
				
		TOGGLE_LED; // main loop LED
		ORANGE_LED_OFF; // unused 
		
		/* read battery voltage */
		voltage = (int16_t)(100*get_voltage()); // in V
		voltage = .3 * voltage + .7 * old_voltage;
		old_voltage = voltage;
		
		/* read current */	
		current = (int16_t)(100*get_current()); // in A
		current = .1 * current + .9 * old_current;
		old_current = current;
		
		/* read temperature */					
		temperature = (int16_t)(10*get_temperature());
		temperature = .5 * temperature + .5 * old_temperature;
		old_temperature = temperature;
		
		/* load analog data into buffer */
		buffer_analog[0] = current; // LSB
		buffer_analog[1] = (current>>8);	  // MSB
		buffer_analog[2] = voltage; // LSB
		buffer_analog[3] = (voltage>>8);	  // MSB
		buffer_analog[4] = temperature;
		buffer_analog[5] = (temperature>>8);
		
		/* write analog data to ACK payload */
		nRF24_uploadACKpay(ACK_PAYLOAD_P0, buffer_analog, ACK_PAYLOAD_LENGTH);
				
		/* wait for incoming data with a timeout of 1s */
// 		timeout = 0;	
// 		start_TMR3();
		while(!nRF_dataReady()) // RX timeout
		{
			if (timeout)
			{
				timeout = 0;
				false_cmd = 1;
				break;
			}
		} // end of RX timeout
		//stop_TMR3();
		
		/* enter Stanby_I mode to save current consumption */	
		nRF_CLEAR_CE; 
		
		/* read RX data */
		nRF_getData(buffer, TX_PAYLOAD_LENGTH); // get the data, put it in buffer	
		
		if (!false_cmd)
		{
			TOGGLE_BLUE_LED;
			
			// get unsigned 8-bit mtr cmd
			mtr_cmd =  buffer[1]; // get LSB
			
			// get signed 8-bit srv cmd
			srv_cmd = buffer[2];
			
			process_mtr_cmd();
			
			move_servo((float)srv_cmd);
		} // end of: if (!false_cmd)
		else
		{
			TOGGLE_RED_LED;
			mtr_cmd = 0;
			process_mtr_cmd();
			false_cmd = 0;
			_delay_ms(1);
		}
		
// 		/* clear RX buffer */
// 		for (int i = 0; i < TX_PAYLOAD_LENGTH; i++)
// 		{
// 			buffer[i] = 0;
// 		}
		
    } // end of: while(1) main loop
}




//&&&&&&&&&&&&&&&&&&&&&& FUNCTIONS &&&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void setup_gpios()
{
	// set LED pins as output
	LED_DDR |= (1<<LED_PIN);
	RED_LED_DDR |= (1<<RED_LED_PIN); 
	BLUE_LED_DDR |= (1<<BLUE_LED_PIN);
	ORANGE_LED_DDR |= (1<<ORANGE_LED_PIN);
	
	// turn off LEDs initially
	LED_OFF;	 // misc. LED, toggles every time we go through the main loop
	RED_LED_OFF; // ON when RX timeout (false_cmd = 1) or TX  timeout (max_rt_count = 1), \
	turned OFF at every start of the main loop
	BLUE_LED_OFF; // toggles everytime we successfully receive data (false_cmd == 0)
	ORANGE_LED_OFF; // ON when TX timeout occurs (max_RT = 1)
	
	IN1_DDR |= (1<<IN1);
	IN2_DDR |= (1<<IN2);
	EN1_DDR |= (1<<EN1);
	EN2_DDR |= (1<<EN2);
	
	SERVO_PWM_DDR |= (1<<SERVO_PWM_PIN);	
	
	move_servo(0);
	
	motor_off();
	
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





//&&&&&&&&&&&&&&&&&&& DC MOTOR &&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void setup_TMR1_pwm()
{
	TCCR1A |= (1 << WGM10) ; // fast PWM
	TCCR1B |= (1 << WGM12) | (1<<CS11); //  prescaler of 8 with f_osc (so 7.82KHz PWM)
}
void set_TMR1A_duty_cycle(int duty_cycle)
{
	//duty_cycle = .256 * duty_cycle - 1;
	// duty cycle from 0 to 255
	if (duty_cycle > 175)
		duty_cycle = 175;
/*	println_int_0(duty_cycle);*/
	OCR1A = (char)((0x00FF) & duty_cycle);
}
void set_TMR1B_duty_cycle(int duty_cycle)
{
	//duty_cycle = .256 * duty_cycle - 1;
	if (duty_cycle > 175)
	duty_cycle = 175;
		//println_int_0(duty_cycle);
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
void process_mtr_cmd()
{
	// low-pass filter
	 mtr_cmd = mtr_cmd*alpha + old_mtr_cmd*(1-alpha);
	 
	// dead band
	if (abs(mtr_cmd) < 1) // deadband (mtr_cmd is from -255 to 255)
	{
		mtr_cmd = 0;
		stop_TMR1A_pwm();
		stop_TMR1B_pwm();
		motor_off();
	}
	else
	{
		motor_on();
	}
	
	if (mtr_cmd > 0 ) // forward direction
	{
		// rate limit
		if (abs(mtr_cmd - old_mtr_cmd) > RATE_LIMIT)
		{
			if (mtr_cmd > old_mtr_cmd)
			{
				// value is rising away from old value
				mtr_cmd = old_mtr_cmd + RATE_LIMIT;
			}
			else
			{
				// value is falling away from old value
				mtr_cmd = old_mtr_cmd - RATE_LIMIT;
			}
		} // if the rate limit is violated
				
		old_mtr_cmd = mtr_cmd;
			
		
			
		stop_TMR1B_pwm();
		start_TMR1A_pwm();
		set_TMR1A_duty_cycle(mtr_cmd);
	}
	else if (mtr_cmd < 0)  // backward direction
	{
		stop_TMR1A_pwm();
		start_TMR1B_pwm();
			
		mtr_cmd *= -1;
		// rate limit
		if (abs(mtr_cmd - old_mtr_cmd) > RATE_LIMIT)
		{
			if (mtr_cmd > old_mtr_cmd)
			{
				// value is rising away from old value
				mtr_cmd = old_mtr_cmd + RATE_LIMIT;
			}
			else
			{
				// value is falling away from old value
				mtr_cmd = old_mtr_cmd - RATE_LIMIT;
			}
		} // if the rate limit is violated
			
		old_mtr_cmd = mtr_cmd;
		set_TMR1B_duty_cycle(abs(mtr_cmd));
	}
	
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

//&&&&&&&&&&&&&& TMR3 for communication timeout &&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void setup_TMR3()
{
	TCCR3A |= (1<<COM3A1) | (1<<COM3A0); // clear OCRA on compare
	TCCR3B |= (1<<CS32) | (1<<CS30) | (1<<WGM32); // 1024 prescaler, CTC mode
	OCR3A = 15620; // 1s period
	TIMSK3 |= (1<<OCIE3A); // set interrupt on OCA compare
	
	stop_TMR3();
}
void start_TMR3()
{
	TCNT3 = 0;
	TIMSK3 |= (1<<OCIE3A); // set interrupt on OCA compare
}
void stop_TMR3()
{
	TIMSK3 &= ~(1<<OCIE3A);
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

//&&&&&&&&&&&&&& Analog reading functions &&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
float get_voltage()
{
	float _voltage = 0;
	// get analog reading
	_voltage = analog_get_average(VOLTAGE_SENSOR, 1);
	// convert to volts
	_voltage *= VREF;
	_voltage /= 1023; 
	// voltage divider scaling factor
	_voltage *= 112.7;
	_voltage /= 11.6; 
	
	return _voltage;
}
float get_current()
{
	float _current = 0;
	// get analog reading
	_current = analog_get_average(CURRENT_SENSOR, 1);
	// convert to mV
	_current *= VREF*1000;
	_current /=  1023; 
	_current -= 444; // 0A offset
	_current /= 400; // 400mV per 1A
	if (_current < 0)
		_current = 0; // cannot be negative
		
	return _current;	
}
float get_temperature()
{
	float _temperature = 0;
	
	_temperature = analog_get_average(TEMP_SENSOR, 1);
	_temperature = 1023 / _temperature - 1;
	_temperature = SERIESRESISTOR / _temperature;
	_temperature = _temperature / THERMISTORNOMINAL;     // (R/Ro)
	_temperature = log(_temperature);                  // ln(R/Ro)
	_temperature /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
	_temperature += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
	_temperature = 1.0 / _temperature;                 // Invert
	_temperature -= 273.15;                         // convert to C
	
	return _temperature;
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&



//&&&&&&&&&&&&&&&&&&& SERVO MOTOR &&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void setup_TMR0_pwm()
{
	TCCR0A |= (1 << COM0A1) | (1 << WGM01) | (1 << WGM00); // fast PWM
	TCCR0B |=  (1 << CS02) | (1 << CS00); // prescaler of 1024
	move_servo(0);

}
void move_servo(float angle)
{
	if (angle < -42)
	{
		angle = -42;
	}
	else if (angle > 45)
	{
		angle = 45;
	}
	angle = 23 + angle*.0889;
	OCR0A = (uint8_t)angle;
}

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&& ISRs &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

//&&&&&&&&&&&&&&&&&&&&&& TMR3 ISR &&&&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

ISR(TIMER3_COMPA_vect)
{
	timeout = 1;
		
}
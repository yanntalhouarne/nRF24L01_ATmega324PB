 
#define F_CPU 16000000

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <math.h>

#include "usart.h"
#include "adc.h"
#include "print.h"
#include "spi.h"
#include "mirf.h" 





 //&&&&&&&&&&&&&&&&& MACROS &&&&&&&&&&&&&&&&&&&&&&
 //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
 
 /*&&& Main loop delay ****/
		#define LOOP_DELAY 10
		
		#define DATA_CMD_DELAY 10
 
/*&&& LEDs &&&*/
	/* LED 1*/
		#define LED1_DDR DDRE
		#define LED1_PORT PORTE
		#define LED1_PIN 4
		#define LED1_ON LED1_PORT |= (1<<LED1_PIN);
		#define LED1_OFF LED1_PORT &= ~(1<<LED1_PIN);
		#define TOGGLE_LED1 LED1_PORT ^= (1<<LED1_PIN);
	/* LED2 */
		#define LED2_DDR DDRB
		#define LED2_PORT PORTB
		#define LED2_PIN 5
		#define LED2_ON LED2_PORT |= (1<<LED2_PIN);
		#define LED2_OFF LED2_PORT &= ~(1<<LED2_PIN);
		#define TOGGLE_LED2 LED2_PORT ^= (1<<LED2_PIN);

/*&&& DC motor &&&*/
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
	/* RATE LIMIT */
		#define RATE_LIMIT_THRESHOLD 25
		#define RATE_LIMIT 10
		
/*&&& Motor command macros &&&*/
		#define MTR_SIDE_DEADBAND 50 // any mtr_cmd under thsi value will be considered a 0 cmd
		#define MTR_CMD 0
		#define ANALOG 1

/*&&& ADC macros &&&*/
		#define VREF 5.08
		#define CURRENT_SENSOR 0
		#define VOLTAGE_SENSOR 1
		#define TEMP_SENSOR 2
		#define SERIESRESISTOR 10000 
		#define THERMISTORNOMINAL 10000
		#define TEMPERATURENOMINAL 25
		#define BCOEFFICIENT 4250


 //&&&&&&&&&&&&& Function protorypes &&&&&&&&&&&&&&
 //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
 
	/*&&& DC motor variables &&&*/
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
 
 /*&&& nRF24L01 variables &&&*/
	char buffer[mirf_PAYLOAD] = {0,0,0,0,0,0}; // for the nRF24L01 receive and transmit data
	char buffer_analog[mirf_PAYLOAD] = {0,0,0,0,0,0};
	char buffer_cmd[mirf_PAYLOAD] = {0,0,0,0,0,0};
		
	char rx_address[5] = {0xD7,0xD7,0xD7,0xD7,0xD7};
	char tx_address[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};	
	volatile int16_t mtr_cmd = 0;	
	volatile int16_t old_mtr_cmd = 0;
	volatile uint16_t unsigned_motor_cmd = 1000;
	float alpha = 0.1;
	volatile uint16_t max_rt_count = 0;
	volatile uint16_t transmission_count = 0;
	volatile float signal_strength = 0;

 /*&&& nRF24L01 variables &&&*/
	volatile uint8_t TMR1_flag = 0;
	
/*&&& communication timeout variables &&&*/
	volatile uint8_t timeout = 0;
	uint8_t nbr_rcv_cmd = 0;
	
/*&&& ADC variables &&&*/
	int16_t current = 32;
	int16_t voltage = 125;
	int16_t temperature = 194;
	
/* &&& Misc &&&*/
	volatile int8_t temp = 0;
	volatile int8_t false_cmd = 0;
	
	
	
	
	
int main(void)
{
	setup_gpios(); 
	
	/* USART setup */
	setup_usart0(BR_38400); // for NEO6 GPS
	
	/* nRF24L01 setup */
	spi1_master_initialize(); // setup device as master for SPI com with nRF24L01
	mirf_init(); // initialize nRF24L01
	mirf_config(); // configure nRF24L01
	mirf_set_RADDR(rx_address);
	mirf_set_TADDR(tx_address);
	
	/* setup TMR3 for communication tiemout */
	setup_TMR3();
	stop_TMR3();

	/* ADC for current and temperature sensor (and joystick indirect joystick mode) */
	setup_adc();

	/* DC motor setup */
	setup_TMR1_pwm(); // setup TMR1 PWM for DC motor
	
	
	sei(); // enable global interrupts
	
	/* setup complete notification */
	flash_LED(10, 50); // flash LED 10 times at intervals of 50ms
	println_0("System initialized...;");
	_delay_ms(10);
	
	LED1_OFF;
	LED2_OFF;


	set_RX_MODE();
	
    while (1)  
    {
		LED1_OFF;
		
		set_RX_MODE();
		
		timeout = 0;
		
		start_TMR3();
		
		while(!mirf_data_ready())
		{
			if (timeout)
			{
				timeout = 0;
				false_cmd = 1;
				break;
			}
		}
		
		stop_TMR3();
		

		
		mirf_CE_lo; // enter Stanby_I mode to save current consumption	
		
	//	cli();
		mirf_get_data(buffer); // get the data, put it in buffer	
		
		for (int i = 0; i < mirf_PAYLOAD; i++)
			buffer_cmd[i] = buffer[i];
	//	sei();
					
				
		// only if communication is successful
		if (!false_cmd)
		{		
			TOGGLE_LED2;
		//	nbr_rcv_cmd++;
			switch (buffer_cmd[0])
			{			
				case MTR_CMD :
		
					
					// get signed 16-bit mtr cmd
					mtr_cmd =  buffer[1]; // get LSB
					mtr_cmd |= (buffer[2]<<8); // get MSB
					
					//println_int_0((int16_t)mtr_cmd);
						
					// low-pass filter
					// mtr_cmd = mtr_cmd*alpha + old_mtr_cmd*(1-alpha);
					

						
				
					process_mtr_cmd();
					
					break;
					
				case ANALOG :
					_delay_ms(1);
					// data sent as a unsigned 16-bit
					buffer_analog[0] = current; // LSB
					buffer_analog[1] = (current>>8);	  // MSB
					buffer_analog[2] = voltage; // LSB
					buffer_analog[3] = (voltage>>8);	  // MSB
					buffer_analog[4] = temperature;
					buffer_analog[5] = (temperature>>8);
					
					/* MIRF stuff */
					mirf_send(buffer_analog, mirf_PAYLOAD);
					
					max_rt_count = 0;
					
					while (!mirf_data_sent())
					{
						if (mirf_read_MAX_RT())
						{
							max_rt_count = 1;
							break;
						}
					}
					if (max_rt_count)
					{
						max_rt_count = 0;
						LED1_ON;
					}
					
					break;
					
				default :
					break;
			}
	
 		}
		else
		{
			mtr_cmd = 0;
			process_mtr_cmd();
 			false_cmd = 0;
			 LED1_ON;
			 _delay_ms(10);
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
	IN1_DDR |= (1<<IN1);
	IN2_DDR |= (1<<IN2);
	EN1_DDR |= (1<<EN1);
	EN2_DDR |= (1<<EN2);
	
	
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
	if (abs(mtr_cmd) < 1) // deadband (mtr_cmd is from -1000 to 1000)
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
			// rate limit
			if (abs(mtr_cmd - old_mtr_cmd) > RATE_LIMIT_THRESHOLD)
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
			if (mtr_cmd > 75 )
				mtr_cmd = 75;
				
			old_mtr_cmd = mtr_cmd;
			
			stop_TMR1B_pwm();
			start_TMR1A_pwm();
			set_TMR1A_duty_cycle(mtr_cmd);
		}
		else if (mtr_cmd < 0)  // backward direction
		{
			if (mtr_cmd < -75)
				mtr_cmd = -75;
			stop_TMR1A_pwm();
			start_TMR1B_pwm();
			
			mtr_cmd *= -1;
			// rate limit
			if (abs(mtr_cmd - old_mtr_cmd) > RATE_LIMIT_THRESHOLD)
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
			if (mtr_cmd > 75 )
			mtr_cmd = 75;
			
			old_mtr_cmd = mtr_cmd;
			set_TMR1B_duty_cycle(abs(mtr_cmd));
		}
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
	OCR3A = 7812; // (7812) 500 ms period
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
	_voltage *= 48;
	_voltage /= 10; 
	
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


//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&& ISRs &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

//&&&&&&&&&&&&&&&&&&&&&& TMR3 ISR &&&&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

ISR(TIMER3_COMPA_vect)
{
	timeout = 1;
	
//	LED2_ON;
	
//  	voltage = (int16_t)(100*get_voltage()); // in V
//  
//  	current = (int16_t)(1000*get_current()); // in A
//  
//  	temperature = (int16_t)(10*get_temperature());
}
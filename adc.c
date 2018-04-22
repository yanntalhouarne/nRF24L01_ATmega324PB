/*
 * adc.c
 *
 * Created: 10/28/2017 12:06:34 AM
 *  Author: talho_000
 */

#include "adc.h"


void setup_adc()
{
    ADMUX |= (1 << REFS0);
    ADCSRA |= (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // enable ADC module, prescaler of 128
}

int analog_read(unsigned char channel)
{
    int analog_in = 0;
    
	//ADMUX = (0x07)&channel; // select channel
	ADMUX = (ADMUX & 0xF8) | (channel & 0x07);
    ADCSRA |= (1 << ADSC); // start conversion
    while ((ADCSRA & (1 << ADIF)) == 0)
        ; // wait for flag

    analog_in = (0x00FF) & (ADCL);       // get lower byte
    analog_in |= (0x2F00) & (ADCH << 8); // get higher byte

    return analog_in;
}

int analog_get_average(unsigned char channel, unsigned char elements) // not tested
{
    int average = 0;
    for (int i = 0; i < elements; i++)
    {
        average += analog_read(channel);
        _delay_ms(DELAY_CONVERSION);
    }

    average /= elements;

    return average;
}


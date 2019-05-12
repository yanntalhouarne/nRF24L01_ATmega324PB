/*
 * print.c
 *
 * Created: 10/12/2017 1:17:12 PM
 *  Author: talho_000
 */ 

#include "print.h"
#include "usart.h"

void print_char_0(char character)
{
	USART0_sendChar(character);
}


void print_0(char char_array[]) // prints to usart
{
	for (int i=0; i<200 ; i++)
	{
		if ( (char_array[i] == NL) || (char_array[i] == EOT) )
			break;
		USART0_sendChar(char_array[i]);
	}
}

void println_0(char char_array[]) // prints to usart
{
	for (int i=0; i<200 ; i++)
	{
		if( (char_array[i] == NL) || (char_array[i] == EOT) )
			break;
		USART0_sendChar(char_array[i]);
		
	}
	USART0_sendChar(0x0A); // NL
}

void print_crln_0(char char_array[]) // prints to usart
{
	for (int i=0; i<200 ; i++)
	{
		if( (char_array[i] == NL) || (char_array[i] == EOT) )
		break;
		USART0_sendChar(char_array[i]);
		
	}
	USART0_sendChar(0x0D); // CR
	USART0_sendChar(0x0A); // NL
}

void println_1(char char_array[]) // prints to usart
{
	for (int i=0; i<200 ; i++)
	{
		if( (char_array[i] == NL) || (char_array[i] == EOT) )
			break;
		USART1_sendChar(char_array[i]);
		
	}
	USART1_sendChar(0x0A); // NL
}

void print_crln_1(char char_array[]) // prints to usart
{
	for (int i=0; i<200 ; i++)
	{
		if( (char_array[i] == NL) || (char_array[i] == EOT) )
		break;
		USART1_sendChar(char_array[i]);
		
	}
	USART1_sendChar(0x0D); // CR
	USART1_sendChar(0x0A); // NL
}

void print_int_0(int number)
{
	if (number == 0)
	print_char_0('0');
	
	int temp = number;
	int num_digit = 0;
	
	while (temp != 0)
	{
		temp = temp /10;
		num_digit++;
	}
	if (number<0) num_digit++;
	char char_array[MAX_STRING_SIZE];
	itoa(number, char_array, 10);

	for (int i=0; i<num_digit ; i++)
	{
		USART0_sendChar(char_array[i]);
	}
}

void println_int_0(int number)
{
	if (number == 0)
	{
		print_char_0('0');
	}
	int temp = number;
	int num_digit = 0;
	
	while (temp != 0)
	{
		temp = temp /10;
		num_digit++;
	}
	if (number<0) num_digit++;
	char char_array[MAX_STRING_SIZE];
	itoa(number, char_array,10);

	for (int i=0; i<num_digit ; i++)
	{
		USART0_sendChar(char_array[i]);
	}
	USART0_sendChar(0x0A); // NL
}

void print_long_0(long number)
{
	long temp = number;
	int num_digit = 0;
	
	while (temp != 0)
	{
		temp = temp /10;
		num_digit++;
	}
	
	if (number<0) num_digit++;
	
	char char_array[MAX_STRING_SIZE];
	ltoa(number, char_array,10);

	for (int i=0; i<num_digit ; i++)
	{
		USART0_sendChar(char_array[i]);
	}
}

void println_long_0(long number)
{
	long temp = number;
	int num_digit = 0;
	
	while (temp != 0)
	{
		temp = temp /10;
		num_digit++;
	}
	if (number<0) num_digit++;
	
	
	char char_array[MAX_STRING_SIZE];
	ltoa(number, char_array, 10);

	for (int i=0; i<num_digit ; i++)
	{
		USART0_sendChar(char_array[i]);
	}
	USART0_sendChar(0x0A); // NL
}

void print_float_0(float number, unsigned char decimals)
{
	char str[8];
	dtostrf(number, 1, (decimals+3), str);
	for (int i=0; i<(decimals) ; i++)
	{
		USART0_sendChar(str[i]);
	}
}

void println_float_0(float number, unsigned char decimals)
{
	char str[8];
	dtostrf(number, 1, (decimals+3), str);
	for (int i=0; i<(decimals) ; i++)
	{
		USART0_sendChar(str[i]);
	}
	USART0_sendChar(0x0A); // NL
}
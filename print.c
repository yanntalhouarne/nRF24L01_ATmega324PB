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
	usart0_send_char(character);
}


void print_0(char char_array[]) // prints to usart
{
	for (int i=0; i<200 ; i++)
	{
		if ( (char_array[i] == NL) || (char_array[i] == EOT) )
			break;
		usart0_send_char(char_array[i]);
	}
}

void println_0(char char_array[]) // prints to usart
{
	for (int i=0; i<200 ; i++)
	{
		if( (char_array[i] == NL) || (char_array[i] == EOT) )
			break;
		usart0_send_char(char_array[i]);
		
	}
	usart0_send_char(0x0A); // NL
}

void print_crln_0(char char_array[]) // prints to usart
{
	for (int i=0; i<200 ; i++)
	{
		if( (char_array[i] == NL) || (char_array[i] == EOT) )
		break;
		usart0_send_char(char_array[i]);
		
	}
	usart0_send_char(0x0D); // CR
	usart0_send_char(0x0A); // NL
}

void println_1(char char_array[]) // prints to usart
{
	for (int i=0; i<200 ; i++)
	{
		if( (char_array[i] == NL) || (char_array[i] == EOT) )
			break;
		usart1_send_char(char_array[i]);
		
	}
	usart1_send_char(0x0A); // NL
}

void print_crln_1(char char_array[]) // prints to usart
{
	for (int i=0; i<200 ; i++)
	{
		if( (char_array[i] == NL) || (char_array[i] == EOT) )
		break;
		usart1_send_char(char_array[i]);
		
	}
	usart1_send_char(0x0D); // CR
	usart1_send_char(0x0A); // NL
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
	itoa(number, char_array,10);

	for (int i=0; i<num_digit ; i++)
	{
		usart0_send_char(char_array[i]);
	}
}

void println_int_0(int number)
{
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
		usart0_send_char(char_array[i]);
	}
	usart0_send_char(0x0A); // NL
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
		usart0_send_char(char_array[i]);
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
		usart0_send_char(char_array[i]);
	}
	usart0_send_char(0x0A); // NL
}

void println_decimal_0(double number, unsigned char decimals) 
{
	char str[4];
	dtostrf(number, 1, (decimals+3), str);
	for (int i=0; i<(decimals) ; i++)
	{
		usart0_send_char(str[i]);
	}
	usart0_send_char(0x0A); // NL
}

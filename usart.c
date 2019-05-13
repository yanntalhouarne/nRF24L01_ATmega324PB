#include <avr/io.h>

#include "usart.h"
#include "print.h"




void USART0_sendChar(unsigned char data)
{
	while (!(UCSR0A & (1 << UDRE)))
	; // wait for data buffer to be empty
	UDR0 = data;
}


unsigned char USART0_receiveChar()
{
	while (!(UCSR0A & (1 << RXC))) // RXCn is set when there are unread data in the receive buffer and cleared when the receive buffer is empty
	;
	return UDR0;
}


void USART0_setup(unsigned char BR)
{
	UCSR0B = (1 << TXEN) | (1 << RXEN); //
	UCSR0C = (1 << UCSZ1) | (1 << UCSZ0);               // 8-bit character size
	UBRR0L = BR;                                        // 51 for 9600 baud rate at 8Mhz
}

void USART0_stopRxInterrupt()
{
	UCSR0B &= ~(1<<RXCIE);                                          
}
void USART0_startRxInterrupt()
{
	UCSR0B |= (1<<RXCIE);
}


void USART1_sendChar(unsigned char data)
{
	while (!(UCSR1A & (1 << UDRE)))
	; // wait for data buffer to be empty
	UDR1 = data;
}

unsigned char USART1_receiveChar()
{
	while (!(UCSR1A & (1 << RXC))) // RXCn is set when there are unread data in the receive buffer and cleared when the receive buffer is empty
	;
	return UDR1;
}

void USART1_receiveString()
{
	int i = 0;
	while(1)
	{
		while (!(UCSR1A & (1 << RXC))) // RXCn is set when there are unread data in the receive buffer and cleared when the receive buffer is empty
		;
	    rcv_string[i] = UDR1;
	    if ((rcv_string[i] == NL) || (i == MAX_STRING_SIZE))
	    {
		   break;
	    }
	    else
	    {
		    i++;
		    
	    }
	}
}

void USART1_setup(unsigned char BR)
{
	UCSR1B = (1 << TXEN) | (1 << RXEN)| (1<<RXCIE); // enable USART1
	UCSR1C = (1 << UCSZ1) | (1 << UCSZ0);               // 8-bit character size
	UBRR1L = BR;                                        // 51 for 9600 baud rate at 8Mhz
}




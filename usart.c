#include <avr/io.h>

#include "usart.h"
#include "print.h"




void usart0_send_char(unsigned char data)
{
	while (!(UCSR0A & (1 << UDRE)))
	; // wait for data buffer to be empty
	UDR0 = data;
}


unsigned char usart0_receive_char()
{
	while (!(UCSR0A & (1 << RXC))) // RXCn is set when there are unread data in the receive buffer and cleared when the receive buffer is empty
	;
	return UDR0;
}


void setup_usart0(unsigned char BR)
{
	UCSR0B = (1 << TXEN) | (1 << RXEN) | (1<<RXCIE); //
	UCSR0C = (1 << UCSZ1) | (1 << UCSZ0);               // 8-bit character size
	UBRR0L = BR;                                        // 51 for 9600 baud rate at 8Mhz
}

void stop_RX0_interrupt()
{
	UCSR0B &= ~(1<<RXCIE);                                          
}
void start_RX0_interrupt()
{
	UCSR0B |= (1<<RXCIE);
}


void usart1_send_char(unsigned char data)
{
	while (!(UCSR1A & (1 << UDRE)))
	; // wait for data buffer to be empty
	UDR1 = data;
}

unsigned char usart1_receive_char()
{
	while (!(UCSR1A & (1 << RXC))) // RXCn is set when there are unread data in the receive buffer and cleared when the receive buffer is empty
	;
	return UDR1;
}

/* usart1_receive_string */
/* dynamically allocates data to store unknown sized string */

void usart1_receive_string()
{
	unsigned char * rcv_ptr;

	int i = 0;
	
	while(1)
	{
		while (!(UCSR1A & (1 << RXC))) // RXCn is set when there are unread data in the receive buffer and cleared when the receive buffer is empty
		;
	    (*rcv_ptr) = UDR1; // update the value at the address pointed by rcv_ptr
		rcv_ptr++; // increment pointer address by one for next byte

	    if (rcv_ptr == NL) 
	    {
		   break;
	    }
		else if (i == MAX_STRING_SIZE)
		{
			rcv_ptr = NULL;
		}
	    else
	    {
		    i++;
	    }
	}
	return rcv_ptr;
}

void setup_usart1(unsigned char BR)
{
	UCSR1B = (1 << TXEN) | (1 << RXEN)| (1<<RXCIE); // enable USART1
	UCSR1C = (1 << UCSZ1) | (1 << UCSZ0);               // 8-bit character size
	UBRR1L = BR;                                        // 51 for 9600 baud rate at 8Mhz
}

unsigned char check_RX()
{
	return receive_string_ready;
}

void clear_RX()
{
	receive_string_ready = 0;
}


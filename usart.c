#include <avr/io.h>

#include "usart.h"
#include "print.h"

struct usart_char * string_header = NULL;


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

struct usart_char * usart1_receive_string()
{
	struct usart_char * new_char;

	int i = 0;

	while(1)
	{
		while (!(UCSR1A & (1 << RXC))) // RXCn is set when there are unread data in the receive buffer and cleared when the receive buffer is empty
		;

		if(!new_char = malloc(sizeof(* new_char))) // allocate enough memory for a usart_char struct
		{
			string_header = NULL;
			break;
		}

		new_char->character = UDR1; // store character in the new_char node

		new_char->next = string_header; // new_node next pointer is previous header pointer 

		string_header = new_char; // new string_header now points to newn_node

	    if (new_char->character == NL)  // if string NULL character is found, break
	    {
		   break; // full string is implemented as a linked list, exit while loop
	    }
		else if (i == MAX_STRING_SIZE) // if we have reached the maximum number of characters allowed
		{
			string_header = NULL; // set the list header to NULL to indicate that maximum size is reached.
			break; // exit while loop
		}
	    else
	    {
		    i++;
	    }
	}

	return string_header;
}

void usart_free_string()
{
	struct usart_char * cur;


	while (cur->next_char != NULL)
		prev = cur;
		string_header = cur->next; // bypass first node
		free(cur); // deallocate memory
		cur = string_header; 
	}
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


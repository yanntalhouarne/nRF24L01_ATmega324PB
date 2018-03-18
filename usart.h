/*
 * usart.h
 *
 * Created: 10/12/2017 1:20:24 PM
 *  Author: talho_000
 */ 

#include <avr/io.h>

#ifndef USART_H_
#define USART_H_

#define MAX_STRING_SIZE 50

#define BR_9600 51 // baud rate for USART (51 for 9600 at 8MHz)


// globals for RX interrupts string
unsigned char rcv_string[MAX_STRING_SIZE];

void usart0_send_char(unsigned char data);
unsigned char usart0_receive_char();
void setup_usart0(unsigned char BR);

void usart1_send_char(unsigned char data);
unsigned char usart1_receive_char();
void usart1_receive_string();
void setup_usart1(unsigned char BR);

#endif /* USART_H_ */
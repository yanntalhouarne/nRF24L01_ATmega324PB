/*
 * usart.h
 *
 * Created: 10/12/2017 1:20:24 PM
 *  Author: talho_000
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>

#ifndef USART_H_
#define USART_H_

#define MAX_STRING_SIZE 64

#define BR_9600 103
#define BR_38400 25
#define BR_115200 8
#define BR_250000 3
#define BR_500000 1
#define BR_1000000 0



// globals for RX interrupts string
unsigned char rcv_string[MAX_STRING_SIZE];
unsigned char receive_string_ready;

void USART0_sendChar(unsigned char data);
unsigned char USART0_receiveChar();
void USART0_setup(unsigned char BR);
void USART0_stopRxInterrupt();
void USART0_startRxInterrupt();

void USART1_sendChar(unsigned char data);
unsigned char USART1_receiveChar();
void USART1_receiveString();
void USART1_setup(unsigned char BR);


#endif /* USART_H_ */
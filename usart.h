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

#define BR_9600 51
 #define BR_3800 25
 #define BR_250000 3
 #define BR_500000 1
 #define BR_1000000 0


// globals for RX interrupts string
unsigned char rcv_string[MAX_STRING_SIZE];
unsigned char receive_string_ready;

void usart0_send_char(unsigned char data);
unsigned char usart0_receive_char();
void setup_usart0(unsigned char BR);
void stop_RX0_interrupt();
void start_RX0_interrupt();

void usart1_send_char(unsigned char data);
unsigned char usart1_receive_char();
void usart1_receive_string();
void setup_usart1(unsigned char BR);
unsigned char check_RX();
void clear_RX();


#endif /* USART_H_ */
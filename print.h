/*
 * print.h
 *
 * Created: 10/12/2017 1:23:11 PM
 *  Author: talho_000
 */ 

#include <avr/io.h>
#include <stddef.h>
#include <stdlib.h>

#ifndef PRINT_H_
#define PRINT_H_

#define EOT 0x3B           // custom End of Text character (not ASCII)
#define NL 0x0A            // ASCII for New Line

void print_char(char character);
void print(char char_array[]);
void println(char char_array[]);
void println_0(char char_array[]);
void print_crln_0(char char_array[]);
void print_crln_1(char char_array[]);
void println_int(int number);
void println_long(long number);
void print_long(long number);
void println_long(long number);
void println_decimal(double number, unsigned char decimals);

#endif /* PRINT_H_ */
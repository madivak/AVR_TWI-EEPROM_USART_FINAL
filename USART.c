
/*
 * TEST-USART.c
 *
 * Created: 8/30/2017 12:14:36 PM
 * Author : madiva
 */ 
#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include "USART.h"

void USART0_Init( unsigned int ubrr)
{
	/* Set the baud rate */
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	/* Enable UART receiver and transmitter */
	UCSR0B = ((1<<RXEN0) | (1<<TXEN0));// | (1<<RXCIE0)); //disabled RXCIE0 so that global interrupt sei() works in main function
	/* Set frame format: 8data, 2stop bit */
	UCSR0C = (3<<UCSZ00);
	//Double speed mode
	UCSR0A = (1<<U2X0);
}
void USART1_Init( unsigned int ubrr)
{	/* Set the baud rate */
	UBRR1H = (unsigned char)(ubrr>>8);
	UBRR1L = (unsigned char)ubrr;
	/* Enable UART receiver and transmitter */
	UCSR1B = ((1<<RXEN1) | (1<<TXEN1));// | (1<<RXCIE1)); //disabled RXCIE0 so that global interrupt sei() works in main function
	/* Set frame format: 8data, 2stop bit */
	UCSR1C = (3<<UCSZ10);
	//Double speed mode
	UCSR1A = (1<<U2X1);
}
int USART0_Transmit( char data0, FILE *stream)
{
	if (data0 == '\n') {
		USART0_Transmit('\r', stream);
	}
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE0)) );	;
	/* Put data into buffer, sends the data */
	UDR0 = data0;
}unsigned char USART0_Receive( FILE *stream )
{
	/* Wait for data to be received */
	while ( !(UCSR0A & (1<<RXC0)));
	/* Get and return received data from buffer */
	return UDR0;
}int USART1_Transmit( char data1, FILE *stream )
{
	if (data1 == '\n') {
		USART1_Transmit('\r', stream);
	}
	/* Wait for empty transmit buffer */
	while ( !( UCSR1A & (1<<UDRE1)) );	;
	/* Put data into buffer, sends the data */
	UDR1 = data1;
}

unsigned char USART1_Receive( FILE *stream )
{
	/* Wait for data to be received */
	while ( !(UCSR1A & (1<<RXC1)));
	/* Get and return received data from buffer */
	return UDR1;
}







// /*
//  * USART.c
//  *
//  * Created: 9/20/2017 2:51:38 PM
//  *  Author: madiv
//  */ 
// #define BAUD 9600
// #define F_CPU 16000000UL
// 
// #include <util/setbaud.h>
// 
// #include <avr/io.h>
// #include <stdio.h>
// #include <avr/interrupt.h>
// #include "AT-SMS.h"
// 
// /* Static Variables */
// static unsigned char UART_RxBuf[UART_RX_BUFFER_SIZE];
// static volatile unsigned char UART_RxHead;
// static volatile unsigned char UART_RxTail;
// static unsigned char UART1_RxBuf[UART1_RX_BUFFER_SIZE];
// static volatile unsigned char UART1_RxHead;
// static volatile unsigned char UART1_RxTail;
// static unsigned char UART_TxBuf[UART_TX_BUFFER_SIZE];//static volatile unsigned char UART_TxHead;
// static volatile unsigned char UART_TxTail;
// static unsigned char UART1_TxBuf[UART1_TX_BUFFER_SIZE];//static volatile unsigned char UART1_TxHead;
// static volatile unsigned char UART1_TxTail;
// 
// void USART0_Init(unsigned char ubrr)
// {
// 	unsigned char x;
// 	/* Set the baud rate */
// 	UBRR0H = (unsigned char)(ubrr>>8);
// 	UBRR0L = (unsigned char)ubrr;
// 	/* Enable UART receiver and transmitter */
// 	UCSR0B = ((1<<RXEN0) | (1<<TXEN0) | (1<<RXCIE0));
// 	/* Set frame format: 8data, 2stop bit */
// 	UCSR0C = (3<<UCSZ00);
// 	//Double speed mode
// 	UCSR0A = (1<<U2X0);
// 	/* Flush receive buffer */
// 	x = 0;
// 	UART_RxTail = x;
// 	UART_RxHead = x;
// 	UART_TxTail = x;
// 	UART_TxHead = x;
// }
// 
// void USART1_Init(unsigned char ubrr)
// {
// 	unsigned char y;
// 	/* Set the baud rate */
// 	UBRR1H = (unsigned char)(ubrr>>8);
// 	UBRR1L = (unsigned char)ubrr;
// 	/* Enable UART receiver and transmitter */
// 	UCSR1B = ((1<<RXEN1) | (1<<TXEN1) | (1<<RXCIE1));
// 	/* Set frame format: 8data, 2stop bit */
// 	UCSR1C = (3<<UCSZ10);
// 	//Double speed mode
// 	UCSR1A = (1<<U2X1);
// 	/* Flush receive buffer */
// 	y = 0;
// 
// 	UART1_RxTail = y;
// 	UART1_RxHead = y;
// 	UART1_TxTail = y;
// 	UART1_TxHead = y;
// }
// 
// /* Interrupt handlers */
// ISR(USART1_RX_vect)
// {
// 	unsigned char data;
// 	unsigned char tmphead;
// 	/* Read the received data */
// 	data = UDR1;
// 	/* Calculate buffer index */
// 	tmphead = (UART1_RxHead + 1) & UART1_RX_BUFFER_MASK;
// 	/* Store new index */
// 	UART1_RxHead = tmphead;
// 	if (tmphead == UART1_RxTail) {
// 		/* ERROR! Receive buffer overflow */
// 	}
// 	/* Store received data in buffer */
// 	UART1_RxBuf[tmphead] = data;
// }
// 
// /* Interrupt handlers */
// ISR(USART0_RX_vect)
// {
// 	unsigned char data;
// 	unsigned char tmphead;
// 	/* Read the received data */
// 	data = UDR0;
// 	/* Calculate buffer index */
// 	tmphead = (UART_RxHead + 1) & UART_RX_BUFFER_MASK;
// 	/* Store new index */
// 	UART_RxHead = tmphead;
// 	if (tmphead == UART_RxTail) {
// 		/* ERROR! Receive buffer overflow */
// 	}
// 	/* Store received data in buffer */
// 	UART_RxBuf[tmphead] = data;
// }
// 
// ISR(USART0_UDRE_vect)
// {
// 	unsigned char tmptail;
// 	/* Check if all data is transmitted */
// 	if (UART_TxHead != UART_TxTail) {
// 		/* Calculate buffer index */
// 		tmptail = ( UART_TxTail + 1 ) & UART_TX_BUFFER_MASK;
// 		/* Store new index */
// 		UART_TxTail = tmptail;
// 		/* Start transmission */
// 		UDR0 = UART_TxBuf[tmptail];
// 		} else {		/* Disable UDRE interrupt *///		UCSR0B &= ~(1<<UDRIE0);//	}
// }
// 
// ISR(USART1_UDRE_vect)
// {
// 	unsigned char tmptail;
// 	/* Check if all data is transmitted */
// 	if (UART1_TxHead != UART1_TxTail) {
// 		/* Calculate buffer index */
// 		tmptail = ( UART1_TxTail + 1 ) & UART1_TX_BUFFER_MASK;
// 		/* Store new index */
// 		UART1_TxTail = tmptail;
// 		/* Start transmission */
// 		UDR1 = UART1_TxBuf[tmptail];
// 		} else {		/* Disable UDRE interrupt *///		UCSR1B &= ~(1<<UDRIE1);//	}
// }
// 
// int uart0_putchar(char c, FILE *stream) 
// {
// // 	if (c == '\n') {
// // 		uart0_putchar('\r', stream);
// // 	}
// // 	/* Wait for empty transmit buffer */
// // 	while ( !( UCSR0A & (1<<UDRE0)));
// // 	/* Put data into buffer, sends the data */
// // 	UDR0 = c;
// 	
// 		unsigned char tmphead;		/* Calculate buffer index *///		tmphead = (UART_TxHead + 1) & UART_TX_BUFFER_MASK;		/* Wait for free space in buffer */
// 		while (tmphead == UART_TxTail);
// 		/* Store data in buffer */
// 		UART_TxBuf[tmphead] = c;
// 		/* Store new index */
// 		UART_TxHead = tmphead;
// 		/* Enable UDRE interrupt */
// 		UCSR0B |= (1<<UDRIE0);	
// }
// 
// int uart1_putchar(char c, FILE *stream)
// {
// 	unsigned char tmphead;	/* Calculate buffer index *///	tmphead = (UART1_TxHead + 1) & UART1_TX_BUFFER_MASK;	/* Wait for free space in buffer */
// 	while (tmphead == UART1_TxTail);
// 	/* Store data in buffer */
// 	UART1_TxBuf[tmphead] = c;
// 	/* Store new index */
// 	UART1_TxHead = tmphead;
// 	/* Enable UDRE interrupt */
// 	UCSR1B |= (1<<UDRIE1);
// }
// 
// int uart0_getchar(FILE *stream) 
// {
// // 	/* Wait for data to be received */
// // 	while ( !(UCSR0A & (1<<RXC0)));
// // 	/* Get and return received data from buffer */
// // 	return UDR0;
// 	
// 		unsigned char tmptail;				/* Wait for incoming data */
// 		while (UART_RxHead == UART_RxTail);
// 		/* Calculate buffer index */
// 		tmptail = (UART_RxTail + 1) & UART_RX_BUFFER_MASK;
// 		/* Store new index */
// 		UART_RxTail = tmptail;
// 		
// 		/* Return data */
// 		return UART_RxBuf[tmptail];
// }
// 
// 
// int uart1_getchar(FILE *stream)
// {
// 	unsigned char tmptail;		/* Wait for incoming data */
// 	while (UART1_RxHead == UART1_RxTail);
// 	/* Calculate buffer index */
// 	tmptail = (UART1_RxTail + 1) & UART1_RX_BUFFER_MASK;
// 	/* Store new index */
// 	UART1_RxTail = tmptail;
// 	
// 	/* Return data */
// 	return UART1_RxBuf[tmptail];
// }
// 
// unsigned char DataInReceiveBuffer( void )
// {
// 	return ( UART_RxHead != UART_RxTail ); //Return 0 (FALSE) if the receive buffer is empty
// }
// unsigned char DataInReceiveBuffer1( void )
// {
// 	return ( UART1_RxHead != UART1_RxTail ); //Return 0 (FALSE) if the receive buffer is empty
// }
// 

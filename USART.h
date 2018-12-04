void USART0_Init( unsigned int ubrr);
void USART1_Init( unsigned int ubrr);
int USART0_Transmit( char data0, FILE *stream);
unsigned char USART0_Receive( FILE *stream );
int USART1_Transmit( char data1, FILE *stream );
unsigned char USART1_Receive( FILE *stream );





/*
 * USART.h
 *
 * Created: 9/20/2017 2:51:25 PM
 *  Author: madiv
 */


//  #ifndef USART_H_
//  #define USART_H_

/* UART Buffer Defines */
// #define UART_RX_BUFFER_SIZE 128
// #define UART1_RX_BUFFER_SIZE 128     /* 2,4,8,16,32,64,128 or 256 bytes */
// #define UART_TX_BUFFER_SIZE 128
// #define UART1_TX_BUFFER_SIZE 128
// #define UART_RX_BUFFER_MASK (UART_RX_BUFFER_SIZE - 1)
// #define UART1_RX_BUFFER_MASK (UART1_RX_BUFFER_SIZE - 1)
// #define UART_TX_BUFFER_MASK (UART_TX_BUFFER_SIZE - 1)
// #define UART1_TX_BUFFER_MASK (UART1_TX_BUFFER_SIZE - 1)
// 
// void USART0_Init(unsigned char ubrr);
// void USART1_Init(unsigned char ubrr);
// int uart0_putchar(char c, FILE *stream);
// int uart1_putchar(char c, FILE *stream);
// int uart0_getchar(FILE *stream);
// int uart1_getchar(FILE *stream);
// unsigned char DataInReceiveBuffer( void );
// unsigned char DataInReceiveBuffer1( void );
/*void USART_putstring(char* StringPtr);*/
/*void USART0_Transmit(unsigned char data);*/
/*#endif / * USART_H_ * /*/

/*
 * USART C library
 * Only 8 data bits, 1 stop bit supported
 */
#include <avr/io.h>
#include <avr/interrupt.h>

#define BAUD 9600
#include <util/setbaud.h>	// for UBBRL_VALUE etc.
#include "usart.h"

void USART_Init(void) {
	/*
         * UBBR = (F_CPU/(8ul * BAUD)) - 1
	 * UBRRL_VALUE = (UBBR & 0xFF)
	 * UBBRH_VALUE = (UBBR >> 8)
	 */
	UBRR0H = UBRRH_VALUE;
	UBRR0L = UBRRL_VALUE;

	/* default: 8 data bits; 1 stop bit */
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); 

	/* enable */
	UCSR0B = (1 << TXEN0) | (1 << RXEN0);
}

void USART_Init_RXinterrupt(void) {
	/* enable RX interrupts */
	UCSR0B |= (1 << RXCIE0);

	/* enable global interrupts */
	sei();
}

void USART_Transmit(uint8_t b) {
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE0)) );

	/* Put data into transmit buffer, sends the data */
	UDR0 = b;
}

uint8_t USART_Receive(void) {
	/* Wait for receive complete */
	while ( !( UCSR0A & (1<<RXC0)) );

	// Return data from receive buffer buffer
	return(UDR0);
}


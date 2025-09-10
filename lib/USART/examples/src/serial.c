/*
 * Example Serial code using USART C library
 */
#include <avr/io.h>

#define BAUD 9600
#include <util/setbaud.h>
#include <usart.h>

int main(void) {
	// init serial
	USART_Init();

	// loop
	while(1) {
		// check for input
		uint8_t ch = USART_Receive();

		// is int? -> write to serial
		if (ch >= '0' && ch <= '9') {
			USART_Transmit(ch);
		}
	}
	
	// never reached
	return(0);
}

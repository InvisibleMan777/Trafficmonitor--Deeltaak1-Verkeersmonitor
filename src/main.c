#include <avr/io.h>
#include <stdbool.h>
#include <usart.h>
#include <stdio.h>
#include <util/delay.h>

int main() {
    int count = 0;
    bool buttonPressed[2] = {false, false};

    //pin 8-11 are outputs
    DDRB |= (1 << DDB0); 
    DDRB |= (1 << DDB1); 
    DDRB |= (1 << DDB2); 
    DDRB |= (1 << DDB3); 

    //port 2 and 3 are pullup inputs
    PORTD |= (1 << PORTD2);
    PORTD |= (1 << PORTD3);

    //initialize serial communication
    USART_Init();

    //main loop
    while (true) {
        //updating the leds (pb0-pb3) to display the count in binary every cycle
        PORTB ^= PORTB ^ (count & 0b00001111);

        //check buttonstate of button0
        switch (buttonPressed[0]) {
            //button0 pressed
            case true:
                //delay for debounce
                _delay_ms(50);

                //button is being released
                if (PIND & (1 << PIND2)) {
                    buttonPressed[0] = false;
                }
                break;

            //button0 not pressed
            case false:
                //button is being pressed
                if (!(PIND & (1 << PIND2))) {
                    count++;

                    //cast to string is neccesairy to transmit the count
                    char str[2400];
                    sprintf(str, "%d", count);

                    //transmit count
                    for (int i = 0; str[i] != '\0'; i++) {
                        USART_Transmit(str[i]);
                    }
                    USART_Transmit('\n');

                    buttonPressed[0] = true;
                }
                break;
        }
    }

    return 0;
}
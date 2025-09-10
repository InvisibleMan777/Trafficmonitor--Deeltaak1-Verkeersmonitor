#include <avr/io.h>
#include <stdbool.h>
#include <usart.h>
#include <stdio.h>
#include <util/delay.h>

int main() {
    int unitCounter = 0; //counter of units that have passed triggers of button0 and button1
    bool buttonPressed[2] = {false, false}; //button states of button0 and button1
    int unitsInProcess = 0; //counter of units between triggers of button0 and button1

    //pin 8-11 are outputs
    DDRB |= (1 << DDB0); 
    DDRB |= (1 << DDB1); 
    DDRB |= (1 << DDB2); 
    DDRB |= (1 << DDB3); 
    //pin 2 and 3 are pullup inputs
    PORTD |= (1 << PORTD2);
    PORTD |= (1 << PORTD3);

    //initialize serial communication
    USART_Init();

    //main loop
    while (true) {
        //updating the leds (pb0-pb3) to display the unit count in binary every cycle
        PORTB ^= PORTB ^ (unitCounter & 0b00001111);

        //check state of button0
        switch (buttonPressed[0]) {
            case true:
                //anti debounce
                _delay_ms(50);

                //button is being released
                if (PIND & (1 << PIND2)) {
                    buttonPressed[0] = false;
                }
                break;

            case false:
                //button is being pressed
                if (!(PIND & (1 << PIND2))) {
                    unitsInProcess++;
                    buttonPressed[0] = true;
                }
                break;
        }

        //check state of button1
        switch (buttonPressed[1]) {
            case true:
                //anti debounce
                _delay_ms(50);

                //button is being released
                if (PIND & (1 << PIND3)) {
                    buttonPressed[1] = false;
                }
                break;

            case false:
                //button is being pressed
                if (!(PIND & (1 << PIND3))) {
                    //if there is at least 1 trigger in process (initialized by button0)
                    if (unitsInProcess > 0) {
                        unitCounter++;
                        unitsInProcess--;

                        //cast to string is neccesairy to transmit the count
                        char str[2400];
                        sprintf(str, "%d", unitCounter);

                        //transmit unit count to serial
                        for (int i = 0; str[i] != '\0'; i++) {
                            USART_Transmit(str[i]);
                        }
                        USART_Transmit('\n');

                        buttonPressed[1] = true;
                    }
                }
                break;
        }
    }

    return 0;
}
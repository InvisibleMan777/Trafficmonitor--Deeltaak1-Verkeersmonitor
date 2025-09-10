#include <avr/io.h>
#include <stdbool.h>
#include <usart.h>
#include <stdio.h>
#include <util/delay.h>

#define SAMPLE_RATE 10

int main() {
    int unitCounter = 0; //counter of units that have passed triggers of button0 and button1
    bool buttonPressed[2] = {false, false}; //button states of button0 and button1
    int unitsInProcess = 0; //counter of units between triggers of button0 and button1

    //pin 8-11 are outputs
    DDRB |= (1 << DDB0 | 1 << DDB1 | 1 << DDB2 | 1 << DDB3); 
    //pin 2 and 3 are pullup inputs
    PORTD |= (1 << PORTD2 | 1 << PORTD3);

    //initialize serial communication
    USART_Init();

    //main loop
    while (true) {
        //samplerate (default 10ms). Lower rates increase risk of debounce issues
        _delay_ms(SAMPLE_RATE);

        //updating the leds (pb0-pb3) to display the unit count in binary every cycle
        PORTB ^= PORTB ^ (unitCounter & (1 << PORTB0 | 1 << PORTB1 | 1 << PORTB2 | 1 << PORTB3));

        //check state of button0
        switch (buttonPressed[0]) {
            case true:
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
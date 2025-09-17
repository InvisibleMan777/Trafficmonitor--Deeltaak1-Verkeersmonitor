#include <avr/io.h>
#include <stdbool.h>
#include <usart.h>
#include <stdio.h>
#include <util/delay.h>
#include <Arduino.h>

#define ANTI_DEBOUNCE_DELAY 200 //time in ms between checks if buttons are released to prevent the negative implications of debounce

//initialize IO pins
void initialiseIO() {
    //initalizing the 4 leds as outputs
    DDRC |= (1 << DDC0 | 1 << DDC1 | 1 << DDC2 | 1 << DDC3); 

    //initalizing the buttons as pullup inputs
    PORTD |= (1 << PORTD2 | 1 << PORTD3);

    //initalizing the switchpins of the 4x7seg display as outputs
    DDRB |= (1 << DDB1 | 1 << DDB2 | 1 << DDB3);
    //turn off all digits as default
    PORTB |= (1 << PORTB1 | 1 << PORTB2 | 1 << PORTB3); 

    //initalizing the data pins of the 4x7seg display as outputs
    DDRD |= (1 << DDD4 | 1 << DDD5 | 1 << DDD6 | 1 << DDD7);
    DDRB |= (1 << DDB0 | 1 << DDB4 | 1 << DDB5);
}

//updating pc0-pc3 (the 4 leds) to display the unit count in binary
void displayCounter(int count) {
    PORTC ^= PORTC ^ (count & (1 << PORTC0 | 1 << PORTC1 | 1 << PORTC2 | 1 << PORTC3));
}

//display a number (0-9) on a specific digit of the 4x7seg display. DigitPort is the port of the digit to be turned on (PB1, PB2 or PB3)
void displayNumberOnDigit(uint8_t number, uint8_t digitPort) {
    //default to 9 if out of range
    if (number > 9) {
        number = 9;
    }

    //turn on digit
    PORTB &= ~(1 << digitPort);

    //define segment positions
    #define SDP4 0
    #define SDP5 1
    #define SDP6 2
    #define SDP7 3
    #define SBP0 4
    #define SBP4 5
    #define SBP5 6
    #define SBP6 7

    //segment mapping for numbers 0-9
    const uint8_t segmentMap[] = {
        0b01011111, // 0
        0b00000110, // 1
        0b00111011, // 2
        0b00101111, // 3
        0b01100110, // 4
        0b01101101, // 5
        0b01111101, // 6
        0b00000111, // 7
        0b01111111, // 8
        0b01101111  // 9
    };

    //get segment pattern for the number
    const uint8_t segments = segmentMap[number];

    //clear segment pins first
    PORTD &= ~((1 << PD4) | (1 << PD5) | (1 << PD6) | (1 << PD7));
    PORTB &= ~((1 << PB0) | (1 << PB4) | (1 << PB5) | (1 << PB6));

    //set segment pins according to segmentMap
    if (segments & (1 << SDP4)) PORTD |= (1 << PD4);
    if (segments & (1 << SDP5)) PORTD |= (1 << PD5);
    if (segments & (1 << SDP6)) PORTD |= (1 << PD6);
    if (segments & (1 << SDP7)) PORTD |= (1 << PD7);
    if (segments & (1 << SBP0)) PORTB |= (1 << PB0);
    if (segments & (1 << SBP4)) PORTB |= (1 << PB4);
    if (segments & (1 << SBP5)) PORTB |= (1 << PB5);
    if (segments & (1 << SBP6)) PORTB |= (1 << PB6);

    //display time
    _delay_ms(1);

    //turn off digit
    PORTB |= (1 << digitPort);
}

int main() {
    unsigned int unitCounter = 0; //counter of units that have passed button0 and button1
    bool buttonPressed[2] = {false, false}; //button states of button0 and button1
    unsigned short int unitsInProcess = 0; //counter of units that have passed button0 but not yet button1
    unsigned long timeSinceLastTrigger = millis(); //time since last button press

    //initialise arduino
    init();
    //initialize io pins
    initialiseIO();
    //initialize serial communication
    USART_Init();

    //main loop
    while (true) {
        //update leds
        displayCounter(unitCounter);
        displayNumberOnDigit(9, PB1);

        //check state of button0
        switch (buttonPressed[0]) {
            case true:
                if (millis() - timeSinceLastTrigger < ANTI_DEBOUNCE_DELAY) {
                    break;
                }

                //button is being released
                if (PIND & (1 << PIND2)) {
                    buttonPressed[0] = false;
                }
                break;

            case false:
                //button is being pressed
                if (!(PIND & (1 << PIND2))) {
                    //not more then 255 units in process
                    if (unitsInProcess < 255) {
                        unitsInProcess++;
                    }       

                    buttonPressed[0] = true;
                    timeSinceLastTrigger = millis();
                }
                break;
        }

        //check state of button1
        switch (buttonPressed[1]) {
            case true:
                if (millis() - timeSinceLastTrigger < ANTI_DEBOUNCE_DELAY) {
                    break;
                }

                //button is being released
                if (PIND & (1 << PIND3)) {
                    buttonPressed[1] = false;
                }
                break;

            case false:
                //button is being pressed
                if (!(PIND & (1 << PIND3))) {
                    if (unitsInProcess > 0) {
                        unitCounter++;
                        unitsInProcess--;

                        //cast to string is neccesairy to transmit the count
                        char str[2400];
                        sprintf(str, "%u", unitCounter);

                        //transmit unit count to serial
                        for (int i = 0; str[i] != '\0'; i++) {
                            USART_Transmit(str[i]);
                        }
                        USART_Transmit('\n');

                        buttonPressed[1] = true;
                        timeSinceLastTrigger = millis();
                    }
                }
                break;
        }
    }

    return 0;
}
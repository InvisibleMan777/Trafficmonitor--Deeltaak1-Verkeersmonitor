#include <avr/io.h>
#include <stdbool.h>
#include <usart.h>
#include <stdio.h>
#include <util/delay.h>
#include <Arduino.h>

#define ANTI_DEBOUNCE_DELAY 200 //time in ms between checks if buttons are released to prevent the negative implications of debounce
#define DISTANCE_BETWEEN_BUTTONS_CM 60 //distance in centimeters between the two buttons, used to calculate speed

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
    //segments
    DDRD |= (1 << DDD4 | 1 << DDD5 | 1 << DDD6 | 1 << DDD7);
    DDRB |= (1 << DDB0 | 1 << DDB4 | 1 << DDB5);
    //decimal point
    DDRC |= (1 << DDC4);
}

//display a number (0-9) on a specific digit of the 4x7seg display. DigitPort is the port of the digit to be turned on (PB1, PB2 or PB3)
void displayNumberOnDigit(uint8_t number, uint8_t digitPort, bool decimalPoint) {
    //default to 9 if out of range
    if (number > 9) {
        number = 9;
    }

    //turn on digit
    PORTB &= ~(1 << digitPort);

    //segment mapping for numbers 0-9 in bytes
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


    //define segment positions in the byte
    #define SPD4 0 // segment mid-low
    #define SPD5 1 // segment right-high
    #define SPD6 2 // segment right-low
    #define SPD7 3 // segment left-high
    #define SPB0 4 // segment left-low
    #define SPB4 5 // segment mid-mid
    #define SPB5 6 // segment mid-high

    //clear segment pins first
    PORTD &= ~((1 << PD4) | (1 << PD5) | (1 << PD6) | (1 << PD7));
    PORTB &= ~((1 << PB0) | (1 << PB4) | (1 << PB5));
    PORTC &= ~(1 << PC4);

    //set segment pins according to segmentMap
    //NOTE: all these if statements could prob be optimized out, some optimization for in the future could be done
    if (segments & (1 << SPD4)) PORTD |= (1 << PD4);
    if (segments & (1 << SPD5)) PORTD |= (1 << PD5);
    if (segments & (1 << SPD6)) PORTD |= (1 << PD6);
    if (segments & (1 << SPD7)) PORTD |= (1 << PD7);
    if (segments & (1 << SPB0)) PORTB |= (1 << PB0);
    if (segments & (1 << SPB4)) PORTB |= (1 << PB4);
    if (segments & (1 << SPB5)) PORTB |= (1 << PB5);

    //turn on decimal point if needed
    if (decimalPoint) {
        PORTC |= (1 << PC4);
    }

     //display time
    _delay_ms(5);

    //turn off digit
    PORTB |= (1 << digitPort);
}

void displaySpeed(int speed) {
    if (speed < 10) {
        displayNumberOnDigit(speed, PB3, false);
    } else if (speed < 100) {
        displayNumberOnDigit(speed % 10, PB3, false);
        displayNumberOnDigit(speed / 10, PB2, true);
    } else {
        displayNumberOnDigit(speed % 10, PB3, false);
        displayNumberOnDigit((speed / 10) % 10, PB2, true);
        displayNumberOnDigit(speed / 100, PB1, false);
    }
}

//updating pc0-pc3 (the 4 leds) to display the unit count in binary
void displayCounter(int count) {
    PORTC ^= PORTC ^ (count & (1 << PORTC0 | 1 << PORTC1 | 1 << PORTC2 | 1 << PORTC3));
}

//calculate speed in meters per second
int calculateSpeed(uint64_t starttime, uint64_t endtime) {
    //prevent division by zero and negative speeds
    if (endtime <= starttime) {
        return 0;
    }
    //time difference in milliseconds
    uint64_t timeDiff = endtime - starttime; 

    //conversion factor = 360 because (3600000 milliseconds in an hour / 10000 to convert cm to hm)
    #define CONVERSION_FACTOR 360

    int speed = (DISTANCE_BETWEEN_BUTTONS_CM * CONVERSION_FACTOR) / timeDiff;

    //cap speed to 100 hm/u
    if (speed > 100) {
        speed = 100;
    }
    
    return speed;
}

int main() {
    uint16_t unitCounter = 0; //counter of units that have passed button0 and button1
    uint8_t unitsInProcess = 0; //counter of units that have passed button0 but not yet button1
    uint64_t timeSinceLastTrigger[2] = {millis(), millis()}; //time since last button press
    uint8_t unitSpeed = 69; //the speed of the unit passing the system, calculated by the time between button0 and button1 being pressed
    uint8_t calculatingSpeed = 0; //temporary variable to hold the calculated speed before it is assigned to unitSpeed
    bool buttonPressed[2] = {false, false}; //button states of button0 and button1


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

        //display nothing during the calculation (unitSpeed will be reset at every button0 press)
        if (unitSpeed > 0) {
            displaySpeed(unitSpeed);
        }

        //calculate speed every cycle after the unitspeed has been reset
        //we do this every cycle so we can stop the calculation prematurely if the speed is very low (<=20hm/u)
        if (unitSpeed == 0) {
            //small delay for checking the speed because otherwise the calculateSpeed function will return 0 emediately after button0 is pressed
            if (calculatingSpeed <= 20 && (millis() - timeSinceLastTrigger[0] > 100)) {
                unitSpeed = 20;
            } else {
                calculatingSpeed = calculateSpeed(timeSinceLastTrigger[0], millis());
            }
        }
        
        //check state of button0
        switch (buttonPressed[0]) {
            case true:
                //anti debounce
                if (millis() - timeSinceLastTrigger[0] < ANTI_DEBOUNCE_DELAY) {
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
                    //reset last speed calculation (used to turn off the display while calculating)
                    unitSpeed = 0;

                    buttonPressed[0] = true;
                    timeSinceLastTrigger[0] = millis();
                }
                break;
        }

        //check state of button1
        switch (buttonPressed[1]) {
            case true:
                //anti debounce
                if (millis() - timeSinceLastTrigger[1] < ANTI_DEBOUNCE_DELAY) {
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
                        //complete the unit process
                        unitCounter++;
                        unitsInProcess--;

                        //cast to string is neccesairy to transmit the count
                        char str[2400];
                        sprintf(str, "%u", calculatingSpeed);

                        //transmit unit count per char to serial
                        for (int i = 0; str[i] != '\0'; i++) {
                            USART_Transmit(str[i]);
                        }
                        USART_Transmit('\n');

                        //if the speedcalculation hasnt been prematury been stopped yet, assign the latest calculated speed to unitSpeed
                        if (unitSpeed == 0) {
                            unitSpeed = calculatingSpeed;
                        }

                        buttonPressed[1] = true;
                        timeSinceLastTrigger[1] = millis();
                    }
                }
                break;
        }
    }

    return 0;
}
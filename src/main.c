#include <avr/io.h>
#include <stdbool.h>
#include <stdio.h>
#include <util/delay.h>
#include <Arduino.h>

#define ANTI_DEBOUNCE_DELAY_MS 200 //time in ms between checks if buttons are released to prevent the negative implications of debounce
#define DISPLAY_REFRESH_INTERVAL_MS 5 //time in ms between updates of the 4x7seg display digits to create the illusion that all digits are on at the same time
#define DISTANCE_BETWEEN_BUTTONS_CM 60 //distance in centimeters between the two buttons, used to calculate speed
#define MAX_UNITS_IN_PROCESS 1 //the maximum number of units that can be in process at the same time
#define MIN_SPEED_DM_PER_S 1 //the minimum speed that can be calculated in dm/s, speeds lower then this will be rounded to this value
#define MAX_SPEED_DM_PER_S 28 //the maximum speed that can be calculated in dm/s, speeds higher then this will be rounded to this value
#define MAX_PROCCESSING_TIME_MS 1000 //the maximum time in ms a unit can be in process (between pressing button0 and button1) before it is discarded, to prevent errors

//button enums for the indexes of buttonPressed and timeSinceLastTrigger arrays
enum Buttons {
    BUTTON0,
    BUTTON1
};

struct Button {
    volatile uint8_t* portregister;
    uint8_t portnumber;
    enum Buttons buttonid;
};

typedef struct Button button;

button button0 = {&PIND, PIND2, BUTTON0};
button button1 = {&PIND, PIND3, BUTTON1};

uint16_t unitCounter = 0; //counter of units that have passed button0 and button1
uint8_t unitsInProcess = 0; //counter of units that have passed button0 but not yet button1
uint64_t timeSinceLastTrigger[2] = {0,0}; //time since last button press of button0 and button1
uint64_t startOfDisplayUpdate = 0; //we use this to decide what digit to update on the 4x7seg display
uint64_t timeSinceLastUnitInProcess = 0; //the time when the latest unit started the process by pressing button0
uint8_t unitSpeed = 69; //the speed of the (latest) unit passing the system in hm/u. defaults to 69 so the segmentleds can be tested at startup
uint8_t calculatingSpeed = 0; //updated every cycle while calculating the speed before being assigned to unitSpeed
bool buttonPressed[2] = {false, false}; //button states of button0 and button1

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

//display a number (0-9) on a specific digit of the 4x7seg display. DigitPort is the switchport of the digit to be turned on (PB1, PB2 or PB3)
void displayNumberOnDigit(uint8_t number, uint8_t digitPort, bool decimalPoint) {
    //default to 9 if out of range
    if (number > 9) {
        number = 9;
    }

    //turn off all digits first
    PORTB |= (1 << PB1 | 1 << PB2 | 1 << PB3);

    //turn on digit by turning port off (making it a ground and allowing current to flow through the digit)
    PORTB &= ~(1 << digitPort);

    //we map the segments that need to be turned on for each number in a byte
    //the positions of segments the byte correspond to the following:
    #define SPD4 0 // segment mid-low
    #define SPD5 1 // segment right-high
    #define SPD6 2 // segment right-low
    #define SPD7 3 // segment left-high
    #define SPB0 4 // segment left-low
    #define SPB4 5 // segment mid-mid
    #define SPB5 6 // segment mid-high

    //byte that maps the segments that need to be turned on
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
}

//calculate speed in decimeters per second based on the time difference between starttime and endtime in milliseconds and using the distance between the buttons in cm
uint8_t calculateSpeed(uint64_t starttime, uint64_t endtime) {
    //prevent division by zero and negative speeds
    if (endtime <= starttime) {
        return MAX_SPEED_DM_PER_S;
    }

    //time difference in milliseconds
    uint64_t timeDiff = endtime - starttime; 

    //conversion factor is 100 because timediff is in ms so multiply by 1000 and then divide by 10 because distance is in cm and we want dm/s
    #define CONVERSION_FACTOR 100

    //the reason we put the conversion factor in the numerator is to prevent loss of precision
    uint8_t speed = round((DISTANCE_BETWEEN_BUTTONS_CM * CONVERSION_FACTOR) / timeDiff);

    //cap speed to MAX_SPEED dm/s, if speed is 0 (too fast to measure) we also set it to MAX_SPEED
    if (speed > MAX_SPEED_DM_PER_S || speed == 0) {
        speed = MAX_SPEED_DM_PER_S;
    }
    return speed;
}

//updating the 4x7seg display to show the speed (given in dm/s) in m/s
void displaySpeed(int speed) {
    uint8_t firstDigit = 0;
    uint8_t secondDigit = 0;
    uint8_t thirdDigit = 0;

    if (speed < 10) {
        thirdDigit = speed;
    } else if (speed < 100) {
        secondDigit = speed / 10;
        thirdDigit = speed % 10;
    } else if (speed < 1000) {
        firstDigit = speed / 10;
        secondDigit = speed / 100;
        thirdDigit = speed % 10;
    //we only have 3 digits so values higher than 999 will be displayed as 999
    } else {
        firstDigit = 9;
        secondDigit = 9;
        thirdDigit = 9;
    }

    //what we're doing here is multiplexing
    //show each digit separately for a very short time interval (DISPLAY_REFRESH_INTERVAL_MS) so that it appears that all digits are on at the same time
    if (startOfDisplayUpdate + DISPLAY_REFRESH_INTERVAL_MS > millis()) {
            displayNumberOnDigit(firstDigit, PB1, false);
    } else if (startOfDisplayUpdate + 2 * DISPLAY_REFRESH_INTERVAL_MS > millis()) {
            //display decimal point on middle digit to show dm/s as m/s (eg 100dm/s = 10.0m/s)
            displayNumberOnDigit(secondDigit, PB2, true);
    } else if (startOfDisplayUpdate + 3 * DISPLAY_REFRESH_INTERVAL_MS > millis()) {
            displayNumberOnDigit(thirdDigit, PB3, false);
    //reset the cycle after the last digit
    } else {
            startOfDisplayUpdate = millis();
    }
}

//updating pc0-pc3 (the 4 leds) to display the unit count in binary
void displayCounter(int count) {
    //A ^ (A ^ B) = B
    //1 ^ (1 ^ 1) = 1 ^ 0 = 1 led is on and needs to stay on
    //1 ^ (1 ^ 0) = 1 ^ 1 = 0 led is on and needs to be turned off
    //0 ^ (0 ^ 1) = 0 ^ 1 = 1 led is off and needs to be turned on
    //0 ^ (0 ^ 0) = 0 ^ 0 = 0 led is off and needs to stay off
    PORTC ^= PORTC ^ (count & (1 << PORTC0 | 1 << PORTC1 | 1 << PORTC2 | 1 << PORTC3));
}

//check if a button has been pressed, returns true if a button press is detected. also handles the button state and anti-debounce so should be called every cycle for each button
bool unitPassedButton(button button) {
    //check state of button
    switch (buttonPressed[button.buttonid]) {
        case true:
            //anti debounce
            if (millis() - timeSinceLastTrigger[button.buttonid] < ANTI_DEBOUNCE_DELAY_MS) {
                break;
            }

            //button is being released
            if (PIND & (1 << (button.portnumber))) {
                buttonPressed[button.buttonid] = false;
            }
            //no fall through because the button cant be pressed and released in the same cycle
            break;

        case false:
            //button is being pressed
            if (!(PIND & (1 << (button.portnumber)))) {
                //set button state
                buttonPressed[button.buttonid] = true;
                //record time of button press
                timeSinceLastTrigger[button.buttonid] = millis();
                return true;
            }
            break;

    }
    //if we reach this point, no button press has been detected
    return false;
};

int main() {
    //initialise arduino
    //NOTE: the only reason we need the arduino library is for the millis() function, which is used for timing
    //this is not very memory efficient, so for a future version, one of the main optimations we could do: would be to replace this with clock interrupts
    init();
    //initialize io pins
    initialiseIO();

    //storage for unitPassedButton return values
    bool unitPassedButtonVar[2] = {false, false}; //array to store if a button press has been detected for button0 and button1

    //main loop
    while (true) {
        //check if units have passed button0 and button1
        unitPassedButtonVar[button0.buttonid] = unitPassedButton(button0);
        unitPassedButtonVar[button1.buttonid] = unitPassedButton(button1);

        //button0 handling
        if (unitPassedButtonVar[button0.buttonid]) {
            //not more then MAX_UNITS_IN_PROCESS units in process
            if (unitsInProcess < MAX_UNITS_IN_PROCESS) {
                //reset speed calculation by setting unitSpeed to 0
                unitSpeed = 0;
                //start the calculation at max speed so it can only go down from there
                calculatingSpeed = MAX_SPEED_DM_PER_S;

                //add unit to process
                unitsInProcess++;
                //record time when the unit started the process
                timeSinceLastUnitInProcess = millis();
            }
        }

        //button1 handling
        if (unitPassedButtonVar[button1.buttonid]) {
            //discard if no units are in process
            if (unitsInProcess > 0) {

                //dont count units if they've been in process for more then MAX_PROCCESSING_TIME_MS
                if (millis() - timeSinceLastUnitInProcess < MAX_PROCCESSING_TIME_MS) {
                    unitCounter++;
                }

                //delete unit from process
                unitsInProcess--;

                //however the speed can still be calculated after MAX_PROCCESSING_TIME_MS because that times out when the speed falls below MIN_SPEED

                //if the speedcalculation hasnt been prematury been stopped yet (speed below MIN_SPEED), assign the latest calculated speed to unitSpeed and therby end the calculation
                if (unitSpeed == 0) {
                    unitSpeed = calculatingSpeed;
                }
            }
        }

        //update leds to display the unit count in binary
        displayCounter(unitCounter);

        //calculate speed while a unit is in process and the unitspeed has been reset
        if (unitSpeed == 0) {
             //end speed calculation prematurely if the calculatingSpeed is already below MIN_SPEED
            if (calculatingSpeed <= MIN_SPEED_DM_PER_S) {
                unitSpeed = MIN_SPEED_DM_PER_S;
            } else {
                //we calculate the speed every cycle so we can stop the calculation prematurely if the speed is very low (<=MIN_SPEED hm/u)
                calculatingSpeed = calculateSpeed(timeSinceLastUnitInProcess, millis());
            }
        }

        //update segmentsdisplay to display the speed
        //display nothing during the calculation (unitSpeed will be reset at button0 press)
        if (unitSpeed > 0) {
            displaySpeed(unitSpeed);
        } else {
            PORTB |= (1 << PORTB1 | 1 << PORTB2 | 1 << PORTB3);
        }
    }


    return 0;
}
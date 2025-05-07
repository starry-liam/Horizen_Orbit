#ifndef BUTLED_HPP
#define BUTLED_HPP

#include <Arduino.h>
#include "constants.hpp"

void ledIndicatorSetup() {
    pinMode(Constants::Pins::led, OUTPUT);
}

class LED {
    private:
        int digits;    
    public:
        void splitInt(int num) {
            for (int d = num; d > 0; digits++) {
                d /= 10;
            }
            int intArray[digits];
            int intRevArray[digits];
            for (int i = 0; i < digits; i++) {
                intArray[i] = num % 10;
                num /= 10;
            }
            int dg = digits;
            for (int i = -1; i < digits; i++) {
                
                intRevArray[i] = intArray[dg];
                dg--;

            }
        }
        void pulse () {
            
        }
        void on() {
            digitalWrite(Constants::Pins::led, HIGH);
        }
        void off() {
            digitalWrite(Constants::Pins::led, LOW);
        }
        void blink(int times, int delayTime) {
            for (int i = 0; i < times; i++) {
                digitalWrite(Constants::Pins::led, HIGH);
                delay(delayTime);
                digitalWrite(Constants::Pins::led, LOW);
                delay(delayTime);
            }
        }
        
};

class Button {
    private:
        bool userButtonState = false;
        bool ctrlButtonState = false;
        int buttonPressed = 0;

    public:
        //get the state of the user button if button is pressed and value is above threshold returns true else false
        bool getUserButtonState() {
            int userButtonValue = analogRead(Constants::Pins::userPin);
            if (userButtonValue > Constants::thresh::buttonThreshold) {
                userButtonState = true;
            } else {
                userButtonState = false;
            }
            return userButtonState;
        }
        //get the state of the control button if button is pressed and value is above threshold returns true else false
        bool getCtrlButtonState() {
            int ctrlButtonValue = analogRead(Constants::Pins::ctrlPin);
            if (ctrlButtonValue > Constants::thresh::buttonThreshold) {
                ctrlButtonState = true;
            } else {
                ctrlButtonState = false;
            }
            return ctrlButtonState;
        }
        int getButtonspressed() {
            // Check the state of the buttons and update buttonPressed accordingly
            // If user button is pressed, return 1
            // If ctrl button is pressed, return 2
            // If both buttons are pressed, return 3
            // If neither button is pressed, return 0
            if (getUserButtonState()) {
                buttonPressed = 1;
            }
            if (getCtrlButtonState()) {
                buttonPressed = 2;
            }
            if (getUserButtonState() && getCtrlButtonState()) {
                buttonPressed = 3;
            } else {
                buttonPressed = 0;
            }
            return buttonPressed;
        }
        
};


#endif // BUTLED_HPP
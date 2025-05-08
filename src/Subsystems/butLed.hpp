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
        int Timedelay;
        int blinkTimes;
        int currentTimes;
        int time;
        int timePassed;
        int timeStart;
        bool done = true;

        

    public:
        void ledRun(int times, int delayTime) {
            if (times == 0) {
                if (done) {
                    off();
                    Serial.println("no input");
                }
                return; // Exit the function if no blinking is required
            }

            if (done) {
                // Initialize blinking process
                done = false;
                Timedelay = delayTime;
                blinkTimes = times;
                currentTimes = 0;
                timeStart = millis();
                Serial.println("input received");
                Serial.print("Blinking LED for ");
                Serial.print(blinkTimes);
                Serial.println(" times");
            }

            unsigned long currentTime = millis(); // Use unsigned long for time calculations

            if (!done && currentTimes < blinkTimes) {
                if (currentTime - timeStart < Timedelay / 2) {
                    on(); // Turn LED on for the first half of the delay
                    Serial.println("LED ON");
                } else if (currentTime - timeStart < Timedelay) {
                    off(); // Turn LED off for the second half of the delay
                    Serial.println("LED OFF");
                } else {
                    currentTimes++; // Increment blink count after one full cycle
                    timeStart = millis(); // Reset the start time for the next blink
                }
            }

            if (currentTimes >= blinkTimes) {
                // Blinking process complete
                done = true;
                off();
                Serial.println("Blinking complete");
            }
        }
    
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

        void on() {
            digitalWrite(Constants::Pins::led, HIGH);
        }
        void off() {
            digitalWrite(Constants::Pins::led, LOW);
        }
        void blink(int times, int delayTime) {
            
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
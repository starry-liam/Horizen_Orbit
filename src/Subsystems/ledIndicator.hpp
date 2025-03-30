#ifndef LEDINDICATOR_HPP
#define LEDINDICATOR_HPP

#include <Arduino.h>
#include "constants.hpp"

void ledIndicatorSetup() {
    pinMode(Constants::Pins::led, OUTPUT);
    digitalWrite(Constants::Pins::led, LOW);
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
        
};

#endif // LEDINDICATOR_HPP
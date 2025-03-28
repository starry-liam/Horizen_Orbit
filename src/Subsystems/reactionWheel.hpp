#ifndef REACTIONWHEEL_HPP
#define REACTIONWHEEL_HPP

#include <Arduino.h>
#include "constants.hpp"
#include "Subsystems/Sensor/gyroAccel.hpp"
#include "Subsystems/Sensor/altimeter.hpp"

void reactionWheelSetup() {
    pinMode(reactionWheelFwd, OUTPUT);
    pinMode(reactionWheelRev, OUTPUT);
    digitalWrite(reactionWheelFwd, LOW);
    digitalWrite(reactionWheelRev, LOW);
}

class Reaction {
    private:
        int reactionWheelSpd;
    public:

        void moveFwd (float speed) {
            reactionWheelSpd = map(speed, 0, 100, 0, 255);
            analogWrite(reactionWheelFwd, reactionWheelSpd);
            digitalWrite(reactionWheelRev, LOW);
        }
        
        void moveRev (float speed) {
            reactionWheelSpd = map(speed, 0, 100, 0, 255);
            analogWrite(reactionWheelRev, reactionWheelSpd);
            digitalWrite(reactionWheelFwd, LOW);
        }

};

#endif // REACTIONWHEEL_HPP
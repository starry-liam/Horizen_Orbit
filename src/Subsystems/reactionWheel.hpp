#ifndef REACTIONWHEEL_HPP
#define REACTIONWHEEL_HPP

#include <Arduino.h>
#include "constants.hpp"
#include "Subsystems/Sensor/gyroAccel.hpp"
#include "Subsystems/Sensor/altimeter.hpp"

void reactionWheelSetup() {
    pinMode(Constants::Pins::reactionWheelRev, OUTPUT);
    pinMode(Constants::Pins::reactionWheelFwd, OUTPUT);
    digitalWrite(Constants::Pins::reactionWheelFwd, LOW);
    digitalWrite(Constants::Pins::reactionWheelRev, LOW);
}

class Reaction {
    private:
        int reactionWheelSpd;
        double prop = 0.0;
        double inte = 0.0;
        double deri = 0.0;
        double kp = Constants::reactionPID::kp; // proportional gain
        double ki = Constants::reactionPID::ki; // integral gain
        double kd = Constants::reactionPID::kd; // derivative gain
        double prevError = 0.0;
        double error = 0.0;
        double curentAng = 0.0;
        double dt = 0.0;
        double prevTime = 0.0;
        double currentTime = 0.0;
        double output = 0.0;
    public:

        void moveFwd (float speed) {
            reactionWheelSpd = map(speed, 0, 180, 0, 255);
            analogWrite(Constants::Pins::reactionWheelFwd, reactionWheelSpd);
            digitalWrite(Constants::Pins::reactionWheelRev, LOW);
        }
        
        void moveRev (float speed) {
            reactionWheelSpd = map(speed, 0, 180, 0, 255);
            analogWrite(Constants::Pins::reactionWheelRev, reactionWheelSpd);
            digitalWrite(Constants::Pins::reactionWheelFwd, LOW);
        }

        void start () {

        }

        float gotoPos (float targetAng, float input) {
            currentTime = millis();
            dt = (currentTime - prevTime) / 1000;
            prevTime = currentTime;

            curentAng = input;
            error = targetAng - curentAng;
            prop = error;
            inte += error * dt;
            deri = (error - prevError) / dt;
            prevError = error;
            output = (kp * prop) + (ki * inte) + (kd * deri);
            if (output > 0) {
                moveFwd(abs(output));
            }
            else if (output < 0) {
                moveRev(abs(output));
            }
            else {
                digitalWrite(Constants::Pins::reactionWheelFwd, LOW);
                digitalWrite(Constants::Pins::reactionWheelRev, LOW);
            }
            return output;
        }

};

#endif // REACTIONWHEEL_HPP
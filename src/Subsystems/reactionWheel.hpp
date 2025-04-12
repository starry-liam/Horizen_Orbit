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
        bool hasBumped = false; // Flag to track if bump has been performed
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
        void moveFwd(float speed) {
            Serial.print("moveFwd: ");
            Serial.println(speed);
            analogWrite(Constants::Pins::reactionWheelFwd, map(speed, 0, 180, 50, 255));
            analogWrite(Constants::Pins::reactionWheelRev, LOW);
        }

        void moveRev(float speed) {
            Serial.print("moveRev: ");
            Serial.println(-speed);
            analogWrite(Constants::Pins::reactionWheelRev, map(speed, 0, 180, 50, 255));
            analogWrite(Constants::Pins::reactionWheelFwd, LOW);
        }

        void bump(float speed, bool forward) {
            // Send a quick high-speed pulse
            if (forward) {
                moveFwd(180); // Full speed forward
                delay(50);    // Short pulse duration
                moveFwd(speed); // Reduce to desired speed
            } else {
                moveRev(180); // Full speed reverse
                delay(50);    // Short pulse duration
                moveRev(speed); // Reduce to desired speed
            }
        }

        void start() {
            // Placeholder for additional initialization logic
        }

        float pid(float targetAng, float input) {
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
            return output;
        }

        void gotoPos(float speed) {
            if (speed < 0) {
                if (abs(speed) < 50) {
                    if (!hasBumped) {
                        bump(abs(speed), false);  // Use bump for reverse direction
                        hasBumped = true;         // Set flag to true after bump
                    } else {
                        moveRev(abs(speed));      // Continue moving at low speed
                    }
                } else {
                    moveRev(abs(speed));          // Move in reverse
                    hasBumped = false;           // Reset flag for next bump
                }
            } else if (speed > 0) {
                if (speed < 50) {
                    if (!hasBumped) {
                        bump(speed, true);        // Use bump for forward direction
                        hasBumped = true;         // Set flag to true after bump
                    } else {
                        moveFwd(speed);           // Continue moving at low speed
                    }
                } else {
                    moveFwd(speed);               // Move forward
                    hasBumped = false;           // Reset flag for next bump
                }
            } else {
                moveFwd(0);                       // Stop forward movement
                moveRev(0);                       // Stop reverse movement
                hasBumped = false;               // Reset flag for next bump
            }
        }
};

#endif // REACTIONWHEEL_HPP
#ifndef VOLTAGE_HPP
#define VOLTAGE_HPP

#include <Arduino.h>
#include "constants.hpp"

class Volt {
    private:
        float voltage;
        float voltageSum;
        float voltageReadings[Constants::avgReadings::VoltAvgReadings];

    public:
        float getVoltage() {
         voltage = analogRead(Constants::Pins::voltChecker); // Read the analog voltage value
            voltage = voltage * (3.3 / 1023.0);
            for (int i; i < Constants::avgReadings::VoltAvgReadings; i++) {
                voltageReadings[i] = voltage;
                voltageSum += voltageReadings[i];
            }
            voltage = voltageSum / Constants::avgReadings::VoltAvgReadings; // Calculate the average voltage
            return voltage;
        }
};

#endif // VOLTAGE_HPP
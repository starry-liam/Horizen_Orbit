#ifndef VOLTAGE_HPP
#define VOLTAGE_HPP

#include <Arduino.h>
#include "constants.hpp"

class Volt {
    private:
        float voltage;
        float voltageSum;
        float voltageReadings[VoltAvgReadings];

    public:
        float getVoltage() {
         voltage = analogRead(voltChecker);
            voltage = voltage * (3.3 / 1023.0);
            for (int i; i < VoltAvgReadings; i++) {
                voltageReadings[i] = voltage;
                voltageSum += voltageReadings[i];
            }
            voltage = voltageSum / VoltAvgReadings;
            return voltage;
        }
};

#endif // VOLTAGE_HPP
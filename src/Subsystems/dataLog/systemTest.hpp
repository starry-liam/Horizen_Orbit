#ifndef SERIALPRINT_HPP
#define SERIALPRINT_HPP

#include <Arduino.h>
#include "Subsystems/Sensor/gyroAccel.hpp"
#include "Subsystems/Sensor/altimeter.hpp"

#include "Subsystems/dataLog/eeprom.hpp"
#include "Subsystems/dataLog/sd.hpp"
#include "Subsystems/voltage.hpp"

void systemTest() {
    BARO alt;
    LED led;
    Button button;
    GYRO GYROAC;
    EEPROM eeprom;
    pinMode(Constants::Pins::led, OUTPUT); // Set LED pin as output
    Serial.println("System Test:");

    
    // Test Gyro and Accelerometer
    GYROAC.getAccel();
    GYROAC.getGyro();
    GYROAC.updateAngles();
    Serial.print("Gyro X: ");
    Serial.print(GYROAC.getGyroX());
    Serial.print(" Gyro Y: ");
    Serial.print(GYROAC.getGyroY());
    Serial.print(" Gyro Z: ");
    Serial.println(GYROAC.getGyroZ());

    Serial.print("Accel X: ");
    Serial.print(GYROAC.getAccelX());
    Serial.print(" Accel Y: ");
    Serial.print(GYROAC.getAccelY());
    Serial.print(" Accel Z: ");
    Serial.println(GYROAC.getAccelZ());

    Serial.print("Angle X: ");
    Serial.print(GYROAC.getAngleX());
    Serial.print(" Angle Y: ");
    Serial.print(GYROAC.getAngleY());
    Serial.print(" Angle Z: ");
    Serial.println(GYROAC.getAngleZ());

    // Test Altimeter
    alt.getAltitude(); // Update altitude reading
    Serial.print(" Altitude: ");
    Serial.print(alt.getAltitude());
    alt.getTemp(); // Update temperature reading
    Serial.print(" Temperature: ");
    Serial.print(alt.getTemp());
    Serial.print(" °C, Pressure: ");
    Serial.print(alt.getPressure());

    // Test Voltage
    Volt voltage;
    Serial.print(" Voltage: ");
    Serial.println(voltage.getVoltage());

    // Test EEPROM and SD (if applicable)
    eeprom.test(alt.getTemp());
    //sdTest();

    //button/led test
    button.getButtonspressed();
    if (button.getButtonspressed() == 1) {
        Serial.println("User button pressed!");
        led.blink(1, 100); // Blink LED
    } else if (button.getButtonspressed() == 2) {
        Serial.println("Control button pressed!");
        led.blink(2, 100); // Blink LED 5 times
    } else if (button.getButtonspressed() == 3) {
        Serial.println("Both buttons pressed!");
        led.blink(3, 100); // Blink LED 7 times
    } else {
        Serial.println("No button pressed.");
        led.off(); // Turn off LED
    }
    
}

#endif // SERIALPRINT_HPP
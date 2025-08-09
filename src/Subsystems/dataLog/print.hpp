#ifndef SERIALPRINT_HPP
#define SERIALPRINT_HPP

#include <Arduino.h>
#include "Subsystems/Sensor/gyroAccel.hpp"
#include "Subsystems/Sensor/altimeter.hpp"

#include "Subsystems/butLed.hpp"

#include "Subsystems/dataLog/eeprom.hpp"
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
    GYROAC.update();
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

}
void comma(){
    Serial.print(",");
}
void printPacket(){
    BARO alt;
    LED led;
    Button button;
    GYRO GYROAC;
    EEPROM eeprom;
    Volt voltage;
    pinMode(Constants::Pins::led, OUTPUT); // Set LED pin as output

    GYROAC.update();
    GYROAC.updateAngles();

    //Serial.print(time);
    //comma();
    Serial.print(alt.getTemp());
    comma();
    Serial.print(alt.getPressure());
    comma();
    Serial.print(alt.getAltitude());
    comma();
    Serial.print(GYROAC.getGyroX());
    comma();
    Serial.print(GYROAC.getGyroY());
    comma();
    Serial.print(GYROAC.getGyroZ());
    comma();
    Serial.print(GYROAC.getAccelX());
    comma();
    Serial.print(GYROAC.getAccelY());
    comma();
    Serial.print(GYROAC.getAccelZ());
    comma();
    Serial.print(GYROAC.getAngleX());
    comma();
    Serial.print(GYROAC.getAngleY());
    comma();
    Serial.print(GYROAC.getAngleZ());
    comma();
    Serial.print(voltage.getVoltage());
    comma();
    Serial.println(button.getButtonspressed());

}

#endif // SERIALPRINT_HPP
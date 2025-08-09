//Library includes
#include <Arduino.h>
#include "constants.hpp"
//subsystems includes
#include "Subsystems/voltage.hpp"
#include "Subsystems/reactionWheel.hpp"
#include "Subsystems/butLed.hpp"
#include "Subsystems/dataLog/eeprom.hpp"
#include "Subsystems/dataLog/print.hpp"
#include "Subsystems/Sensor/altimeter.hpp"
#include "Subsystems/Sensor/gyroAccel.hpp"
//function includes
#include "Functions/writeData.hpp"




//initialize class entities
GYRO GYROAC;
BARO alt;
LED led;
Reaction REACTION;

void setup() { //main setup function
   
   //set baud rates
   Serial.begin(Constants::Serial::serial_baud); //main serial port for debugging
   Serial1.begin(Constants::Serial::serial1_baud); //serial port for data logging or other purposes
    
    //initialize subsystems
    altimeterSetup();
    alt.altZero(); //set the altitude zero point
    GYROAC.gyroAccelSetup();
    GYROAC.getOffsets();
    //initialize class entities
}
void loop() { //main loop
    printPacket();
}
//Library includes
#include <Arduino.h>
#include "constants.hpp"
//subsystems includes
#include "Subsystems/voltage.hpp"
#include "Subsystems/reactionWheel.hpp"
#include "Subsystems/ledIndicator.hpp"
#include "Subsystems/dataLog/eeprom.hpp"
#include "Subsystems/dataLog/sd.hpp"
#include "Subsystems/Camera.hpp"
#include "Subsystems/Sensor/altimeter.hpp"
#include "Subsystems/Sensor/gyroAccel.hpp"
//function includes
#include "Functions/setReactionpos.hpp"
#include "Functions/writeData.hpp"




void setup() { //main setup function
   
   //set baud rates
   Serial.begin(serial_baud);
   Serial1.begin(serial1_baud);
    
    //initialize subsystems
    altimeterSetup();
    gyroAccelSetup();
    reactionWheelSetup();
    

    //initialize class entities
    BARO alt;
    GYRO gyro;
    Reaction react;
    Volt voltmeter;
}

void loop() { //main loop

}
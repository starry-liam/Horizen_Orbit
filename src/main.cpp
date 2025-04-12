//Library includes
#include <Arduino.h>
#include "constants.hpp"
//subsystems includes
#include "Subsystems/voltage.hpp"
#include "Subsystems/reactionWheel.hpp"
#include "Subsystems/ledIndicator.hpp"
#include "Subsystems/dataLog/eeprom.hpp"
#include "Subsystems/dataLog/sd.hpp"
#include "Subsystems/dataLog/systemTest.hpp"
#include "Subsystems/Camera.hpp"
#include "Subsystems/Sensor/altimeter.hpp"
#include "Subsystems/Sensor/gyroAccel.hpp"
//function includes
#include "Functions/writeData.hpp"




//initialize class entities
GYRO GYROAC;
BARO alt;
Reaction REACTION;
EEPROM eeprom;

void setup() { //main setup function
   
   //set baud rates
   Serial.begin(Constants::Serial::serial_baud); //main serial port for debugging
   Serial1.begin(Constants::Serial::serial1_baud); //serial port for data logging or other purposes
    
    //initialize subsystems
    altimeterSetup();
    gyroAccelSetup();
    GYROAC.getOffsets();
    //initialize class entities
}
void loop() { //main loop
    
    GYROAC.updateAngles();
    GYROAC.getGyro();
    //Serial.print(GYROAC.getAngleZ());
    //Serial.print(", ");
    float output = REACTION.pid(0, GYROAC.getAngleZ());
    REACTION.gotoPos(output);
    //Serial.println(output);
    delay(50);

    //  REACTION.moveFwd(180);
    //  delay(5000);
    //  REACTION.moveFwd(90);
    //  delay(5000);
    //  REACTION.moveFwd(0);
    //  delay(5000);
    //  REACTION.moveRev(90);
    //  delay(5000);
    //  REACTION.moveRev(180);
    //  delay(5000);
    //  REACTION.moveRev(90);
    //  delay(5000);
    //  REACTION.moveRev(0);
    //  delay(5000);
    //  REACTION.moveFwd(90);
    

}
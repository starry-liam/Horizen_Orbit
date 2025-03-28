#include <Arduino.h>
#include "constants.hpp"
//subsystems
#include "Subsystems/voltage.hpp"
#include "Subsystems/reactionWheel.hpp"
#include "Subsystems/ledIndicator.hpp"
#include "Subsystems/dataLog/eeprom.hpp"
#include "Subsystems/dataLog/sd.hpp"
#include "Subsystems/Camera.hpp"
#include "Subsystems/Sensor/altimeter.hpp"
#include "Subsystems/Sensor/gyroAccel.hpp"


void setup() {
   Serial.begin(serial_baud);
   Serial1.begin(serial1_baud);
    altimeterSetup();
    



}

void loop() {

}
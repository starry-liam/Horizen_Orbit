
#ifndef ALTIMETER_HPP
#define ALTIMETER_HPP

#include <Arduino.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include "constants.hpp"

Adafruit_BMP3XX bmp;

void altimeterSetup() {
  if (!bmp.begin_I2C()) {
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}

class BARO {
    private:
        float altitude;
        float altitudeReadings[Constants::avgReadings::AltAvgReadings];
        float altitudeSum;
    
    public:
        float getAltitude() {
            for (int i; i < Constants::avgReadings::AltAvgReadings; i++) {
                altitudeReadings[i] = bmp.readAltitude(Constants::altimeter::seaLevelPressure); // Read altitude in meters
                altitudeSum += altitudeReadings[i];
            }
            altitude = altitudeSum / Constants::avgReadings::AltAvgReadings; // Calculate the average altitude
            return altitude;

        }

        float getTemp() {
            return bmp.readTemperature();
        }

        float getPressure() {
            return bmp.readPressure();
        }


};

#endif // ALTIMETER_HPP

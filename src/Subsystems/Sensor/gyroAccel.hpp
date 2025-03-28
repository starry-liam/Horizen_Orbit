#ifndef GYROACCEL_HPP
#define GYROACCEL_HPP

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>

#include "constants.hpp"

Adafruit_MPU6050 mpu;

void gyroAccelSetup() {
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1);
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    delay(100);

        
}

class GYRO {
    private:
        float accelX;
        float accelY;
        float accelZ;
        float gyroX;
        float gyroY;
        float gyroZ;
    
    public:
        void getAccel() {
            sensors_event_t accel;
            mpu.getAccelerometerSensor()->getEvent(&accel);
            accelX = accel.acceleration.x;
            accelY = accel.acceleration.y;
            accelZ = accel.acceleration.z;
        }

        void getGyro() {
            sensors_event_t gyro;
            mpu.getGyroSensor()->getEvent(&gyro);
            gyroX = gyro.gyro.x;
            gyroY = gyro.gyro.y;
            gyroZ = gyro.gyro.z;
        }

        float getAccelX() {
            return accelX;
        }

        float getAccelY() {
            return accelY;
        }

        float getAccelZ() {
            return accelZ;
        }

        float getGyroX() {
            return gyroX;
        }

        float getGyroY() {
            return gyroY;
        }

        float getGyroZ() {
            return gyroZ;
        }
};
#endif // GYROACCEL_HPP
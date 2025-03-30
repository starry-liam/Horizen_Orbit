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
        float gyroXOffset = 0.0;
        float gyroYOffset = 0.0;
        float gyroZOffset = 0.0;

        float angleX = 0.0;
        float angleY = 0.0;
        float angleZ = 0.0;

        unsigned long lastUpdateTime = 0;

    public:
        
        void getOffsets() {
            gyroXOffset = getGyroX();
            gyroYOffset = getGyroY();
            gyroZOffset = getGyroZ();
        }
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
            updateAngles();
            gyroX = gyro.gyro.x - gyroXOffset;
            gyroY = gyro.gyro.y - gyroYOffset;
            gyroZ = gyro.gyro.z - gyroZOffset;
        }

        void updateAngles() {
            unsigned long currentTime = millis();
            float deltaTime = (currentTime - lastUpdateTime) / 1000.0; // Convert to seconds

            if (lastUpdateTime != 0) {
                angleX += gyroX * deltaTime * (180.0 / PI); // Convert to degrees
                angleY += gyroY * deltaTime * (180.0 / PI); // Convert to degrees
                angleZ += gyroZ * deltaTime * (180.0 / PI); // Convert to degrees

                // Constrain angles to -180 to 180 degrees
                angleX = constrainAngle(angleX);
                angleY = constrainAngle(angleY);
                angleZ = constrainAngle(angleZ);
            }

            lastUpdateTime = currentTime;
        }

    private:
        float constrainAngle(float angle) {
            // Constrain angle to -180 to 180 degrees
            while (angle > 180.0) angle -= 360.0;
            while (angle < -180.0) angle += 360.0;
            return angle;
        }

    public:
        float getAngleX() {
            return angleX;
        }

        float getAngleY() {
            return angleY;
        }

        float getAngleZ() {
            return angleZ;
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
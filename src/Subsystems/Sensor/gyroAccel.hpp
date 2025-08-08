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
        float accelX = 0.0;
        float accelY = 0.0;
        float accelZ = 0.0;
        float gyroX = 0.0;
        float gyroY = 0.0;
        float gyroZ = 0.0;
        float gyroXOffset = 0.0;
        float gyroYOffset = 0.0;
        float gyroZOffset = 0.0;

        float angleX = 0.0;
        float angleY = 0.0;
        float angleZ = 0.0;

        unsigned long lastUpdateTime = 0;
        float constrainAngle(float angle) {
            // Constrain angle to -180 to 180 degrees
            while (angle > 180.0) angle -= 360.0;
            while (angle < -180.0) angle += 360.0;
            return angle;
        }

        float lowPassFilter(float currentValue, float previousValue, float alpha) {
            return alpha * currentValue + (1 - alpha) * previousValue;
        }
    public:
        void getOffsets() {
            float sumX = 0, sumY = 0, sumZ = 0;
            const int numSamples = 100;

            for (int i = 0; i < numSamples; i++) {
                sensors_event_t gyro;
                mpu.getGyroSensor()->getEvent(&gyro);
                sumX += gyro.gyro.x;
                sumY += gyro.gyro.y;
                sumZ += gyro.gyro.z;
                delay(10); // Small delay between samples
            }

            gyroXOffset = sumX / numSamples;
            gyroYOffset = sumY / numSamples;
            gyroZOffset = sumZ / numSamples;
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

            // Apply low-pass filter to reduce noise
            gyroX = lowPassFilter(gyro.gyro.x - gyroXOffset, gyroX, 0.9);
            gyroY = lowPassFilter(gyro.gyro.y - gyroYOffset, gyroY, 0.9);
            gyroZ = lowPassFilter(gyro.gyro.z - gyroZOffset, gyroZ, 0.9);

            updateAngles();
        }

        void updateAngles() {
            unsigned long currentTime = millis();
            float deltaTime = (currentTime - lastUpdateTime) * 0.001f; // ms → seconds

            if (lastUpdateTime != 0) {
                // Convert gyro readings to degrees/sec
                const float radToDeg = 180.0f / PI;
                angleX += gyroX * deltaTime * radToDeg;
                angleY += gyroY * deltaTime * radToDeg;
                angleZ += gyroZ * deltaTime * radToDeg;

                // Calculate accelerometer-based angles
                float accelAngleX = atan2(accelY, accelZ) * radToDeg;
                float accelAngleY = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * radToDeg;

                // Apply complementary filter (gyro = high-pass, accel = low-pass)
                const float alpha = 0.98f;
                angleX = alpha * angleX + (1.0f - alpha) * accelAngleX;
                angleY = alpha * angleY + (1.0f - alpha) * accelAngleY;

                // Keep angles within -180° to +180°
                angleX = constrainAngle(angleX);
                angleY = constrainAngle(angleY);
                angleZ = constrainAngle(angleZ);
            }

            lastUpdateTime = currentTime;
        }
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
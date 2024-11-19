//
// Created by Robbie on 11/13/24.
//

#ifndef JOYSTICKIMUDEMO_IMUFUNCTIONS_H
#define JOYSTICKIMUDEMO_IMUFUNCTIONS_H

#include <Arduino.h>
#include <Adafruit_ICM20948.h>

/**
 * Initializes the IMU with the given parameters
 * @param icm - An instance of the Adafruit_ICM20948 class for the IMU
 * @param accelRang - The range of the accelerometer using the icm20948_accel_range_t enum
 * @param gyroRang  - The range of the gyroscope using the icm20948_gyro_range_t enum
 * @param magDataRate - The data rate of the magnetometer using the ak09916_data_rate_t enum
 */
void imuInit(Adafruit_ICM20948 &icm, icm20948_accel_range_t accelRang, icm20948_gyro_range_t gyroRang, ak09916_data_rate_t magDataRate);

/**
 * Prints the IMU data to the serial monitor
 * @param icm - An instance of the Adafruit_ICM20948 class for the IMU
 */
void printImuData(Adafruit_ICM20948 &icm);

#endif //JOYSTICKIMUDEMO_IMUFUNCTIONS_H

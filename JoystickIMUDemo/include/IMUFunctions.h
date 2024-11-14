//
// Created by Robbie on 11/13/24.
//

#ifndef JOYSTICKIMUDEMO_IMUFUNCTIONS_H
#define JOYSTICKIMUDEMO_IMUFUNCTIONS_H

#include <Arduino.h>
#include <Adafruit_ICM20948.h>

void imuInit(Adafruit_ICM20948 &icm, icm20948_accel_range_t accelRang, icm20948_gyro_range_t gyroRang, ak09916_data_rate_t magDataRate);
void printImuData(Adafruit_ICM20948 &icm);

#endif //JOYSTICKIMUDEMO_IMUFUNCTIONS_H


#include <Adafruit_ICM20X.h>
//#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
//#include "IMUFunctions.h"
#include "ADCFunctions.h"
#include "JoystickFunctions.h"

Adafruit_ADS1115 adc;	// Construct an ads1115
//Adafruit_ICM20948 icm;
//uint16_t measurement_delay_us = 65535; // Delay between measurements for testing

void setup(void) {
    Serial.begin(115200);
    // while (!Serial) {
    //     delay(10); // will pause Zero, Leonardo, etc until serial console opens
    // }

    delay(2000); // Give a second to open the Serial Monitor
    Serial.println("Starting joystick demo...");
    //imuInit(icm, ICM20948_ACCEL_RANGE_2_G, ICM20948_GYRO_RANGE_250_DPS, AK09916_MAG_DATARATE_10_HZ);

    //adcInit(adc, 0x48); //default address

}

void loop() {
    // RefSpeed joystickValues = joystickToSpeed(adc);
    //
    // Serial.println("Right Speed: " + String(joystickValues.rightSpeed));
    // Serial.println("Left Speed: " + String(joystickValues.leftSpeed));
    // delay(100);

    Serial.println("Test print");

    // IMUData imuData{};
    // //printImuData(icm);
    // //printADC(adc);
    // imuData = getIMUData(icm);
    // Serial.print("Accel X: ");
    // Serial.print(imuData.accel_x);
    // Serial.print(" \tY: ");
    // Serial.print(imuData.accel_y);
    // Serial.print(" \tZ: ");
    // Serial.println(imuData.accel_z);
    //
    // Serial.print("Gyro X: ");
    // Serial.print(imuData.gyro_x);
    // Serial.print(" \tY: ");
    // Serial.print(imuData.gyro_y);
    // Serial.print(" \tZ: ");
    // Serial.println(imuData.gyro_z);
    //
    // Serial.print("Mag X: ");
    // Serial.print(imuData.mag_x);
    // Serial.print(" \tY: ");
    // Serial.print(imuData.mag_y);
    // Serial.print(" \tZ: ");
    // Serial.println(imuData.mag_z);
    // delay(100);
}

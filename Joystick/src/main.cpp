#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_ICM20948.h>
#include "ADCFunctions.h"
#include "JoystickFunctions.h"
#include "UltrasonicFunctions.h"
#include "IMUFunctions.h"
#include "PWMFunctions.h"

#ifdef ROS
#include <microRosFunctions.h>

#endif

Adafruit_ADS1115 joystickAdc;	// Construct an ads1115
Adafruit_ICM20948 icm;

uint8_t dutyCycle0 = 25;


void setup(void) {
    Serial.begin(115200);

    #ifdef ROS
        const char* nodeName = "sensors_node";
        const char* topicName = "sensors";
        microRosSetup(1, nodeName, topicName);
    #endif

    while (!Serial) {
        delay(10);
    }

    adcInit(joystickAdc, 0x48); //default address

    setPWM(PWM_PIN0, 25000, dutyCycle0);

}

void loop() {
    IMUData imuData = getIMUData(icm);
    refSpeed omegaRef = joystickToSpeed(joystickAdc);
    uint16_t usDistance = ultrasonicDistance(joystickAdc, 2);

    Serial.print("Right Speed: ");
    Serial.println(omegaRef.rightSpeed);
    Serial.print("Left Speed: ");
    Serial.println(omegaRef.leftSpeed);
    Serial.print("Ultrasonic Distance: ");
    Serial.println(usDistance);

    Serial.print("Accel X: ");
    Serial.print(imuData.accel_x);
    Serial.print(" \tY: ");
    Serial.print(imuData.accel_y);
    Serial.print(" \tZ: ");
    Serial.println(imuData.accel_z);

    Serial.print("Gyro X: ");
    Serial.print(imuData.gyro_x);
    Serial.print(" \tY: ");
    Serial.print(imuData.gyro_y);
    Serial.print(" \tZ: ");
    Serial.println(imuData.gyro_z);

    Serial.print("Mag X: ");
    Serial.print(imuData.mag_x);
    Serial.print(" \tY: ");
    Serial.print(imuData.mag_y);
    Serial.print(" \tZ: ");
    Serial.println(imuData.mag_z);

    #ifdef ROS
        transmitMsg(omegaRef);

    #elif DEBUG

    #endif

}
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_ICM20948.h>
#include "ADCFunctions.h"
#include "JoystickFunctions.h"
#include "UltrasonicFunctions.h"
#include "FingerprintFunctions.h"
#include "PIRFunctions.h"
#include "IMUFunctions.h"
#include "FanFunctions.h"



#if defined(ROS) || defined(ROS_DEBUG)
#include <microRosFunctions.h>
#endif

Adafruit_ADS1115 joystickAdc;
Adafruit_ADS1115 ultrasonicAdc;// Construct an ads1115
Adafruit_ICM20948 icm;

uint8_t dutyCycle0 = 25;

void setup(void) {
    Serial.begin(115200);

    #ifdef ROS

    const char* nodeName = "sensors_node";
    const char* sensorTopicName = "sensors";
    const char* fingerprintTopicName = "fingerprint";
    microRosSetup(1, nodeName, sensorTopicName, fingerprintTopicName);

    #elif ROS_DEBUG

    const char* nodeName = "sensors_node";
    const char* topicName = "refSpeed";
    microRosSetup(1, nodeName, topicName);
    #endif

    while (!Serial) {
        delay(10);
    }

    adcInit(joystickAdc, 0x48); //default address
    imuInit(icm, ICM20948_ACCEL_RANGE_2_G, ICM20948_GYRO_RANGE_250_DPS, AK09916_MAG_DATARATE_10_HZ);
    setupFingerprint();
    setFanIndividual(FAN_0, dutyCycle0);
}

void loop() {
    RefSpeed omegaRef = joystickToSpeed(joystickAdc);
    //uint16_t usDistance = ultrasonicDistance(joystickAdc, 2);
    USData usDistances = allUltrasonicDistance(joystickAdc, ultrasonicAdc);
    PIRSensors pirSensors = readAllPIR();
    uint8_t fingerID = getFingerprintID();
    IMUData imuData = getIMUData(icm);
    FanSpeeds fanSpeeds = getAllFanSpeeds();


    #ifdef ROS

    transmitMsg(omegaRef, usDistances, pirSensors, fanSpeeds, imuData);

    if(fingerID != 2){
        publishFingerprint(fingerID);
    }
    #elif ROS_DEBUG


    transmitMsg(omegaRef);


    #elif DEBUG
    Serial.print("Right Speed: ");
    Serial.println(omegaRef.rightSpeed);
    Serial.print("Left Speed: ");
    Serial.println(omegaRef.leftSpeed);
    //Serial.print("Ultrasonic Distance: ");
    //Serial.println(usDistance);
    Serial.print("PIR 0 struct: ");
    Serial.println(pirSensors.pir0);
    Serial.print("PIR 1: ");
    Serial.println(pirSensors.pir1);
    Serial.print("PIR 2: ");
    Serial.println(pirSensors.pir2);
    Serial.print("PIR 3: ");
    Serial.println(pirSensors.pir3);
    Serial.print("Fingerprint ID: ");
    Serial.println(fingerID);
    delay(50);

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
    #endif

}
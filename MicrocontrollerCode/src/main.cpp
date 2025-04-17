#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_ICM20948.h>
#include <micro_ros_platformio.h>
#include <hardware/watchdog.h>
#include "ADCFunctions.h"
#include "JoystickFunctions.h"
#include "UltrasonicFunctions.h"
#include "FingerprintFunctions.h"
#include "PIRFunctions.h"
#include "IMUFunctions.h"
#include "FanFunctions.h"
#include "LightFunctions.h"
#include "LidarFunctions.h"



#if defined(ROS) || defined(ROS_DEBUG)
#include <microRosFunctions.h>
#endif

Adafruit_ADS1115 joystickAdc;
Adafruit_ADS1115 ultrasonicAdc;// Construct an ads1115
Adafruit_ICM20948 icm;




void setup() {
    Serial.begin(115200);

    while(!Serial){
        delay(10); //wait for serial
    }

    delay(5000);

    FanDutyCycles startDutyCycles{};
    startDutyCycles.fan_0_duty_cycle = 0;
    startDutyCycles.fan_1_duty_cycle = 0;
    startDutyCycles.fan_2_duty_cycle = 0;
    startDutyCycles.fan_3_duty_cycle = 0;

    adcInit(ultrasonicAdc, 0x49); //default address
    adcInit(joystickAdc, 0x48); //default address
    imuInit(icm, ICM20948_ACCEL_RANGE_2_G, ICM20948_GYRO_RANGE_250_DPS, AK09916_MAG_DATARATE_10_HZ);
    setupFingerprint();
    setAllFans(startDutyCycles);
    setupRPMCounter();
    setupLight();
    setupLidar();

    #ifdef ROS

    const char* nodeName = "sensors_node";
    const char* sensorTopicName = "sensors";
    const char* fingerprintTopicName = "fingerprint";

    while(!microRosSetup(1, nodeName, sensorTopicName, fingerprintTopicName));

    #elif ROS_DEBUG

    const char* nodeName = "sensors_node";
    const char* topicName = "refSpeed";
    while(!microRosSetup(1, nodeName, topicName));
    #endif

    while (!Serial) {
        delay(10);
    }


}

unsigned long lastFingerprintTime = 0;

unsigned long lastMicroRosTime = 0;

void loop() {

    unsigned long currentMillis = millis();

    //TODO might want to figure out how to put these on core1 so that they can run in parallel
    //uint32_t start = millis();
    RefSpeed omegaRef = joystickToSpeed(joystickAdc);
    //uint32_t joystickTime = millis() - start;
    USData usDistances = allUltrasonicDistance(joystickAdc, ultrasonicAdc);
    //uint32_t ultrasonicTime = millis() - start - joystickTime;
    PIRSensors pirSensors = readAllPIR();
    //uint32_t pirTime = millis() - start - joystickTime - ultrasonicTime;
    //uint8_t fingerID = getFingerprintID(); //TODO might want to put this on a timer so that it runs less frequently
    //uint32_t fingerprintTime = millis() - start - joystickTime - ultrasonicTime - pirTime;
    IMUData imuData = getIMUData(icm);
    //uint32_t imuTime = millis() - start - joystickTime - ultrasonicTime - pirTime - fingerprintTime;
    FanSpeeds fanSpeeds = getAllFanSpeeds(); //TODO might want to put on a timer as well
    //uint32_t fanTime = millis() - start - joystickTime - ultrasonicTime - pirTime - fingerprintTime - imuTime;

    uint8_t fingerID = 2;
    if (currentMillis - lastFingerprintTime >= 5000) {
        lastFingerprintTime = currentMillis;
        fingerID = getFingerprintID();
        //Serial.println("Fingerprint ID: " + String(fingerID));
    }

    if(currentMillis - lastMicroRosTime >= 25000){
        lastMicroRosTime = currentMillis;
        // checking the MicroROS connection to make sure that it is still connected
//        if (!rmw_uros_ping_agent(100, 10)) {
//            Serial.println("Lost agent connection. Rebooting...");
//            //watchdog_enable(1, 1); // 2000 ms (2 seconds) timeout
//            //while(true);
//        }
        checkConnection();
    }



    #ifdef ROS

    transmitMsg(omegaRef, usDistances, pirSensors, fanSpeeds, imuData);

    if(fingerID != 2){
        publishFingerprint(fingerID);
    }
    #elif ROS_DEBUG


    transmitMsg(omegaRef);


    #elif DEBUG

    // Serial.print("Joystick Time: ");
    // Serial.println(joystickTime);
    // Serial.print("Ultrasonic Time: ");
    // Serial.println(ultrasonicTime);
    // Serial.print("PIR Time: ");
    // Serial.println(pirTime);
    // Serial.print("Fingerprint Time: ");
    // Serial.println(fingerprintTime);
    // Serial.print("IMU Time: ");
    // Serial.println(imuTime);
    // Serial.print("Fan Time: ");
    // Serial.println(fanTime);

    // Serial.print("Fan0 Speed: ");
    // Serial.println(fanSpeeds.fan_speed_0);
    // Serial.print("Fan1 Speed: ");
    // Serial.println(fanSpeeds.fan_speed_1);
    // Serial.print("Fan2 Speed: ");
    // Serial.println(fanSpeeds.fan_speed_2);
    // Serial.print("Fan3 Speed: ");
    // Serial.println(fanSpeeds.fan_speed_3);
    // Serial.print("Right Speed: ");
    // Serial.println(omegaRef.rightSpeed);
    // Serial.print("Left Speed: ");
    // Serial.println(omegaRef.leftSpeed);
    // //Serial.print("Ultrasonic Distance: ");
    // //Serial.println(usDistance);
    // Serial.print("PIR 0 struct: ");
    // Serial.println(pirSensors.pir0);
    // Serial.print("PIR 1: ");
    // Serial.println(pirSensors.pir1);
    // Serial.print("PIR 2: ");
    // Serial.println(pirSensors.pir2);
    // Serial.print("PIR 3: ");
    // Serial.println(pirSensors.pir3);
    // Serial.print("Fingerprint ID: ");
    // Serial.println(fingerID);
    //
    //
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
    #endif

}
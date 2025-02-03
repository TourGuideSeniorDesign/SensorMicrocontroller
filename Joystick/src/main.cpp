#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include "ADCFunctions.h"
#include "JoystickFunctions.h"
#include "UltrasonicFunctions.h"
#include "FingerprintFunctions.h"
#include "PIRFunctions.h"

#ifdef ROS
#include <microRosFunctions.h>

#endif

Adafruit_ADS1115 joystickAdc;	// Construct an ads1115

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
    setupFingerprint();

}

void loop() {

    RefSpeed omegaRef = joystickToSpeed(joystickAdc);
    int16_t usDistance = ultrasonicDistance(joystickAdc, 2);
    PIRSensors pirSensors = readAllPIR();
    uint8_t fingerID = getFingerprintID();

    Serial.print("Right Speed: ");
    Serial.println(omegaRef.rightSpeed);
    Serial.print("Left Speed: ");
    Serial.println(omegaRef.leftSpeed);
    Serial.print("Ultrasonic Distance: ");
    Serial.println(usDistance);
    Serial.print("PIR 0: ");
    Serial.println(pirSensors.pir0);
    Serial.print("PIR 1: ");
    Serial.println(pirSensors.pir1);
    Serial.print("PIR 2: ");
    Serial.println(pirSensors.pir2);
    Serial.print("PIR 3: ");
    Serial.println(pirSensors.pir3);
    Serial.print("Fingerprint ID: ");
    Serial.println(fingerID);
    delay(1000);

    #ifdef ROS

    transmitMsg(omegaRef);

    #elif DEBUG

    #endif

}
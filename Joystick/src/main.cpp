#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include "ADCFunctions.h"
#include "JoystickFunctions.h"

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

}

void loop() {

    refSpeed omegaRef = joystickToSpeed(joystickAdc);

    #ifdef ROS
        transmitMsg(omegaRef);

    #elif DEBUG
        Serial.print("Right Speed: ");
        Serial.println(omegaRef.rightSpeed);
        Serial.print("Left Speed: ");
        Serial.println(omegaRef.leftSpeed);
    #endif

}
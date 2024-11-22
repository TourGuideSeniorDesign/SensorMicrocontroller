#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include "ADCFunctions.h"
#include "JoystickFunctions.h"

Adafruit_ADS1115 joystickAdc;	// Construct an ads1115


void setup(void) {
    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }


    adcInit(joystickAdc, 0x48); //default address


}

void loop() {

    refSpeed omegaRef = joystickToSpeed(joystickAdc);
    Serial.print("Left Speed: ");
    Serial.println(omegaRef.leftSpeed);
    Serial.print("Right Speed: ");
    Serial.println(omegaRef.rightSpeed);


    delay(300); // Delay for 1 second

}
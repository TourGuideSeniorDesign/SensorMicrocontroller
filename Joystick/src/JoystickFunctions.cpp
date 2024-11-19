//
// Created by Robbie on 11/18/24.
//

#include "JoystickFunctions.h"
#include <Adafruit_ADS1X15.h>

refSpeed joystickToSpeed(Adafruit_ADS1115 &adc){
    int forwardJoystick = adc.readADC_SingleEnded(0); //a0 is forward/backward
    int sidewaysJoystick = adc.readADC_SingleEnded(1); //a1 is left/right

    /*
     * Joystick middle values: ~8500
     * a0 middle value: ~8500
     * a1 middle value: ~8300
     * a0 deadzone 10000 - 6500
     * a1 deadzone 11000 - 6000
     * Joystick Min: 0
     * Joystick Max: 17390
     */

    //setting the speeds
    //TODO calculate the correct speeds based on the inputs
    refSpeed speeds{};
    speeds.leftSpeed = forwardJoystick;
    speeds.rightSpeed = sidewaysJoystick;

    //Lookup table to determine the direction
    switch(forwardJoystick){
        case 0 ... 6499:
            speeds.dir = BACKWARD;
            break;
        case 6500 ... 10000: //deadzone
            speeds.dir = STOPPED;
            break;
        case 10001 ... 17500:
            speeds.dir = FORWARD;
            break;
        default:
            speeds.dir = STOPPED;
            Serial.println("Default case");
            break;
    }

    return speeds;
}

//A simple lookup table
const char* directionToString(direction dir) {
    switch (dir) {
        case STOPPED: return "STOPPED";
        case FORWARD: return "FORWARD";
        case BACKWARD: return "BACKWARD";
        default: return "UNKNOWN";
    }
}
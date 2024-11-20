//
// Created by Robbie on 11/18/24.
//

#include "JoystickFunctions.h"
#include <Adafruit_ADS1X15.h>
#include <algorithm>

int diffParam = 30;
int deadzoneParam = 30;

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
     * Output is a value -100 to 100 for the speed of the motor
     */

    //Converting the speeds so they start around 0 and then go positive and negative
    forwardJoystick = forwardJoystick - 8500;
    sidewaysJoystick = sidewaysJoystick - 8400;

    /*Serial.print("Forward joystick: ");
    Serial.println(forwardJoystick);
    Serial.print("Sideways joystick: ");
    Serial.println(sidewaysJoystick);*/

    //setting the speeds
    refSpeed speeds{};
    float k = 0.011; //used for scaling
    speeds.leftSpeed = static_cast<int8_t>(clamp(k * (forwardJoystick - sidewaysJoystick), -100.0f, 100.0f));
    speeds.rightSpeed = static_cast<int8_t>(clamp(k * (forwardJoystick + sidewaysJoystick), -100.0f, 100.0f));

    //Deadzone
    if(speeds.leftSpeed < deadzoneParam && speeds.leftSpeed > -deadzoneParam && speeds.rightSpeed < deadzoneParam && speeds.rightSpeed > -deadzoneParam){
        speeds.leftSpeed = 0;
        speeds.rightSpeed = 0;
        return speeds;
    }

    //Middle zone to have the same speed
    int diff = abs(speeds.leftSpeed - speeds.rightSpeed);
    if(diff > -diffParam && diff < diffParam){
        // Check if the speeds are positive or negative and set accordingly
        if (speeds.leftSpeed > 0 && speeds.rightSpeed > 0) {
            // Both speeds are positive, set the smaller one to the larger one
            if (speeds.leftSpeed < speeds.rightSpeed) {
                speeds.leftSpeed = speeds.rightSpeed;
            } else {
                speeds.rightSpeed = speeds.leftSpeed;
            }
        } else if (speeds.leftSpeed < 0 && speeds.rightSpeed < 0) {
            // Both speeds are negative, set the larger one to the smaller one
            if (speeds.leftSpeed > speeds.rightSpeed) {
                speeds.leftSpeed = speeds.rightSpeed;
            } else {
                speeds.rightSpeed = speeds.leftSpeed;
            }
        }
    }

    return speeds;
}

template <typename T>
T clamp(T value, T min, T max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

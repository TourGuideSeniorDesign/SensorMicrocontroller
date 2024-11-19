//
// Created by Robbie on 11/18/24.
//

#ifndef JOYSTICK_JOYSTICKFUNCTIONS_H
#define JOYSTICK_JOYSTICKFUNCTIONS_H

#include <Adafruit_ADS1X15.h>

/**
 * Enum representing the direction of movement.
 */
enum direction {
    STOPPED,  ///< The wheelchair is stopped.
    FORWARD,  ///< The wheelchair is forward.
    BACKWARD  ///< The wheelchair is backward.
};

/**
 * Struct representing the reference speed and direction.
 */
struct refSpeed {
    int leftSpeed;      ///< Speed of the left wheel.
    int rightSpeed;     ///< Speed of the right wheel.
    direction dir;      ///< Direction of movement.
};

/**
 * Reads a value from the joystick connected to the ADC and returns the reference speeds
 * @param adc An instance of the adc the joystick is connected to
 * @return A refSpeed for the wheelchair containing the wheel speeds and the direction
 */
refSpeed joystickToSpeed(Adafruit_ADS1115 &adc);

/**
 * Returns the string corresponding to the enum value
 * @param dir The enum to convert
 * @return A string for the enum
 */
const char* directionToString(direction dir);

#endif //JOYSTICK_JOYSTICKFUNCTIONS_H

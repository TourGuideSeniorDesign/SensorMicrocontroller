//
// Created by Robbie on 12/6/24.
//

#ifndef SUBSCRIBERDEMO_JOYSTICKFUNCTIONS_H
#define SUBSCRIBERDEMO_JOYSTICKFUNCTIONS_H

#include <Arduino.h>

/**
 * Struct representing the reference speed and direction.
 */
struct refSpeed {
    int8_t leftSpeed;      ///< Speed of the left wheel.
    int8_t rightSpeed;     ///< Speed of the right wheel.
};

#endif //SUBSCRIBERDEMO_JOYSTICKFUNCTIONS_H

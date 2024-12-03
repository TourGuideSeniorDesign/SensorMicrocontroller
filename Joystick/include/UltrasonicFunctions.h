//
// Created by Robbie on 12/3/24.
//

#ifndef JOYSTICK_ULTRASONICFUNCTIONS_H
#define JOYSTICK_ULTRASONICFUNCTIONS_H
#include <Adafruit_ADS1X15.h>

/**
 * Returns the distance read by an ultrasonic sensor.
 * @param adc An instance of the ADC that the sensor is attached to.
 * @param pinNumber The pin number of the ADC that the sensor is attached to.
 * @return Returns the distance on the ultrasonic sensor, measured in cm. Returns -1 if the pin is misconfigured.
 */
uint16_t ultrasonicDistance(Adafruit_ADS1115 &adc, uint8_t pinNumber);

#endif //JOYSTICK_ULTRASONICFUNCTIONS_H

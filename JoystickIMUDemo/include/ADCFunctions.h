//
// Created by Robbie on 11/14/24.
//

#ifndef JOYSTICKIMUDEMO_ADCFUNCTIONS_H
#define JOYSTICKIMUDEMO_ADCFUNCTIONS_H

#include <Arduino.h>
#include <Adafruit_ADS1X15.h>

void adcInit(Adafruit_ADS1115 &adc, uint8_t i2c_addr);
void printADC(Adafruit_ADS1115 &adc);

#endif //JOYSTICKIMUDEMO_ADCFUNCTIONS_H

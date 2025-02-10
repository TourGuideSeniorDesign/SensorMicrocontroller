//
// Created by Robbie on 2/9/25.
//

#ifndef MICROCONTROLLERCODE_FANFUNCTIONS_H
#define MICROCONTROLLERCODE_FANFUNCTIONS_H

#include "Arduino.h"

struct FanSpeeds{
    int16_t fan_speed_0;
    int16_t fan_speed_1;
    int16_t fan_speed_2;
    int16_t fan_speed_3;
};

struct FanDutyCycles{
    int8_t fan_0_duty_cycle;
    int8_t fan_1_duty_cycle;
    int8_t fan_2_duty_cycle;
    int8_t fan_3_duty_cycle;
};


#define FAN_0 16
#define FAN_1 18
#define FAN_2 20
#define FAN_3 22



//TODO add fuctions to read individual and all fan speeds

void setFanIndividual(uint8_t fan, uint8_t dutyCycle);

void setAllFans(FanDutyCycles dutyCycles);

FanSpeeds getAllFanSpeeds();

#endif //MICROCONTROLLERCODE_FANFUNCTIONS_H

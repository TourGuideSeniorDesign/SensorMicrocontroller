//
// Created by Robbie on 2/9/25.
//

#include <Arduino.h>
#include "FanFunctions.h"
#include "PWMFunctions.h"

static float frequency = 25000;

void setFanIndividual(uint8_t fan, uint8_t dutyCycle){
    setPWM(fan, frequency, dutyCycle);
}

void setAllDutyCycles(FanDutyCycles dutyCycles){
    setPWM(FAN_0, frequency, dutyCycles.fan_0_duty_cycle);
    setPWM(FAN_1, frequency, dutyCycles.fan_1_duty_cycle);
    setPWM(FAN_2, frequency, dutyCycles.fan_2_duty_cycle);
    setPWM(FAN_3, frequency, dutyCycles.fan_3_duty_cycle);
}

//TODO figure out how to read the actual fan speeds
FanSpeeds getAllFanSpeeds(){
    FanSpeeds speeds{};
    speeds.fan_speed_0 = 0;
    speeds.fan_speed_1 = 0;
    speeds.fan_speed_2 = 0;
    speeds.fan_speed_3 = 0;
    return speeds;
}
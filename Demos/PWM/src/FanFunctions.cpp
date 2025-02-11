//
// Created by Robbie on 2/9/25.
//

#include <Arduino.h>
#include "FanFunctions.h"
#include "PWMFunctions.h"
#include "hardware/pio.h"
#include "fanRPM.pio.h"

static float frequency = 25000;

static volatile uint32_t pulse_count = 0;
static uint32_t rpm = 0;

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

void setupRPMCounter(){
    pinMode(TACH_0, INPUT);
    attachInterrupt(digitalPinToInterrupt(TACH_0), handleTachInterrupt, FALLING);
}

static void handleTachInterrupt() {
    pulse_count++;
}

uint32_t getRPM(){
    static uint32_t last_pulse_count = 0;
    static uint32_t last_time = 0;

    uint32_t current_time = millis();
    if (current_time - last_time >= 1000) {  // Calculate RPM every second
        uint32_t pulses = pulse_count - last_pulse_count;
        last_pulse_count = pulse_count;
        last_time = current_time;

        // Assuming the fan gives 2 pulses per revolution
        rpm = (pulses * 60) / 2;

    }
    return rpm;
}




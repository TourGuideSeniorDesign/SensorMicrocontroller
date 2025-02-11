//
// Created by Robbie on 2/9/25.
//

#include <Arduino.h>
#include "FanFunctions.h"
#include "PWMFunctions.h"
#include "hardware/pio.h"
#include "fanRPM.pio.h"

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

#define TACH_PIN 21 // Example tachometer pin

PIO pio = pio0;
uint sm = 0;

void initRPMCounter() {
    uint offset = pio_add_program(pio, &rpm_counter_program);
    sm = pio_claim_unused_sm(pio, true);
    rpm_counter_program_init(pio, sm, offset, TACH_PIN);
}

uint32_t getRPM() {
    // Assuming the fan gives 2 pulses per revolution
    const uint pulses_per_revolution = 2;
    const uint32_t interval_ms = 1000; // 1 second interval

    // Enable the state machine
    pio_sm_set_enabled(pio, sm, true);

    // Wait for the interval
    sleep_ms(interval_ms);

    // Disable the state machine
    pio_sm_set_enabled(pio, sm, false);

    // Read the pulse count
    uint32_t pulse_count = pio_sm_get(pio, sm);

    // Calculate RPM
    uint32_t rpm = (pulse_count / pulses_per_revolution) * (60000 / interval_ms);

    return rpm;
}


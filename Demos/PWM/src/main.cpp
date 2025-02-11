#include <Arduino.h>
#include "PWMFunctions.h"
#include "FanFunctions.h"

#define PWM_PIN0 16  // Choose a valid PWM-capable GPIO pin

void setup() {
    setFanIndividual(PWM_PIN0, 25);  // 1 kHz, 50% duty cycle
    initRPMCounter();
}

void loop() {
    uint32_t rpm = getRPM();
    Serial.print("Fan RPM: ");
    Serial.println(rpm);
    delay(1000);
}
#include "Arduino.h"
#include "PWMFunctions.h"

#define PWM_PIN0 16  // Choose a valid PWM-capable GPIO pin

void setup() {
    setPWM(PWM_PIN0, 25000, 20);  // 1 kHz, 50% duty cycle
}

void loop() {
    // Do nothing, PWM runs independently
}
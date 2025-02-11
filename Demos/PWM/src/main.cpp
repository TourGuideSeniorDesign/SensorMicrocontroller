#include <Arduino.h>
#include "PWMFunctions.h"
#include "FanFunctions.h"



#define TACH_0 17  // Define the GPIO pin for the tachometer

volatile uint32_t pulse_count = 0;

void handleTachInterrupt() {
    pulse_count++;
}

void setup() {
    setFanIndividual(FAN_0, 75);  // 1 kHz, 50% duty cycle
    //setFanIndividual(TACH_0, 25);  // 1 kHz, 50% duty cycle
    pinMode(TACH_0, INPUT);
    attachInterrupt(digitalPinToInterrupt(TACH_0), handleTachInterrupt, FALLING);

    Serial.begin(115200);

}

void loop() {
    static uint32_t last_pulse_count = 0;
    static uint32_t last_time = 0;

    uint32_t current_time = millis();
    if (current_time - last_time >= 1000) {  // Calculate RPM every second
        uint32_t pulses = pulse_count - last_pulse_count;
        last_pulse_count = pulse_count;
        last_time = current_time;

        // Assuming the fan gives 2 pulses per revolution
        uint32_t rpm = (pulses * 60) / 2;

        Serial.print("RPM: ");
        Serial.println(rpm);
    }

}
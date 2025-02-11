#include <Arduino.h>
#include "PWMFunctions.h"
#include "FanFunctions.h"
#include "pico/multicore.h"




void setup() {
    Serial.begin(115200);
    delay(1000);
    setFanIndividual(FAN_0, 25);  // 1 kHz, 50% duty cycle
    //setFanIndividual(TACH_0, 25);  // 1 kHz, 50% duty cycle
    setupRPMCounter();
}

void loop() {
    const uint32_t rpm = getRPM();  // Receive RPM value from second core
    Serial.print("RPM: ");
    Serial.println(rpm);

}


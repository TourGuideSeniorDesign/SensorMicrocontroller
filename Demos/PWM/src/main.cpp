#include <Arduino.h>
#include "PWMFunctions.h"
#include "FanFunctions.h"
#include "pico/bootrom.h"


void setup() {
    Serial.begin(115200);
    delay(1000);
    setFanIndividual(FAN_0, 25);  // 1 kHz, 50% duty cycle
    //setFanIndividual(TACH_0, 25);  // 1 kHz, 50% duty cycle
    setupRPMCounter();
}

void loop() {
    const uint32_t rpm = getRPM();  // Receive RPM value from second core
    const FanSpeeds fan_speeds = getAllFanSpeeds();
    Serial.print("Single RPM: ");
    Serial.println(rpm);
    Serial.println("Multi RPM: ");
    Serial.println(fan_speeds.fan_speed_0);
    if ((fan_speeds.fan_speed_0 == 0 || rpm == 0) && millis() > 3000) {
        Serial.println("Fan stopped spinning. Resetting...");
        reset_usb_boot(0, 0);
    }

}


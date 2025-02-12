#include <Arduino.h>
#include "microRosFunctions.h"

void setup() {
// write your initialization code here

    Serial.begin(115200);
    Serial1.begin(115200);
    delay(2000);
    Serial1.println("Serial1 is working");

    pinMode(LED_BUILTIN, OUTPUT);

    digitalWrite(LED_BUILTIN, HIGH);
    delay(3000);
    digitalWrite(LED_BUILTIN, LOW);

    const char* nodeName = "test_node";
#ifdef DEBUG
    const char* topicName = "sensors";
#else
    const char* topicName = "refspeed";
#endif
    microRosSetup(1, nodeName, topicName);
}

void loop() {
// write your code here
    checkSubs();
}
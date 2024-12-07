#include <Arduino.h>
#include "microRosFunctions.h"

void setup() {
// write your initialization code here

    Serial.begin(115200);

    const char* nodeName = "test_node";
    const char* topicName = "refspeed";
    microRosSetup(1, nodeName, topicName);
}

void loop() {
// write your code here
}
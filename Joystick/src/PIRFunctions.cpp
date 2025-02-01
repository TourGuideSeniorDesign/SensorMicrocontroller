//
// Functions created by Nam
// Modified by Robbie
//

#include <Arduino.h>
#include "PIRFunctions.h"

//TODO figure out best way to accept all the PIR pins and initialize them
void setupPIR() {
    //pinMode(PIRout, INPUT); // initialize sensor as an input
    Serial.println("PIR detected");
}

bool readPIR(uint8_t pirPin) {
    return digitalRead(pirPin);
}

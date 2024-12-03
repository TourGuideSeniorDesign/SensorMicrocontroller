//
// Created by Robbie on 12/3/24.
//

#include "UltrasonicFunctions.h"
#include <Adafruit_ADS1X15.h>
#include "ADCFunctions.h"

//TODO tune these numbers to get a more accurate reading
#define  MAX_RANG      (520)//the max measurement value of the module is 520cm(a little bit longer than  effective max range)
#define ADC_SOLUTION 26230 //Max value for a 16 bit dac at 3.3V

//Distance Formula: Distance=Vout(mV)×520/Vin(mV)



uint16_t ultrasonicDistance(Adafruit_ADS1115 &adc, uint8_t pinNumber){
    if(pinNumber > 3){
        Serial.println("Please select a pin between 0 and 3");
        return -1;
    }
    //uint16_t distance = bitToMv(adc, pinNumber) * MAX_RANG / ADC_SOLUTION;
    uint16_t distance = adc.readADC_SingleEnded(pinNumber) * MAX_RANG / ADC_SOLUTION;

#ifdef DEBUG
    Serial.print("Raw Reading: ");
    Serial.println(adc.readADC_SingleEnded(pinNumber));
    Serial.print("Ultrasonic Distance: ");
    Serial.println(distance);
#endif
    return distance;
}
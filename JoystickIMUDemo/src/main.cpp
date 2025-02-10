
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include "IMUFunctions.h"
#include "ADCFunctions.h"

Adafruit_ADS1115 adc;	// Construct an ads1115
Adafruit_ICM20948 icm;
uint16_t measurement_delay_us = 65535; // Delay between measurements for testing

void setup(void) {
    Serial.begin(115200);
    while (!Serial) {
        delay(10); // will pause Zero, Leonardo, etc until serial console opens
    }


    imuInit(icm, ICM20948_ACCEL_RANGE_2_G, ICM20948_GYRO_RANGE_250_DPS, AK09916_MAG_DATARATE_10_HZ);

    adcInit(adc, 0x48); //default address

}

void loop() {

    //printImuData(icm);
    printADC(adc);

    delay(100);


}
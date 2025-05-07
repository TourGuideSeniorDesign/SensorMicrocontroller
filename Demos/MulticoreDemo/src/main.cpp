    //Demo for multicore architecture on the Pico

    #include <Arduino.h>
    #include "RefSpeed.h"
    #include "pico/multicore.h"
    #include "hardware/clocks.h"
    //#include "pico/stdlib.h"



    int32_t packRefSpeed(const refSpeed& s) {
        return (static_cast<int32_t>(s.leftSpeed) << 8) | static_cast<uint8_t>(s.rightSpeed);
    }

    refSpeed unpackRefSpeed(int32_t packed) {
        return {
            static_cast<int8_t>((packed >> 8) & 0xFF),
            static_cast<int8_t>(packed & 0xFF)
        };
    }

    void core1Task() {
        int8_t left = 0;
        int8_t right = 0;
        while (true) {
            const refSpeed s = {left, right};
            multicore_fifo_push_blocking(packRefSpeed(s));
            left++;
            right--;
            //sleep_ms(500);
        }
    }

    void setup() {
        //set_sys_clock_khz(250000, true);
        Serial.begin(115200); // start I2C communication protocol


        while(!Serial){
            delay(10); //wait for serial
        }

        sleep_ms(2000);
        uint32_t clock_speed = clock_get_hz(clk_sys) / 1000000;
        Serial.println("Clock Speed: " + String(clock_speed) + " MHz");

        sleep_ms(2000);


        multicore_launch_core1(core1Task);

    }


    void loop() {

        int32_t packed = multicore_fifo_pop_blocking();
        refSpeed s = unpackRefSpeed(packed);
        Serial.println("Core 1 Speeds:");
        Serial.print("L: "); Serial.println(s.leftSpeed);
        Serial.print("R: "); Serial.println(s.rightSpeed);
        Serial.println("");
    }
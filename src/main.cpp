#include <Arduino.h>
#include "Lego_I2C.hpp"

LegoI2C lego;

void setup() {
    Serial.begin(115200);
    Serial.println("LegoExpander");
    lego.install();
}

char c = 'a';

void loop() {
    if (Serial.available()) {
        char ch = Serial.read();
        switch (ch) {
            case '\n':
                break;
            case '\r':
                Serial.write('\n');
                break;
            default:
                c = ch;
                Serial.print("c set to ");
                Serial.println(c);
                break;
        }
    }
    if (lego.available()) {
        uint8_t b = lego.read();
        Serial.println(b);
        switch (b) {
        case 3:
            lego.write(c);
            break;
        case 4:
            for(uint8_t i = 0; i != 8; ++i)
                lego.write(c+i);
        }
    }
}
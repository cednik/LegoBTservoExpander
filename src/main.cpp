#include <Arduino.h>

#include "driver/i2c.h"

void setup() {
    Serial.begin(115200);
    Serial.println("LegoExpander");

    i2c_config_t i2c_config;
    i2c_config.sda_io_num = GPIO_NUM_22;
    i2c_config.scl_io_num = GPIO_NUM_23;
    i2c_config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_config.mode = I2C_MODE_SLAVE;
    i2c_config.slave.addr_10bit_en = I2C_ADDR_BIT_7;
    i2c_config.slave.slave_addr = 4;
     ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_config));
     ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, i2c_config.mode, 1024, 1024, 0));
}

#include "hexdump.h"

char c = 'a';
uint8_t cnt = 0;

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
    uint8_t buffer[16];
    int size = i2c_slave_read_buffer(I2C_NUM_0, buffer, 1, 1 / portTICK_RATE_MS);
    if (size == ESP_FAIL) {
        Serial.println("I2C read failed");
    } else if (size == 0) {
        //Serial.println("I2C: notihing to read");
    } else {
        i2c_slave_write_buffer(I2C_NUM_0, &cnt, 1, 1000 / portTICK_RATE_MS);
        ++cnt;
        hexDump("I2C read:", buffer, size);
        if (buffer[0] == 3 || buffer[0] == 4) {
            int stop = buffer[0] == 3 ? 1 : 8;
            for(int i = 0; i != stop; ++i)
                buffer[i] = c+i;
            size = i2c_slave_write_buffer(I2C_NUM_0, buffer, stop, 1000 / portTICK_RATE_MS);
            if (size == ESP_FAIL) {
                Serial.println("I2C write failed");
            } else if (size == 0) {
                Serial.println("I2C: write buffer full");
            } else {
                Serial.print("I2C: written ");
                Serial.print(size);
                Serial.println(" bytes");
                hexDump("I2C write:", buffer, size);
            }
        }
    }
}
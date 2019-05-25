#include <Arduino.h>
#include <BluetoothSerial.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial bt;

#include <vector>
#include <string>

class LegoProtocol {
public:
    LegoProtocol()
        :m_buffer()
    {
        m_buffer.reserve(2);
    }

    bool push_byte(uint8_t v) {
        m_buffer.push_back(v);
        return m_buffer.size() >= 2 && m_buffer.size() == packet_length();
    }

    void clear() {
        m_buffer.clear();
    }

    uint16_t packet_length() const { return read_at<uint16_t>(0); }
    uint16_t message_counter() const { return read_at<uint16_t>(2); }
    uint8_t cmd_type() const { return read_at<uint8_t>(4); }
    uint8_t cmd_number() const { return read_at<uint8_t>(5); }
    uint8_t mailbox_length() const { return read_at<uint8_t>(6); }
    std::string mailbox() const { return std::string(reinterpret_cast<const char*>(m_buffer.data() + 7)); }
    uint16_t payload_length() const { return read_at<uint16_t>(7 + mailbox_length() + 1); }
    const uint8_t* payload() const { return m_buffer.data() + 7 + mailbox_length() + 1 + 2; }

private:
    template <typename T>
    const T& read_at (uint16_t index) const {
        return *reinterpret_cast<const T*>(m_buffer.data() + index);
    }

    std::vector<uint8_t> m_buffer;
};

LegoProtocol lego;

#include "driver/i2c.h"

void setup() {
    Serial.begin(115200);
    Serial.println("LegoBTservoExpander");
    if (!bt.begin("Expander")) {
        Serial.println("Can not initialize bluetooth!");
        for(;;){}
    }

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

bool is_connected = false;

template <class Stream, typename Value>
void print_val(Stream& stream, std::string intro, const Value& value) {
    stream.print(intro.c_str());
    stream.print(": ");
    stream.println(value);
}

#include "hexdump.h"

char c = 'a';
uint8_t cnt = 0;

void loop() {
    bool con = bt.hasClient();
    if (con != is_connected) {
        is_connected = con;
        if (is_connected) {
            Serial.println("Client connected");
        } else {
            Serial.println("Client disconnected");
        }
    }
    if (bt.available() && lego.push_byte(bt.read())) {
        Serial.println("Lego packet arrived:");
        print_val(Serial, "\tlength  ", lego.packet_length());
        print_val(Serial, "\tmsg cnt ", lego.message_counter());
        print_val(Serial, "\tcmd type", lego.cmd_type());
        print_val(Serial, "\tcmd num ", lego.cmd_number());
        print_val(Serial, "\tname len", lego.mailbox_length());
        print_val(Serial, "\tname    ", lego.mailbox().c_str());
        print_val(Serial, "\tdata len", lego.payload_length());
        hexDump("\tdata:", lego.payload(), lego.payload_length());
        lego.clear();
    }
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
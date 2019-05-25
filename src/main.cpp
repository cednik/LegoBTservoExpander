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

void setup() {
    Serial.begin(115200);
    Serial.println("LegoBTservoExpander");
    if (!bt.begin("Expander")) {
        Serial.println("Can not initialize bluetooth!");
        for(;;){}
    }
}

bool is_connected = false;

template <class Stream, typename Value>
void print_val(Stream& stream, std::string intro, const Value& value) {
    stream.print(intro.c_str());
    stream.print(": ");
    stream.println(value);
}

#include "hexdump.h"

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
}
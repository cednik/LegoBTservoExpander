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
    std::string mailbox() const { return std::string(static_cast<const char*>(m_buffer.data() + 7); }

private:
    template <typename T>
    const T& read_at (uint16_t index) const {
        return *static_cast<T*>(m_buffer.data() + index);
    }

    std::vector<uint8_t> m_buffer;
};

void setup() {
    Serial.begin(115200);
    Serial.println("LegoBTservoExpander");
    if (!bt.begin("Expander")) {
        Serial.println("Can not initialize bluetooth!");
        for(;;){}
    }
}

bool is_connected = false;

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
}
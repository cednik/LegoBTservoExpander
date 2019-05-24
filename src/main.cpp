#include <Arduino.h>
#include <BluetoothSerial.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial bt;

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
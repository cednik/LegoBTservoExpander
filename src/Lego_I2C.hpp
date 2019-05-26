#include "driver/i2c.h"
#include <mutex>
#include <queue>
#include <functional>

class LegoI2C {
public:
    typedef uint8_t addr_t;
    typedef std::function<void(LegoI2C&)> callback_t;

    LegoI2C(i2c_port_t i2c = I2C_NUM_0);

    bool install(addr_t addr = 4,
                 gpio_num_t sda = GPIO_NUM_22,
                 gpio_num_t scl = GPIO_NUM_23,
                 gpio_pullup_t pullup_en = GPIO_PULLUP_ENABLE );

    void register_callback(callback_t callback);

    size_t available();

    uint8_t read();

    size_t write(uint8_t v);

    size_t write(uint8_t* data, size_t size);

private:
    void process();
    void write_cnt();

    const i2c_port_t m_i2c;
    uint8_t m_msg_cnt;
    std::mutex m_tx_mutex;
    std::mutex m_rx_mutex;
    std::queue<uint8_t> m_rx_buff;
    callback_t m_callback;

    static void process_trampoline(void* cookie);
};
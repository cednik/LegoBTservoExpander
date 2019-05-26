#include "Lego_I2C.hpp"
#include <esp_log.h>

#define TAG "Lego_I2C"

LegoI2C::LegoI2C(i2c_port_t i2c)
    : m_i2c(i2c),
      m_msg_cnt(0),
      m_tx_mutex(),
      m_rx_mutex(),
      m_rx_buff(),
      m_callback()
{}

bool LegoI2C::install(addr_t addr, gpio_num_t sda, gpio_num_t scl, gpio_pullup_t pullup_en) {
    i2c_config_t i2c_config;
    i2c_config.sda_io_num = sda;
    i2c_config.scl_io_num = scl;
    i2c_config.sda_pullup_en = pullup_en;
    i2c_config.scl_pullup_en = pullup_en;
    i2c_config.mode = I2C_MODE_SLAVE;
    i2c_config.slave.addr_10bit_en = I2C_ADDR_BIT_7;
    i2c_config.slave.slave_addr = addr;
    ESP_ERROR_CHECK(i2c_param_config(m_i2c, &i2c_config));
    ESP_ERROR_CHECK(i2c_driver_install(m_i2c, i2c_config.mode, 128, 128, 0));
    xTaskCreate(&LegoI2C::process_trampoline, "LegoI2C_loop", 4096, this, 2, NULL);
    return true;
}

void LegoI2C::process() {
    uint8_t buffer = 0;
    int64_t last_read = esp_timer_get_time();
    write_cnt();
    for (;;) {
        int size = i2c_slave_read_buffer(m_i2c, &buffer, 1, 1000 / portTICK_RATE_MS);
        if (size == ESP_FAIL) {
            ESP_LOGE(TAG, "I2C reading failed!");
        } else if (size > 0) {
            int64_t t = esp_timer_get_time();
            if ((t - last_read) > 3000) {
                write_cnt();
            }
            last_read = t;
            m_rx_mutex.lock();
            m_rx_buff.push(buffer);
            m_rx_mutex.unlock();
            if (m_callback)
                m_callback(*this);
        }
    }
}

void LegoI2C::register_callback(callback_t callback) {
    m_callback = callback;
}

size_t LegoI2C::available() {
    std::lock_guard<std::mutex> guard(m_rx_mutex);
    return m_rx_buff.size();
}

uint8_t LegoI2C::read() {
    std::lock_guard<std::mutex> guard(m_rx_mutex);
    if (m_rx_buff.empty())
        return 0;
    uint8_t v = m_rx_buff.front();
    m_rx_buff.pop();
    return v;
}

size_t LegoI2C::write(uint8_t v) {
    return write(&v, 1);
}

size_t LegoI2C::write(uint8_t* data, size_t size) {
    std::lock_guard<std::mutex> guard(m_tx_mutex);
    return i2c_slave_write_buffer(m_i2c, data, size, 1000 / portTICK_RATE_MS);
}

void LegoI2C::write_cnt() {
    write(m_msg_cnt++);
}

void LegoI2C::process_trampoline(void* cookie) {
    ((LegoI2C*)cookie)->process();
}

#ifndef ST25R100_DEV_STRUCTS_H
#define ST25R100_DEV_STRUCTS_H

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

struct nfc_data {
    uint32_t write_buffer;
    uint32_t read_buffer;
};

struct nfc_cfg {
    struct spi_dt_spec spi_bus;
    struct gpio_dt_spec reset_gpio;
    struct gpio_dt_spec irq_gpio;
};

#endif
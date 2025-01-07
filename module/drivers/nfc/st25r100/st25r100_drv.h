#ifndef ST25R100_DRV_H
#define ST25R100_DRV_H

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/sensor.h>

struct nfc_data {
    const struct device *dev;
	struct gpio_callback gpio_cb;
#if defined(CONFIG_ST25R_TRIGGER_OWN_THREAD)
	K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_ST25R_THREAD_STACK_SIZE);
	struct k_thread thread;
	struct k_sem gpio_sem;
#elif defined(CONFIG_ST25R_TRIGGER_GLOBAL_THREAD)
	struct k_work work;
#endif
};

struct nfc_cfg {
    struct spi_dt_spec spi_bus;
    struct gpio_dt_spec reset_gpio;
    struct gpio_dt_spec irq_gpio;
};

int st25r_spi_init(const struct device *dev);
int st25r_init_interrupt(const struct device *dev);

#endif
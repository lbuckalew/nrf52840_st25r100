// Copyright © 2023 Vouch.io LLC

#define DT_DRV_COMPAT st_st25r

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "rfal_platform.h"
#include "st25r200_irq.h"
#include "st25r100_drv.h"

LOG_MODULE_DECLARE(ST25R);

static void st25r_handle_interrupt(const struct device *dev)
{
	const struct nfc_cfg *cfg = dev->config;

	st25r200Isr();

	gpio_pin_interrupt_configure_dt(&cfg->irq_gpio, GPIO_INT_EDGE_TO_ACTIVE);
}

static void st25r_gpio_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	struct nfc_data *st25r = CONTAINER_OF(cb, struct nfc_data, gpio_cb);
	const struct nfc_cfg *cfg = st25r->dev->config;

	if ((pins & BIT(cfg->irq_gpio.pin)) == 0U)
    {
		return;
	}

	gpio_pin_interrupt_configure_dt(&cfg->irq_gpio, GPIO_INT_DISABLE);

#if defined(CONFIG_ST25R_TRIGGER_OWN_THREAD)
	k_sem_give(&st25r->gpio_sem);
#elif defined(CONFIG_ST25R_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&st25r->work);
#endif /* CONFIG_ST25R_TRIGGER_OWN_THREAD */
}

#ifdef CONFIG_ST25R_TRIGGER_OWN_THREAD
static void st25r_thread(struct nfc_data *st25r)
{
	while (1)
    {
		k_sem_take(&st25r->gpio_sem, K_FOREVER);
		st25r_handle_interrupt(st25r->dev);
	}
}
#endif

#ifdef CONFIG_ST25R_TRIGGER_GLOBAL_THREAD
static void st25r_work_cb(struct k_work *work)
{
	struct nfc_data *st25r = CONTAINER_OF(work, struct nfc_data, work);
	st25r_handle_interrupt(st25r->dev);
}
#endif

static gpio_pin_t s_int_pin;
static const struct device *s_int_port;

gpio_pin_t platform_st25r_int_pin()
{
    return s_int_pin;
}

const struct device *platform_st25r_int_port()
{
    return s_int_port;
}

int st25r_init_interrupt(const struct device *dev)
{
	struct nfc_data *st25r = dev->data;
	const struct nfc_cfg *cfg = dev->config;
	int ret;

	if (!device_is_ready(cfg->irq_gpio.port))
    {
		LOG_ERR("%s: device %s is not ready", dev->name, cfg->irq_gpio.port->name);
		return -ENODEV;
	}

	s_int_pin = cfg->irq_gpio.pin;
	s_int_port = cfg->irq_gpio.port;

	st25r->dev = dev;

#if defined(CONFIG_ST25R_TRIGGER_OWN_THREAD)
	k_sem_init(&st25r->gpio_sem, 0, K_SEM_MAX_LIMIT);

	k_thread_create(&st25r->thread,
                    st25r->thread_stack,
                    CONFIG_ST25R_THREAD_STACK_SIZE,
		            (k_thread_entry_t)st25r_thread, st25r,
		            0, NULL, K_PRIO_COOP(CONFIG_ST25R_THREAD_PRIORITY),
		            0, K_NO_WAIT);
#elif defined(CONFIG_ST25R_TRIGGER_GLOBAL_THREAD)
	st25r->work.handler = st25r_work_cb;
#endif

	ret = gpio_pin_configure_dt(&cfg->irq_gpio, GPIO_INPUT);
	if (ret < 0)
    {
		LOG_ERR("Could not configure gpio");
		return ret;
	}

	gpio_init_callback(&st25r->gpio_cb, st25r_gpio_callback, BIT(cfg->irq_gpio.pin));

	if (gpio_add_callback(cfg->irq_gpio.port, &st25r->gpio_cb) < 0)
    {
		LOG_ERR("Could not set gpio callback");
		return -EIO;
	}

	return gpio_pin_interrupt_configure_dt(&cfg->irq_gpio, GPIO_INT_EDGE_TO_ACTIVE);
}
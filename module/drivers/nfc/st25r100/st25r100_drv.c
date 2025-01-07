#define DT_DRV_COMPAT st_st25r100

#include <zephyr/init.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>

#include "st25r100_drv.h"

LOG_MODULE_REGISTER(ST25R);

static int st25r_init(const struct device *dev)
{
	if (st25r_spi_init(dev))
    {
	    return -EINVAL;
	}

    if (st25r_init_interrupt(dev) < 0)
    {
        LOG_ERR("Failed to initialize interrupts");
        return -EIO;
    }

	return 0;
}

#define ST25R_DEFINE(inst)\
	static struct nfc_data st25r_data_##inst;\
	static const struct nfc_cfg st25r_device_config_##inst = {\
		.spi_bus = SPI_DT_SPEC_INST_GET(0, SPI_OP_MODE_MASTER |\
                                        SPI_MODE_CPHA |\
                                        SPI_WORD_SET(8)|\
                                        SPI_HOLD_ON_CS |\
                                        SPI_LOCK_ON, 0),\
		.irq_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, irq_gpios, { 0 }),\
	};\
	DEVICE_DT_INST_DEFINE(inst,\
                            st25r_init,\
                            NULL,\
			                &st25r_data_##inst,\
                            &st25r_device_config_##inst,\
                            POST_KERNEL,\
			                90, NULL);\

DT_INST_FOREACH_STATUS_OKAY(ST25R_DEFINE)
#include "st25r100_drv.h"

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/random/random.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

// This is how to write non-maintainable overly complicated code, but I'm having fun.
struct dummy_bytes {
	uint8_t a: 8;
	uint8_t b: 8;
	uint8_t c: 8;
	uint8_t d: 8;
};
union dummy_data {
	struct dummy_bytes bytes;
	uint32_t parent_val;
} write_data;

int main(void)
{
	const struct device *dev = device_get_binding(DEVICE_DT_NAME(DT_NODELABEL(st25)));
	if (dev == NULL)
	{
		LOG_ERR("Could not find NFC device.");
		return -ENODEV;
	}
	LOG_INF("NFC device found. It loves you very much.");

	while (1)
	{
		// Random val for write to test read matches
		write_data.parent_val = sys_rand32_get();
		LOG_INF("%x: %x %x %x %x",
				write_data.parent_val,
				write_data.bytes.a,
				write_data.bytes.b,
				write_data.bytes.c,
				write_data.bytes.d);

		// unsafe {
		st25_write_tag(dev, 0, (uint8_t*)&write_data); // very brain

		uint8_t read_buf[3*4] = {0};
		st25_read_tag(dev, 0, 3, read_buf);
		for (int i=0; i<sizeof(read_buf); i++)
		{
			printk("%x ", read_buf[i]);
		}
		printk("\n");

		// Check block 0 to verify write
		union dummy_data check;
		check.parent_val = *(uint32_t*)read_buf; // everything is fine
		memset(read_buf, 0, sizeof(read_buf));

		if (check.parent_val != write_data.parent_val)
		{
			LOG_ERR("Written bytes did not match read bytes. (read 0x%x)", check.parent_val);
		}
		// }

		k_msleep(5000);
	}
	return 0;
}

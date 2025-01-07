#include "st25r100_dev_structs.h"

#define DT_DRV_COMPAT st_st25r100

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

#define ST25_REG_OP 0x00
#define ST25_OP_WU_EN_FIELD 0b00000001
#define ST25_OP_RD_EN_FIELD 0b00000010
#define ST25_OP_AM_EN_FIELD 0b00001000
#define ST25_OP_RX_EN_FIELD 0b00010000
#define ST25_OP_TX_EN_FIELD 0b00100000

#define ST25_REG_REGULATOR_CFG 0x02
#define ST25_REGULATOR_CFG_REGD_FIELD 0b11100000

#define ST25_REG_TX_DRIVER 0x03
#define ST25_DRES 0b1111

#define ST25_REG_TX_MOD1 0x04
#define ST25_TX_MOD1_REG_AM 0b1
#define ST25_TX_MOD1_INDEX 0b11110000

#define ST25_REG_RX_DIGITAL 0x08
#define ST25_RX_DIGITAL_LPF 0b111 << 4
#define ST25_RX_DIGITAL_HPF 0b11 << 2
#define ST25_RX_DIGITAL_AGC 0b1 << 7

#define ST25_REG_CORREL_1 0x09

#define ST25_REG_CORREL_5 0x0D
#define ST25_CORREL_5_FSUBC 1 << 4

#define ST25_REG_DR1 0x0F
#define ST25_DR1_OSC_OK_FIELD 0b00100000

#define ST25_REG_PROTOCOL 0x12
#define ST25_PROTOCOL_MODE_FIELD 0b00001111
#define ST25_PROTOCOL_MODE_NFCV 0b101

#define ST25_REG_TX_PROTOCOL 0x13
#define ST25_TX_PROTOCOL_TR_AM 0b111 << 4

#define ST25_REG_FIFO_FRAME1 0x34
#define ST25_FIFO_FRAME2_NTXMSB_FIELD 0xFF

#define ST25_REG_FIFO_FRAME2 0x35
#define ST25_FIFO_FRAME2_NTXLSB_FIELD 0b11111000

#define ST25_REG_FIFO_SR1 0x36

#define ST25_REG_FIFO_SR2 0x37

#define ST25_REG_IRQ_SR1 0x3C

#define ST25_REG_IRQ_SR2 0x3D

#define ST25_REG_IRQ_SR3 0x3E

#define ST25_REG_ID 0x3F
#define ST25_VAL_ID 0xA9

#define ST25_REG_FIFO 0x5F

#define ST25_CMD_SET_DEFAULT 0x60
#define ST25_CMD_STOP_ACT 0x62
#define ST25_CMD_CLR_FIFO 0x64
#define ST25_CMD_CLR_RX_GAIN 0X66
#define ST25_CMD_ADJ_REG 0x68
#define ST25_CMD_TX_DATA 0x6A
#define ST25_CMD_TX_EOF 0x6C
#define ST25_CMD_UNMASK_RX 0x72

struct gpio_callback irq_cb_data;

LOG_MODULE_REGISTER(st25r100, CONFIG_LOG_DEFAULT_LEVEL);

struct irq_work {
    struct k_work work;
    const struct device *dev;
} st25_irq_work;

struct irq_bitfield {
    // IRQ reg 1
    uint8_t i_rfu0: 1;
    uint8_t i_txe: 1;
    uint8_t i_rxs: 1;
    uint8_t i_rxe: 1;
    uint8_t i_rx_reset: 1;
    uint8_t i_wl: 1;
    uint8_t i_col: 1;
    uint8_t i_subc_start: 1;
    // IRQ reg 2
    uint8_t i_sfe: 1;
    uint8_t i_hfe: 1;
    uint8_t i_par: 1;
    uint8_t i_crc: 1;
    uint8_t i_rfu1: 2;
    uint8_t i_nre: 1;
    uint8_t i_gpe: 1;
    // IRQ reg 3
    uint8_t i_osc: 1;
    uint8_t i_wut: 1;
    uint8_t i_wui: 1;
    uint8_t i_wuq: 1;
    uint8_t i_dct: 1;
    uint8_t i_rfu2: 3;
};

union {
    struct irq_bitfield bits;
    uint32_t val;
} irq_status;



static int st25_read_reg(const struct device *dev, uint8_t reg, uint8_t *buff, size_t len)
{
    int err;
    const struct nfc_cfg *cfg = dev->config;

    uint8_t tx_buffer = reg | 0x80; // Set MSB read bit to 1
	struct spi_buf tx_spi_buf = {.buf = (void *)&tx_buffer, .len = 1};
	struct spi_buf_set tx_spi_buf_set = {.buffers = &tx_spi_buf, .count = 1};
	struct spi_buf rx_spi_bufs = {.buf = buff, .len = len};
	struct spi_buf_set rx_spi_buf_set = {.buffers = &rx_spi_bufs, .count = 1};

	err = spi_transceive_dt(&cfg->spi_bus, &tx_spi_buf_set, &rx_spi_buf_set);
	if (err < 0)
    {
		LOG_ERR("spi transceive failed, err: %d", err);
		return err;
	}
}

static int st25_write_reg(const struct device *dev, uint8_t reg, uint8_t *buff, size_t len)
{
    int err;
    const struct nfc_cfg *cfg = dev->config;

    uint8_t tx_buf[len + 1];
    memset(tx_buf, reg, 1);
    memcpy(tx_buf + 1, buff, len);

    struct spi_buf tx_spi_buf = {.buf = tx_buf, .len = sizeof(tx_buf)};
	struct spi_buf_set tx_spi_buf_set = {.buffers = &tx_spi_buf, .count = 1};

    err = spi_write_dt(&cfg->spi_bus, &tx_spi_buf_set);
	if (err < 0) {
		LOG_ERR("spi_write_dt() failed, err %d", err);
		return err;
	}

    return err;
}

static int st25_write_cmd(const struct device *dev, uint8_t cmd)
{
    int err;
    const struct nfc_cfg *cfg = dev->config;

    uint8_t tx_buf = cmd;
    struct spi_buf tx_spi_buf = {.buf = &tx_buf, .len = 1};
	struct spi_buf_set tx_spi_buf_set = {.buffers = &tx_spi_buf, .count = 1};

    err = spi_write_dt(&cfg->spi_bus, &tx_spi_buf_set);
	if (err < 0) {
		LOG_ERR("spi_write_dt() failed, err %d", err);
		return err;
	}

    return err;
}

static void st25_irq_wh(struct k_work *w)
{
    struct irq_work *work_ctx = CONTAINER_OF(w, struct irq_work, work);
    const struct device *dev = work_ctx->dev;
    const struct nfc_cfg *cfg = dev->config;

    uint8_t data[4];
    int rc = st25_read_reg(dev, ST25_REG_IRQ_SR1, data, sizeof(data));

    if (rc == 0)
    {
        irq_status.val = (data[3] << 16) + (data[2] << 8) + data[1];
        LOG_INF("i_rfu0: %d", irq_status.bits.i_rfu0);
        LOG_INF("i_txe: %d", irq_status.bits.i_txe);
        LOG_INF("i_rxs: %d", irq_status.bits.i_rxs);
        LOG_INF("i_rxe: %d", irq_status.bits.i_rxe);
        LOG_INF("i_rx_reset: %d", irq_status.bits.i_rx_reset);
        LOG_INF("i_wl: %d", irq_status.bits.i_wl);
        LOG_INF("i_col: %d", irq_status.bits.i_col);
        LOG_INF("i_subc_start: %d", irq_status.bits.i_subc_start);
        LOG_INF("i_sfe: %d", irq_status.bits.i_sfe);
        LOG_INF("i_hfe: %d", irq_status.bits.i_hfe);
        LOG_INF("i_par: %d", irq_status.bits.i_par);
        LOG_INF("i_crc: %d", irq_status.bits.i_crc);
        LOG_INF("i_rfu1: %d", irq_status.bits.i_rfu1);
        LOG_INF("i_nre: %d", irq_status.bits.i_nre);
        LOG_INF("i_gpe: %d", irq_status.bits.i_gpe);
        LOG_INF("i_osc: %d", irq_status.bits.i_osc);
        LOG_INF("i_wut: %d", irq_status.bits.i_wut);
        LOG_INF("i_wui: %d", irq_status.bits.i_wui);
        LOG_INF("i_wuq: %d", irq_status.bits.i_wuq);
        LOG_INF("i_dct: %d", irq_status.bits.i_dct);
        LOG_INF("i_rfu2: %d", irq_status.bits.i_rfu2);
    }
    else
    {
        LOG_WRN("IRQ status registers could not be read.");
    }

    if (gpio_pin_get_dt(&cfg->irq_gpio) != 0)
    {
        LOG_ERR("IRQ pin was not de-asserted.");
    }

}

static void st25_irq_cb()
{
    LOG_WRN("Interrupt triggered.");
    k_work_submit(&st25_irq_work.work);
}

static void st25_reset(const struct device *dev)
{
    st25_write_cmd(dev, ST25_CMD_SET_DEFAULT);
}

static int st25_ready_mode(const struct device *dev)
{
    // Config operation and general registers to put into ready mode
    uint8_t temp = FIELD_PREP(ST25_OP_RD_EN_FIELD, 1);
    LOG_WRN("Ready mode writing 0x%x", temp);
    int rc = st25_write_reg(dev, ST25_REG_OP, &temp, 1);

    // Wait for oscillator to establish
    int attempts = 0;
    while(attempts < 5)
    {
        if (irq_status.bits.i_osc) break;
        attempts++;
        k_msleep(5);
    }
    if (attempts == 5)
    {
        LOG_WRN("Oscillator didn't stabize in time.");
        return -ETIME;
    }

    // Enable TX, RX, and AM regulator
    uint8_t data[2];
    rc = st25_read_reg(dev, ST25_REG_OP, data, 2);
    data[1] |= (FIELD_PREP(ST25_OP_RX_EN_FIELD, 1) |
        FIELD_PREP(ST25_OP_TX_EN_FIELD, 1) |
        FIELD_PREP(ST25_OP_AM_EN_FIELD, 1));
    LOG_ERR("writing %x to OP register", data[1]);
    rc = st25_write_reg(dev, ST25_REG_OP, &data[1], 1);

    // Configure AM regulator
    temp = FIELD_PREP(ST25_TX_MOD1_REG_AM, 1) | FIELD_PREP(ST25_TX_MOD1_INDEX, 4);
    rc = st25_write_reg(dev, ST25_REG_TX_MOD1, &temp, 1);
    temp = FIELD_PREP(ST25_TX_PROTOCOL_TR_AM, 7);
    rc = st25_write_reg(dev, ST25_REG_TX_PROTOCOL, &temp, 1);

    // Set RFO resistance
    temp = FIELD_PREP(ST25_DRES, 0);
    rc = st25_write_reg(dev, ST25_REG_TX_DRIVER, &temp, 1);

    // Configure correlator regulator 5
    temp = FIELD_PREP(ST25_CORREL_5_FSUBC, 1) + 3;
    rc = st25_write_reg(dev, ST25_REG_CORREL_5, &temp, 1);

    // Configure rx digital filters
    temp = FIELD_PREP(ST25_RX_DIGITAL_LPF, 2) |
        FIELD_PREP(ST25_RX_DIGITAL_HPF, 0) |
        FIELD_PREP(ST25_RX_DIGITAL_AGC, 1);
    rc = st25_write_reg(dev, ST25_REG_RX_DIGITAL, &temp, 1);

    // Configure correlator 1
    temp = 0xC0;
    rc = st25_write_reg(dev, ST25_REG_CORREL_1, &temp, 1);

    LOG_INF("Adjust regulators command");
    rc = st25_write_cmd(dev, ST25_CMD_ADJ_REG);

    return rc;
}

static int st25_read_tag(const struct device *dev)
{
    int rc = 0;

    // Set mode and bitrates
    uint8_t mode = FIELD_PREP(ST25_PROTOCOL_MODE_FIELD, ST25_PROTOCOL_MODE_NFCV);
    rc = st25_write_reg(dev, ST25_REG_PROTOCOL, &mode, 1);

    // Stop all activities
    LOG_INF("Stop activities command");
    rc = st25_write_cmd(dev, ST25_CMD_STOP_ACT);

    // Clear RX gain
    LOG_INF("Clear activities command");
    rc = st25_write_cmd(dev, ST25_CMD_CLR_RX_GAIN);

    // Configure timers

    // Clear FIFO
    rc = st25_write_cmd(dev, ST25_CMD_CLR_FIFO);

    // Define TX length
    int read_cmd_len = 3;
    uint8_t bytes_to_tx = FIELD_PREP(ST25_FIFO_FRAME2_NTXLSB_FIELD, read_cmd_len) | FIELD_PREP(0b111, 1);
    rc = st25_write_reg(dev, ST25_REG_FIFO_FRAME2, &bytes_to_tx, 1);

    // Write TX bytes to FIFO
    uint8_t tx_bytes[] = {0x02, 0x20, 0x01};
    rc = st25_write_reg(dev, ST25_REG_FIFO, tx_bytes, 3);

    // uint8_t read1[5] = {0};
    // rc = st25_read_reg(dev, ST25_REG_FIFO, read1, 3);
    // LOG_ERR("%x %x %x", read1[1], read1[2], read1[3]);

    // Send direct command to tx
    // rc = st25_write_cmd(dev, ST25_CMD_UNMASK_RX);

    LOG_INF("TX data command");
    rc = st25_write_cmd(dev, ST25_CMD_TX_DATA);

    rc = st25_write_cmd(dev, ST25_CMD_TX_EOF);
}

static int st25_config_gpio(const struct device *dev)
{
    const struct nfc_cfg *cfg = dev->config;
    int status = 0;

    // Retrieve actual GPIO devices
    if (!device_is_ready(cfg->reset_gpio.port))
    {
        LOG_ERR("Could not find reset pin gpio device.");
        return -ENODEV;
    }
    if (!device_is_ready(cfg->irq_gpio.port))
    {
        LOG_ERR("Could not find IRQ pin gpio device.");
        return -ENODEV;
    }

    // Configure pins
    status = gpio_pin_configure_dt(&cfg->reset_gpio, GPIO_OUTPUT_INACTIVE);
    if (status != 0)
    {
        LOG_ERR("Error %d configuring gpio=>reset", status);
        return status;
    }

    status = gpio_pin_configure_dt(&cfg->irq_gpio, GPIO_INPUT);
    if (status != 0)
    {
        LOG_ERR("Error %d configuring gpio=>irq", status);
        return status;
    }
    gpio_pin_interrupt_configure_dt(&cfg->irq_gpio, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&irq_cb_data, st25_irq_cb, BIT(cfg->irq_gpio.pin));
    gpio_add_callback(cfg->irq_gpio.port, &irq_cb_data);

    return status;
}

static int st25r100_init(const struct device *dev)
{
    int rc;

    k_msleep(5000);

    // Config the work handler for IRQs before config GPIO
    st25_irq_work.dev = dev;
    k_work_init(&st25_irq_work.work, st25_irq_wh);

    rc = st25_config_gpio(dev);

    st25_reset(dev);
    k_msleep(100);

    // Check for chip ID
    uint8_t buf[2] = {0};
    rc = st25_read_reg(dev, ST25_REG_ID, buf, 2);
    if (rc != 0)
    {
        LOG_WRN("Failed to read chip ID: %d", rc);
        return -EIO;
    }
    if (buf[1] != ST25_VAL_ID)
    {
        LOG_ERR("Chip ID did not match (expected 0x%x, got 0x%x)", ST25_VAL_ID, buf[1]);
        return -ENODEV;
    }
    LOG_INF("ST25 chip ID matched.");


    // TEMP
    st25_ready_mode(dev);
    st25_read_tag(dev);

    struct nfc_cfg *c = dev->config;
    LOG_WRN("irq pin: %d", gpio_pin_get_dt(&c->irq_gpio));

    return 0;
}

static struct nfc_data st25_drv_data;
static const struct nfc_cfg st25_drv_cfg = {
    .spi_bus = SPI_DT_SPEC_INST_GET(0, SPI_OP_MODE_MASTER | \
					    0 << 1 |		            \
					    1 << 2 |		            \
					    SPI_WORD_SET(8) |		    \
					    SPI_TRANSFER_MSB,
					    0U),
    .reset_gpio = GPIO_DT_SPEC_INST_GET(0, reset_gpios),
    .irq_gpio = GPIO_DT_SPEC_INST_GET(0, irq_gpios),
};

DEVICE_DT_INST_DEFINE(0, st25r100_init,
    NULL,
    &st25_drv_data,
    &st25_drv_cfg,
    POST_KERNEL, 92,
    NULL
);
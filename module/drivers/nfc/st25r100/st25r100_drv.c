# include "st25r100_drv.h"
#include "st25r100_dev_structs.h"

#define DT_DRV_COMPAT st_st25r100

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

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

static int st25_reg_read(const struct device *dev, uint8_t reg, uint8_t *buff, size_t len)
{
    int err;
    const struct nfc_cfg *cfg = dev->config;

    uint8_t temp_buf[len + 1];

    uint8_t tx_buffer = reg | 0x80; // Set MSB read bit to 1
	struct spi_buf tx_spi_buf = {.buf = (void *)&tx_buffer, .len = 1};
	struct spi_buf_set tx_spi_buf_set = {.buffers = &tx_spi_buf, .count = 1};
	struct spi_buf rx_spi_bufs = {.buf = temp_buf, .len = sizeof(temp_buf)};
	struct spi_buf_set rx_spi_buf_set = {.buffers = &rx_spi_bufs, .count = 1};

	err = spi_transceive_dt(&cfg->spi_bus, &tx_spi_buf_set, &rx_spi_buf_set);
	if (err < 0)
    {
		LOG_ERR("spi transceive failed, err: %d", err);
		return err;
	}

    memcpy(buff, temp_buf + 1, len);
    return 0;
}

static int st25_reg_write(const struct device *dev, uint8_t reg, uint8_t *buff, size_t len)
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
    LOG_INF("Writing to %x:", reg);
    for (int i=0; i<len; i++)
    {
        LOG_INF("%x,", buff[i]);
    }

    return err;
}

static int st25_cmd_write(const struct device *dev, uint8_t cmd)
{
    int err;
    const struct nfc_cfg *cfg = dev->config;

    uint8_t tx_buf = cmd;
    struct spi_buf tx_spi_buf = {.buf = &tx_buf, .len = 1};
	struct spi_buf_set tx_spi_buf_set = {.buffers = &tx_spi_buf, .count = 1};

    err = spi_write_dt(&cfg->spi_bus, &tx_spi_buf_set);
    LOG_INF("Direct command: %x", cmd);
	if (err < 0) {
		LOG_ERR("spi_write_dt() failed, err %d", err);
		return err;
	}

    return err;
}

static int st25_reg_set_bits(const struct device *dev, uint8_t reg, uint8_t mask)
{
    // Read register
    uint8_t old_val;
    int rc = st25_reg_read(dev, reg, &old_val, 1);

    // Set new bits based on the mask
    if (rc == 0)
    {
        uint8_t new_val = old_val | mask;
        rc = st25_reg_write(dev, reg, &new_val, 1);
        LOG_INF("Set bits @ 0x%x: 0x%x -> 0x%x", reg, old_val, new_val);
    }

    return rc;
}

static int st25_reg_clear_bits(const struct device *dev, uint8_t reg, uint8_t mask)
{
    // Read register
    uint8_t old_val;
    int rc = st25_reg_read(dev, reg, &old_val, 1);

    // Set new bits based on the mask
    if (rc == 0)
    {
        uint8_t new_val = old_val & ~mask;
        rc = st25_reg_write(dev, reg, &new_val, 1);
        LOG_INF("Clear bits @ 0x%x: 0x%x -> 0x%x", reg, old_val, new_val);
    }

    return rc;
}

static int st25_reg_write_masked(const struct device *dev, uint8_t reg, uint8_t value, uint8_t mask)
{
    // Read register
    uint8_t old_val;
    int rc = st25_reg_read(dev, reg, &old_val, 1);

    // Set new bits based on the mask
    if (rc == 0)
    {
        uint8_t new_val = old_val & ~mask;
        new_val |= value;
        rc = st25_reg_write(dev, reg, &new_val, 1);
        LOG_INF("Write bits @ 0x%x: 0x%x -> 0x%x", reg, old_val, new_val);
    }

    return rc;
}

static void st25_irq_wh(struct k_work *w)
{
    struct irq_work *work_ctx = CONTAINER_OF(w, struct irq_work, work);
    const struct device *dev = work_ctx->dev;
    const struct nfc_cfg *cfg = dev->config;

    uint8_t data[3];
    int rc = st25_reg_read(dev, ST25_REG_IRQ_SR1, data, sizeof(data));

    if (rc == 0)
    {
        irq_status.val = (data[2] << 16) + (data[1] << 8) + data[0];
        // LOG_INF("i_rfu0: %d", irq_status.bits.i_rfu0);
        // LOG_INF("i_txe: %d", irq_status.bits.i_txe);
        // LOG_INF("i_rxs: %d", irq_status.bits.i_rxs);
        // LOG_INF("i_rxe: %d", irq_status.bits.i_rxe);
        // LOG_INF("i_rx_reset: %d", irq_status.bits.i_rx_reset);
        // LOG_INF("i_wl: %d", irq_status.bits.i_wl);
        // LOG_INF("i_col: %d", irq_status.bits.i_col);
        // LOG_INF("i_subc_start: %d", irq_status.bits.i_subc_start);
        // LOG_INF("i_sfe: %d", irq_status.bits.i_sfe);
        // LOG_INF("i_hfe: %d", irq_status.bits.i_hfe);
        // LOG_INF("i_par: %d", irq_status.bits.i_par);
        // LOG_INF("i_crc: %d", irq_status.bits.i_crc);
        // LOG_INF("i_rfu1: %d", irq_status.bits.i_rfu1);
        // LOG_INF("i_nre: %d", irq_status.bits.i_nre);
        // LOG_INF("i_gpe: %d", irq_status.bits.i_gpe);
        // LOG_INF("i_osc: %d", irq_status.bits.i_osc);
        // LOG_INF("i_wut: %d", irq_status.bits.i_wut);
        // LOG_INF("i_wui: %d", irq_status.bits.i_wui);
        // LOG_INF("i_wuq: %d", irq_status.bits.i_wuq);
        // LOG_INF("i_dct: %d", irq_status.bits.i_dct);
        // LOG_INF("i_rfu2: %d", irq_status.bits.i_rfu2);
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

static int st25_wake(const struct device *dev)
{
    const struct nfc_cfg *cfg = dev->config;
    gpio_pin_set_dt(&cfg->reset_gpio, 0);

    // Config operation and general registers to put into ready mode
    uint8_t temp = FIELD_PREP(ST25_OP_EN_FIELD, 1);
    int rc = st25_reg_set_bits(dev, ST25_REG_OP, ST25_OP_EN_FIELD);

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

    // Wait for agd_ok bit
    attempts = 0;
    while(attempts < 5)
    {
        rc = st25_reg_read(dev, ST25_REG_DR1, &temp, 1);
        if (temp & ST25_DR1_AGD_OK_FIELD) break;
        attempts++;
        k_msleep(5);
    }
    if (attempts == 5)
    {
        LOG_WRN("AGD bit not set in time.");
        return -ETIME;
    }

    return 0;
}

static int st25_sleep(const struct device *dev)
{
    const struct nfc_cfg *cfg = dev->config;
    gpio_pin_set_dt(&cfg->reset_gpio, 1);
    return 0;
}

static int st25_ready_mode(const struct device *dev)
{
    st25_wake(dev);

    // Pullup resistor configuration
    uint8_t temp = ST25_GENERAL_MISO_PD1 | ST25_GENERAL_MISO_PD2;
    int rc = st25_reg_set_bits(dev, ST25_REG_GENERAL, temp);

    // Enable am regulator and set modulation index to 4 (12%)
    rc = st25_reg_set_bits(dev, ST25_REG_OP, ST25_OP_AM_EN_FIELD);
    temp = ST25_TX_MOD1_REG_AM | FIELD_PREP(ST25_TX_MOD1_AM_INDEX, 4);
    LOG_INF("mod1: %x", temp);
    rc = st25_reg_write_masked(dev,
                                ST25_REG_TX_MOD1,
                                temp,
                                ST25_TX_MOD1_REG_AM | ST25_TX_MOD1_AM_INDEX);

    // Set RFO driver resistance
    rc = st25_reg_clear_bits(dev, ST25_REG_TX_DRIVER, ST25_TX_DRIVER_DRES);

    // Enable subcarrier and set IIR
    temp = FIELD_PREP(ST25_CORREL_5_SUBC_EN, 1) | FIELD_PREP(ST25_CORREL_5_IIR_F, 4);
    rc = st25_reg_write_masked(dev, ST25_REG_CORREL_5, temp, ST25_CORREL_5_SUBC_EN | ST25_CORREL_5_IIR_F);

    // Set afe gain
    temp = FIELD_PREP(ST25_ANALOG_2_AFE_TD, 4);
    rc = st25_reg_write_masked(dev, ST25_REG_ANALOG_2, temp, ST25_ANALOG_2_AFE_TD);

    // Calibrate regulators
    rc = st25_reg_clear_bits(dev, ST25_REG_GENERAL, ST25_GENERAL_REG_S);
    rc = st25_cmd_write(dev, ST25_CMD_ADJ_REG);
    // Wait for i_dict irq
    int attempts = 0;
    while(attempts < 5)
    {
        if (irq_status.bits.i_dct) break;
        attempts++;
        k_msleep(5);
    }
    if (attempts == 5)
    {
        LOG_WRN("Adjust regulator cmd timeout.");
        return -ETIME;
    }

    // Set no response timer value
    temp = 0x41;
    rc = st25_reg_write(dev, ST25_REG_MASK_RX_TIMER, &temp, 1);

    // Operation mode
    temp = FIELD_PREP(ST25_PROTOCOL_MODE_FIELD, ST25_PROTOCOL_MODE_NFCV);
    rc = st25_reg_write_masked(dev, ST25_REG_PROTOCOL, temp, ST25_PROTOCOL_MODE_FIELD);

    // Use OOK tx modulation
    rc = st25_reg_clear_bits(dev, ST25_REG_PROTOCOL, ST25_TX_PROTOCOL_TR_AM);

    // Configure filters (lpf = 2 hpf = 0)
    temp = FIELD_PREP(ST25_RX_DIGITAL_HPF | ST25_RX_DIGITAL_LPF, 8);
    rc = st25_reg_write_masked(dev,
                                ST25_REG_RX_DIGITAL,
                                temp,
                                ST25_RX_DIGITAL_HPF | ST25_RX_DIGITAL_LPF);

    // Configure IIR
    temp = FIELD_PREP(ST25_CORREL_1_IIR1 | ST25_CORREL_1_IIR2, 0xC0);
    rc = st25_reg_write_masked(dev,
                                ST25_REG_CORREL_1,
                                temp,
                                ST25_CORREL_1_IIR1 | ST25_CORREL_1_IIR2);

    // Parity and crc for TX
    rc = st25_reg_set_bits(dev, ST25_REG_TX_PROTOCOL, ST25_TX_PROTOCOL_CRC | ST25_TX_PROTOCOL_PAR);

    // Parity and CRC for RX
    rc = st25_reg_set_bits(dev, ST25_REG_RX_PROTOCOL, ST25_RX_PROTOCOL_CRC | ST25_RX_PROTOCOL_PAR);

    // Enable AGC
    rc = st25_reg_set_bits(dev, ST25_REG_RX_DIGITAL, ST25_RX_DIGITAL_AGC);

    st25_sleep(dev);

    return rc;
}

static int st25_read_tag(const struct device *dev, uint8_t start_blk, uint8_t *buf, uint8_t blks)
{
    int rc = 0;

    st25_wake(dev);

    // Enable TX and RX
    int set = FIELD_PREP(ST25_OP_RX_EN_FIELD, 1) | FIELD_PREP(ST25_OP_TX_EN_FIELD, 1);
    rc = st25_reg_write_masked(dev, ST25_REG_OP, set, ST25_OP_RX_EN_FIELD | ST25_OP_TX_EN_FIELD);

    // Wait for guard time
    k_msleep(6);

    // Stop all activities
    rc = st25_cmd_write(dev, ST25_CMD_STOP_ACT);

    // Clear RX gain
    rc = st25_cmd_write(dev, ST25_CMD_CLR_RX_GAIN);

    // Unmask RX
    st25_cmd_write(dev, ST25_CMD_UNMASK_RX);

    // Define TX length and write to FIFO
    uint8_t req_payload[NFC_READ_MLT_BLOCKS_PL_LEN] = {0};
    req_payload[0] = 0x02; // Op flags
    req_payload[1] = 0x23; // Read multiple blocks command
    req_payload[2] = start_blk; // Start reading block num
    req_payload[3] = start_blk + blks - 1; // End block
    uint8_t temp = FIELD_PREP(ST25_FIFO_FRAME2_NTXLSB_FIELD, NFC_READ_MLT_BLOCKS_PL_LEN);
    st25_reg_write_masked(dev, ST25_REG_FIFO_FRAME2, temp, ST25_FIFO_FRAME2_NTXLSB_FIELD);
    st25_reg_write(dev, ST25_REG_FIFO, req_payload, NFC_READ_MLT_BLOCKS_PL_LEN);

    uint8_t test;
    st25_reg_read(dev, ST25_REG_FIFO_SR1, &test, 1);
    LOG_INF("fsr1 %x", test);
    st25_reg_read(dev, ST25_REG_FIFO_SR2, &test, 1);
    LOG_INF("fsr2 %x", test);

    st25_cmd_write(dev, ST25_CMD_TX_DATA);

    // Wait for rxe irq
    int attempts = 0;
    while(attempts < 10)
    {
        if (irq_status.bits.i_rxe) break;
        attempts++;
        k_msleep(5);
    }
    if (attempts == 10)
    {
        LOG_WRN("RXE timeout.");
        return -ETIME;
    }

    uint16_t bytes_in_fifo = 0;
    st25_reg_read(dev, ST25_REG_FIFO_SR1, &test, 1);
    bytes_in_fifo = test;
    LOG_INF("fsr1 %x", test);
    st25_reg_read(dev, ST25_REG_FIFO_SR2, &test, 1);
    LOG_INF("fsr2 %x", test);
    bytes_in_fifo |= (uint16_t)(test << 8);

    uint8_t fifo_buf[bytes_in_fifo];
    rc = st25_reg_read(dev, ST25_REG_FIFO, fifo_buf, bytes_in_fifo);

    if (blks*4 >= bytes_in_fifo - 3)
    {
        memcpy(buf, fifo_buf + 1, bytes_in_fifo);
    }

    st25_sleep(dev);

    return 0;
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

    k_msleep(2000);

    // Config the work handler for IRQs before config GPIO
    st25_irq_work.dev = dev;
    k_work_init(&st25_irq_work.work, st25_irq_wh);

    rc = st25_config_gpio(dev);

    // Set to default state
    st25_cmd_write(dev, ST25_CMD_SET_DEFAULT);
    k_msleep(100);

    // Check for chip ID
    uint8_t wai;
    rc = st25_reg_read(dev, ST25_REG_ID, &wai, 1);
    if (rc != 0)
    {
        LOG_WRN("Failed to read chip ID: %d", rc);
        return -EIO;
    }
    if (wai != ST25_VAL_ID)
    {
        LOG_ERR("Chip ID did not match (expected 0x%x, got 0x%x)", ST25_VAL_ID, wai);
        return -ENODEV;
    }
    LOG_INF("ST25 chip ID matched.");

    st25_ready_mode(dev);
    uint8_t tag_data[3*4] = {0};
    st25_read_tag(dev, 0, tag_data, 3);
    for (int i=0; i<sizeof(tag_data); i++)
    {
        printk("%x ", tag_data[i]);
    }
    printk("\n");

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
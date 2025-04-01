/*
 * Copyright (c) 2025 Texas Instruments Incorporated
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#define DT_DRV_COMPAT ti_omap_mcspi

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(omap_mcspi);

#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/pinctrl.h>
#include "spi_context.h"

#define OMAP_MCSPI_MAX_DIVIDER   (4096)
#define OMAP_MCSPI_MAX_FREQUENCY (48000000)
#define OMAP_MCSPI_NUM_CHANNELS  (4)

struct omap_mcspi_regs {
	uint8_t RESERVED_1[0x110];   /**< Reserved, offset: 0x00 - 0x110 */
	volatile uint32_t SYSCONFIG; /**< Configuration register, offset: 0x110 */
	volatile uint32_t SYSSTATUS; /**< Status information register, offset: 0x114 */
	uint8_t RESERVED_2[0x10];    /**< Reserved, offset: 0x114 - 0x128 */
	volatile uint32_t MODULCTRL; /**< MCSPI configuration register, offset: 0x128 */
	volatile struct {
		volatile uint32_t CHCONF; /**< Configuration register, offset: 0x12C + (0x14 * i) */
		volatile uint32_t CHSTAT; /**< Status register, offset: 0x130 + (0x14 * i) */
		volatile uint32_t CHCTRL; /**< Control register, offset: 0x134 + (0x14 * i) */
		volatile uint32_t TX;     /**< TX register, offset: 0x138 + (0x14 * i) */
		volatile uint32_t RX;     /**< RX register, offset: 0x13C + (0x14 * i) */
	} CHAN[OMAP_MCSPI_NUM_CHANNELS];
};

/* Configuration Register */
#define OMAP_MCSPI_SYSCONFIG_SOFTRESET BIT(1) /* Software Reset */

/* Status Register */
#define OMAP_MCSPI_SYSSTATUS_RESETDONE BIT(0) /* Internal Reset Monitoring */

/* MCSPI Configuration Register */
#define OMAP_MCSPI_MODULCTRL_SINGLE  BIT(0) /* Single channel mode (controller only) */
#define OMAP_MCSPI_MODULCTRL_MS      BIT(2) /* Peripheral mode */
#define OMAP_MCSPI_MODULCTRL_SYSTEST BIT(3) /* System test mode */

/* Channel Configuration */
#define OMAP_MCSPI_CHCONF_CLKG        BIT(29)         /* Clock divider granularity */
#define OMAP_MCSPI_CHCONF_FORCE       BIT(20)         /* Manual SPIEN assertion */
#define OMAP_MCSPI_CHCONF_TURBO       BIT(19)         /* Turbo mode */
#define OMAP_MCSPI_CHCONF_IS          BIT(18)         /* Input select */
#define OMAP_MCSPI_CHCONF_DPE1        BIT(17)         /* Transmission enabled for data line 1 */
#define OMAP_MCSPI_CHCONF_DPE0        BIT(16)         /* Transmission enabled for data line 0 */
#define OMAP_MCSPI_CHCONF_TRM         GENMASK(13, 12) /* TX/RX mode */
#define OMAP_MCSPI_CHCONF_TRM_TX_ONLY BIT(13)         /* TX/RX mode - Transmit only */
#define OMAP_MCSPI_CHCONF_TRM_RX_ONLY BIT(12)         /* TX/RX mode - Receive only */
#define OMAP_MCSPI_CHCONF_WL          GENMASK(11, 7)  /* SPI word length */
#define OMAP_MCSPI_CHCONF_EPOL        BIT(6)          /* SPIEN polarity */
#define OMAP_MCSPI_CHCONF_CLKD        GENMASK(5, 2)   /* Frequency divider for SPICLK */
#define OMAP_MCSPI_CHCONF_POL         BIT(1)          /* SPICLK polarity */
#define OMAP_MCSPI_CHCONF_PHA         BIT(0)          /* SPICLK phase */

/* Channel Control Register */
#define OMAP_MCSPI_CHCTRL_EXTCLK GENMASK(15, 8) /* Clock ratio extension (concat with CLKD) */
#define OMAP_MCSPI_CHCTRL_EN     BIT(0)         /* Channel enable */

/* Channel Status Register */
#define OMAP_MCSPI_CHSTAT_EOT BIT(2) /* End of transfer status */
#define OMAP_MCSPI_CHSTAT_TXS BIT(1) /* Transmitter register empty status */
#define OMAP_MCSPI_CHSTAT_RXS BIT(0) /* Receiver register full status */

#define DEV_CFG(dev)  ((const struct omap_mcspi_cfg *)(dev)->config)
#define DEV_DATA(dev) ((struct omap_mcspi_data *)(dev)->data)

struct omap_mcspi_cfg {
	struct omap_mcspi_regs *regs;
	const struct pinctrl_dev_config *pinctrl;
	uint8_t num_cs;
	uint32_t clock_frequency;
};

struct omap_mcspi_data {
	struct spi_context ctx;
	uint8_t dfs; /* data frame size - word length in bytes */
	uint32_t chconf;
	uint32_t chctrl;
};

static void omap_mcspi_enable_channel(const struct device *dev, bool enable)
{
	const struct omap_mcspi_cfg *cfg = DEV_CFG(dev);
	struct omap_mcspi_regs *regs = cfg->regs;
	uint8_t chan = DEV_DATA(dev)->ctx.config->slave;

	regs->CHAN[chan].CHCTRL |= FIELD_PREP(OMAP_MCSPI_CHCTRL_EN, enable);
}

static void omap_mcspi_set_mode(const struct device *dev, bool is_peripheral)
{
	const struct omap_mcspi_cfg *cfg = DEV_CFG(dev);
	struct omap_mcspi_regs *regs = cfg->regs;
	uint32_t modulctrl = regs->MODULCTRL;

	/* disable system test mode */
	modulctrl &= ~(OMAP_MCSPI_MODULCTRL_SYSTEST);

	/* set controller or peripheral (master/slave) */
	if (is_peripheral) {
		modulctrl |= OMAP_MCSPI_MODULCTRL_MS;
	} else {
		modulctrl &= ~OMAP_MCSPI_MODULCTRL_MS;

		/* We only support single-mode for now
		 * TODO: add multi-mode
		 */
		modulctrl |= OMAP_MCSPI_MODULCTRL_SINGLE;
	}

	regs->MODULCTRL = modulctrl;
}

static uint32_t omap_mcspi_calc_divisor(uint32_t speed_hz, uint32_t ref_hz)
{
	uint32_t div;

	for (div = 0; div < 15; div++) {
		if (speed_hz >= (ref_hz >> div)) {
			return div;
		}
	}

	return 15;
}

static void omap_mcspi_configure_clk_freq(const struct device *dev, uint32_t speed_hz,
					  uint32_t ref_hz)
{
	struct omap_mcspi_data *data = DEV_DATA(dev);
	uint32_t clkd = 0, clkg, div = 0, extclk = 0;

	speed_hz = MIN(speed_hz, ref_hz);
	if (speed_hz < (ref_hz / OMAP_MCSPI_MAX_DIVIDER)) {
		speed_hz = ref_hz >> clkd;
		clkd = omap_mcspi_calc_divisor(speed_hz, ref_hz);
		clkg = 0;
	} else {
		div = (ref_hz + speed_hz - 1) / speed_hz;
		speed_hz = ref_hz / div;
		clkd = (div - 1) & 0xf;
		clkg = 1;
		extclk = (div - 1) >> 4;
	}

	data->chconf &= ~OMAP_MCSPI_CHCONF_CLKD;
	data->chconf |= FIELD_PREP(OMAP_MCSPI_CHCONF_CLKD, clkd);

	if (clkg) {
		data->chconf |= OMAP_MCSPI_CHCONF_CLKG;
		data->chctrl &= ~OMAP_MCSPI_CHCTRL_EXTCLK;
		data->chctrl |= FIELD_PREP(OMAP_MCSPI_CHCTRL_EXTCLK, extclk);
	} else {
		data->chconf &= ~OMAP_MCSPI_CHCONF_CLKG;
	}
}

static int omap_mcspi_configure(const struct device *dev, const struct spi_config *config,
				const struct spi_buf_set *tx_bufs,
				const struct spi_buf_set *rx_bufs)
{
	const struct omap_mcspi_cfg *cfg = DEV_CFG(dev);
	struct omap_mcspi_data *data = DEV_DATA(dev);
	struct omap_mcspi_regs *regs = cfg->regs;
	struct spi_context *ctx = &data->ctx;
	uint8_t chan = config->slave;
	uint8_t word_size = SPI_WORD_SIZE_GET(config->operation);

	if (spi_context_configured(ctx, config)) {
		/* This configuration is already in use */
		return 0;
	}

	if (chan >= cfg->num_cs) {
		LOG_ERR("invalid slave selected");
		return -EINVAL;
	}

	if ((config->operation & SPI_HALF_DUPLEX) && tx_bufs && rx_bufs) {
		LOG_ERR("cannot transmit and receive simultaneously with half duplex");
		return -EPROTO;
	}

	if (word_size < 4 || word_size > 32) {
		LOG_ERR("invalid word size");
		return -EINVAL;
	}

	/* update data frame size (word size in bytes) */
	if (word_size <= 8) {
		data->dfs = 1;
	} else if (word_size <= 16) {
		data->dfs = 2;
	} else { /* word_size <= 32 */
		data->dfs = 4;
	}

	/* only when MODULCTRL_SINGLE is set */
	ARRAY_FOR_EACH(regs->CHAN, ch) {
		if (ch != chan) {
			regs->CHAN[ch].CHCTRL &= ~OMAP_MCSPI_CHCTRL_EN;
			regs->CHAN[ch].CHCONF &= ~OMAP_MCSPI_CHCONF_FORCE;
		}
	}

	/* set mode */
	omap_mcspi_set_mode(dev, config->operation & SPI_OP_MODE_SLAVE);

	/* update cached registers */
	data->chconf = regs->CHAN[chan].CHCONF;
	data->chctrl = regs->CHAN[chan].CHCTRL;

	/* configure word length */
	data->chconf &= ~OMAP_MCSPI_CHCONF_WL;
	data->chconf |= FIELD_PREP(OMAP_MCSPI_CHCONF_WL, word_size - 1);

	if (config->operation & SPI_MODE_LOOP) {
		/* d0-in d0-out, loopback */
		data->chconf &= ~(OMAP_MCSPI_CHCONF_IS);
		data->chconf |= (OMAP_MCSPI_CHCONF_DPE1);
		data->chconf &= ~(OMAP_MCSPI_CHCONF_DPE0);
	} else {
		/* d0-in d1-out, default */
		data->chconf &= ~(OMAP_MCSPI_CHCONF_IS);
		data->chconf &= ~(OMAP_MCSPI_CHCONF_DPE1);
		data->chconf |= (OMAP_MCSPI_CHCONF_DPE0);
	}

	/* configure spien polarity */
	if (!(config->operation & SPI_CS_ACTIVE_HIGH)) {
		data->chconf |= OMAP_MCSPI_CHCONF_EPOL;
	} else {
		data->chconf &= ~OMAP_MCSPI_CHCONF_EPOL;
	}

	/* set clk polarity */
	if (config->operation & SPI_MODE_CPOL) {
		data->chconf |= OMAP_MCSPI_CHCONF_POL;
	} else {
		data->chconf &= ~OMAP_MCSPI_CHCONF_POL;
	}

	/* set clk phase */
	if (config->operation & SPI_MODE_CPHA) {
		data->chconf |= OMAP_MCSPI_CHCONF_PHA;
	} else {
		data->chconf &= ~OMAP_MCSPI_CHCONF_PHA;
	}

	/* set force */
	if (!config->cs.gpio.port) {
		data->chconf |= OMAP_MCSPI_CHCONF_FORCE;
	} else {
		data->chconf &= ~OMAP_MCSPI_CHCONF_FORCE;
	}

	omap_mcspi_configure_clk_freq(dev, config->frequency, cfg->clock_frequency);

	/* save config in the context */
	ctx->config = config;
	return 0;
}

static int omap_mcspi_wait_for_reg_bit(volatile uint32_t *reg, uint8_t bit)
{
	uint64_t timeout = k_uptime_get();

	while (!(*reg & bit)) {
		/* 1s timeout */
		if (k_uptime_get() - timeout < 1000) {
			return -ETIMEDOUT;
		}

		k_busy_wait(5000); /* 5 ms */
	}

	return 0;
}

static int omap_mcspi_transceive_pio(const struct device *dev, size_t count)
{
	const struct omap_mcspi_cfg *cfg = DEV_CFG(dev);
	struct omap_mcspi_data *data = DEV_DATA(dev);
	struct omap_mcspi_regs *regs = cfg->regs;
	struct spi_context *ctx = &data->ctx;
	const uint8_t chan = ctx->config->slave;
	const bool turbo = data->chconf & OMAP_MCSPI_CHCONF_TURBO;

	volatile uint32_t *chstat = &regs->CHAN[chan].CHSTAT;
	volatile uint32_t *tx_reg = &regs->CHAN[chan].TX;
	volatile uint32_t *rx_reg = &regs->CHAN[chan].RX;

#define WAIT_FOR_STAT(txrx)                                                                        \
	if (omap_mcspi_wait_for_reg_bit(chstat, OMAP_MCSPI_CHSTAT_##txrx) < 0) {                   \
		LOG_ERR(#txrx "timed out");                                                        \
		return count;                                                                      \
	}

#define PROCESS_TXRX()                                                                             \
	do {                                                                                       \
		count--;                                                                           \
                                                                                                   \
		if (tx) {                                                                          \
			WAIT_FOR_STAT(TXS); /* tx */                                               \
			*tx_reg = *tx++;                                                           \
		}                                                                                  \
                                                                                                   \
		if (rx) {                                                                          \
			if (count == 1 && !tx && turbo) {                                          \
				WAIT_FOR_STAT(RXS); /* extra rx for turbo */                       \
				*rx++ = *rx_reg;                                                   \
				count = 0;                                                         \
			}                                                                          \
                                                                                                   \
			WAIT_FOR_STAT(RXS); /* rx */                                               \
			*rx++ = *rx_reg;                                                           \
		}                                                                                  \
	} while (count)

	if (data->dfs == 1) {
		const uint8_t *tx = ctx->tx_buf;
		uint8_t *rx = ctx->rx_buf;

		PROCESS_TXRX();
	} else if (data->dfs == 2) {
		const uint16_t *tx = (const uint16_t *)ctx->tx_buf;
		uint16_t *rx = (uint16_t *)ctx->rx_buf;

		PROCESS_TXRX();
	} else {
		const uint32_t *tx = (const uint32_t *)ctx->tx_buf;
		uint32_t *rx = (uint32_t *)ctx->rx_buf;

		PROCESS_TXRX();
	}

#undef PROCESS_TXRX
#undef WAIT_FOR_STAT

	/* for TX_ONLY mode, be sure all words have shifted out */
	if (!ctx->rx_buf) {
		if (omap_mcspi_wait_for_reg_bit(chstat, OMAP_MCSPI_CHSTAT_TXS) < 0) {
			LOG_ERR("TXS timed out");
		} else if (omap_mcspi_wait_for_reg_bit(chstat, OMAP_MCSPI_CHSTAT_EOT) < 0) {
			LOG_ERR("EOT timed out");
		}

		/* disable chan to purge rx datas received in TX_ONLY transfer,
		 * otherwise these rx datas will affect the direct following
		 * RX_ONLY transfer.
		 */
		omap_mcspi_enable_channel(dev, false);
	}

	return count;
}

static int omap_mcspi_transceive_one(const struct device *dev)
{
	const struct omap_mcspi_cfg *cfg = DEV_CFG(dev);
	struct omap_mcspi_data *data = DEV_DATA(dev);
	struct omap_mcspi_regs *regs = cfg->regs;
	struct spi_context *ctx = &data->ctx;
	const uint8_t chan = ctx->config->slave;
	const uint8_t *tx = ctx->tx_buf, *rx = ctx->rx_buf;
	size_t count = 0;
	int ret = 0, rv;

	/* disable channel */
	omap_mcspi_enable_channel(dev, false);

	/* configure chconf */
	data->chconf &= ~OMAP_MCSPI_CHCONF_TRM;
	if (!tx) {
		data->chconf |= OMAP_MCSPI_CHCONF_TRM_RX_ONLY;
		/* more than one word */
		if (ctx->rx_len > 1) {
			data->chconf |= OMAP_MCSPI_CHCONF_TURBO;
		}

		count = ctx->rx_len;
	} else if (!rx) {
		data->chconf |= OMAP_MCSPI_CHCONF_TRM_TX_ONLY;
		count = ctx->tx_len;
	} else {
		count = MIN(ctx->tx_len, ctx->rx_len);
	}

	/* write chconf and chctrl */
	regs->CHAN[chan].CHCONF = data->chconf;
	regs->CHAN[chan].CHCTRL = data->chctrl;

	/* enable channel */
	omap_mcspi_enable_channel(dev, true);

	if (!tx) {
		/* for rx-only, we must write something to TX */
		regs->CHAN[chan].TX = 0;
	}

	/* we only support PIO for now
	 * TODO: add DMA
	 */
	rv = omap_mcspi_transceive_pio(dev, count);
	if (rv) {
		ret = -EIO;
	}

	return ret;
}

static int omap_mcspi_transceive(const struct device *dev, const struct spi_config *config,
				 const struct spi_buf_set *tx_bufs,
				 const struct spi_buf_set *rx_bufs)
{
	struct omap_mcspi_data *data = DEV_DATA(dev);
	struct spi_context *ctx = &data->ctx;
	int ret = 0, rv;

	if (!tx_bufs && !rx_bufs) {
		return 0;
	}

	spi_context_lock(ctx, false, NULL, NULL, config);

	ret = omap_mcspi_configure(dev, config, tx_bufs, rx_bufs);
	if (ret) {
		LOG_ERR("An error occurred in the SPI configuration");
		goto cleanup;
	}

	spi_context_buffers_setup(ctx, tx_bufs, rx_bufs, data->dfs);

	while (spi_context_tx_on(ctx) || spi_context_rx_on(ctx)) {
		rv = omap_mcspi_transceive_one(dev);
		if (rv < 0) {
			LOG_ERR("One transaction failed: %s", strerror(rv));
		}

		/* next rx buffer */
		if (ctx->rx_buf) {
			spi_context_update_rx(ctx, data->dfs, ctx->rx_len);
		}

		/* next tx buffer */
		if (ctx->tx_buf) {
			spi_context_update_tx(ctx, data->dfs, ctx->tx_len);
		}
	}

cleanup:
	spi_context_release(ctx, ret);
	return ret;
}

static int omap_mcspi_init(const struct device *dev)
{
	const struct omap_mcspi_cfg *cfg = DEV_CFG(dev);
	struct omap_mcspi_data *data = DEV_DATA(dev);
	struct omap_mcspi_regs *regs = cfg->regs;
	volatile uint32_t *sysstatus = &regs->SYSSTATUS;
	int ret;

	if (cfg->num_cs > OMAP_MCSPI_NUM_CHANNELS) {
		LOG_ERR("chipselect count cannot be greater than max channel count");
		return -EINVAL;
	}

	ret = pinctrl_apply_state(cfg->pinctrl, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("failed to apply pinctrl");
		return ret;
	}

	/* Software Reset */
	regs->SYSCONFIG |= OMAP_MCSPI_SYSCONFIG_SOFTRESET;

	/* Wait till reset is done */
	ret = omap_mcspi_wait_for_reg_bit(sysstatus, OMAP_MCSPI_SYSSTATUS_RESETDONE);
	if (ret < 0) {
		LOG_ERR("RESETDONE timed out");
		return ret;
	}

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static int omap_mcspi_release(const struct device *dev, const struct spi_config *spi_cfg)
{
	struct omap_mcspi_data *data = DEV_DATA(dev);

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static DEVICE_API(spi, omap_mcspi_api) = {
	.transceive = omap_mcspi_transceive,
	.release = omap_mcspi_release,
};

#define OMAP_MCSPI_INIT(n)                                                                         \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static struct omap_mcspi_cfg omap_mcspi_config_##n = {                                     \
		.regs = (struct omap_mcspi_regs *)DT_INST_REG_ADDR(n),                             \
		.pinctrl = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                      \
		.clock_frequency = DT_INST_PROP_OR(n, clock_frequency, OMAP_MCSPI_MAX_FREQUENCY),  \
		.num_cs = DT_INST_PROP(n, ti_spi_num_cs),                                          \
	};                                                                                         \
                                                                                                   \
	static struct omap_mcspi_data omap_mcspi_data_##n = {                                      \
		SPI_CONTEXT_INIT_LOCK(omap_mcspi_data_##n, ctx),                                   \
		SPI_CONTEXT_INIT_SYNC(omap_mcspi_data_##n, ctx),                                   \
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(n), ctx)};                             \
                                                                                                   \
	SPI_DEVICE_DT_INST_DEFINE(n, omap_mcspi_init, NULL, &omap_mcspi_data_##n,                  \
				  &omap_mcspi_config_##n, POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,   \
				  &omap_mcspi_api);

DT_INST_FOREACH_STATUS_OKAY(OMAP_MCSPI_INIT)

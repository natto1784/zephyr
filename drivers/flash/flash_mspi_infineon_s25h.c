/*
 * Copyright (c) 2025 Siemens Mobility GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "jesd216.h"
#include "mspi_ti_k3.h"
#include "spi_nor.h"
#include "zephyr/sys/printk.h"
#define DT_DRV_COMPAT infineon_s25h_flash

#include "flash_mspi_infineon_s25h.h"

#include <zephyr/devicetree.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/mspi.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys_clock.h>
#include <zephyr/sys/device_mmio.h>

LOG_MODULE_REGISTER(flash_mspi_infineon_s25h, CONFIG_FLASH_LOG_LEVEL);

#define JESD216D_SFDP_BFP_DW_NUM (20)

#define CMD_REPEAT(cmd) ((cmd << 8) | cmd)

struct flash_mspi_infineon_s25h_commands {
	uint32_t read_any_reg;
	uint32_t write_any_reg;
	uint32_t write_enable;
	uint32_t write_disable;
	uint32_t read_flash;
	uint32_t write_flash;
	uint32_t erase_256k;
	uint32_t enter_4_byte_addr_mode;
	uint32_t read_jedec_id;
	uint32_t read_sfdp;
	uint32_t reset_enable;
	uint32_t reset;
};

struct flash_mspi_infineon_s25h_dummy_cycles {
	uint8_t volatile_latency;
	uint8_t memory_latency;
	uint8_t read_flash;
};

static const struct flash_mspi_infineon_s25h_commands cmds_single = {
	.read_any_reg = INF_MSPI_S25H_OPCODE_READ_ANY_REGISTER,
	.write_any_reg = INF_MSPI_S25H_OPCODE_WRITE_ANY_REGISTER,
	.write_enable = SPI_NOR_CMD_WREN,
	.write_disable = SPI_NOR_CMD_WRDI,
	.read_flash = SPI_NOR_CMD_READ,
	.write_flash = SPI_NOR_CMD_PP,
	.erase_256k = SPI_NOR_CMD_BE,
	.enter_4_byte_addr_mode = SPI_NOR_CMD_4BA,
	.read_jedec_id = SPI_NOR_CMD_RDID,
	.read_sfdp = INF_MSPI_S25H_OPCODE_READ_SFDP,
	.reset_enable = SPI_NOR_CMD_RESET_EN,
	.reset = SPI_NOR_CMD_RESET_MEM,
};

static const struct flash_mspi_infineon_s25h_dummy_cycles dummy_cycles_single = {
	.volatile_latency = INF_MSPI_S25H_CYCLES_VRGLAT_SINGLE,
	.memory_latency = INF_MSPI_S25H_CYCLES_MEMLAT_SINGLE,
	.read_flash = INF_MSPI_S25H_CYCLES_READ_SINGLE,
};

static const struct flash_mspi_infineon_s25h_commands cmds_quad_sdr = {
	.read_any_reg = INF_MSPI_S25H_OPCODE_READ_ANY_REGISTER,
	.write_any_reg = INF_MSPI_S25H_OPCODE_WRITE_ANY_REGISTER,
	.write_enable = SPI_NOR_CMD_WREN,
	.write_disable = SPI_NOR_CMD_WRDI,
	.read_flash = INF_MSPI_S25H_OPCODE_READ_FLASH_QUAD_SDR,
	.write_flash = SPI_NOR_CMD_PP,
	.erase_256k = SPI_NOR_CMD_BE,
	.enter_4_byte_addr_mode = SPI_NOR_CMD_4BA,
	.read_jedec_id = INF_MSPI_S25H_OPCODE_READ_JEDEC_ID_QUAD,
	.read_sfdp = INF_MSPI_S25H_OPCODE_READ_SFDP,
	.reset_enable = SPI_NOR_CMD_RESET_EN,
	.reset = SPI_NOR_CMD_RESET_MEM,
};

static const struct flash_mspi_infineon_s25h_dummy_cycles dummy_cycles_quad_sdr = {
	.volatile_latency = INF_MSPI_S25H_CYCLES_VRGLAT_QUAD,
	.memory_latency = INF_MSPI_S25H_CYCLES_MEMLAT_QUAD_SDR,
	.read_flash = INF_MSPI_S25H_CYCLES_MEMLAT_QUAD_SDR,
};

static const struct flash_mspi_infineon_s25h_commands cmds_octal_sdr = {
	.read_any_reg = CMD_REPEAT(INF_MSPI_S25H_OPCODE_READ_ANY_REGISTER),
	.write_any_reg = CMD_REPEAT(INF_MSPI_S25H_OPCODE_WRITE_ANY_REGISTER),
	.write_enable = CMD_REPEAT(SPI_NOR_CMD_WREN),
	.write_disable = CMD_REPEAT(SPI_NOR_CMD_WRDI),
	.read_flash = CMD_REPEAT(INF_MSPI_S25H_OPCODE_READ_FLASH_OCTAL_SDR),
	.write_flash = CMD_REPEAT(SPI_NOR_CMD_PP_4B),
	.erase_256k = CMD_REPEAT(SPI_NOR_CMD_BE_4B),
	.enter_4_byte_addr_mode = CMD_REPEAT(SPI_NOR_CMD_4BA),
	.read_jedec_id = CMD_REPEAT(SPI_NOR_CMD_RDID),
	.read_sfdp = INF_MSPI_S25H_OPCODE_READ_SFDP,
	.reset_enable = CMD_REPEAT(SPI_NOR_CMD_RESET_EN),
	.reset = CMD_REPEAT(SPI_NOR_CMD_RESET_MEM),
};

static const struct flash_mspi_infineon_s25h_dummy_cycles dummy_cycles_octal = {
	.volatile_latency = INF_MSPI_S25H_CYCLES_VRGLAT_OCTAL,
	.memory_latency = INF_MSPI_S25H_CYCLES_MEMLAT_OCTAL,
	.read_flash = INF_MSPI_S25H_CYCLES_MEMLAT_OCTAL,
};

struct flash_mspi_infineon_s25h_cfg {
	DEVICE_MMIO_ROM;
	const struct device *bus;
	const struct pinctrl_dev_config *pinctrl;
	k_timeout_t reset_startup_duration;
	const struct mspi_dev_cfg mspi_dev_cfg;
	const struct flash_pages_layout page_layout;
	const struct flash_parameters parameters;
	uint8_t jedec_id[SPI_NOR_MAX_ID_LEN];
	const struct mspi_dev_id dev_id;
};

struct flash_mspi_infineon_s25h_data {
	struct flash_mspi_infineon_s25h_dummy_cycles dummy_cycles;
	struct flash_mspi_infineon_s25h_commands cmds;
	struct mspi_dev_cfg mspi_dev_cfg;
};

static enum jesd216_mode_type flash_mspi_infineon_s25h_to_jesd216_mode(enum mspi_io_mode mode,
								       enum mspi_data_rate rate)
{
	switch (mode) {
	case MSPI_IO_MODE_SINGLE:
		return JESD216_MODE_111;
	case MSPI_IO_MODE_DUAL_1_1_2:
		return JESD216_MODE_112;
	case MSPI_IO_MODE_DUAL_1_2_2:
		return JESD216_MODE_122;
	case MSPI_IO_MODE_DUAL:
		return JESD216_MODE_222;
	case MSPI_IO_MODE_QUAD_1_1_4:
		return JESD216_MODE_114;
	case MSPI_IO_MODE_QUAD_1_4_4:
		return JESD216_MODE_144;
	case MSPI_IO_MODE_QUAD:
		switch (rate) {
		case MSPI_DATA_RATE_SINGLE:
			return JESD216_MODE_444;
		case MSPI_DATA_RATE_DUAL:
			return JESD216_MODE_44D4D;
		default:
			return JESD216_MODE_LIMIT;
		}
	case MSPI_IO_MODE_OCTAL_1_1_8:
		return JESD216_MODE_118;
	case MSPI_IO_MODE_OCTAL_1_8_8:
		return JESD216_MODE_188;
	case MSPI_IO_MODE_OCTAL:
		switch (rate) {
		case MSPI_DATA_RATE_SINGLE:
			return JESD216_MODE_888;
		case MSPI_DATA_RATE_DUAL:
			return JESD216_MODE_8D8D8D;
		default:
			return JESD216_MODE_LIMIT;
		}
	default:
		return JESD216_MODE_LIMIT;
	}
}

#ifdef CONFIG_MSPI_TI_K3
/* forward declaration */
static int flash_mspi_infineon_s25h_verify_jedec_id(const struct device *dev);
static int flash_mspi_infineon_s25h_calibrate_cadence(const struct device *dev)
{
	const struct flash_mspi_infineon_s25h_cfg *config = dev->config;
	struct mspi_ti_k3_timing_cfg timing_cfg = {.rd_delay = 0xf};
	int ret = 0;

	ret = flash_mspi_infineon_s25h_verify_jedec_id(dev);
	if (ret == 0) {
		return 0;
	}

	do {
		ret = mspi_timing_config(config->bus, &config->dev_id,
					 MSPI_TI_K3_TIMING_PARAM_RD_DELAY, &timing_cfg);
		if (ret < 0) {
			LOG_ERR("failed to configure read delay for controller");
			return ret;
		}

		ret = flash_mspi_infineon_s25h_verify_jedec_id(dev);

		timing_cfg.rd_delay--;
	} while (ret < 0 && timing_cfg.rd_delay > 0);

	if (ret < 0) {
		LOG_ERR("failed to configure read delay for controller");
	}
	return ret;
}
#endif /* CONFIG_MSPI_TI_K3 */

static int flash_mspi_infineon_s25h_prepare_mspi_bus(const struct device *dev)
{
	const struct flash_mspi_infineon_s25h_cfg *config = dev->config;
	struct flash_mspi_infineon_s25h_data *data = dev->data;
	int ret = 0;

	ret = mspi_dev_config(config->bus, &config->dev_id,
			      MSPI_DEVICE_CONFIG_CE_NUM | MSPI_DEVICE_CONFIG_IO_MODE |
				      MSPI_DEVICE_CONFIG_CPP | MSPI_DEVICE_CONFIG_CE_POL |
				      MSPI_DEVICE_CONFIG_DQS | MSPI_DEVICE_CONFIG_DATA_RATE |
				      MSPI_DEVICE_CONFIG_ENDIAN,
			      &data->mspi_dev_cfg);
	if (ret < 0) {
		return ret;
	}

#ifdef CONFIG_MSPI_TI_K3
	if (data->mspi_dev_cfg.io_mode != MSPI_IO_MODE_SINGLE) {
		ret = flash_mspi_infineon_s25h_calibrate_cadence(dev);
		if (ret < 0) {
			return ret;
		}
	}
#endif /* CONFIG_MSPI_TI_K3 */

	return 0;
}

static int flash_mspi_infineon_s25h_reset(const struct device *dev)
{
	int ret = 0;
	const struct flash_mspi_infineon_s25h_cfg *config = dev->config;
	struct flash_mspi_infineon_s25h_data *data = dev->data;

	const struct mspi_xfer_packet reset_packets[] = {
		{
			.dir = MSPI_TX,
			.cmd = data->cmds.reset_enable,
			.num_bytes = 0,
		},
		{
			.dir = MSPI_TX,
			.cmd = data->cmds.reset,
			.num_bytes = 0,
		},
	};

	struct mspi_xfer xfer = {
		INF_MSPI_S25H_DEFAULT_XFER_DATA,
		.cmd_length = data->mspi_dev_cfg.cmd_length,
		.rx_dummy = 0,
		.addr_length = 0,
		.num_packet = 2,
		.packets = reset_packets,
		.timeout = INF_MSPI_S25H_DEFAULT_MSPI_TIMEOUT,
	};

	ret = mspi_transceive(config->bus, &config->dev_id, &xfer);
	if (ret < 0) {
		return ret;
	}

	k_sleep(config->reset_startup_duration);

	return 0;
}

static int flash_mspi_infineon_s25h_set_writing_forbidden(const struct device *dev,
							  bool writing_forbidden)
{
	const struct flash_mspi_infineon_s25h_cfg *config = dev->config;
	struct flash_mspi_infineon_s25h_data *data = dev->data;
	uint32_t cmd = writing_forbidden ? data->cmds.write_disable : data->cmds.write_enable;
	const struct mspi_xfer_packet packet = {
		.dir = MSPI_TX,
		.cmd = cmd,
		.num_bytes = 0,
	};

	struct mspi_xfer xfer = {
		INF_MSPI_S25H_DEFAULT_XFER_DATA_SINGLE_CMD,
		.cmd_length = data->mspi_dev_cfg.cmd_length,
		.packets = &packet,
		.timeout = INF_MSPI_S25H_DEFAULT_MSPI_TIMEOUT,
	};

	return mspi_transceive(config->bus, &config->dev_id, &xfer);
}

static int flash_mspi_infineon_s25h_rw_any_register(const struct device *dev, uint32_t address,
						    uint8_t *value, enum mspi_xfer_direction dir)
{
	int ret;
	const struct flash_mspi_infineon_s25h_cfg *config = dev->config;
	struct flash_mspi_infineon_s25h_data *data = dev->data;
	/* ddr requires even number of bytes */
	uint8_t num_bytes;
	uint32_t cmd;
	uint8_t rx_dummy;
	uint8_t data_buf[2] = {0xf, 0xf};

	if (dir == MSPI_TX) {
		ret = flash_mspi_infineon_s25h_set_writing_forbidden(dev, false);
		if (ret < 0) {
			LOG_ERR("Error disabling write protection before changing configuration");
			return ret;
		}
		data_buf[0] = *value;
	}

	if (dir == MSPI_RX) {
		num_bytes = (data->mspi_dev_cfg.data_rate == MSPI_DATA_RATE_DUAL ? 2 : 1);
		cmd = data->cmds.read_any_reg;
		if (address >= INF_MSPI_S25H_ADDRESS_VOLATILE_BASE) {
			rx_dummy = data->dummy_cycles.volatile_latency;
		} else {
			rx_dummy = data->dummy_cycles.memory_latency;
		}
	} else {
		num_bytes = 1;
		cmd = data->cmds.write_any_reg;
		rx_dummy = 0;
	}

	const struct mspi_xfer_packet packet = {
		.dir = dir,
		.cmd = cmd,
		.num_bytes = num_bytes,
		.data_buf = data_buf,
		.address = address,
	};

	struct mspi_xfer xfer = {
		INF_MSPI_S25H_DEFAULT_XFER_DATA,
		.cmd_length = data->mspi_dev_cfg.cmd_length,
		.addr_length = data->mspi_dev_cfg.addr_length,
		.rx_dummy = rx_dummy,
		.packets = &packet,
		.num_packet = 1,
		.timeout = INF_MSPI_S25H_DEFAULT_MSPI_TIMEOUT,
	};

	printk("mspi xfer %x %x %x\n", cmd, num_bytes, address);

	ret = mspi_transceive(config->bus, &config->dev_id, &xfer);
	if (ret < 0) {
		return ret;
	}

	if (dir == MSPI_RX) {
		*value = data_buf[0];
	}

	printk(" buf = %x %x %u\n", data_buf[0], data_buf[1], dir);

	return 0;
}

static int flash_mspi_infineon_s25h_is_write_protection_enabled(const struct device *dev,
								uint8_t *is_enabled)
{
	int ret = 0;
	uint8_t val = 0;

	ret = flash_mspi_infineon_s25h_rw_any_register(dev, INF_MSPI_S25H_ADDRESS_VOLATILE_STATUS_1,
						       &val, MSPI_RX);
	if (ret < 0) {
		return ret;
	}
	*is_enabled = (val & INF_MSPI_S25H_STATUS_1_WRPGEN_BIT);
	return 0;
}

static int flash_mspi_infineon_s25h_wait_for_idle(const struct device *dev, uint32_t timeout_ms)
{
	int ret = 0;
	uint8_t status_1 = 0;
	uint32_t retries = timeout_ms / INF_MSPI_S25H_TIMEOUT_IDLE_RETRY_INTERVAL_MS;

	ret = flash_mspi_infineon_s25h_rw_any_register(dev, INF_MSPI_S25H_ADDRESS_VOLATILE_STATUS_1,
						       &status_1, MSPI_RX);
	if (ret < 0) {
		return ret;
	}

	while ((status_1 & INF_MSPI_S25H_STATUS_1_RDYBSY_BIT) != 0 && retries > 0) {
		k_sleep(INF_MSPI_S25H_TIMEOUT_IDLE_RETRY_INTERVAL);
		ret = flash_mspi_infineon_s25h_rw_any_register(
			dev, INF_MSPI_S25H_ADDRESS_VOLATILE_STATUS_1, &status_1, MSPI_RX);
		if (ret < 0) {
			return ret;
		}
		--retries;
	}

	if (retries == 0) {
		LOG_ERR("Waiting for flash to enter idle. Status 1 register: 0x%X", status_1);
		return -EIO;
	}

	return 0;
}

static int flash_mspi_infineon_s25h_read_jedec_id(const struct device *dev, uint8_t *buf)
{
	int ret = 0;
	const struct flash_mspi_infineon_s25h_cfg *config = dev->config;
	struct flash_mspi_infineon_s25h_data *data = dev->data;
	/* ddr requires even number of bytes */
	uint8_t num_bytes = (data->mspi_dev_cfg.data_rate == MSPI_DATA_RATE_DUAL ? 4 : 3);
	/* RDIDN takes 4 dummy address bytes in octal mode */
	uint8_t addr_length =
		(data->mspi_dev_cfg.io_mode == MSPI_IO_MODE_OCTAL ? data->mspi_dev_cfg.addr_length
								  : 0);

	printk("cmd = %x %x %x %x %x\n", data->cmds.read_jedec_id, num_bytes, addr_length,
	       data->mspi_dev_cfg.cmd_length, data->dummy_cycles.volatile_latency);
	uint8_t data_buf[4] = {0};

	const struct mspi_xfer_packet packet = {
		.dir = MSPI_RX,
		.cmd = data->cmds.read_jedec_id,
		.num_bytes = num_bytes,
		.data_buf = data_buf,
		.address = 0,
	};

	struct mspi_xfer xfer = {
		INF_MSPI_S25H_DEFAULT_XFER_DATA,
		.cmd_length = data->mspi_dev_cfg.cmd_length,
		.addr_length = addr_length,
		.rx_dummy = data->dummy_cycles.volatile_latency,
		.packets = &packet,
		.num_packet = 1,
		.timeout = INF_MSPI_S25H_DEFAULT_MSPI_TIMEOUT,
	};

	ret = mspi_transceive(config->bus, &config->dev_id, &xfer);
	if (ret < 0) {
		LOG_ERR("Error reading JEDEC id");
		return ret;
	}

	memcpy(buf, data_buf, 3);

	return 0;
}

static int flash_mspi_infineon_s25h_read(const struct device *dev, off_t addr, void *data,
					 size_t size)
{
	int ret = 0;
	bool requires_cleanup = false;
	const struct flash_mspi_infineon_s25h_cfg *config = dev->config;
	struct flash_mspi_infineon_s25h_data *dev_data = dev->data;

	/* The S25H allows for continuous read operations which happen by sending
	 * 0xAX after an address. The driver doesn't implement this which is why we
	 * don't want this and instead wait for 2 cycles. However since the flash
	 * pins could be in high impedance state from the MSPI controller after the
	 * address was sent an address ending with 0xA could put the flash into a
	 * continuous read mode. To prevent this the driver will read the jedec id,
	 * if the address wasn't aligned to prevent accidentally fulfilling the
	 * requirement.
	 */
	if (dev_data->mspi_dev_cfg.io_mode == MSPI_IO_MODE_QUAD && (addr % 16 != 0)) {
		requires_cleanup = true;
	}

	ret = flash_mspi_infineon_s25h_prepare_mspi_bus(dev);
	if (ret < 0) {
		LOG_ERR("Error setting up the MSPI bus for the flash device");
		return ret;
	}

	const struct mspi_xfer_packet packet = {
		.address = addr,
		.cmd = dev_data->cmds.read_flash,
		.data_buf = data,
		.dir = MSPI_RX,
		.num_bytes = size,
	};

	struct mspi_xfer xfer = {
		INF_MSPI_S25H_DEFAULT_XFER_DATA,
		.cmd_length = dev_data->mspi_dev_cfg.cmd_length,
		.addr_length = dev_data->mspi_dev_cfg.addr_length,
		.rx_dummy = dev_data->dummy_cycles.read_flash,
		.packets = &packet,
		.num_packet = 1,
		/* 20 milliseconds + 1 ms per 4KiB; for 256 KiB this results in 84 ms */
		.timeout = (size / 4096) + 20,
	};

	ret = mspi_transceive(config->bus, &config->dev_id, &xfer);
	if (ret < 0) {
		return ret;
	}

	if (requires_cleanup) {
		uint8_t unused[3];
		(void)unused;
		return flash_mspi_infineon_s25h_read_jedec_id(dev, unused);
	}

	return ret;
}

static int flash_mspi_infineon_s25h_single_block_write(const struct device *dev,
						       const struct mspi_xfer *xfer_write)
{
	int ret;
	const struct flash_mspi_infineon_s25h_cfg *config = dev->config;
	uint8_t status_1;

	ret = flash_mspi_infineon_s25h_set_writing_forbidden(dev, false);
	if (ret < 0) {
		LOG_ERR("Error disabling write protection before trying to write data into "
			"flash");
		return ret;
	}

	ret = flash_mspi_infineon_s25h_rw_any_register(dev, INF_MSPI_S25H_ADDRESS_VOLATILE_STATUS_1,
						       &status_1, MSPI_RX);
	if (ret < 0) {
		return ret;
	}
	ret = mspi_transceive(config->bus, &config->dev_id, xfer_write);
	if (ret < 0) {
		LOG_ERR("Error writing flash memory");
		return ret;
	}

	ret = flash_mspi_infineon_s25h_wait_for_idle(dev,
						     INF_MSPI_S25H_TIMEOUT_IDLE_WRITE_BLOCK_MS);
	if (ret < 0) {
		LOG_ERR("Error waiting for flash to enter idle after writing");
		return ret;
	}

	ret = flash_mspi_infineon_s25h_rw_any_register(dev, INF_MSPI_S25H_ADDRESS_VOLATILE_STATUS_1,
						       &status_1, MSPI_RX);
	if (ret < 0) {
		LOG_ERR("Error reading back status 1 register to confirm valid write");
		return ret;
	}

	if (status_1 & INF_MSPI_S25H_STATUS_1_PRGERR_BIT) {
		LOG_ERR("Last programming transaction wasn't successful");
		return -EIO;
	}

	return 0;
}

static int flash_mspi_infineon_s25h_write(const struct device *dev, off_t addr,
					  const void *transmission_data, size_t size)
{
	int ret = 0;
	struct flash_mspi_infineon_s25h_data *dev_data = dev->data;
	uint8_t old_write_protection;
	uint8_t *data_buf = (uint8_t *)transmission_data;

	/* Check whether we are not aligned and would write over a block boundary */
	if ((addr % INF_MSPI_S25H_WRITE_BLOCK_SIZE) != 0 &&
	    (addr % INF_MSPI_S25H_WRITE_BLOCK_SIZE) + size >= INF_MSPI_S25H_WRITE_BLOCK_SIZE) {
		LOG_ERR("Non-aligned write that goes above another block isn't supported");
		return -ENOSYS;
	}

	struct mspi_xfer_packet packet_write = {
		.cmd = dev_data->cmds.write_flash,
		.dir = MSPI_TX,
	};

	struct mspi_xfer xfer_write = {
		INF_MSPI_S25H_DEFAULT_XFER_DATA,
		.cmd_length = dev_data->mspi_dev_cfg.cmd_length,
		.addr_length = dev_data->mspi_dev_cfg.addr_length,
		.rx_dummy = 0,
		.packets = &packet_write,
		.num_packet = 1,
		/* 20 milliseconds + 1 ms per 16KiB; for 256 KiB this results in 36 ms */
		.timeout = (size / 16384) + 20,
	};

	ret = flash_mspi_infineon_s25h_is_write_protection_enabled(dev, &old_write_protection);
	if (ret < 0) {
		LOG_ERR("Error querying whether write protection is enabled");
		return ret;
	}

	uint32_t remaining_bytes = size;
	uint32_t write_index = 0;
	uint32_t current_transaction_transfer_size;

	do {
		current_transaction_transfer_size =
			MIN(remaining_bytes, INF_MSPI_S25H_WRITE_BLOCK_SIZE);
		packet_write.num_bytes = current_transaction_transfer_size;
		packet_write.address = addr + (write_index * INF_MSPI_S25H_WRITE_BLOCK_SIZE);
		packet_write.data_buf = &data_buf[write_index * INF_MSPI_S25H_WRITE_BLOCK_SIZE];

		ret = flash_mspi_infineon_s25h_single_block_write(dev, &xfer_write);
		if (ret < 0) {
			return ret;
		}

		remaining_bytes -= current_transaction_transfer_size;
		++write_index;

	} while (remaining_bytes > 0);

	if (old_write_protection) {
		ret = flash_mspi_infineon_s25h_set_writing_forbidden(dev);
		if (ret < 0) {
			LOG_ERR("Error re-enabling write protection after writing data into flash");
			return ret;
		}
	}

	return 0;
}

static int flash_mspi_infineon_s25h_erase(const struct device *dev, off_t addr, size_t size)
{
	int ret = 0;
	const struct flash_mspi_infineon_s25h_cfg *config = dev->config;
	struct flash_mspi_infineon_s25h_data *data = dev->data;
	uint8_t old_write_protection;

	struct mspi_xfer_packet packet_erase = {
		.cmd = data->cmds.erase_256k,
		.data_buf = NULL,
		.num_bytes = 0,
		.dir = MSPI_TX,
	};

	const struct mspi_xfer xfer_erase = {
		INF_MSPI_S25H_DEFAULT_XFER_DATA,
		.cmd_length = data->mspi_dev_cfg.cmd_length,
		.addr_length = data->mspi_dev_cfg.addr_length,
		.rx_dummy = 0,
		.packets = &packet_erase,
		.num_packet = 1,
		/* 20 milliseconds + 1 ms per 16KiB; for 256 KiB this results in 36 ms */
		.timeout = (size / 16384) + 20,
	};

	if (addr % INF_MSPI_S25H_ERASE_SECTOR_SIZE != 0) {
		LOG_WRN("Erase sector is not aligned! This might erase data you don't want to "
			"erase");
	}

	ret = flash_mspi_infineon_s25h_is_write_protection_enabled(dev, &old_write_protection);
	if (ret < 0) {
		LOG_ERR("Error querying whether write protection is enabled");
		return ret;
	}

	uint32_t count = size / INF_MSPI_S25H_ERASE_SECTOR_SIZE;

	if (size % INF_MSPI_S25H_ERASE_SECTOR_SIZE != 0) {
		++count;
	}

	for (uint32_t i = 0; i < count; ++i) {
		packet_erase.address = addr + (i * INF_MSPI_S25H_ERASE_SECTOR_SIZE);
		ret = flash_mspi_infineon_s25h_set_writing_forbidden(dev, false);
		if (ret < 0) {
			LOG_ERR("Error disabling write protection before flash erase");
			return ret;
		}

		ret = mspi_transceive(config->bus, &config->dev_id, &xfer_erase);
		if (ret) {
			LOG_ERR("Error sending erase command");
			return ret;
		}

		ret = flash_mspi_infineon_s25h_wait_for_idle(
			dev, INF_MSPI_S25H_TIMEOUT_IDLE_ERASE_SECTOR_MS);
		if (ret < 0) {
			LOG_ERR("Error waiting for flash to enter idle after erasing");
			return ret;
		}
	}

	if (old_write_protection) {
		ret = flash_mspi_infineon_s25h_set_writing_forbidden(dev, false);
		if (ret < 0) {
			LOG_ERR("Error re-enabling write protection after flash erase");
			return ret;
		}
	}

	return 0;
}

static const struct flash_parameters *
flash_mspi_infineon_s25h_get_parameters(const struct device *dev)
{
	const struct flash_mspi_infineon_s25h_cfg *config = dev->config;

	return &config->parameters;
}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
static void flash_mspi_infineon_s25h_pages_layout(const struct device *dev,
						  const struct flash_pages_layout **layout,
						  size_t *layout_size)
{
	const struct flash_mspi_infineon_s25h_cfg *config = dev->config;

	*layout = &config->page_layout;
	*layout_size = 1;
}
#endif

static int flash_mspi_infineon_s25h_compare_jedec_id(const struct device *dev)
{
}

static int flash_mspi_infineon_s25h_verify_jedec_id(const struct device *dev)
{
	const struct flash_mspi_infineon_s25h_cfg *config = dev->config;
	uint8_t id[3];
	int ret;

	ret = flash_mspi_infineon_s25h_read_jedec_id(dev, id);
	if (ret < 0) {
		LOG_ERR("Error reading JEDEC ids from flash");
		return ret;
	}

	uint8_t manufacturer_id = id[0];
	uint8_t expected_manufacturer_id = config->jedec_id[0];
	uint16_t dev_id = (id[1] << 8) | id[2];
	uint16_t expected_dev_id = (config->jedec_id[1] << 8) | config->jedec_id[2];

	if (manufacturer_id != expected_manufacturer_id || dev_id != expected_dev_id) {
		LOG_ERR("Rear JEDEC ids don't match expected ids. The communication is "
			"broken or the non-volatile flash configuration is something "
			"unexpected");
		LOG_ERR("Read manufacturer id: 0x%02X. Expected: 0x%02X", manufacturer_id,
			expected_manufacturer_id);
		LOG_ERR("Read device id: 0x%04X. Expected: 0x%04X", dev_id, expected_dev_id);
		return -EIO;
	}

	return 0;
}

static int flash_mspi_infineon_s25h_switch_to_octal_transfer(const struct device *dev)
{
	int ret;
	struct flash_mspi_infineon_s25h_data *data = dev->data;
	enum mspi_data_rate rate = data->mspi_dev_cfg.data_rate;
	uint8_t cfg_value;

	if (data->mspi_dev_cfg.addr_length != 4) {
		LOG_ERR("4 byte address mode is required for octal mode");
		return -EINVAL;
	}

	if (data->mspi_dev_cfg.cmd_length != 2) {
		LOG_ERR("2 byte command length is required for octal mode");
		return -EINVAL;
	}
	data->mspi_dev_cfg.io_mode = MSPI_IO_MODE_SINGLE;
	data->mspi_dev_cfg.data_rate = MSPI_DATA_RATE_SINGLE;
	data->mspi_dev_cfg.addr_length = 4;
	data->mspi_dev_cfg.cmd_length = 1;

	ret = flash_mspi_infineon_s25h_rw_any_register(dev, INF_MSPI_S25H_ADDRESS_VOLATILE_CFG_5,
						       &cfg_value, MSPI_RX);
	if (ret < 0) {
		LOG_ERR("Error reading flash register");
		return ret;
	}

	cfg_value |= INF_MSPI_S25H_CFG_5_OPI_IT_BIT;

	if (rate == MSPI_DATA_RATE_SINGLE) {
		cfg_value &= ~INF_MSPI_S25H_CFG_5_SDRDDR_BIT;
	} else if (rate == MSPI_DATA_RATE_DUAL) {
		cfg_value |= INF_MSPI_S25H_CFG_5_SDRDDR_BIT;
	} else {
		LOG_ERR("Invalid data rate for octal mode");
		return -EINVAL;
	}

	ret = flash_mspi_infineon_s25h_rw_any_register(dev, INF_MSPI_S25H_ADDRESS_VOLATILE_CFG_5,
						       &cfg_value, MSPI_TX);
	if (ret < 0) {
		LOG_ERR("Error writing flash register");
		return ret;
	}

	/* update commands and cycles */
	data->cmds = cmds_octal_sdr;
	data->dummy_cycles = dummy_cycles_octal;
	if (rate == MSPI_DATA_RATE_DUAL) {
		data->cmds.read_flash = CMD_REPEAT(INF_MSPI_S25H_OPCODE_READ_FLASH_OCTAL_DDR);
	}

	data->mspi_dev_cfg.io_mode = MSPI_IO_MODE_OCTAL;
	data->mspi_dev_cfg.data_rate = MSPI_DATA_RATE_DUAL;
	data->mspi_dev_cfg.addr_length = 4;
	data->mspi_dev_cfg.cmd_length = 2;
	ret = flash_mspi_infineon_s25h_prepare_mspi_bus(dev);
	if (ret < 0) {
		LOG_ERR("Error switching MSPI mode to octal data width");
		return ret;
	}

	/* verify jedec ID again */
	ret = flash_mspi_infineon_s25h_verify_jedec_id(dev);
	if (ret < 0) {
		LOG_ERR("JEDEC ID mismatch after switching to octal MSPI mode. Communication is "
			"broken");
		return ret;
	}

	return 0;
}

static int flash_mspi_infineon_s25h_switch_to_quad_transfer(const struct device *dev)
{
	int ret;
	struct flash_mspi_infineon_s25h_data *data = dev->data;

	uint8_t cfg_value;

	ret = flash_mspi_infineon_s25h_rw_any_register(dev, INF_MSPI_S25H_ADDRESS_VOLATILE_CFG_1,
						       &cfg_value, MSPI_RX);
	if (ret < 0) {
		LOG_ERR("Error reading flash register");
		return ret;
	}

	cfg_value |= INF_MSPI_S25H_CFG_1_QUADIT_BIT;

	ret = flash_mspi_infineon_s25h_rw_any_register(dev, INF_MSPI_S25H_ADDRESS_VOLATILE_CFG_1,
						       &cfg_value, MSPI_TX);
	if (ret < 0) {
		LOG_ERR("Error writing flash register");
		return ret;
	}

	/* set address + data to 4 lanes */
	data->mspi_dev_cfg.io_mode = MSPI_IO_MODE_QUAD_1_4_4;
	ret = flash_mspi_infineon_s25h_prepare_mspi_bus(dev);
	if (ret < 0) {
		LOG_ERR("Error switching MSPI mode to 4 lane data width");
		return ret;
	}

	/* update commands */
	data->cmds = cmds_quad_sdr;
	data->dummy_cycles = dummy_cycles_quad_sdr;
	if (data->mspi_dev_cfg.data_rate == MSPI_DATA_RATE_DUAL) {
		data->cmds.read_flash = INF_MSPI_S25H_OPCODE_READ_FLASH_QUAD_DDR;
		data->dummy_cycles.read_flash = INF_MSPI_S25H_CYCLES_MEMLAT_QUAD_DDR;
	}

	ret = flash_mspi_infineon_s25h_rw_any_register(dev, INF_MSPI_S25H_ADDRESS_VOLATILE_CFG_1,
						       &cfg_value, MSPI_RX);
	if (ret < 0) {
		LOG_ERR("Error reading flash register");
		return ret;
	}

	ret = flash_mspi_infineon_s25h_verify_jedec_id(dev);
	if (ret < 0) {
		LOG_ERR("JEDEC ID mismatch after switching to 4 lane MSPI. Communication is "
			"broken");
		return ret;
	}

	/* set command to 4 lanes */
	ret = flash_mspi_infineon_s25h_rw_any_register(dev, INF_MSPI_S25H_ADDRESS_VOLATILE_CFG_2,
						       &cfg_value, MSPI_RX);
	if (ret < 0) {
		LOG_ERR("Error reading flash register");
		return ret;
	}

	cfg_value |= INF_MSPI_S25H_CFG_2_QPI_IT_BIT;

	ret = flash_mspi_infineon_s25h_rw_any_register(dev, INF_MSPI_S25H_ADDRESS_VOLATILE_CFG_2,
						       &cfg_value, MSPI_TX);
	if (ret < 0) {
		LOG_ERR("Error writing flash register");
		return ret;
	}

	data->mspi_dev_cfg.io_mode = MSPI_IO_MODE_QUAD;
	ret = flash_mspi_infineon_s25h_prepare_mspi_bus(dev);
	if (ret < 0) {
		LOG_ERR("Error switching bus mode to full quad MSPI mode");
		return ret;
	}

	ret = flash_mspi_infineon_s25h_verify_jedec_id(dev);
	if (ret < 0) {
		LOG_ERR("JEDEC ID mismatch after switching to full quad MSPI mode. Communication "
			"is broken");
		return ret;
	}

	return 0;
}

static int flash_mspi_infineon_s25h_switch_mode(const struct device *dev)
{
	struct flash_mspi_infineon_s25h_data *data = dev->data;

	switch (data->mspi_dev_cfg.io_mode) {
	case MSPI_IO_MODE_SINGLE:
		/* already in 1s1s1s */
		return 0;
	case MSPI_IO_MODE_QUAD:
		return flash_mspi_infineon_s25h_switch_to_quad_transfer(dev);
	case MSPI_IO_MODE_OCTAL:
		return flash_mspi_infineon_s25h_switch_to_octal_transfer(dev);
	default:
		return -ENOTSUP;
	}
}

static int flash_mspi_infineon_s25h_disable_hybrid_sector_mode(const struct device *dev)
{
	/* This driver needs the hybrid sector mode to be disabled. So if it's found to be turned on
	 * it gets changed. This requires changing the non-volatile configuration and also a reset
	 */
	int ret = 0;
	uint8_t conf3 = 0;

	ret = flash_mspi_infineon_s25h_rw_any_register(dev, INF_MSPI_S25H_ADDRESS_VOLATILE_CFG_3,
						       &conf3, MSPI_RX);
	if (ret < 0) {
		LOG_ERR("Error reading volatile configuration register 3");
		return ret;
	}

	if ((conf3 & INF_MSPI_S25H_CFG_3_UNHYSA_BIT) == 0) {
		LOG_ERR("Flash is in hybrid sector mode. Changing non-volatile config to correct "
			"this %x",
			conf3);

		conf3 |= INF_MSPI_S25H_CFG_3_UNHYSA_BIT;

		LOG_ERR("meow %x", conf3);
		ret = flash_mspi_infineon_s25h_rw_any_register(
			dev, INF_MSPI_S25H_ADDRESS_NON_VOLATILE_CFG_3, &conf3, MSPI_TX);
		if (ret < 0) {
			LOG_ERR("Error changing non-volatile configuration of flash");
			return ret;
		}
		ret = flash_mspi_infineon_s25h_wait_for_idle(dev,
							     INF_MSPI_S25H_TIMEOUT_IDLE_STARTUP);
		if (ret < 0) {
			LOG_ERR("Error waiting for flash to enter idle after disabling hybrid "
				"sector mode");
			return ret;
		}

		ret = flash_mspi_infineon_s25h_reset(dev);
		if (ret < 0) {
			LOG_ERR("Error resetting flash via reset command");
			return ret;
		}
		conf3 = 0;
		ret = flash_mspi_infineon_s25h_rw_any_register(
			dev, INF_MSPI_S25H_ADDRESS_VOLATILE_CFG_3, &conf3, MSPI_RX);
		if (ret < 0) {
			LOG_ERR("Error reading volatile config 3 register of flash");
			return ret;
		}

		if ((conf3 & INF_MSPI_S25H_CFG_3_UNHYSA_BIT) == 0) {
			LOG_ERR("Changing the flash configuration to Uniform mode didn't work %x",
				conf3);
			return -EIO;
		}

		ret = flash_mspi_infineon_s25h_set_writing_forbidden(dev);
		if (ret < 0) {
			LOG_ERR("Error re-enabling the write protection");
			return ret;
		}
	}

	return 0;
}

static int flash_mspi_infineon_s25h_enter_4_byte_address_mode(const struct device *dev)
{
	int ret = 0;
	const struct flash_mspi_infineon_s25h_cfg *config = dev->config;
	struct flash_mspi_infineon_s25h_data *data = dev->data;

	const struct mspi_xfer_packet enter_4_byte_cmd = {
		.dir = MSPI_TX,
		.cmd = data->cmds.enter_4_byte_addr_mode,
		.num_bytes = 0,
	};

	struct mspi_xfer xfer = {
		INF_MSPI_S25H_DEFAULT_XFER_DATA,
		.cmd_length = data->mspi_dev_cfg.cmd_length,
		.rx_dummy = 0,
		.addr_length = 0,
		.num_packet = 1,
		.packets = &enter_4_byte_cmd,
		.timeout = INF_MSPI_S25H_DEFAULT_MSPI_TIMEOUT,
	};

	ret = mspi_transceive(config->bus, &config->dev_id, &xfer);
	if (ret < 0) {
		LOG_ERR("Error sending command to enter 4 byte address mode");
		return ret;
	}

	data->mspi_dev_cfg.addr_length = 4;
	ret = flash_mspi_infineon_s25h_prepare_mspi_bus(dev);
	if (ret < 0) {
		LOG_ERR("Error setting up MSPI bus after changing address length");
		return ret;
	}

	ret = flash_mspi_infineon_s25h_verify_jedec_id(dev);
	if (ret < 0) {
		LOG_ERR("Error verifying JEDEC id after entering 4 byte address mode");
		return ret;
	}

	return 0;
}

static int flash_mspi_infineon_s25h_read_sfdp(const struct device *dev, off_t addr, void *sfdp,
					      size_t size)
{

	int ret = 0;
	const struct flash_mspi_infineon_s25h_cfg *config = dev->config;
	struct flash_mspi_infineon_s25h_data *data = dev->data;

	const struct mspi_xfer_packet packet = {
		.dir = MSPI_RX,
		.cmd = data->cmds.read_sfdp,
		.num_bytes = size,
		.data_buf = sfdp,
		.address = addr,
	};

	struct mspi_xfer xfer = {
		INF_MSPI_S25H_DEFAULT_XFER_DATA,
		.cmd_length = data->mspi_dev_cfg.cmd_length,
		.addr_length = data->mspi_dev_cfg.addr_length,
		/* Read SFDP transaction is always 8 dummy cycles */
		.rx_dummy = 8,
		.packets = &packet,
		.num_packet = 1,
		.timeout = INF_MSPI_S25H_DEFAULT_MSPI_TIMEOUT,
	};

	ret = mspi_transceive(config->bus, &config->dev_id, &xfer);
	if (ret < 0) {
		LOG_ERR("Error reading SFDP");
		return ret;
	}

	return 0;
}

static int flash_mspi_infineon_s25h_check_mode(const struct device *dev)
{
	struct flash_mspi_infineon_s25h_data *data = dev->data;
	struct jesd216_sfdp_header header;
	int ret;

	ret = flash_mspi_infineon_s25h_read_sfdp(dev, 0, &header, sizeof(header));
	if (ret < 0) {
		LOG_ERR("Failed to get SFDP header");
		return ret;
	}

	for (int i = 0; i < 8; i++) {
		printk("hi %x\n", *((uint8_t *)(&header) + i));
	}

	if (header.magic != JESD216_SFDP_MAGIC) {
		LOG_ERR("SFDP magic 0x%08x invalid", header.magic);
		return -EINVAL;
	}

	LOG_DBG("SFDP v%u.%u AP %x with %u parameter headers", header.rev_major, header.rev_minor,
		header.access, header.nph + 1);

	printk("SFDP v%u.%u AP %x with %u parameter headers", header.rev_major, header.rev_minor,
	       header.access, header.nph + 1);
	struct jesd216_param_header pheaders[header.nph + 1];

	ret = flash_mspi_infineon_s25h_read_sfdp(dev, sizeof(header), pheaders, sizeof(pheaders));
	for (int i = 0; i < 48; i++) {
		printk("hi %x\n", *((uint8_t *)(pheaders) + i));
	}

	printk("hmm1");
	if (ret < 0) {
		LOG_ERR("Failed to get all parameter headers");
		return ret;
	}

	for (int i = 0; i <= header.nph; i++) {
		struct jesd216_param_header *cur = &pheaders[i];
		uint16_t id = jesd216_param_id(cur);

		if (id == JESD216_SFDP_PARAM_ID_BFP) {
			enum jesd216_mode_type mode = flash_mspi_infineon_s25h_to_jesd216_mode(
				data->mspi_dev_cfg.io_mode, data->mspi_dev_cfg.data_rate);
			union {
				uint32_t raw[JESD216D_SFDP_BFP_DW_NUM]; /* 20 dwords (jesd216d) */
				struct jesd216_bfp bfp;
			} bfpu;
			uint32_t bfp_addr = (cur->ptp[2] << 16) | (cur->ptp[1] << 8) | cur->ptp[0];

			if (cur->len_dw != JESD216D_SFDP_BFP_DW_NUM) {
				LOG_ERR("Number of dwords in BFP is not %u",
					JESD216D_SFDP_BFP_DW_NUM);
				return -EINVAL;
			}

			ret = flash_mspi_infineon_s25h_read_sfdp(dev, bfp_addr, bfpu.raw,
								 sizeof(bfpu.raw));
			if (ret < 0) {
				LOG_ERR("Failed to read basic flash parameter table");
				return ret;
			}

			ret = jesd216_bfp_read_support(cur, &bfpu.bfp, mode, NULL);
			if (ret < 0) {
				LOG_ERR("I/O mode is not supported");
				return -ENOSYS;
			}
		}
	}
	return 0;
}

static int flash_mspi_infineon_s25h_init(const struct device *dev)
{
	const struct flash_mspi_infineon_s25h_cfg *config = dev->config;
	struct flash_mspi_infineon_s25h_data *data = dev->data;
	struct mspi_dev_cfg dev_cfg = data->mspi_dev_cfg;
	volatile int a = 1;
	int ret = 0;

	ret = pinctrl_apply_state(config->pinctrl, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("Failed to apply pinctrl");
		return ret;
	}

	printk("pre reset\n");
	/* reset the flash */
	ret = flash_mspi_infineon_s25h_reset(dev);
	if (ret < 0) {
		LOG_ERR("Error resetting flash device");
		return ret;
	}
	printk("post reset\n");

	data->mspi_dev_cfg.io_mode = MSPI_IO_MODE_SINGLE;
	data->mspi_dev_cfg.data_rate = MSPI_DATA_RATE_SINGLE;
	data->mspi_dev_cfg.addr_length = 3;
	data->mspi_dev_cfg.cmd_length = 1;

	/* configure mspi for 1s-1s-1s */
	ret = flash_mspi_infineon_s25h_prepare_mspi_bus(dev);
	if (ret < 0) {
		LOG_ERR("Error switching MSPI configuration to 1s1s1s");
		return ret;
	}

	printk("pre check mode\n");
	uint8_t conf3;

	ret = flash_mspi_infineon_s25h_rw_any_register(dev, INF_MSPI_S25H_ADDRESS_VOLATILE_CFG_3,
						       &conf3, MSPI_RX);
	if (ret < 0) {
		LOG_ERR("Error reading volatile configuration register 3");
		return ret;
	}
	printk("conf3 = %u\n", conf3);

	/* verify whether the specified I/O mode is supported */
	ret = flash_mspi_infineon_s25h_check_mode(dev);
	if (ret < 0) {
		return ret;
	}
	printk("post check mode\n");

	/* verify jedec id from DT */
	ret = flash_mspi_infineon_s25h_verify_jedec_id(dev);
	if (ret < 0) {
		return ret;
	}

	/* disable hybrid sector mode i.e, make all blocks 256k in size */
	ret = flash_mspi_infineon_s25h_disable_hybrid_sector_mode(dev);
	if (ret < 0) {
		return ret;
	}

	if (dev_cfg.addr_length == 4) {
		/* enable 4 byte address mode for all modes */
		ret = flash_mspi_infineon_s25h_enter_4_byte_address_mode(dev);
		if (ret < 0) {
			return ret;
		}
	}

	/* restore original mspi configuration */
	data->mspi_dev_cfg = dev_cfg;
	ret = flash_mspi_infineon_s25h_switch_mode(dev);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static DEVICE_API(flash, flash_mspi_infineon_s25h_driver_api) = {
	.read = flash_mspi_infineon_s25h_read,
	.write = flash_mspi_infineon_s25h_write,
	.erase = flash_mspi_infineon_s25h_erase,
	.get_parameters = flash_mspi_infineon_s25h_get_parameters,
#if defined(CONFIG_FLASH_JESD216_API)
	.read_jedec_id = flash_mspi_infineon_s25h_read_jedec_id,
#endif
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	.page_layout = flash_mspi_infineon_s25h_pages_layout,
#endif
};

#define INFINEON_MSPI_FLASH_S25H_CHECK_PROP_IS_UNDEFINED(n, prop)                                  \
	BUILD_ASSERT(DT_NODE_HAS_PROP(DT_DRV_INST(n), prop) == 0,                                  \
		     "The Infineon S25H driver ignores the property " #prop ". Don't use it")

/* Check for ignored/wrong values in the devicetree */
#define INFINEON_MSPI_FLASH_S25H_CHECK_DEVICETREE_CONFIG(n)                                        \
	INFINEON_MSPI_FLASH_S25H_CHECK_PROP_IS_UNDEFINED(n, rx_dummy);                             \
	INFINEON_MSPI_FLASH_S25H_CHECK_PROP_IS_UNDEFINED(n, tx_dummy);                             \
	INFINEON_MSPI_FLASH_S25H_CHECK_PROP_IS_UNDEFINED(n, read_command);                         \
	INFINEON_MSPI_FLASH_S25H_CHECK_PROP_IS_UNDEFINED(n, write_command);                        \
	INFINEON_MSPI_FLASH_S25H_CHECK_PROP_IS_UNDEFINED(n, xip_config);                           \
	INFINEON_MSPI_FLASH_S25H_CHECK_PROP_IS_UNDEFINED(n, scramble_config);                      \
	INFINEON_MSPI_FLASH_S25H_CHECK_PROP_IS_UNDEFINED(n, ce_break_config);                      \
	BUILD_ASSERT((DT_ENUM_HAS_VALUE(DT_DRV_INST(n), command_length, INSTR_1_BYTE) <= 2),       \
		     "The Infineon S25H chip uses only 1 byte opcodes")

#define INFINFEON_MSPI_FLASH_S25H_DEFINE(n)                                                        \
	INFINEON_MSPI_FLASH_S25H_CHECK_DEVICETREE_CONFIG(n);                                       \
	PINCTRL_DT_DEFINE(DT_DRV_INST(n));                                                         \
	static const struct flash_mspi_infineon_s25h_cfg flash_mspi_infineon_s25h_cfg_##n = {      \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(n)),                                              \
		.bus = DEVICE_DT_GET(DT_BUS(DT_DRV_INST(n))),                                      \
		.pinctrl = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                      \
		.reset_startup_duration = K_USEC(DT_INST_PROP(n, reset_startup_time_us)),          \
		.dev_id = MSPI_DEVICE_ID_DT_INST(n),                                               \
		.page_layout =                                                                     \
			{                                                                          \
				.pages_count = DT_INST_PROP(n, flash_size) /                       \
					       DT_INST_PROP(n, erase_block_size),                  \
				.pages_size = DT_INST_PROP(n, erase_block_size),                   \
			},                                                                         \
		.parameters =                                                                      \
			{                                                                          \
				.erase_value = 0xFF,                                               \
				.write_block_size = DT_INST_PROP(n, write_block_size),             \
			},                                                                         \
		.jedec_id = DT_INST_PROP(n, jedec_id),                                             \
	};                                                                                         \
	static struct flash_mspi_infineon_s25h_data flash_mspi_infineon_s25h_data_##n = {          \
		.mspi_dev_cfg = MSPI_DEVICE_CONFIG_DT_INST(n),                                     \
		.cmds = cmds_single,                                                               \
		.dummy_cycles = dummy_cycles_single,                                               \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, flash_mspi_infineon_s25h_init, NULL,                              \
			      &flash_mspi_infineon_s25h_data_##n,                                  \
			      &flash_mspi_infineon_s25h_cfg_##n, POST_KERNEL,                      \
			      CONFIG_FLASH_INIT_PRIORITY, &flash_mspi_infineon_s25h_driver_api);

DT_INST_FOREACH_STATUS_OKAY(INFINFEON_MSPI_FLASH_S25H_DEFINE)

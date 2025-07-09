#include "zephyr/drivers/regulator.h"
#include "zephyr/sd/sd_spec.h"
#include "zephyr/sys/util.h"
#define DT_DRV_COMPAT ti_sdhc

#include <errno.h>
#include <zephyr/drivers/sdhc.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

#ifdef CONFIG_DCACHE
#include <zephyr/cache.h>
#endif /* CONFIG_DCACHE */

LOG_MODULE_REGISTER(ti_sdhc, CONFIG_SDHC_LOG_LEVEL);

struct ti_ss_regs {
	uint8_t RESERVED_1[0x14];
	volatile uint32_t CTL_CFG_2;
	volatile uint32_t CTL_CFG_3;
	uint8_t RESERVED_4[0xE4];
	volatile uint32_t PHY_CTRL_1;
	uint8_t RESERVED_2[0x08];
	volatile uint32_t PHY_CTRL_4;
	volatile uint32_t PHY_CTRL_5;
	uint8_t RESERVED_3[0x1C];
	volatile uint32_t PHY_STAT_1;
};

/* Controller Config 2 Register */
#define TI_CTL_CFG_2_SLOTTYPE GENMASK(31, 30)

/* PHY Control 1 Register */
#define TI_PHY_CTRL_1_DR_TY       GENMASK(22, 20)
#define TI_PHY_CTRL_1_DLL_TRM_ICP GENMASK(7, 4)
#define TI_PHY_CTRL_1_ENDLL       BIT(1)

/* PHY Control 4 Register */
#define TI_PHY_CTRL_4_STRBSEL            GENMASK(31, 24)
#define TI_PHY_CTRL_4_STRBSEL_4BIT       GENMASK(27, 24)
#define TI_PHY_CTRL_4_OTAPDLYENA         BIT(20)
#define TI_PHY_CTRL_4_OTAPDLYSEL         GENMASK(15, 12)
#define TI_PHY_CTRL_4_ITAPCHGWIN         BIT(9)
#define TI_PHY_CTRL_4_ITAPDLYENA         BIT(8)
#define TI_PHY_CTRL_4_ITAPDLYSEL         GENMASK(4, 0)
#define TI_PHY_CTRL_4_ITAPDLYSEL_VAL_MAX (32)

/* PHY Control 5 Register */
#define TI_PHY_CTRL_5_SETDLYTXCLK            BIT(17)
#define TI_PHY_CTRL_5_SETDLYRXCLK            BIT(16)
#define TI_PHY_CTRL_5_FRQSEL                 GENMASK(10, 8)
#define TI_PHY_CTRL_5_FRQSEL_VAL_200_170_MHZ (0x0)
#define TI_PHY_CTRL_5_FRQSEL_VAL_170_140_MHZ (0x1)
#define TI_PHY_CTRL_5_FRQSEL_VAL_140_110_MHZ (0x2)
#define TI_PHY_CTRL_5_FRQSEL_VAL_110_80_MHZ  (0x3)
#define TI_PHY_CTRL_5_FRQSEL_VAL_80_50_MHZ   (0x4)
#define TI_PHY_CTRL_5_FRQSEL100              BIT(9)
#define TI_PHY_CTRL_5_FRQSEL50               BIT(8)
#define TI_PHY_CTRL_5_CLKBUFSEL              GENMASK(2, 0)

/* PHY Status 1 Register */
#define TI_PHY_STAT_1_DLLRDY BIT(0)

struct ti_hc_regs {
	uint8_t RES[0x4];
	volatile uint16_t BLOCK_SIZE;
	volatile uint16_t BLOCK_COUNT;
	volatile uint32_t ARGUMENT1;
	volatile uint16_t TRANSFER_MODE; /*todo*/
	volatile uint16_t COMMAND;
	volatile uint16_t RESPONSE_0;
	volatile uint16_t RESPONSE_1;
	volatile uint16_t RESPONSE_2;
	volatile uint16_t RESPONSE_3;
	volatile uint16_t RESPONSE_4;
	volatile uint16_t RESPONSE_5;
	volatile uint16_t RESPONSE_6;
	volatile uint16_t RESPONSE_7;
	volatile uint32_t DATA_PORT;
	volatile uint32_t PRESENTSTATE;
	volatile uint8_t HOST_CONTROL1;
	volatile uint8_t POWER_CONTROL;
	uint8_t RES2[0x2];
	volatile uint16_t CLOCK_CONTROL;
	uint8_t RES3[0x1];
	volatile uint8_t SOFTWARE_RESET;
	volatile uint16_t NORMAL_INTR_STS;
	volatile uint16_t ERROR_INTR_STS;
	volatile uint16_t NORMAL_INTR_STS_ENA;
	volatile uint16_t ERROR_INTR_STS_ENA;
	volatile uint16_t NORMAL_INTR_SIG_ENA;
	volatile uint16_t ERROR_INTR_SIG_ENA;
	uint8_t REG[0x02];
	volatile uint16_t HOST_CONTROL2;
	volatile uint64_t CAPABILITIES;
	volatile uint64_t MAX_CURRENT_CAP;
	uint8_t RESSS[0x8];
	volatile uint64_t ADMA_SYS_ADDRESS;
};

/* Block Size */
#define TI_BLOCK_SIZE_XFER_BLK_SIZE GENMASK(11, 0)

/* Transfer Mode */
#define TI_TRANSFER_MODE_DATA_XFER_DIR          BIT(4)
#define TI_TRANSFER_MODE_AUTO_CMD_ENA           GENMASK(3, 2)
#define TI_TRANSFER_MODE_AUTO_CMD_ENA_VAL_CMD12 (0x1)
#define TI_TRANSFER_MODE_AUTO_CMD_ENA_VAL_CMD23 (0x2)
#define TI_TRANSFER_MODE_BLK_CNT_ENA            BIT(1)

/* Command */
#define TI_COMMAND_CMD_INDEX                     GENMASK(13, 8)
#define TI_COMMAND_CMD_TYPE                      GENMASK(7, 6)
#define TI_COMMAND_CMD_TYPE_VAL_NORMAL           (0x0)
#define TI_COMMAND_DATA_PRESENT                  BIT(5)
#define TI_COMMAND_CMD_INDEX_CHK_ENA             BIT(4)
#define TI_COMMAND_CMD_CRC_CHK_ENA               BIT(3)
#define TI_COMMAND_RESP_TYPE_SEL                 GENMASK(1, 0)
#define TI_COMMAND_RESP_TYPE_SEL_VAL_NONE        (0x0)
#define TI_COMMAND_RESP_TYPE_SEL_VAL_LEN_136     (0x1)
#define TI_COMMAND_RESP_TYPE_SEL_VAL_LEN_48      (0x2)
#define TI_COMMAND_RESP_TYPE_SEL_VAL_LEN_48_BUSY (0x3)

/* Present State */
#define TI_PRESENTSTATE_CARD_INSERTED BIT(16)
#define TI_PRESENTSTATE_INHIBIT_DAT   BIT(1)
#define TI_PRESENTSTATE_INHIBIT_CMD   BIT(0)

/* Host Control 1 */
#define TI_HOST_CONTROL1_EXT_DATA_WIDTH       BIT(5)
#define TI_HOST_CONTROL1_HIGH_SPEED_ENA       BIT(2)
#define TI_HOST_CONTROL1_DATA_WIDTH           BIT(1)
#define TI_HOST_CONTROL1_DATA_WIDTH           BIT(1)
#define TI_HOST_CONTROL1_DMA_SELECT           GENMASK(4, 3)
#define TI_HOST_CONTROL1_DMA_SELECT_VAL_ADMA2 (0x2)

/* Power Control */
#define TI_POWER_CONTROL_SD_BUS_VOLTAGE          GENMASK(3, 1)
#define TI_POWER_CONTROL_SD_BUS_VOLTAGE_VAL_V3P3 (0x7)
#define TI_POWER_CONTROL_SD_BUS_VOLTAGE_VAL_V3P0 (0x6)
#define TI_POWER_CONTROL_SD_BUS_VOLTAGE_VAL_V1P8 (0x5)
#define TI_POWER_CONTROL_SD_BUS_POWER            BIT(0)

/* Clock Control */
#define TI_CLOCK_CONTROL_SDCLK_FRQSEL             GENMASK(15, 8)
#define TI_CLOCK_CONTROL_SDCLK_FRQSEL_UPBITS      GENMASK(7, 6)
#define TI_CLOCK_CONTROL_SDCLK_FRQSEL_VAL_MAX     (0x3FF)
#define TI_CLOCK_CONTROL_SDCLK_FRQSEL_VAL_MASK_HI (0x300)
#define TI_CLOCK_CONTROL_SDCLK_FRQSEL_VAL_MASK_LO (0x0FF)
#define TI_CLOCK_CONTROL_CLKGEN_SEL               BIT(5)
#define TI_CLOCK_CONTROL_SD_CLK_ENA               BIT(2)
#define TI_CLOCK_CONTROL_INT_CLK_STABLE           BIT(1)
#define TI_CLOCK_CONTROL_INT_CLK_ENA              BIT(0)

/* Software Reset */
#define TI_SOFTWARE_RESET_SWRST_FOR_ALL BIT(0)

/* Normal Interrupt Bits (common to several registers) */
#define TI_NORMAL_INTR_CARD_REMOVAL   BIT(7)
#define TI_NORMAL_INTR_CARD_INSERTION BIT(6)
#define TI_NORMAL_INTR_BUF_RD_READY   BIT(5)
#define TI_NORMAL_INTR_BUF_WR_READY   BIT(4)
#define TI_NORMAL_INTR_XFER_COMPLETE  BIT(1)
#define TI_NORMAL_INTR_CMD_COMPLETE   BIT(0)

/* Error interrupt bits */
#define TI_ERROR_INTR_ALL GENMASK(15, 0)

/* Host Control 2 */
#define TI_HOST_CONTROL2_ADMA2_LEN_MODE             BIT(10)
#define TI_HOST_CONTROL2_V1P8_SIGNAL_ENA            BIT(3)
#define TI_HOST_CONTROL2_UHS_MODE_SELECT            GENMASK(2, 0)
#define TI_HOST_CONTROL2_UHS_MODE_SELECT_VAL_SDR12  (0x0)
#define TI_HOST_CONTROL2_UHS_MODE_SELECT_VAL_SDR25  (0x1)
#define TI_HOST_CONTROL2_UHS_MODE_SELECT_VAL_SDR50  (0x2)
#define TI_HOST_CONTROL2_UHS_MODE_SELECT_VAL_SDR104 (0x3)
#define TI_HOST_CONTROL2_UHS_MODE_SELECT_VAL_DDR50  (0x4)
#define TI_HOST_CONTROL2_UHS_MODE_SELECT_VAL_HS400  (0x5)

/* Capabilities */
#define TI_CAPABILITIES_BUS_HS400_SUPPORT BIT64(63)

/* Max Current Capabilities */
#define TI_MAX_CURRENT_CAP_VDD2_1P8V GENMASK64(39, 32)
#define TI_MAX_CURRENT_CAP_VDD1_1P8V GENMASK64(23, 16)
#define TI_MAX_CURRENT_CAP_VDD1_3P0V GENMASK64(15, 8)
#define TI_MAX_CURRENT_CAP_VDD1_3P3V GENMASK64(7, 0)

struct ti_adma2_descriptor_table {
	bool valid: 1;
	bool end: 1;
	bool interrupt: 1;
	uint8_t action: 3;
	uint16_t reserved: 10;
	uint16_t length: 16;
	uint32_t address: 32;
} __packed;

#define TI_ADMA2_DESC_ACTION_LINK (0x3)
#define TI_ADMA2_DESC_ACTION_TRAN (0x2)

struct ti_tap_delay_config {
	bool itap_delay_enable;
	uint8_t itap_delay_value;
	bool otap_delay_enable;
	uint8_t otap_delay_value;
};

/* Miscellaneous */
#define TI_TUNING_MODE_MAX (SDHC_TIMING_HS400 + 1)

/* SDHC configuration. */
struct ti_config {
	DEVICE_MMIO_NAMED_ROM(host);
	DEVICE_MMIO_NAMED_ROM(subsys);
	void (*irq_func)(const struct device *dev);
	struct ti_tap_delay_config delay_config[TI_TUNING_MODE_MAX];
	const struct device *vmmc;
	const struct device *vqmmc;
	uint8_t clkbuf_sel;
	bool strobe_sel_4_bit;
	uint8_t strobe_sel;
	bool freq_sel_2_bit;
	uint8_t drive_impedance;
	uint8_t current_trim;
};

struct ti_data {
	DEVICE_MMIO_NAMED_RAM(host);
	DEVICE_MMIO_NAMED_RAM(subsys);
#ifdef CONFIG_TI_SDHC_ENABLE_ADMA
	struct ti_adma2_descriptor_table descs[MAX(CONFIG_TI_SDHC_ADMA_DESC_LEN, 1)];
#endif /* CONFIG_TI_SDHC_ENABLE_ADMA */
	struct sdhc_host_props props;
	struct sdhc_io ios;
	struct k_event irq_event;
#define TI_IRQ_EVENT_ERROR_INTR(intr) (intr << 16)
	sdhc_interrupt_cb_t callback;
	void *user_data;
};

#define DEV_CFG(dev)     ((const struct ti_config *)dev->config)
#define DEV_DATA(dev)    ((struct ti_data *)dev->data)
#define DEV_HC_REGS(dev) ((struct ti_hc_regs *)DEVICE_MMIO_NAMED_GET(dev, host))
#define DEV_SS_REGS(dev) ((struct ti_ss_regs *)DEVICE_MMIO_NAMED_GET(dev, subsys))

static int ti_reset(const struct device *dev)
{
	struct ti_hc_regs *hc_regs = DEV_HC_REGS(dev);
	int retries = 100;

	/* do software reset */
	hc_regs->SOFTWARE_RESET |= TI_SOFTWARE_RESET_SWRST_FOR_ALL;

	/* wait for completion */
	while (hc_regs->SOFTWARE_RESET & TI_SOFTWARE_RESET_SWRST_FOR_ALL) {
		if (retries-- == 0) {
			LOG_ERR("Timed out while resetting all circuits\n");
			return -ETIMEDOUT;
		}
		k_busy_wait(100);
	}

	return 0;
}

static ALWAYS_INLINE k_timeout_t ti_timeout_from_msec(int timeout)
{
	return (timeout == SDHC_TIMEOUT_FOREVER ? K_FOREVER : K_MSEC(timeout));
}

static int ti_init_resp_type(enum sd_rsp_type type, uint8_t *resp_type_sel, bool *idx_check,
			     bool *crc_check)
{
	switch (type & SDHC_NATIVE_RESPONSE_MASK) {
	case SD_RSP_TYPE_NONE:
		*resp_type_sel = TI_COMMAND_RESP_TYPE_SEL_VAL_NONE;
		*idx_check = false;
		*crc_check = false;
		break;
	case SD_RSP_TYPE_R2:
		*resp_type_sel = TI_COMMAND_RESP_TYPE_SEL_VAL_LEN_136;
		*idx_check = false;
		*crc_check = true;
		break;
	case SD_RSP_TYPE_R3:
	case SD_RSP_TYPE_R4:
		*resp_type_sel = TI_COMMAND_RESP_TYPE_SEL_VAL_LEN_48;
		*idx_check = false;
		*crc_check = false;
		break;
	case SD_RSP_TYPE_R1:
	case SD_RSP_TYPE_R6:
	case SD_RSP_TYPE_R5:
	case SD_RSP_TYPE_R7:
		*resp_type_sel = TI_COMMAND_RESP_TYPE_SEL_VAL_LEN_48;
		*idx_check = true;
		*crc_check = true;
		break;
	case SD_RSP_TYPE_R1b:
	case SD_RSP_TYPE_R5b:
		*resp_type_sel = TI_COMMAND_RESP_TYPE_SEL_VAL_LEN_48_BUSY;
		*idx_check = true;
		*crc_check = true;
		break;
	default:
		LOG_ERR("Unsupported response type");
		return -ENOTSUP;
	}

	return 0;
}

static void ti_read_cmd_resp(const struct device *dev, struct sdhc_command *cmd)
{
	enum sd_rsp_type type = cmd->response_type;
	struct ti_hc_regs *hc_regs = DEV_HC_REGS(dev);

	uint32_t r01 = (hc_regs->RESPONSE_1 << 16) | hc_regs->RESPONSE_0;
	uint32_t r23 = (hc_regs->RESPONSE_3 << 16) | hc_regs->RESPONSE_2;
	uint32_t r45 = (hc_regs->RESPONSE_5 << 16) | hc_regs->RESPONSE_4;
	uint32_t r67 = (hc_regs->RESPONSE_7 << 16) | hc_regs->RESPONSE_6;

	switch (type & SDHC_NATIVE_RESPONSE_MASK) {
	case SD_RSP_TYPE_NONE:
		cmd->response[3] = 0;
		cmd->response[2] = 0;
		cmd->response[1] = 0;
		cmd->response[0] = 0;
		break;
	case SD_RSP_TYPE_R2:
		/* REP[119:0] */
		/* shift by 1 byte to make it [127:8] for parsing */
		cmd->response[3] = ((r67 & GENMASK(23, 0)) << 8) | (r45 >> 24);
		cmd->response[2] = (r45 << 8) | (r23 >> 24);
		cmd->response[1] = (r23 << 8) | (r01 >> 24);
		cmd->response[0] = (r01 << 8);
		break;
	default:
		/* REP[31:0] */
		cmd->response[3] = 0;
		cmd->response[2] = 0;
		cmd->response[1] = 0;
		cmd->response[0] = r01;
	}

	printk("r01 %x %x\n", r01, r67);
}

static int ti_request_cmd_send(const struct device *dev, struct sdhc_command *cmd,
			       bool data_present)
{
	struct ti_hc_regs *hc_regs = DEV_HC_REGS(dev);
	struct ti_data *data = DEV_DATA(dev);
	uint32_t tries = cmd->retries + 1;
	bool idx_chk;
	bool crc_chk;
	uint8_t resp_type_sel;
	uint32_t events;
	int rv;

	rv = ti_init_resp_type(cmd->response_type, &resp_type_sel, &idx_chk, &crc_chk);
	if (rv != 0) {
		return rv;
	}
	printk("presentstate = %x\n", hc_regs->PRESENTSTATE);

	if (hc_regs->PRESENTSTATE & TI_PRESENTSTATE_INHIBIT_CMD) {
		LOG_ERR("command line is already busy");
		return -EBUSY;
	}

	while (tries--) {
		k_event_clear(&data->irq_event, TI_NORMAL_INTR_CMD_COMPLETE);

		printk("type idx crc %u %u %u\n", resp_type_sel, idx_chk, crc_chk);
		hc_regs->ARGUMENT1 = cmd->arg;
		uint16_t xx = FIELD_PREP(TI_COMMAND_CMD_INDEX, cmd->opcode) |
			      FIELD_PREP(TI_COMMAND_CMD_TYPE, TI_COMMAND_CMD_TYPE_VAL_NORMAL) |
			      FIELD_PREP(TI_COMMAND_RESP_TYPE_SEL, resp_type_sel) |
			      FIELD_PREP(TI_COMMAND_CMD_INDEX_CHK_ENA, idx_chk) |
			      FIELD_PREP(TI_COMMAND_CMD_CRC_CHK_ENA, crc_chk) |
			      FIELD_PREP(TI_COMMAND_DATA_PRESENT, data_present);

		printk("command %x %u\n", xx, cmd->opcode);
		hc_regs->COMMAND = xx;
		printk("cmd and data2 = %x\n\n", hc_regs->PRESENTSTATE);
		events = k_event_wait(
			&data->irq_event,
			(TI_NORMAL_INTR_CMD_COMPLETE | TI_IRQ_EVENT_ERROR_INTR(TI_ERROR_INTR_ALL)),
			false, ti_timeout_from_msec(cmd->timeout_ms));

		if (events & TI_NORMAL_INTR_CMD_COMPLETE) {
			ti_read_cmd_resp(dev, cmd);
			return 0;
		} else if (events & TI_IRQ_EVENT_ERROR_INTR(TI_ERROR_INTR_ALL)) {
			printk("presentstate = %x\n\n", hc_regs->PRESENTSTATE);
			LOG_ERR("error occurred while sending command, events = %x", events);
			return -EIO;
		} else {
			LOG_ERR("timeout occurred while sending command");
			printk("cmd and data3 = %x\n\n", hc_regs->PRESENTSTATE);
		}
	}

	return -ETIMEDOUT;
}

#ifndef CONFIG_TI_SDHC_ENABLE_AUTO_STOP
static int ti_request_stop_transmission(const struct device *dev)
{
	int rv;
	struct sdhc_command stop_cmd = {
		.opcode = SD_STOP_TRANSMISSION,
		.arg = 0,
		.response_type = SD_RSP_TYPE_NONE,
		.retries = 0,
		.timeout_ms = 1000,
	};

	rv = ti_request_cmd_send(dev, &stop_cmd, false);
	if (rv != 0) {
		LOG_ERR("failed to stop transmission");
	}

	return rv;
}
#endif /* !CONFIG_TI_SDHC_ENABLE_AUTO_STOP */

#ifdef CONFIG_TI_SDHC_ENABLE_ADMA
static int ti_request_data_setup_adma(const struct device *dev, struct sdhc_data *dat,
				      bool is_write)
{
	struct ti_hc_regs *hc_regs = DEV_HC_REGS(dev);
	struct ti_data *data = DEV_DATA(dev);
	uint8_t *address = dat->data;
	uint16_t host_control1;
	int i;

	if (ARRAY_SIZE(data->descs) < dat->blocks) {
		LOG_ERR("number of descriptors %u is less than the number of blocks %u",
			ARRAY_SIZE(data->descs), dat->blocks);
		return -EINVAL;
	}

#ifdef CONFIG_DCACHE
	if (is_write) {
		sys_cache_data_flush_range(dat->data, dat->blocks * dat->block_size);
	}
#endif /* CONFIG_DACHE */

	/* n - 1 blocks */
	for (i = 0; i < dat->blocks - 1; i++) {
		data->descs[i].valid = true;
		data->descs[i].end = false;
		data->descs[i].interrupt = false;
		data->descs[i].action = TI_ADMA2_DESC_ACTION_LINK;
		data->descs[i].length = dat->block_size;
		data->descs[i].address = (uint32_t)address;
		address += dat->block_size;
	}

	/* last block */
	data->descs[i].valid = true;
	data->descs[i].end = true;
	data->descs[i].interrupt = true;
	data->descs[i].action = TI_ADMA2_DESC_ACTION_TRAN;
	data->descs[i].length = dat->block_size;
	data->descs[i].address = (uint32_t)address;

	/* write descriptor address */
	hc_regs->ADMA_SYS_ADDRESS = (uint64_t)data->descs;

	/* configure host_control1 */
	host_control1 = hc_regs->HOST_CONTROL1;
	host_control1 &= ~TI_HOST_CONTROL1_DMA_SELECT;
	host_control1 |=
		FIELD_PREP(TI_HOST_CONTROL1_DMA_SELECT, TI_HOST_CONTROL1_DMA_SELECT_VAL_ADMA2);
	hc_regs->HOST_CONTROL1 = host_control1;

	/* configure host control 2 */
	/* adma 16 bit length mode */
	hc_regs->HOST_CONTROL2 &= ~TI_HOST_CONTROL2_ADMA2_LEN_MODE;

	return 0;
}
#else
static int ti_request_data_write(const struct device *dev, struct sdhc_data *dat)
{
	struct ti_hc_regs *hc_regs = DEV_HC_REGS(dev);
	struct ti_data *data = DEV_DATA(dev);
	uint32_t *data_32;
	uint8_t *data_8 = dat->data;
	uint32_t block_cnt = dat->blocks;
	uint32_t events;

	dat->bytes_xfered = 0;

	while (block_cnt > 0) {
		data_32 = (uint32_t *)(data_8 + dat->bytes_xfered);

		events = k_event_wait(&data->irq_event, TI_NORMAL_INTR_BUF_WR_READY, false,
				      ti_timeout_from_msec(dat->timeout_ms));
		k_event_clear(&data->irq_event, TI_NORMAL_INTR_BUF_WR_READY);

		if ((events & TI_NORMAL_INTR_BUF_WR_READY) == 0) {
			LOG_ERR("timeout occurred while writing to data port");
			return -EIO;
		}

		for (int i = 0; i < DIV_ROUND_UP(dat->block_size, 4); i++) {
			hc_regs->DATA_PORT = *(data_32++);
		}

		dat->bytes_xfered += dat->block_size;
		block_cnt--;
	};

	return 0;
}

static int ti_request_data_read(const struct device *dev, struct sdhc_data *dat)
{
	struct ti_hc_regs *hc_regs = DEV_HC_REGS(dev);
	struct ti_data *data = DEV_DATA(dev);
	uint32_t *data_32;
	uint8_t *data_8 = dat->data;
	uint32_t block_cnt = dat->blocks;
	uint32_t events;
	printk("cnt size %u %u\n", block_cnt, dat->block_size);

	dat->bytes_xfered = 0;

	while (block_cnt > 0) {
		data_32 = (uint32_t *)(data_8 + dat->bytes_xfered);

		events = k_event_wait(&data->irq_event, TI_NORMAL_INTR_BUF_RD_READY, false,
				      ti_timeout_from_msec(dat->timeout_ms));
		k_event_clear(&data->irq_event, TI_NORMAL_INTR_BUF_RD_READY);

		if ((events & TI_NORMAL_INTR_BUF_RD_READY) == 0) {
			LOG_ERR("timeout occurred while reading from data port");
			return -EIO;
		}

		for (int i = 0; i < DIV_ROUND_UP(dat->block_size, 4); i++) {
			*(data_32++) = hc_regs->DATA_PORT;
		}

		dat->bytes_xfered += dat->block_size;
		block_cnt--;
	};

	return 0;
}
#endif /* CONFIG_TI_SDHC_ENABLE_ADMA */

static int ti_request_data_setup(const struct device *dev, struct sdhc_data *dat, bool is_write)
{
	struct ti_hc_regs *hc_regs = DEV_HC_REGS(dev);
	struct ti_data *data = DEV_DATA(dev);
	uint16_t transfer_mode = hc_regs->TRANSFER_MODE;

#ifdef CONFIG_TI_SDHC_ENABLE_ADMA
	int rv = ti_request_data_setup_adma(dev, dat, is_write);
	if (rv != 0) {
		return rv;
	}
#endif /* CONFIG_TI_SDHC_ENABLE_ADMA */

	hc_regs->BLOCK_SIZE = FIELD_PREP(TI_BLOCK_SIZE_XFER_BLK_SIZE, dat->block_size);

	if (is_write) {
		transfer_mode &= ~TI_TRANSFER_MODE_DATA_XFER_DIR;
	} else {
		transfer_mode |= TI_TRANSFER_MODE_DATA_XFER_DIR;
	}

	if (dat->blocks > 1) {
#ifdef CONFIG_TI_SDHC_ENABLE_AUTO_STOP
		hc_regs->BLOCK_COUNT = dat->blocks;
		transfer_mode |= TI_TRANSFER_MODE_BLK_CNT_ENA;

		transfer_mode &= ~TI_TRANSFER_MODE_AUTO_CMD_ENA;

		/* mandatory for sdr104 */
		if (IS_ENABLED(CONFIG_TI_SDHC_ENABLE_ADMA) &&
		    data->ios.timing == SDHC_TIMING_SDR104) {
			transfer_mode |= FIELD_PREP(TI_TRANSFER_MODE_AUTO_CMD_ENA,
						    TI_TRANSFER_MODE_AUTO_CMD_ENA_VAL_CMD23);
		} else {
			transfer_mode |= FIELD_PREP(TI_TRANSFER_MODE_AUTO_CMD_ENA,
						    TI_TRANSFER_MODE_AUTO_CMD_ENA_VAL_CMD12);
		}
#else
		hc_regs->BLOCK_COUNT = 0;
		transfer_mode &= ~TI_TRANSFER_MODE_BLK_CNT_ENA;
		transfer_mode &= ~TI_TRANSFER_MODE_AUTO_CMD_ENA;
#endif /* CONFIG_TI_SDHC_ENABLE_AUTO_STOP */
	}

	hc_regs->TRANSFER_MODE = transfer_mode;

	return 0;
}

static ALWAYS_INLINE bool ti_is_cmd_write(uint32_t opcode)
{
	return opcode == SD_WRITE_SINGLE_BLOCK || opcode == SD_WRITE_MULTIPLE_BLOCK;
}

static int ti_request(const struct device *dev, struct sdhc_command *cmd, struct sdhc_data *dat)
{
	struct ti_data *data = DEV_DATA(dev);
	bool data_present = dat != NULL;
	bool is_write = ti_is_cmd_write(cmd->opcode);
	int rv;

        data->irq_event.events = 0;

	if (data_present) {
		rv = ti_request_data_setup(dev, dat, is_write);
		if (rv != 0) {
			return rv;
		}
	}
	printk("datapresent = %u\n", data_present);

	rv = ti_request_cmd_send(dev, cmd, data_present);
	if (rv != 0) {
		return rv;
	}

	if (data_present) {
		uint32_t events;

#ifndef CONFIG_TI_SDHC_ENABLE_ADMA
		if (is_write) {
			rv = ti_request_data_write(dev, dat);
		} else {
			rv = ti_request_data_read(dev, dat);
		}

		if (rv != 0) {
			return rv;
		}

#ifndef CONFIG_TI_SDHC_ENABLE_AUTO_STOP
		rv = ti_request_stop_transmission(dev);
		if (rv != 0) {
			return rv;
		}
#endif /* !CONFIG_TI_SDHC_ENABLE_AUTO_STOP */
#endif /* !CONFIG_TI_SDHC_ENABLE_ADMA */

		events = k_event_wait(&data->irq_event, TI_NORMAL_INTR_XFER_COMPLETE, false,
				      ti_timeout_from_msec(dat->timeout_ms));
		if (events & TI_NORMAL_INTR_XFER_COMPLETE) {
			return 0;
		} else if (events & TI_IRQ_EVENT_ERROR_INTR(TI_ERROR_INTR_ALL)) {
			LOG_ERR("error occurred while sending command, events = %x", events);
			return -EIO;
		} else {
			LOG_ERR("timeout occurred while completing data transfer");
			return -ETIMEDOUT;
		}
	}

	return 0;
}

static int ti_configure_clock(const struct device *dev, enum sdhc_clock_speed clock,
			      bool enable_clock)
{
	struct ti_hc_regs *hc_regs = DEV_HC_REGS(dev);
	struct ti_data *data = DEV_DATA(dev);
	uint32_t multiplier = data->props.host_caps.clk_multiplier;
	uint32_t base = MHZ(data->props.host_caps.sd_base_clk);
	bool prog_clk_mode = false;
	uint16_t frqsel = 0;
	uint16_t divisor = 0;

	printk("base %uHz clock %uHz multi %u\n", base, clock, multiplier);

	hc_regs->CLOCK_CONTROL = 0;

	if (clock == 0) {
		return 0;
	}

	/* Programmable Clock Mode */
	if (multiplier != 0) {
		for (frqsel = 0; frqsel <= TI_CLOCK_CONTROL_SDCLK_FRQSEL_VAL_MAX; frqsel++) {
			divisor = frqsel + 1;
			if ((base * multiplier) / divisor <= clock) {
				prog_clk_mode = true;
				goto write_register;
			}
		}
	}

	/* 10-bit Divided Clock Mode */
	if (base <= clock) {
		frqsel = 0;
		divisor = 1;
	} else {
		for (frqsel = 1; frqsel <= TI_CLOCK_CONTROL_SDCLK_FRQSEL_VAL_MAX; frqsel++) {
			divisor = frqsel << 1;
			if (base / divisor <= clock) {
				break;
			}
		}

		if (frqsel > TI_CLOCK_CONTROL_SDCLK_FRQSEL_VAL_MAX) {
			LOG_ERR("Configured clock speed %uHz is too low", clock);
			return -EINVAL;
		}
	}

write_register: {
	uint8_t frqsel_lo = FIELD_GET(TI_CLOCK_CONTROL_SDCLK_FRQSEL_VAL_MASK_LO, frqsel);
	uint8_t frqsel_hi = FIELD_GET(TI_CLOCK_CONTROL_SDCLK_FRQSEL_VAL_MASK_HI, frqsel);

	hc_regs->CLOCK_CONTROL = FIELD_PREP(TI_CLOCK_CONTROL_SDCLK_FRQSEL, frqsel_lo) |
				 FIELD_PREP(TI_CLOCK_CONTROL_SDCLK_FRQSEL_UPBITS, frqsel_hi) |
				 FIELD_PREP(TI_CLOCK_CONTROL_CLKGEN_SEL, prog_clk_mode) |
				 TI_CLOCK_CONTROL_INT_CLK_ENA;
}

	if (enable_clock || 1) {
		while (!(hc_regs->CLOCK_CONTROL & TI_CLOCK_CONTROL_INT_CLK_STABLE)) {
		}

		hc_regs->CLOCK_CONTROL |= TI_CLOCK_CONTROL_SD_CLK_ENA;
	}
	printk("stable\n");

	return 0;
}

static int ti_configure_delay_locked_loop(const struct device *dev, enum sdhc_timing_mode mode,
					  enum sdhc_clock_speed clock)
{
	struct ti_ss_regs *ss_regs = DEV_SS_REGS(dev);
	const struct ti_config *config = DEV_CFG(dev);
	uint32_t phy_ctrl_5;
	uint32_t phy_ctrl_1;
	int retries;

	/* read phy_ctrl5 */
	phy_ctrl_5 = ss_regs->PHY_CTRL_5;

	/* modify phy_ctrl5 */
	if (config->freq_sel_2_bit) {
		switch (clock) {
		case MHZ(200):
			phy_ctrl_5 &= ~TI_PHY_CTRL_5_FRQSEL100;
			phy_ctrl_5 &= ~TI_PHY_CTRL_5_FRQSEL50;
			break;
		case MHZ(100):
			phy_ctrl_5 |= TI_PHY_CTRL_5_FRQSEL100;
			phy_ctrl_5 &= ~TI_PHY_CTRL_5_FRQSEL50;
			break;
		default:
			phy_ctrl_5 &= ~TI_PHY_CTRL_5_FRQSEL100;
			phy_ctrl_5 |= TI_PHY_CTRL_5_FRQSEL50;
		}
	} else {
		uint8_t frqsel;
		if (clock <= MHZ(200) && clock > MHZ(170)) {
			frqsel = TI_PHY_CTRL_5_FRQSEL_VAL_200_170_MHZ;
		} else if (clock <= MHZ(170) && clock > MHZ(140)) {
			frqsel = TI_PHY_CTRL_5_FRQSEL_VAL_170_140_MHZ;
		} else if (clock <= MHZ(140) && clock > MHZ(110)) {
			frqsel = TI_PHY_CTRL_5_FRQSEL_VAL_140_110_MHZ;
		} else if (clock <= MHZ(110) && clock > MHZ(80)) {
			frqsel = TI_PHY_CTRL_5_FRQSEL_VAL_110_80_MHZ;
		} else {
			frqsel = TI_PHY_CTRL_5_FRQSEL_VAL_80_50_MHZ;
		}
		phy_ctrl_5 &= ~TI_PHY_CTRL_5_FRQSEL;
		phy_ctrl_5 |= FIELD_PREP(TI_PHY_CTRL_5_FRQSEL, frqsel);
	}

	/* write phy_ctrl5 */
	ss_regs->PHY_CTRL_5 = phy_ctrl_5;

	/* read phy_ctrl1 */
	phy_ctrl_1 = ss_regs->PHY_CTRL_1;

	/* modify phy_ctrl1 */
	phy_ctrl_1 &= ~(TI_PHY_CTRL_1_DR_TY | TI_PHY_CTRL_1_DLL_TRM_ICP);
	phy_ctrl_1 |= FIELD_PREP(TI_PHY_CTRL_1_DR_TY, config->drive_impedance) |
		      FIELD_PREP(TI_PHY_CTRL_1_DLL_TRM_ICP, config->current_trim) |
		      TI_PHY_CTRL_1_ENDLL;

	/* write phy_ctrl1 */
	ss_regs->PHY_CTRL_1 = phy_ctrl_1;

	/* poll ready state for 1 second */
	retries = 1000;
	while ((ss_regs->PHY_STAT_1 & TI_PHY_STAT_1_DLLRDY) == 0) {
		if (retries-- == 0) {
			LOG_ERR("Timed out while waiting for DLL to be ready");
			return -ETIMEDOUT;
		}
		k_busy_wait(1000);
	}

	return 0;
}

static void ti_configure_delay_chain(const struct device *dev, enum sdhc_timing_mode mode)
{
	struct ti_ss_regs *ss_regs = DEV_SS_REGS(dev);
	const struct ti_config *config = DEV_CFG(dev);
	struct ti_tap_delay_config delay_config = config->delay_config[mode];
        uint32_t phy_ctrl_5;

	/* disable DLL for now */
	ss_regs->PHY_CTRL_1 &= ~TI_PHY_CTRL_1_ENDLL;

	/* read */
	phy_ctrl_5 = ss_regs->PHY_CTRL_5;

	/* modify */
	if (delay_config.itap_delay_enable) {
		phy_ctrl_5 |= TI_PHY_CTRL_5_SETDLYRXCLK;
	} else {
		phy_ctrl_5 &= ~TI_PHY_CTRL_5_SETDLYRXCLK;
	}

	if (delay_config.otap_delay_enable) {
		phy_ctrl_5 |= TI_PHY_CTRL_5_SETDLYTXCLK;
	} else {
		phy_ctrl_5 &= ~TI_PHY_CTRL_5_SETDLYTXCLK;
	}

	/* write */
	ss_regs->PHY_CTRL_5 = phy_ctrl_5;
}

static void ti_configure_tap_delays(const struct device *dev,
				    const struct ti_tap_delay_config *delay_config)
{
	struct ti_ss_regs *ss_regs = DEV_SS_REGS(dev);
	uint32_t phy_ctrl_4;

	/* read phy_ctrl4 */
	phy_ctrl_4 = ss_regs->PHY_CTRL_4;

	/* modify phy_ctrl4 */
	phy_ctrl_4 &= ~(TI_PHY_CTRL_4_ITAPDLYENA | TI_PHY_CTRL_4_ITAPDLYSEL |
			TI_PHY_CTRL_4_OTAPDLYENA | TI_PHY_CTRL_4_OTAPDLYSEL);
	phy_ctrl_4 |= FIELD_PREP(TI_PHY_CTRL_4_ITAPDLYENA, delay_config->itap_delay_enable) |
		      FIELD_PREP(TI_PHY_CTRL_4_ITAPDLYSEL, delay_config->itap_delay_value) |
		      FIELD_PREP(TI_PHY_CTRL_4_OTAPDLYENA, delay_config->otap_delay_enable) |
		      FIELD_PREP(TI_PHY_CTRL_4_OTAPDLYSEL, delay_config->otap_delay_value);

	/* write phy_ctrl4 */
	ss_regs->PHY_CTRL_4 |= TI_PHY_CTRL_4_ITAPCHGWIN;
	ss_regs->PHY_CTRL_4 = phy_ctrl_4;
	ss_regs->PHY_CTRL_4 &= ~TI_PHY_CTRL_4_ITAPCHGWIN;
}

static int ti_configure_timing_8_bit(const struct device *dev, enum sdhc_timing_mode mode,
				     enum sdhc_clock_speed clock)
{
	struct ti_ss_regs *ss_regs = DEV_SS_REGS(dev);
	const struct ti_config *config = DEV_CFG(dev);
	uint32_t phy_ctrl_4;
	uint32_t phy_ctrl_5;
	int rv;

	printk("configuring timing rn\n");

	/* configure itap and otap delay */
	ti_configure_tap_delays(dev, &config->delay_config[mode]);

	/* read phy_ctrl4 and phy_ctrl5 */
	phy_ctrl_4 = ss_regs->PHY_CTRL_4;
	phy_ctrl_5 = ss_regs->PHY_CTRL_5;

	/* modify phy_ctrl4 */
	if (mode == SDHC_TIMING_HS400) {
		uint32_t strobe_sel_mask;
		if (config->strobe_sel_4_bit) {
			strobe_sel_mask = TI_PHY_CTRL_4_STRBSEL_4BIT;
		} else {
			strobe_sel_mask = TI_PHY_CTRL_4_STRBSEL;
		}

		phy_ctrl_4 &= ~strobe_sel_mask;
		phy_ctrl_4 |= FIELD_PREP(strobe_sel_mask, config->strobe_sel);
	}

	/* modify phy_ctrl5 */
	phy_ctrl_5 &= ~TI_PHY_CTRL_5_CLKBUFSEL;
	phy_ctrl_5 |= FIELD_PREP(TI_PHY_CTRL_5_CLKBUFSEL, config->clkbuf_sel);

	/* write phy_ctrl4 and phy_ctrl5 */
	ss_regs->PHY_CTRL_4 = phy_ctrl_4;
	ss_regs->PHY_CTRL_5 = phy_ctrl_5;

	switch (mode) {
	case SDHC_TIMING_LEGACY:
	case SDHC_TIMING_HS:
	case SDHC_TIMING_SDR12:
	case SDHC_TIMING_SDR25:
		ti_configure_delay_chain(dev, mode);
		break;
	case SDHC_TIMING_SDR50:
	case SDHC_TIMING_SDR104:
	case SDHC_TIMING_DDR50:
	case SDHC_TIMING_DDR52:
	case SDHC_TIMING_HS200:
	case SDHC_TIMING_HS400: {
		rv = ti_configure_delay_locked_loop(dev, mode, clock);
		if (rv != 0) {
			return rv;
		}
	}
	}

	return 0;
}

static int ti_configure_timing_4_bit(const struct device *dev, enum sdhc_timing_mode mode)
{
	struct ti_ss_regs *ss_regs = DEV_SS_REGS(dev);
	const struct ti_config *config = DEV_CFG(dev);
	uint32_t phy_ctrl_5;

	/* configure itap and otap delay */
	ti_configure_tap_delays(dev, &config->delay_config[mode]);

	/* read phy_ctrl5 */
	phy_ctrl_5 = ss_regs->PHY_CTRL_5;

	/* modify phy_ctrl5 */
	phy_ctrl_5 &= ~TI_PHY_CTRL_5_CLKBUFSEL;
	phy_ctrl_5 |= FIELD_PREP(TI_PHY_CTRL_5_CLKBUFSEL, config->clkbuf_sel);

	/* write phy_ctrl5 */
	ss_regs->PHY_CTRL_5 = phy_ctrl_5;

	return 0;
}

static int ti_configure_timing(const struct device *dev, enum sdhc_timing_mode mode,
			       enum sdhc_clock_speed clock)
{
	struct ti_data *data = DEV_DATA(dev);
	struct ti_hc_regs *hc_regs = DEV_HC_REGS(dev);
	uint8_t uhs_mode = 0;

        if (clock == 0)
          return 0;

        printk("mode = %u %u\n", mode, clock);

	switch (mode) {
	case SDHC_TIMING_LEGACY:
		hc_regs->HOST_CONTROL1 &= ~TI_HOST_CONTROL1_HIGH_SPEED_ENA;
		break;
	case SDHC_TIMING_HS:
		hc_regs->HOST_CONTROL1 |= TI_HOST_CONTROL1_HIGH_SPEED_ENA;
		break;
	case SDHC_TIMING_SDR12:
		uhs_mode = TI_HOST_CONTROL2_UHS_MODE_SELECT_VAL_SDR12;
		break;
	case SDHC_TIMING_SDR25:
		uhs_mode = TI_HOST_CONTROL2_UHS_MODE_SELECT_VAL_SDR25;
		hc_regs->HOST_CONTROL1 |= TI_HOST_CONTROL1_HIGH_SPEED_ENA;
		break;
	case SDHC_TIMING_SDR50:
		uhs_mode = TI_HOST_CONTROL2_UHS_MODE_SELECT_VAL_SDR50;
		break;
	case SDHC_TIMING_SDR104:
	case SDHC_TIMING_HS200:
		uhs_mode = TI_HOST_CONTROL2_UHS_MODE_SELECT_VAL_SDR104;
		break;
	case SDHC_TIMING_DDR50:
	case SDHC_TIMING_DDR52:
		uhs_mode = TI_HOST_CONTROL2_UHS_MODE_SELECT_VAL_DDR50;
		break;
	case SDHC_TIMING_HS400:
		uhs_mode = TI_HOST_CONTROL2_UHS_MODE_SELECT_VAL_HS400;
		break;
	}

	if (mode >= SDHC_TIMING_SDR12) {
		hc_regs->HOST_CONTROL2 &= ~TI_HOST_CONTROL2_UHS_MODE_SELECT;
		hc_regs->HOST_CONTROL2 |= FIELD_PREP(TI_HOST_CONTROL2_UHS_MODE_SELECT, uhs_mode);
	}

	/* configure timing mode */
	if (data->props.host_caps.bus_8_bit_support) {
		/* for 8bit bus */
		return ti_configure_timing_8_bit(dev, mode, clock);
	} else {
		/* for 4bit bus */
		return ti_configure_timing_4_bit(dev, mode);
	}
}

static void ti_configure_bus_width(const struct device *dev, enum sdhc_bus_width width)
{
	uint8_t host_control1 = DEV_HC_REGS(dev)->HOST_CONTROL1;
	struct ti_data *data = DEV_DATA(dev);

	if (width == SDHC_BUS_WIDTH8BIT) {
		host_control1 &= ~TI_HOST_CONTROL1_DATA_WIDTH;
		host_control1 |= TI_HOST_CONTROL1_EXT_DATA_WIDTH;
	} else {
		if (data->props.host_caps.bus_8_bit_support) {
			host_control1 &= ~TI_HOST_CONTROL1_EXT_DATA_WIDTH;
		}
		if (width == SDHC_BUS_WIDTH4BIT) {
			host_control1 |= TI_HOST_CONTROL1_DATA_WIDTH;
		} else {
			host_control1 &= ~TI_HOST_CONTROL1_DATA_WIDTH;
		}
	}

	DEV_HC_REGS(dev)->HOST_CONTROL1 = host_control1;
}

static int ti_configure_voltage(const struct device *dev, enum sd_voltage voltage)
{
	const struct ti_config *config = DEV_CFG(dev);
	struct ti_hc_regs *hc_regs = DEV_HC_REGS(dev);
	uint8_t power_control = hc_regs->POWER_CONTROL;
	uint16_t host_control2 = hc_regs->HOST_CONTROL2;
	uint32_t uV = 0;
	int rv;

	power_control &= ~TI_POWER_CONTROL_SD_BUS_VOLTAGE;

	switch (voltage) {
	case SD_VOL_1_8_V: {
		uV = 1800000;

		power_control |= FIELD_PREP(TI_POWER_CONTROL_SD_BUS_VOLTAGE,
					    TI_POWER_CONTROL_SD_BUS_VOLTAGE_VAL_V1P8);
		host_control2 |= TI_HOST_CONTROL2_V1P8_SIGNAL_ENA;
		break;
	}
	case SD_VOL_3_0_V: {
		uV = 3000000;

		power_control |= FIELD_PREP(TI_POWER_CONTROL_SD_BUS_VOLTAGE,
					    TI_POWER_CONTROL_SD_BUS_VOLTAGE_VAL_V3P0);
		host_control2 &= ~TI_HOST_CONTROL2_V1P8_SIGNAL_ENA;
		break;
	}
	case SD_VOL_3_3_V: {
		uV = 3300000;

		power_control |= FIELD_PREP(TI_POWER_CONTROL_SD_BUS_VOLTAGE,
					    TI_POWER_CONTROL_SD_BUS_VOLTAGE_VAL_V3P3);
		host_control2 &= ~TI_HOST_CONTROL2_V1P8_SIGNAL_ENA;
		break;
	}
	case SD_VOL_1_2_V: {
		LOG_ERR("1.2V not supported");
		return -ENOTSUP;
	}
	}
	printk("uv = %u\n", uV);

	if (config->vqmmc != NULL) {
		if (regulator_is_supported_voltage(config->vqmmc, uV, uV)) {
			rv = regulator_set_voltage(config->vqmmc, uV, uV);
			if (rv != 0) {
				LOG_ERR("Failed to change regulator voltage");
				return rv;
			}
		}
	}

	hc_regs->POWER_CONTROL = power_control;
	hc_regs->HOST_CONTROL2 = host_control2;

	return 0;
}

static int ti_configure_power(const struct device *dev, enum sdhc_power power_mode)
{
	const struct ti_config *config = DEV_CFG(dev);
	struct ti_hc_regs *hc_regs = DEV_HC_REGS(dev);
	int rv;

	switch (power_mode) {
	case SDHC_POWER_ON: {
		if (config->vmmc != NULL) {
			rv = regulator_enable(config->vmmc);
			if (rv != 0) {
				LOG_ERR("Failed to enable VMMC regulator");
				return rv;
			}
		}

		/* enable power */
		hc_regs->POWER_CONTROL |= TI_POWER_CONTROL_SD_BUS_POWER;

		break;
	}

	case SDHC_POWER_OFF: {
		if (config->vmmc != NULL) {
			rv = regulator_disable(config->vmmc);
			if (rv != 0) {
				LOG_ERR("Failed to disable VMMC regulator");
				return rv;
			}
		}

		/* disable power */
		hc_regs->POWER_CONTROL &= ~TI_POWER_CONTROL_SD_BUS_POWER;
	}
	}

	k_msleep(1000);

	return 0;
}

static int ti_set_io(const struct device *dev, struct sdhc_io *ios)
{
	struct ti_data *data = DEV_DATA(dev);
	int rv;

	if (ios->clock < data->props.f_min || ios->clock > data->props.f_max) {
		LOG_ERR("Invalid clock frequency: %uHz", ios->clock);
		return -EINVAL;
	}

	if (ios->bus_width == SDHC_BUS_WIDTH8BIT &&
	    data->props.host_caps.bus_8_bit_support == false) {
		LOG_ERR("Bus width not supported");
		return -ENOTSUP;
	}

	if (ios->bus_mode == SDHC_BUSMODE_OPENDRAIN) {
		LOG_ERR("Open drain is not supported");
		return -ENOTSUP;
	}

	/* configure bus width */
	if (ios->bus_width != data->ios.bus_width) {
		ti_configure_bus_width(dev, ios->bus_width);
	}

	/* configure voltage */
	if (ios->signal_voltage != data->ios.signal_voltage) {
		rv = ti_configure_voltage(dev, ios->signal_voltage);
		if (rv != 0) {
			return rv;
		}
	}

	/* configure timing */
	if (ios->timing != data->ios.timing || ios->clock != data->ios.clock) {
		rv = ti_configure_timing(dev, ios->timing, ios->clock);
		if (rv != 0) {
			return rv;
		}
	}

	/* set clock */
	if (ios->clock != data->ios.clock || ios->clock == 0) {
		rv = ti_configure_clock(dev, ios->clock, ios->power_mode == SDHC_POWER_ON);
		if (rv != 0) {
			return rv;
		}
	}

	/* configure power */
	if (ios->power_mode != data->ios.power_mode) {
		rv = ti_configure_power(dev, ios->power_mode);
		if (rv != 0) {
			return rv;
		}
	}

	/* save */
	data->ios = *ios;

	return 0;
}

static int ti_execute_tuning(const struct device *dev)
{
	const struct ti_config *config = DEV_CFG(dev);
	struct ti_data *data = DEV_DATA(dev);
	enum sdhc_timing_mode timing = data->ios.timing;
	struct ti_tap_delay_config delay_config = config->delay_config[timing];

	switch (timing) {
	case SDHC_TIMING_SDR104:
	case SDHC_TIMING_HS200:
		break;
	case SDHC_TIMING_SDR50:
		if (data->props.host_caps.sdr50_needs_tuning) {
			break;
		}
	case SDHC_TIMING_LEGACY:
	case SDHC_TIMING_HS:
	case SDHC_TIMING_SDR12:
	case SDHC_TIMING_SDR25:
	case SDHC_TIMING_DDR50:
	case SDHC_TIMING_DDR52:
	case SDHC_TIMING_HS400:
	default:
		LOG_ERR("invalid timing mode for tuning");
		return -ENOTSUP;
		break;
	}

	delay_config.itap_delay_enable = true;

	for (int i = 0; i < TI_PHY_CTRL_4_ITAPDLYSEL_VAL_MAX; i++) {
		delay_config.itap_delay_value = i;

		/* configure itap value */
		ti_configure_tap_delays(dev, &delay_config);
	}

	return 0;
}

static int ti_get_card_present(const struct device *dev)
{
	return !!(DEV_HC_REGS(dev)->PRESENTSTATE & TI_PRESENTSTATE_CARD_INSERTED);
}

static int ti_card_busy(const struct device *dev)
{
	return !!(DEV_HC_REGS(dev)->PRESENTSTATE &
		  (TI_PRESENTSTATE_INHIBIT_DAT | TI_PRESENTSTATE_INHIBIT_CMD));
}

static int ti_enable_interrupt(const struct device *dev, sdhc_interrupt_cb_t callback, int sources,
			       void *user_data)
{
	struct ti_hc_regs *hc_regs = DEV_HC_REGS(dev);
	struct ti_data *data = DEV_DATA(dev);

	data->callback = callback;
	data->user_data = user_data;

	if (sources & SDHC_INT_SDIO) {
		return -ENOTSUP;
	}

	if (sources & SDHC_INT_INSERTED) {
		hc_regs->NORMAL_INTR_SIG_ENA |= TI_NORMAL_INTR_CARD_INSERTION;
		hc_regs->NORMAL_INTR_STS_ENA |= TI_NORMAL_INTR_CARD_INSERTION;
	};

	if (sources & SDHC_INT_REMOVED) {
		hc_regs->NORMAL_INTR_SIG_ENA |= TI_NORMAL_INTR_CARD_REMOVAL;
		hc_regs->NORMAL_INTR_STS_ENA |= TI_NORMAL_INTR_CARD_REMOVAL;
	};

	return 0;
}

static int ti_disable_interrupt(const struct device *dev, int sources)
{
	struct ti_hc_regs *hc_regs = DEV_HC_REGS(dev);

	if (sources & SDHC_INT_SDIO) {
		return -ENOTSUP;
	}

	if (sources & SDHC_INT_INSERTED) {
		hc_regs->NORMAL_INTR_SIG_ENA &= ~TI_NORMAL_INTR_CARD_INSERTION;
		hc_regs->NORMAL_INTR_STS_ENA &= ~TI_NORMAL_INTR_CARD_INSERTION;
	};

	if (sources & SDHC_INT_REMOVED) {
		hc_regs->NORMAL_INTR_SIG_ENA &= ~TI_NORMAL_INTR_CARD_REMOVAL;
		hc_regs->NORMAL_INTR_STS_ENA &= ~TI_NORMAL_INTR_CARD_REMOVAL;
	};

	return 0;
}

static void ti_init_host_props(const struct device *dev)
{
	struct ti_hc_regs *hc_regs = DEV_HC_REGS(dev);
	struct ti_data *data = dev->data;
	struct sdhc_host_props *props = &data->props;
	uint64_t max_current_caps = hc_regs->MAX_CURRENT_CAP;
	uint64_t caps = hc_regs->CAPABILITIES;

	/* max current */
	props->max_current_180 = FIELD_GET(TI_MAX_CURRENT_CAP_VDD1_1P8V, max_current_caps) << 2;
	props->max_current_300 = FIELD_GET(TI_MAX_CURRENT_CAP_VDD1_3P0V, max_current_caps) << 2;
	props->max_current_330 = FIELD_GET(TI_MAX_CURRENT_CAP_VDD1_3P3V, max_current_caps) << 2;

	/* copy capabilities to bitfield struct */
	BUILD_ASSERT(sizeof(props->host_caps) == sizeof(caps),
		     "SDHCI host capabilities do not fit the register");
	props->host_caps = *(const struct sdhc_host_caps *)(&caps);

	/* extra capabilities */
	props->hs400_support = !!(caps & TI_CAPABILITIES_BUS_HS400_SUPPORT);
	props->bus_4_bit_support = true;
}

static int ti_get_host_props(const struct device *dev, struct sdhc_host_props *props)
{
	struct ti_data *data = dev->data;

	*props = data->props;

	return 0;
}

static int ti_init(const struct device *dev)
{
	const struct ti_config *config = DEV_CFG(dev);
	struct ti_data *data = DEV_DATA(dev);
	struct ti_hc_regs *hc_regs;
	struct ti_ss_regs *ss_regs;
	uint16_t normal_intr;
	uint16_t error_intr;
	uint32_t ctl_cfg_2;
	int rv;

	DEVICE_MMIO_NAMED_MAP(dev, host, K_MEM_CACHE_NONE);
	DEVICE_MMIO_NAMED_MAP(dev, subsys, K_MEM_CACHE_NONE);

	hc_regs = DEV_HC_REGS(dev);
	ss_regs = DEV_SS_REGS(dev);

	config->irq_func(dev);
	k_event_init(&data->irq_event);

	rv = ti_reset(dev);
	if (rv != 0) {
		LOG_ERR("failed to reset the controller");
		return rv;
	}

	ss_regs->PHY_CTRL_1 |= 1;
	k_msleep(100);
	printk("stat = %x\n", ss_regs->PHY_STAT_1);

	ti_init_host_props(dev);

	/* set slot type */
	ctl_cfg_2 = ss_regs->CTL_CFG_2;
	ctl_cfg_2 &= ~TI_CTL_CFG_2_SLOTTYPE;
	ctl_cfg_2 |= FIELD_PREP(TI_CTL_CFG_2_SLOTTYPE, data->props.host_caps.slot_type);
	ss_regs->CTL_CFG_2 = ctl_cfg_2;

	/* enable interrupts */
	normal_intr = TI_NORMAL_INTR_CMD_COMPLETE | TI_NORMAL_INTR_XFER_COMPLETE |
		      TI_NORMAL_INTR_BUF_RD_READY | TI_NORMAL_INTR_BUF_WR_READY;
	error_intr = TI_ERROR_INTR_ALL;

	hc_regs->NORMAL_INTR_SIG_ENA |= normal_intr;
	hc_regs->NORMAL_INTR_STS_ENA |= normal_intr;
	hc_regs->ERROR_INTR_SIG_ENA |= error_intr;
	hc_regs->ERROR_INTR_STS_ENA |= error_intr;

	printk("presentstate0 = %x\n", hc_regs->PRESENTSTATE);
	return 0;
}

static void ti_isr(const struct device *dev)
{
	struct ti_hc_regs *hc_regs = DEV_HC_REGS(dev);
	struct ti_data *data = DEV_DATA(dev);

	uint16_t nstatus = hc_regs->NORMAL_INTR_STS;
	uint16_t estatus = hc_regs->ERROR_INTR_STS;
	printk("interrupt = 0x%x 0x%x\n", nstatus, estatus);

	if (nstatus & TI_NORMAL_INTR_CMD_COMPLETE) {
		hc_regs->NORMAL_INTR_STS = TI_NORMAL_INTR_CMD_COMPLETE;
		k_event_post(&data->irq_event, TI_NORMAL_INTR_CMD_COMPLETE);
	}

	if (nstatus & TI_NORMAL_INTR_XFER_COMPLETE) {
		hc_regs->NORMAL_INTR_STS = TI_NORMAL_INTR_XFER_COMPLETE;
		k_event_post(&data->irq_event, TI_NORMAL_INTR_XFER_COMPLETE);
	}

	if (nstatus & TI_NORMAL_INTR_BUF_WR_READY) {
		hc_regs->NORMAL_INTR_STS = TI_NORMAL_INTR_BUF_WR_READY;
		k_event_post(&data->irq_event, TI_NORMAL_INTR_BUF_WR_READY);
	}

	if (nstatus & TI_NORMAL_INTR_BUF_RD_READY) {
		hc_regs->NORMAL_INTR_STS = TI_NORMAL_INTR_BUF_RD_READY;
		k_event_post(&data->irq_event, TI_NORMAL_INTR_BUF_RD_READY);
	}

	if (nstatus & TI_NORMAL_INTR_CARD_INSERTION) {
		hc_regs->NORMAL_INTR_STS |= TI_NORMAL_INTR_CARD_INSERTION;

		if (ti_get_card_present(dev)) {
			data->callback(dev, SDHC_INT_INSERTED, data->user_data);
		}
	}

	if (nstatus & TI_NORMAL_INTR_CARD_REMOVAL) {
		hc_regs->NORMAL_INTR_STS |= TI_NORMAL_INTR_CARD_REMOVAL;

		if (!ti_get_card_present(dev)) {
			data->callback(dev, SDHC_INT_REMOVED, data->user_data);
		}
	}

	if (estatus) {
		hc_regs->ERROR_INTR_STS |= estatus;
		k_event_post(&data->irq_event, TI_IRQ_EVENT_ERROR_INTR(estatus));
	}
}

static DEVICE_API(sdhc, ti_api) = {
	.reset = ti_reset,
	.request = ti_request,
	.set_io = ti_set_io,
	.enable_interrupt = ti_enable_interrupt,
	.disable_interrupt = ti_disable_interrupt,
	.get_card_present = ti_get_card_present,
	/*.execute_tuning = ti_execute_tuning, */
	.card_busy = ti_card_busy,
	.get_host_props = ti_get_host_props,
};

#define TI_TIMING_DELAY(n, timing)                                                                 \
	{                                                                                          \
		.itap_delay_enable = DT_INST_NODE_HAS_PROP(n, ti_itap_del_sel_##timing),           \
		.itap_delay_value = DT_INST_PROP_OR(n, ti_itap_del_sel_##timing, 0),               \
		.otap_delay_enable = DT_INST_NODE_HAS_PROP(n, ti_otap_del_sel_##timing),           \
		.otap_delay_value = DT_INST_PROP_OR(n, ti_otap_del_sel_##timing, 0),               \
	}

#define TI_TIMING_DELAY_LIST(n)                                                                    \
	{                                                                                          \
		[SDHC_TIMING_LEGACY] = TI_TIMING_DELAY(n, legacy),                                 \
		[SDHC_TIMING_HS] = TI_TIMING_DELAY(n, hs),                                         \
		[SDHC_TIMING_SDR12] = TI_TIMING_DELAY(n, sdr12),                                   \
		[SDHC_TIMING_SDR25] = TI_TIMING_DELAY(n, sdr25),                                   \
		[SDHC_TIMING_SDR50] = TI_TIMING_DELAY(n, sdr50),                                   \
		[SDHC_TIMING_SDR104] = TI_TIMING_DELAY(n, sdr104),                                 \
		[SDHC_TIMING_DDR50] = TI_TIMING_DELAY(n, ddr50),                                   \
		[SDHC_TIMING_DDR52] = TI_TIMING_DELAY(n, ddr52),                                   \
		[SDHC_TIMING_HS200] = TI_TIMING_DELAY(n, hs200),                                   \
		[SDHC_TIMING_HS400] = TI_TIMING_DELAY(n, hs400),                                   \
	}

#define TI_INIT(n)                                                                                 \
	static void ti_##n##_irq_func(const struct device *dev)                                    \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), ti_isr,                     \
			    DEVICE_DT_INST_GET(n), DT_INST_IRQ(0, flags));                         \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}                                                                                          \
                                                                                                   \
	static const struct ti_config ti_##n##_config = {                                          \
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(host, DT_DRV_INST(n)),                          \
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(subsys, DT_DRV_INST(n)),                        \
		.irq_func = ti_##n##_irq_func,                                                     \
		.delay_config = TI_TIMING_DELAY_LIST(n),                                           \
		.clkbuf_sel = DT_INST_PROP(n, ti_clkbuf_sel),                                      \
		.strobe_sel = DT_INST_PROP_OR(n, ti_strobe_sel, 0),                                \
		.strobe_sel_4_bit = DT_INST_PROP(n, ti_strobe_sel_4_bit),                          \
		.freq_sel_2_bit = DT_INST_PROP(n, ti_freq_sel_2_bit),                              \
		.drive_impedance = DT_INST_PROP_OR(n, ti_driver_strength_ohm, 0),                  \
		.current_trim = DT_INST_PROP_OR(n, ti_trm_icp, 0),                                 \
		.vmmc = DEVICE_DT_GET_OR_NULL(DT_PHANDLE(DT_DRV_INST(n), vmmc_supply)),            \
		.vqmmc = DEVICE_DT_GET_OR_NULL(DT_PHANDLE(DT_DRV_INST(n), vqmmc_supply)),          \
	};                                                                                         \
                                                                                                   \
	static struct ti_data ti_##n##_data = {                                                    \
		.props =                                                                           \
			{                                                                          \
				.f_min = DT_INST_PROP(n, min_bus_freq),                            \
				.f_max = DT_INST_PROP(n, max_bus_freq),                            \
				.power_delay = DT_INST_PROP(n, power_delay_ms),                    \
				.hs200_support = DT_INST_PROP(n, mmc_hs200_1_8v),                  \
			},                                                                         \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, &ti_init, NULL, &ti_##n##_data, &ti_##n##_config, POST_KERNEL,    \
			      CONFIG_SDHC_INIT_PRIORITY, &ti_api);

DT_INST_FOREACH_STATUS_OKAY(TI_INIT)

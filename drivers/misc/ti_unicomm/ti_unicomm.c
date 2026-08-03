/*
 * Copyright (c) 2026 Texas Instruments Incorporated
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_unicomm

#include <stdint.h>
#include <errno.h>
#include <zephyr/device.h>

#define RSTCTL_KEY_UNLOCK       0xB1000000U
#define RSTCTL_STICKY_BIT_CLEAR 0x00000002U
#define RSTCTL_ASSERT_RESET     0x00000001U

#define PWREN_ENABLE  0x00000001U
#define PWREN_DISABLE 0x00000000U
#define PWREN_KEY     0x26000000U

typedef struct {
	uint32_t RESERVED0[512];
	volatile uint32_t PWREN;
	volatile uint32_t RSTCTL;
	uint32_t RESERVED1[2];
	volatile uint32_t STAT;
	uint32_t RESERVED2[570];
	volatile uint32_t IPMODE;
} UNICOMM_Regs_t;

enum IPMode {
	IPMODE_UART,
	IPMODE_SPI,
	IPMODE_I2CC,
	IPMODE_I2CT,
};

struct ti_unicomm_config {
	void *inst_base;
};

struct ti_unicomm_data {
	enum IPMode ip_mode;
};

static int ti_unicomm_init(const struct device *dev)
{
	const struct ti_unicomm_config *cfg = dev->config;

	volatile UNICOMM_Regs_t *unicomm = (UNICOMM_Regs_t *)cfg->inst_base;

	/* Reset and enable power */
	unicomm->RSTCTL = RSTCTL_KEY_UNLOCK | RSTCTL_STICKY_BIT_CLEAR | RSTCTL_ASSERT_RESET;
	unicomm->PWREN = PWREN_KEY | PWREN_ENABLE;

	return 0;
}

#define TI_UNICOMM_INIT(idx)                                                                       \
	BUILD_ASSERT(DT_INST_CHILD_NUM_STATUS_OKAY(idx) == 1,                                      \
		     "UNICOMM node should have one active child!");                                \
                                                                                                   \
	static const struct ti_unicomm_config ti_unicomm_cfg_##idx = {                             \
		.inst_base = (void *)DT_INST_REG_ADDR(idx)};                                       \
                                                                                                   \
	static struct ti_unicomm_data ti_unicomm_data_##idx = {.ip_mode = IPMODE_UART};            \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(idx, &ti_unicomm_init, NULL, &ti_unicomm_data_##idx,                 \
			      &ti_unicomm_cfg_##idx, PRE_KERNEL_1, 0, NULL);

DT_INST_FOREACH_STATUS_OKAY(TI_UNICOMM_INIT)

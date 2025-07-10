/*
 * Copyright (c) 2025 Texas Instruments
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/arch/common/sys_io.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/drivers/misc/ti_k3_mmr/ti_k3_mmr.h>

#define DT_DRV_COMPAT ti_k3_mmr

#define TI_K3_MMR_PARTITION_SIZE         (0x4000)
#define TI_K3_MMR_PARTITION_OFFSET(part) (part * TI_K3_MMR_PARTITION_SIZE)

#define TI_K3_MMR_KICK0_OFFSET (0x1008)
#define TI_K3_MMR_KICK1_OFFSET (0x100C)

#define TI_K3_MMR_KICK0_LOCK_VAL   (0x0U)
#define TI_K3_MMR_KICK0_UNLOCK_VAL (0x68EF3490U)
#define TI_K3_MMR_KICK1_LOCK_VAL   (0x0U)
#define TI_K3_MMR_KICK1_UNLOCK_VAL (0xD172BC5AU)

struct ti_k3_mmr_config {
	DEVICE_MMIO_ROM;
	uint32_t size;
};

struct ti_k3_mmr_data {
	DEVICE_MMIO_RAM;
};

int z_impl_ti_k3_mmr_lock(const struct device *dev, uint8_t partition)
{

	const struct ti_k3_mmr_config *cfg = dev->config;
	uint32_t addr = DEVICE_MMIO_GET(dev);
	uint32_t part_off = TI_K3_MMR_PARTITION_OFFSET(partition);

	if (part_off + TI_K3_MMR_KICK1_OFFSET >= cfg->size) {
		return -ENOMEM;
	}

	addr += part_off;

	sys_write32(TI_K3_MMR_KICK0_LOCK_VAL, addr + TI_K3_MMR_KICK0_OFFSET);
	if (TI_K3_MMR_KICK0_LOCK_VAL != sys_read32(addr + TI_K3_MMR_KICK0_OFFSET)) {
		return -EIO;
	}

	sys_write32(TI_K3_MMR_KICK1_LOCK_VAL, addr + TI_K3_MMR_KICK1_OFFSET);
	if (TI_K3_MMR_KICK1_LOCK_VAL != sys_read32(addr + TI_K3_MMR_KICK1_OFFSET)) {
		return -EIO;
	}

	return 0;
}

int z_impl_ti_k3_mmr_unlock(const struct device *dev, uint8_t partition)
{
	const struct ti_k3_mmr_config *cfg = dev->config;
	uint32_t addr = DEVICE_MMIO_GET(dev);
	uint32_t part_off = TI_K3_MMR_PARTITION_OFFSET(partition);

	if (part_off + TI_K3_MMR_KICK1_OFFSET >= cfg->size) {
		return -ENOMEM;
	}

	addr += part_off;

	sys_write32(TI_K3_MMR_KICK0_UNLOCK_VAL, addr + TI_K3_MMR_KICK0_OFFSET);
	if (TI_K3_MMR_KICK0_UNLOCK_VAL != sys_read32(addr + TI_K3_MMR_KICK0_OFFSET)) {
		return -EIO;
	}

	sys_write32(TI_K3_MMR_KICK1_UNLOCK_VAL, addr + TI_K3_MMR_KICK1_OFFSET);
	if (TI_K3_MMR_KICK1_UNLOCK_VAL != sys_read32(addr + TI_K3_MMR_KICK1_OFFSET)) {
		return -EIO;
	}

	return 0;
}

static int ti_k3_mmr_init(const struct device *dev)
{
	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	return 0;
}

#ifdef CONFIG_USERSPACE
#include <zephyr/syscall_handler.h>

static inline int z_vrfy_ti_k3_mmr_lock(const struct device *dev, uint8_t partition)
{
	return z_impl_ti_k3_mmr_lock(dev, partition);
}
#include <zephyr/syscalls/ti_k3_mmr_lock_mrsh.c>

static inline int z_vrfy_ti_k3_mmr_unlock(const struct device *dev, uint8_t partition)
{
	return z_impl_ti_k3_mmr_unlock(dev, partition);
}
#include <zephyr/syscalls/ti_k3_mmr_unlock_mrsh.c>
#endif /* CONFIG_USERSPACE */

#define TI_K3_MMR_INIT(n)                                                                          \
	static const struct ti_k3_mmr_config ti_k3_mmr_config_##n = {                              \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(n)),                                              \
		.size = DT_INST_REG_SIZE(n),                                                       \
	};                                                                                         \
	static struct ti_k3_mmr_data ti_k3_mmr_data_##n;                                           \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, ti_k3_mmr_init, NULL, &ti_k3_mmr_data_##n, &ti_k3_mmr_config_##n, \
			      PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, NULL);

DT_INST_FOREACH_STATUS_OKAY(TI_K3_MMR_INIT);

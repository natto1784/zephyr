# SPDX-License-Identifier: Apache-2.0

zephyr_syscall_header(${ZEPHYR_BASE}/include/zephyr/drivers/misc/ti_k3_mmr/ti_k3_mmr.h)

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_TI_K3_MMR ti_k3_mmr.c)


/*
 * Copyright (c) 2026 Texas Instruments Incorporated
 * Copyright (c) 2026 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/comparator.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(comparator_sample, LOG_LEVEL_INF);

static const struct device *comp_dev = DEVICE_DT_GET(DT_ALIAS(sample_comp));

static void comparator_callback(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	LOG_INF("comparator output changed, output = %d", comparator_get_output(dev));
}

int main(void)
{
	int ret;

	if (!device_is_ready(comp_dev)) {
		LOG_ERR("comparator device not ready");
		return 0;
	}

	ret = comparator_set_trigger_callback(comp_dev, comparator_callback, NULL);
	if (ret < 0) {
		LOG_ERR("failed to set trigger callback (%d)", ret);
		return 0;
	}

	ret = comparator_set_trigger(comp_dev, COMPARATOR_TRIGGER_BOTH_EDGES);
	if (ret < 0) {
		LOG_ERR("failed to set trigger (%d)", ret);
		return 0;
	}

	LOG_INF("comparator sample started, waiting for trigger events");

	while (1) {
		k_sleep(K_FOREVER);
	}

	return 0;
}

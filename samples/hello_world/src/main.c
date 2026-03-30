/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/devicetree.h"
#include "zephyr/drivers/pwm.h"
#include "zephyr/irq.h"
#include "zephyr/sys_clock.h"
#include <stdio.h>
#include <zephyr/kernel.h>

#define EPWM DT_NODELABEL(main_epwm0)
#define ADDR DEVICE_MMIO_GET(DEVICE_DT_GET(EPWM))
#define ECAP DT_NODELABEL(main_ecap0)
uint64_t timeout;

#define OUTPUT (1000U)

int a = 0;
void isr()
{
	if (sys_read16(ADDR + 0x36) & 1) {
		if (++a % OUTPUT == 0) {
			printf("well %llu\n", k_uptime_get() - timeout);
		}
		uint16_t etclr = sys_read16(ADDR + 0x38);
		etclr |= BIT(0);
		sys_write16(etclr, ADDR + 0x38);
	}
}

int main(void)
{
	printf("Hello World! %s\n", CONFIG_BOARD_TARGET);
	IRQ_CONNECT(DT_IRQN(EPWM), DT_IRQ(EPWM, priority), isr, NULL, DT_IRQ(EPWM, flags));
	irq_enable(DT_IRQN(EPWM));

	uint16_t etsel = sys_read16(ADDR + 0x32);
	etsel |= BIT(3);
	etsel |= BIT(0);
	etsel &= ~BIT(1);
	etsel &= ~BIT(2);
	printf("etsel %u\n", etsel);
	sys_write16(etsel, ADDR + 0x32);

	uint16_t etps = sys_read16(ADDR + 0x34);
	etps |= BIT(0);
	etps &= ~BIT(1);
	sys_write16(etps, ADDR + 0x34);
	printf("etps %u\n", etps);
	const struct device *dev = DEVICE_DT_GET(EPWM);
	uint64_t freq;

	if (pwm_get_cycles_per_sec(dev, 0, &freq) < 0) {
		printf("failed to get freq");
		return -1;
	};

	timeout = k_uptime_get();
	if (pwm_set(dev, 0, NSEC_PER_SEC / OUTPUT, ((NSEC_PER_SEC/100) * 15) / (OUTPUT ),
		    PWM_POLARITY_NORMAL) < 0) {
		printf("failed to set cycles");
		return -1;
	}
	uint64_t period = 0, pulse = 0;
	k_timeout_t timeout = K_MSEC(2 * OUTPUT);

	int www = pwm_capture_usec(DEVICE_DT_GET(ECAP), 0,
				     PWM_CAPTURE_TYPE_BOTH | PWM_CAPTURE_MODE_SINGLE |
					     PWM_POLARITY_NORMAL,
				     &period, &pulse, timeout);
	printf("%llu %llu %d\n", period, pulse, www);
	printf("success\n");

	return 0;
}

/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 * Copyright (c) 2025 Texas Instruments Incorporated
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * EQEP Capture Sample Application
 *
 * This example demonstrates Enhanced Quadrature Encoder Pulse (EQEP) functionality
 * using Zephyr's counter subsystem. It shows:
 * 1. Position counting and direction detection
 * 2. Frequency calculation
 * 3. Counter interrupt handling
 * 4. Basic EQEP counter operations
 *
 * Based on TI MCU+ SDK EQEP capture example, adapted for Zephyr.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/counter/ti_am3352_eqep.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(eqep_capture, LOG_LEVEL_INF);

/* EQEP device configuration */
#define EQEP_NODE DT_NODELABEL(main_eqep0)
#define GPIO_NODE DT_NODELABEL(main_gpio0_0)

/* Sample configuration */
#define EQEP_EVENT_FREQ           10
#define EQEP_TEST_FREQ            500
#define EQEP_POSITION_RESET_VALUE 0
#define EQEP_MAX_POSITION         0xFFFFFFFF

#ifdef CONFIG_SOC_AM62L3_A53
#define EQEP_A_GPIO_PIN (18)
#define EQEP_B_GPIO_PIN (19)
#define EQEP_S_GPIO_PIN (20)
#define EQEP_I_GPIO_PIN (21)
#endif

/* Global variables */
static const struct device *eqep_dev = DEVICE_DT_GET(EQEP_NODE);
static const struct device *gpio_dev = DEVICE_DT_GET(GPIO_NODE);
struct k_sem sem;
static volatile uint32_t interrupt_count = 0;
uint32_t position_count[EQEP_EVENT_FREQ];

/* Counter alar callback for periodic position reading */
static void alarm_callback(const struct device *dev, uint8_t chan_id, uint32_t ticks,
			   void *user_data)
{
  printk("callback %u\n", interrupt_count);
	if (interrupt_count < EQEP_EVENT_FREQ) {
		position_count[interrupt_count] = ticks;
		counter_set_channel_alarm(eqep_dev, chan_id, user_data);
	}
	if (interrupt_count == (EQEP_EVENT_FREQ - 1)) {
		k_sem_give(&sem);
		counter_cancel_channel_alarm(eqep_dev, chan_id);
	}
	interrupt_count++;
}

static int test_eqep_basic_operations(void)
{
	uint32_t top_value;
	uint32_t frequency;

	printk("\n=== EQEP Basic Operations Test ===\n");

	/* Test counter device readiness */
	if (!device_is_ready(eqep_dev)) {
		printk("ERROR: EQEP device not ready\n");
		return -ENODEV;
	}

	/* Get counter frequency */
	frequency = counter_get_frequency(eqep_dev);
	printk("EQEP Counter frequency: %u Hz\n", frequency);

	/* Get top value (maximum count) */
	top_value = counter_get_top_value(eqep_dev);
	printk("EQEP Counter top value: %u\n", top_value);

	return 0;
}

enum ti_eqep_direction {
	TI_EQEP_DIRECTION_CLOCKWISE,
	TI_EQEP_DIRECTION_COUNTER_CLOCKWISE
};

void ti_eqep_generate_pattern(enum ti_eqep_direction dir, bool index)
{
	uint32_t simulated_delay = ((1000 * 1000) / EQEP_TEST_FREQ) / 4;
	gpio_pin_configure(gpio_dev, EQEP_A_GPIO_PIN, GPIO_OUTPUT);
	gpio_pin_configure(gpio_dev, EQEP_B_GPIO_PIN, GPIO_OUTPUT);
	gpio_pin_configure(gpio_dev, EQEP_I_GPIO_PIN, GPIO_OUTPUT);
	gpio_pin_configure(gpio_dev, EQEP_S_GPIO_PIN, GPIO_OUTPUT);
  uint32_t t = k_uptime_get();

	for (int i = 0; i < EQEP_EVENT_FREQ; i++) {
		for (int j = 0; j < (EQEP_TEST_FREQ / EQEP_EVENT_FREQ); j++) {
			if (dir == TI_EQEP_DIRECTION_CLOCKWISE) {
				gpio_pin_set(gpio_dev, EQEP_A_GPIO_PIN, 0);
				gpio_pin_set(gpio_dev, EQEP_B_GPIO_PIN, 1);
				k_busy_wait(simulated_delay);

				gpio_pin_set(gpio_dev, EQEP_A_GPIO_PIN, 0);
				gpio_pin_set(gpio_dev, EQEP_B_GPIO_PIN, 0);
				k_busy_wait(simulated_delay);

				gpio_pin_set(gpio_dev, EQEP_A_GPIO_PIN, 1);
				gpio_pin_set(gpio_dev, EQEP_B_GPIO_PIN, 0);
				k_busy_wait(simulated_delay);

				gpio_pin_set(gpio_dev, EQEP_A_GPIO_PIN, 1);
				gpio_pin_set(gpio_dev, EQEP_B_GPIO_PIN, 1);
				k_busy_wait(simulated_delay);
			} else {
				gpio_pin_set(gpio_dev, EQEP_A_GPIO_PIN, 0);
				gpio_pin_set(gpio_dev, EQEP_B_GPIO_PIN, 1);
				k_busy_wait(simulated_delay);

				gpio_pin_set(gpio_dev, EQEP_A_GPIO_PIN, 1);
				gpio_pin_set(gpio_dev, EQEP_B_GPIO_PIN, 1);
				k_busy_wait(simulated_delay);

				gpio_pin_set(gpio_dev, EQEP_A_GPIO_PIN, 1);
				gpio_pin_set(gpio_dev, EQEP_B_GPIO_PIN, 0);
				k_busy_wait(simulated_delay);

				gpio_pin_set(gpio_dev, EQEP_A_GPIO_PIN, 0);
				gpio_pin_set(gpio_dev, EQEP_B_GPIO_PIN, 0);
				k_busy_wait(simulated_delay);
			}
		}
    uint32_t ticks;
	counter_get_value(eqep_dev, &ticks);
  printk("loop %d = %u, time = %lld\n", i, ticks, k_uptime_get() - t);
  t = k_uptime_get();
		if (index) {
			gpio_pin_set(gpio_dev, EQEP_I_GPIO_PIN, 1);
			k_usleep(simulated_delay);
			gpio_pin_set(gpio_dev, EQEP_I_GPIO_PIN, 0);
		}
	}
}

int ti_eqep_init_quadrature_wave(void)
{
	struct ti_eqep_dec_cfg dec_cfg = {
		.source = TI_EQEP_SRC_QUADRATURE,
		.swap_inputs = false,
		.rising_edge_only = true,
	};

	ti_eqep_configure_decoder(eqep_dev, &dec_cfg);

	struct ti_eqep_qep_cfg qep_cfg = {
		.reset_mode = TI_EQEP_RESET_MODE_INDEX,
		.index_latch = TI_EQEP_INDEX_LATCH_RISING,
	};

	ti_eqep_configure_qep(eqep_dev, &qep_cfg);

	uint32_t ticks;
	int rv = counter_start(eqep_dev);

	rv += counter_get_value(eqep_dev, &ticks);
	printk("initial = %u\n", ticks);

	return rv;
}

int ti_eqep_init_freq_calc(void)
{
	struct ti_eqep_dec_cfg dec_cfg = {
		.source = TI_EQEP_SRC_UP,
		.swap_inputs = false,
		.rising_edge_only = false,
	};

	ti_eqep_configure_decoder(eqep_dev, &dec_cfg);

	struct ti_eqep_qep_cfg qep_cfg = {
		.reset_mode = TI_EQEP_RESET_MODE_UNIT_TIME,
    .capture_latch = TI_EQEP_CAPTURE_LATCH_TIMEOUT
	};

	ti_eqep_configure_qep(eqep_dev, &qep_cfg);

	uint32_t ticks;
	int rv = counter_start(eqep_dev);

  struct ti_eqep_cap_cfg cap_cfg = {
    .clock_prescaler = TI_EQEP_CAP_CLK_DIV_128,
    .unit_position_prescaler = TI_EQEP_UNIT_POS_DIV_8,
    .enable = true
  };
	ti_eqep_configure_capture(eqep_dev, &cap_cfg);

	rv += counter_get_value(eqep_dev, &ticks);
	printk("initial = %u\n", ticks);

	return rv;
}

void print_count()
{
	for (int i = 0; i < EQEP_EVENT_FREQ; i++) {
		printk("captured %d = %d\n", i, position_count[i]);
	}
}

uint32_t calculate_freq() {
  uint32_t freq = 0;
  for(int i = 1; i <EQEP_EVENT_FREQ; i++) {
    freq += position_count[i] * 25;
  }
  freq /= 9;
  return freq;
}

int main(void)
{
	int ret;
	k_sem_init(&sem, 0, 1);

	printk("EQEP Capture Sample Application Started\n");
	printk("Board: %s\n", CONFIG_BOARD_TARGET);
	printk("EQEP device: %s\n", eqep_dev->name);

	/* Run test sequence */
	ret = test_eqep_basic_operations();
	if (ret != 0) {
		printk("Basic operations test failed: %d\n", ret);
		return ret;
	}

	uint32_t ticks;

	struct counter_alarm_cfg idx_cfg = {.callback = alarm_callback};
	struct counter_alarm_cfg tout_cfg = {
		.callback = alarm_callback,
		.ticks = counter_get_frequency(eqep_dev) / 50,
	};
  tout_cfg.user_data = &tout_cfg;
	idx_cfg.user_data = &idx_cfg;
	ret = counter_set_channel_alarm(eqep_dev, TI_EQEP_ALARM_CHAN_INDEX, &idx_cfg);
	if (ret != 0) {
		printk("ERROR: failed to set alarm for index");
		return ret;
	}

	interrupt_count = 0;
	ret = ti_eqep_init_quadrature_wave();
	if (ret != 0) {
		printk("ERROR: failed to init quadrature wave");
		return ret;
	}
	ti_eqep_generate_pattern(TI_EQEP_DIRECTION_CLOCKWISE, true);
	k_sem_take(&sem, K_FOREVER);
	print_count();

	ret = counter_set_channel_alarm(eqep_dev, TI_EQEP_ALARM_CHAN_INDEX, &idx_cfg);
	if (ret != 0) {
		printk("ERROR: failed to set alarm for index");
		return ret;
	}
	interrupt_count = 0;
	ret = ti_eqep_init_quadrature_wave();
	if (ret != 0) {
		printk("ERROR: failed to init quadrature wave");
		return ret;
	}
	ti_eqep_generate_pattern(TI_EQEP_DIRECTION_COUNTER_CLOCKWISE, true);
	k_sem_take(&sem, K_FOREVER);
	print_count();

  	ret = counter_set_channel_alarm(eqep_dev, TI_EQEP_ALARM_CHAN_TIMEOUT, &tout_cfg);
	if (ret != 0) {
		printk("ERROR: failed to set alarm for timeout");
		return ret;
	}
	interrupt_count = 0;
	ret = ti_eqep_init_freq_calc();
	ti_eqep_generate_pattern(TI_EQEP_DIRECTION_CLOCKWISE, false);
	k_sem_take(&sem, K_FOREVER);
	print_count();
  printk("freq = %u\n", calculate_freq());


	/* todo init pattern */
	printk("\n=== EQEP Capture Sample Complete ===\n");
	printk("All tests completed successfully!\n");

	return 0;
}

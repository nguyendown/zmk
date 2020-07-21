/*
 * Copyright (c) 2020 Peter Johanson
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <device.h>
#include <drivers/gpio.h>
#include <sys/util.h>

struct ec11_config {
	const char *a_label;
	const u8_t a_pin;
	const u8_t a_flags;

	const char *b_label;
	const u8_t b_pin;
	const u8_t b_flags;

	const u8_t resolution;
};

enum ec11_pin_state {
	EC11_A_PIN_STATE,
	EC11_B_PIN_STATE
};

struct ec11_data {
	struct device *a;
	struct device *b;
	u8_t ab_state;
	s8_t pulses;
	s8_t ticks;
	s8_t delta;

#ifdef CONFIG_EC11_TRIGGER
	struct device *gpio;
	struct gpio_callback a_gpio_cb;
	struct gpio_callback b_gpio_cb;
	struct device *dev;

	sensor_trigger_handler_t handler;
	struct sensor_trigger trigger;

#if defined(CONFIG_EC11_TRIGGER_OWN_THREAD)
	K_THREAD_STACK_MEMBER(thread_stack, CONFIG_EC11_THREAD_STACK_SIZE);
	struct k_sem gpio_sem;
	struct k_thread thread;
#elif defined(CONFIG_EC11_TRIGGER_GLOBAL_THREAD)
	struct k_work work;
#endif

#endif /* CONFIG_EC11_TRIGGER */
};

#ifdef CONFIG_EC11_TRIGGER

int ec11_trigger_set(struct device *dev,
		       const struct sensor_trigger *trig,
		       sensor_trigger_handler_t handler);

int ec11_init_interrupt(struct device *dev);
#endif

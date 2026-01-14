/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "led_indication.h"
#include <dk_buttons_and_leds.h>
#include <zephyr/kernel.h>

/* Work items for LED blinking */
static struct k_work_delayable gradient_led_work;
static struct k_work_delayable forward_led_work;
static struct k_work_delayable attention_led_work;

/* Blink counters */
static int gradient_blink_count = 0;
static int forward_blink_count = 0;

/* Attention state */
static bool attention_active = false;

/* LED assignments */
#define LED_GRADIENT    DK_LED3  /* LED for gradient received */
#define LED_FORWARD     DK_LED2  /* LED for data forwarded */
#define LED_SINK        DK_LED1  /* LED for sink received */
#define LED_BACKPROP    DK_LED1  /* LED 0 for backprop received */

/* Blink parameters */
#define BLINK_COUNT_MAX       6    /* 6 toggles = 3 complete on/off cycles */
#define BLINK_INTERVAL_MS     100  /* 100ms between each toggle */
#define ATTENTION_INTERVAL_MS 30   /* 30ms for fast attention pattern rotation */

/**
 * @brief Work handler for gradient LED blinking
 *
 * Blinks LED_GRADIENT to indicate gradient message received.
 *
 * @param work Pointer to work item (unused)
 */
static void gradient_led_handler(struct k_work *work)
{
    static bool led_state = false;
    
    if (gradient_blink_count < BLINK_COUNT_MAX) {
        if (led_state) {
            dk_set_led_on(LED_GRADIENT);
        } else {
            dk_set_led_off(LED_GRADIENT);
        }
        led_state = !led_state;
        gradient_blink_count++;
        k_work_schedule(&gradient_led_work, K_MSEC(BLINK_INTERVAL_MS));
    } else {
        gradient_blink_count = 0;
        dk_set_led_off(LED_GRADIENT);
    }
}

/**
 * @brief Work handler for forward LED blinking
 *
 * Blinks LED_FORWARD to indicate data is being forwarded.
 *
 * @param work Pointer to work item (unused)
 */
static void forward_led_handler(struct k_work *work)
{
    static bool led_state = false;
    
    if (forward_blink_count < BLINK_COUNT_MAX) {
        if (led_state) {
            dk_set_led_on(LED_FORWARD);
        } else {
            dk_set_led_off(LED_FORWARD);
        }
        led_state = !led_state;
        forward_blink_count++;
        k_work_schedule(&forward_led_work, K_MSEC(BLINK_INTERVAL_MS));
    } else {
        forward_blink_count = 0;
        dk_set_led_off(LED_FORWARD);
    }
}

/**
 * @brief Work handler for attention LED pattern
 *
 * Displays rotating LED pattern during mesh attention state.
 *
 * @param work Pointer to work item (unused)
 */
static void attention_led_handler(struct k_work *work)
{
    static int idx = 0;
    const uint8_t pattern[] = {
        BIT(0) | BIT(1),
        BIT(1) | BIT(2),
        BIT(2) | BIT(3),
        BIT(3) | BIT(0),
    };

    if (attention_active) {
        dk_set_leds(pattern[idx++ % ARRAY_SIZE(pattern)]);
        k_work_reschedule(&attention_led_work, K_MSEC(ATTENTION_INTERVAL_MS));
    } else {
        dk_set_leds(DK_NO_LEDS_MSK);
    }
}

void led_indication_init(void)
{
    k_work_init_delayable(&gradient_led_work, gradient_led_handler);
    k_work_init_delayable(&forward_led_work, forward_led_handler);
    k_work_init_delayable(&attention_led_work, attention_led_handler);
}

void led_indicate_gradient_received(void)
{
    gradient_blink_count = 0;
    k_work_schedule(&gradient_led_work, K_NO_WAIT);
}

void led_indicate_data_forwarded(void)
{
    forward_blink_count = 0;
    k_work_schedule(&forward_led_work, K_NO_WAIT);
}

void led_indicate_sink_received(void)
{
    static bool led_state = false;
    led_state = !led_state;
    dk_set_led(LED_SINK, led_state);
}

void led_indicate_backprop_received(void)
{
    static bool led_state = false;
    led_state = !led_state;
    dk_set_led(LED_BACKPROP, led_state);
}

void led_indicate_attention(bool on)
{
    attention_active = on;
    if (on) {
        k_work_reschedule(&attention_led_work, K_NO_WAIT);
    }
    /* When off, the handler will stop and clear LEDs */
}

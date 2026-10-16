/* SPDX-License-Identifier: GPL-2.0 */
/*
 * gt_hal_esp32.h - ESP32 (ESP-IDF / FreeRTOS) implementation of the gt_hal.h
 * contract. This is the ONLY file you write to run the exact same i2c_slave +
 * gt9xx core on an ESP32: fill in the four pin primitives against your board
 * and the emulator comes up unchanged.
 *
 * Build the core with -DGT_PLATFORM_ESP32. Status: reference port - the pin
 * mechanics below are the standard ESP-IDF way; verify the timing-critical
 * gt_gpio_get2() fast path on your target (see the note there) before trusting
 * it at 400 kHz.
 */
#ifndef _GT_HAL_ESP32_H_
#define _GT_HAL_ESP32_H_

#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/gpio_reg.h"

#include "gt_types.h"

/* A GPIO line is just its pad number on the ESP32. */
struct gt_gpio {
	gpio_num_t num;
};

/* ---- pin I/O ------------------------------------------------------------- */

GT_INLINE u32 gt_gpio_get(const struct gt_gpio *p)
{
	return (u32)gpio_get_level(p->num);
}

/*
 * Atomic two-line sample. Per README section 5 ("the point that will kill
 * this most easily"): gpio_get_level() is an APB function call (~hundreds of
 * ns) - too slow to reliably catch a 2.5 us bit at 400 kHz. Read GPIO_IN_REG
 * ONCE and extract both bits instead. Requires SDA and SCL both on pads 0..31
 * (enforced by the pin map in the .ino - GPIO32+ needs GPIO_IN1_REG instead).
 */
GT_INLINE void gt_gpio_get2(const struct gt_gpio *a, const struct gt_gpio *b,
			    u32 *va, u32 *vb)
{
	uint32_t in = REG_READ(GPIO_IN_REG);
	*va = (in >> a->num) & 1u;
	*vb = (in >> b->num) & 1u;
}

/*
 * Fast register-level output write, mirroring the gt_gpio_get2() fast-read
 * path above. gpio_set_level() is a full ESP-IDF driver call (parameter
 * checks + its own internal critical section) costing on the order of a few
 * hundred ns - fine for init, but it is called from sda_drive_bit() on EVERY
 * bit of the I2C_PH_TX phase (the host reading our touch report, i.e. the
 * exact path that determines touch responsiveness). At 400 kHz a bit is only
 * ~2.5us (low phase ~1.2us) wide, so that overhead - stacked on top of the
 * portENTER_CRITICAL region already held by i2c_slave_run() - can eat enough
 * of the setup-time budget to make the host sample a stale/transitioning bit,
 * which shows up as intermittent read errors -> retries -> perceived lag.
 *
 * ESP32 output pins split across two 32-bit banks: 0-31 via GPIO_OUT_REG,
 * 32-39 via GPIO_OUT1_REG. Each bank has W1TS/W1TC (write-1-to-set/clear)
 * shadow registers so a single-bit change is one atomic register write with
 * no read-modify-write race.
 */
GT_INLINE void gt_gpio_write_fast(u32 pin, u32 level)
{
	if (pin < 32) {
		if (level)
			REG_WRITE(GPIO_OUT_W1TS_REG, 1UL << pin);
		else
			REG_WRITE(GPIO_OUT_W1TC_REG, 1UL << pin);
	} else {
		if (level)
			REG_WRITE(GPIO_OUT1_W1TS_REG, 1UL << (pin - 32));
		else
			REG_WRITE(GPIO_OUT1_W1TC_REG, 1UL << (pin - 32));
	}
}

/*
 * Open-drain lines (SDA/SCL) are configured once as INPUT_OUTPUT_OD in
 * gt_gpio_init_od(); then level 0 pulls low and level 1 floats to the external
 * pull-up. The pad stays readable in this mode.
 */
GT_INLINE void gt_gpio_low(const struct gt_gpio *p)
{
	gt_gpio_write_fast((u32)p->num, 0);
}

GT_INLINE void gt_gpio_release(const struct gt_gpio *p)
{
	gt_gpio_write_fast((u32)p->num, 1);
}

/* Push-pull (INT): drive both rails. Pad is INPUT_OUTPUT so it stays readable. */
GT_INLINE void gt_gpio_set(const struct gt_gpio *p, u32 v)
{
	gt_gpio_write_fast((u32)p->num, v ? 1 : 0);
}

/*
 * Tri-state a push-pull line: unlike gt_gpio_release (which is the open-drain
 * "let the pull-up win"), this genuinely floats the pad by turning it into an
 * input - needed when the host briefly drives INT during reset for I2C-address
 * selection. gt_gpio_init_pp() restores push-pull drive afterwards.
 */
GT_INLINE void gt_gpio_hiz(const struct gt_gpio *p)
{
	gpio_set_direction(p->num, GPIO_MODE_INPUT);
}

/* ---- one-time line setup ------------------------------------------------- */

GT_INLINE void gt_gpio_init_od(const struct gt_gpio *p)
{
	gpio_config_t c = {
		.pin_bit_mask = 1ULL << p->num,
		.mode	      = GPIO_MODE_INPUT_OUTPUT_OD,
		.pull_up_en   = GPIO_PULLUP_DISABLE,	/* bus has external pull-ups */
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type    = GPIO_INTR_DISABLE,
	};
	gpio_config(&c);
	gpio_set_level(p->num, 1);		/* released */
}

GT_INLINE void gt_gpio_init_pp(const struct gt_gpio *p, u32 v)
{
	gpio_config_t c = {
		.pin_bit_mask = 1ULL << p->num,
		.mode	      = GPIO_MODE_INPUT_OUTPUT,	/* push-pull, still readable */
		.pull_up_en   = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type    = GPIO_INTR_DISABLE,
	};
	gpio_config(&c);
	gpio_set_level(p->num, v ? 1 : 0);
}

GT_INLINE void gt_gpio_init_in(const struct gt_gpio *p)
{
	gpio_config_t c = {
		.pin_bit_mask = 1ULL << p->num,
		.mode	      = GPIO_MODE_INPUT,
		.pull_up_en   = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type    = GPIO_INTR_DISABLE,
	};
	gpio_config(&c);
}

/* ---- time source --------------------------------------------------------- */

GT_INLINE u32 gt_now_ms(void)
{
	return (u32)(esp_timer_get_time() / 1000);	/* us -> ms */
}

/* ---- real-time guards ----------------------------------------------------
 *
 * The poll loop should own one core (pin the task with
 * xTaskCreatePinnedToCore(..., APP_CPU_NUM)). GT_IRQ_SAVE/RESTORE wrap each I2C
 * transaction in a critical section so nothing preempts a byte. gt_esp32_mux is
 * defined once in the ESP32 platform .c.
 */
extern portMUX_TYPE gt_esp32_mux;

typedef unsigned int GT_IRQ_FLAGS_T;

#define GT_IRQ_SAVE(flags)	do { (void)(flags); portENTER_CRITICAL(&gt_esp32_mux); } while (0)
#define GT_IRQ_RESTORE(flags)	do { (void)(flags); portEXIT_CRITICAL(&gt_esp32_mux); } while (0)
#define GT_CPU_RELAX()		do { } while (0)
#define GT_WATCHDOG_KICK()	do { } while (0)	/* or esp_task_wdt_reset() */
#define GT_YIELD()		taskYIELD()

#endif /* _GT_HAL_ESP32_H_ */
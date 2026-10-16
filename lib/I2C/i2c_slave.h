/* SPDX-License-Identifier: GPL-2.0 */
/*
 * i2c_slave - a portable software (bit-banged) I2C *slave* engine.
 *
 * WHY THIS EXISTS
 * ---------------
 * Some hosts must be answered by a device whose SoC has no I2C slave silicon
 * (the RK3568 I2C controller is master-only; the ESP32 slave peripheral is too
 * limited for 16-bit-addressed touch traffic). So we bit-bang the slave side of
 * the protocol on plain GPIO.
 *
 * The engine is protocol-agnostic: it knows how to be an I2C slave at one 7-bit
 * address and hands each byte to an upper layer through struct i2c_slave_ops.
 * All pin access goes through gt_hal.h, so the same code runs on any target
 * that provides the HAL (see gt_hal_rk3568.h / gt_hal_esp32.h).
 *
 * REAL-TIME MODEL
 * ---------------
 * i2c_slave_run() tight-polls SCL/SDA and drives an edge-triggered FSM; the
 * caller runs it on a dedicated, high-priority thread pinned to one core. At
 * 400 kHz a bit lasts 2.5 us and the loop iterates in a few hundred ns, so
 * every edge is oversampled. During an active transaction the loop wraps itself
 * in GT_IRQ_SAVE/RESTORE so nothing preempts a byte (see cfg.guard_irq).
 *
 * The struct is public and embeddable so no allocation is needed on an MCU:
 * put one in your device object and call i2c_slave_init() on it.
 */
#ifndef _I2C_SLAVE_H_
#define _I2C_SLAVE_H_

#include "gt_types.h"
#include "gt_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Callbacks run in the poll-thread context (interrupts off, must not sleep).
 * Every handler must be O(1) and must not allocate or block.
 */
struct i2c_slave_ops {
	/* (Repeated) START addressed to us. @is_read = R/W bit (true = read). */
	void (*start)(void *ctx, bool is_read);
	/* Master wrote one byte to us. */
	void (*write_byte)(void *ctx, u8 byte);
	/* Master reads one byte from us; fill *byte (define it even if idle). */
	void (*read_byte)(void *ctx, u8 *byte);
	/* Transaction ended (STOP or de-addressed). */
	void (*stop)(void *ctx);
	/* Bus idle: safe point to publish state from other contexts. May be NULL. */
	void (*idle)(void *ctx);

	/*
	 * Optional gate checked when our address is matched: return false to NACK
	 * it and appear absent on the bus (used to deliberately look "dead" so the
	 * master re-initialises us). NULL means always addressable.
	 */
	bool (*addressable)(void *ctx);
};

/* Runtime counters, for debugging / health checks. */
struct i2c_slave_stats {
	u64 starts;	/* START conditions seen (any address) */
	u64 addr_match;	/* transactions addressed to us */
	u64 addr_miss;	/* transactions to a different address */
	u64 reads;	/* read transactions served */
	u64 writes;	/* write transactions served */
	u64 stops;	/* STOP conditions seen */
	u64 timeouts;	/* bus hangs recovered by timeout */
};

/* Per-byte phase of the transaction. */
enum i2c_slave_phase {
	I2C_PH_IDLE,	/* between transactions */
	I2C_PH_ADDR,	/* clocking in the address byte */
	I2C_PH_RX,	/* master writing data to us */
	I2C_PH_TX,	/* master reading data from us */
};

struct i2c_slave_config {
	struct gt_gpio sda;	/* data line  (open-drain) */
	struct gt_gpio scl;	/* clock line (open-drain, input to us) */
	u8   addr;		/* our 7-bit slave address */
	bool guard_irq;		/* wrap each transaction in GT_IRQ_SAVE/RESTORE */
	const struct i2c_slave_ops *ops;
	void *ctx;
};

struct i2c_slave {
	struct gt_gpio sda;
	struct gt_gpio scl;
	u8   addr;
	bool guard_irq;

	const struct i2c_slave_ops *ops;
	void *ctx;

	volatile bool stop;	/* set by i2c_slave_stop() to end i2c_slave_run() */

	/* --- FSM state (poll-thread private) --- */
	enum i2c_slave_phase phase;
	u32 prev_scl, prev_sda;
	int nbit;		/* data bits moved in the current byte (0..8) */
	bool ack_stage;		/* in the 9th (ACK) clock of this byte */
	u8  shift_in;		/* byte being received */
	u8  shift_out;		/* byte being transmitted */
	bool is_read;		/* current transaction direction */
	bool matched;		/* current address was ours */
	u32  master_ack;	/* master's ACK bit sampled during a read */

	struct i2c_slave_stats stats;
};

/*
 * Initialise @s from @cfg and put the bus lines in the idle (released) state.
 * Returns 0 or a negative errno-style code. Does not start any thread - the
 * caller drives i2c_slave_run().
 */
int i2c_slave_init(struct i2c_slave *s, const struct i2c_slave_config *cfg);

/*
 * Run the poll loop until i2c_slave_stop() is called. Meant to be the body of a
 * dedicated high-priority thread/task pinned to one core.
 */
void i2c_slave_run(struct i2c_slave *s);

/* Ask i2c_slave_run() to return (safe to call from another context). */
void i2c_slave_stop(struct i2c_slave *s);

/* Snapshot the statistics. */
void i2c_slave_get_stats(const struct i2c_slave *s, struct i2c_slave_stats *out);

#ifdef __cplusplus
}
#endif

#endif /* _I2C_SLAVE_H_ */

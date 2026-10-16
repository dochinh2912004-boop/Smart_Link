// SPDX-License-Identifier: GPL-2.0
/*
 * i2c_slave - portable bit-banged I2C slave engine. See i2c_slave.h for the
 * design overview. One edge-triggered state machine, fed by a tight GPIO poll
 * loop, implements the slave side of the protocol.
 *
 * All line handling is open-drain, exactly like real I2C:
 *   - "drive low" = pull the line to 0 (become an output at the latched 0);
 *   - "release"   = let the external pull-up bring it to 1 (become an input);
 *   - "read"      = sample the live level.
 * We never drive a hard 1, so we can never fight the master.
 */

#include "i2c_slave.h"
#include "gt_hal.h"

/*
 * Poll iterations to wait for a bus edge before assuming the master abandoned
 * the transaction and the bus hung mid-byte. A few hundred ns per iteration
 * puts this in the low-millisecond range: far longer than any legitimate
 * 400 kHz bit, short enough to recover promptly.
 */
#define BUS_TIMEOUT_SPINS	200000u

/* ---- line access (open-drain, via the HAL) ------------------------------- */

GT_INLINE u32 read_scl(struct i2c_slave *s)	{ return gt_gpio_get(&s->scl); }
GT_INLINE u32 read_sda(struct i2c_slave *s)	{ return gt_gpio_get(&s->sda); }
GT_INLINE void sda_drive_low(struct i2c_slave *s) { gt_gpio_low(&s->sda); }
GT_INLINE void sda_release(struct i2c_slave *s)	{ gt_gpio_release(&s->sda); }

/* Drive data bit @pos (0 = MSB) of @val onto SDA (open-drain). */
GT_INLINE void sda_drive_bit(struct i2c_slave *s, u8 val, int pos)
{
	if ((val >> (7 - pos)) & 1)
		sda_release(s);		/* logic 1 -> float high */
	else
		sda_drive_low(s);	/* logic 0 -> pull low */
}

/* ---- state machine ------------------------------------------------------- */

static void fsm_start(struct i2c_slave *s)
{
	/* (Repeated) START: begin clocking in a fresh address byte. */
	s->phase = I2C_PH_ADDR;
	s->nbit = 0;
	s->ack_stage = false;
	s->shift_in = 0;
	sda_release(s);
	s->stats.starts++;
}

static void fsm_stop(struct i2c_slave *s)
{
	if (s->phase != I2C_PH_IDLE && s->matched && s->ops->stop)
		s->ops->stop(s->ctx);
	s->phase = I2C_PH_IDLE;
	s->nbit = 0;
	s->ack_stage = false;
	sda_release(s);
	s->stats.stops++;
}

/* Sample point: SCL just went high. */
static void fsm_scl_rising(struct i2c_slave *s, u32 sda)
{
	switch (s->phase) {
	case I2C_PH_ADDR:
	case I2C_PH_RX:
		if (s->nbit < 8) {
			s->shift_in = (s->shift_in << 1) | (u8)sda;
			s->nbit++;
		}
		/* At nbit==8 the ACK bit is handled on falling edges. */
		break;
	case I2C_PH_TX:
		/* The 9th clock of a transmitted byte: read the master's ACK. */
		if (s->nbit == 8 && s->ack_stage)
			s->master_ack = sda;
		break;
	default:
		break;
	}
}

/* Setup point: SCL just went low. Safe to change SDA here. */
static void fsm_scl_falling(struct i2c_slave *s)
{
	switch (s->phase) {
	case I2C_PH_ADDR:
		if (s->nbit != 8)
			break;
		if (!s->ack_stage) {
			/* Byte complete: decide address match and drive ACK. */
			s->is_read = s->shift_in & 1;
			s->matched = ((s->shift_in >> 1) == s->addr);
			/* Upper layer may tell us to play dead (NACK) for now. */
			if (s->matched && s->ops->addressable &&
			    !s->ops->addressable(s->ctx))
				s->matched = false;
			if (s->matched)
				sda_drive_low(s);	/* ACK  */
			else
				sda_release(s);		/* NACK */
			s->ack_stage = true;
		} else {
			/* ACK clock finished. */
			sda_release(s);
			s->ack_stage = false;
			s->nbit = 0;
			if (!s->matched) {
				s->stats.addr_miss++;
				s->phase = I2C_PH_IDLE;	/* ignore until STOP */
				break;
			}
			s->stats.addr_match++;
			if (s->ops->start)
				s->ops->start(s->ctx, s->is_read);
			if (s->is_read) {
				s->stats.reads++;
				s->phase = I2C_PH_TX;
				s->shift_out = 0xff;
				if (s->ops->read_byte)
					s->ops->read_byte(s->ctx, &s->shift_out);
				sda_drive_bit(s, s->shift_out, 0);
				s->nbit = 1;
			} else {
				s->stats.writes++;
				s->phase = I2C_PH_RX;
				s->shift_in = 0;
			}
		}
		break;

	case I2C_PH_RX:
		if (s->nbit != 8)
			break;
		if (!s->ack_stage) {
			sda_drive_low(s);	/* ACK the received byte */
			s->ack_stage = true;
		} else {
			sda_release(s);
			s->ack_stage = false;
			if (s->ops->write_byte)
				s->ops->write_byte(s->ctx, s->shift_in);
			s->nbit = 0;
			s->shift_in = 0;
		}
		break;

	case I2C_PH_TX:
		if (s->nbit < 8) {
			sda_drive_bit(s, s->shift_out, s->nbit);
			s->nbit++;
		} else if (!s->ack_stage) {
			/* All 8 bits clocked out; release SDA for master's ACK. */
			sda_release(s);
			s->ack_stage = true;
		} else {
			/* master_ack was sampled on the preceding rising edge. */
			s->ack_stage = false;
			if (s->master_ack) {
				/* NACK: master is done reading. */
				s->phase = I2C_PH_IDLE;
				sda_release(s);
			} else {
				/* ACK: send the next byte. */
				s->shift_out = 0xff;
				if (s->ops->read_byte)
					s->ops->read_byte(s->ctx, &s->shift_out);
				sda_drive_bit(s, s->shift_out, 0);
				s->nbit = 1;
			}
		}
		break;

	default:
		break;
	}
}

/* Process one (scl, sda) sample. */
GT_INLINE void fsm_step(struct i2c_slave *s, u32 scl, u32 sda)
{
	if (scl && s->prev_scl) {
		/* SCL stable high: SDA transitions are START/STOP. */
		if (s->prev_sda && !sda)
			fsm_start(s);
		else if (!s->prev_sda && sda)
			fsm_stop(s);
	} else if (scl && !s->prev_scl) {
		fsm_scl_rising(s, sda);
	} else if (!scl && s->prev_scl) {
		fsm_scl_falling(s);
	}
	s->prev_scl = scl;
	s->prev_sda = sda;
}

/* ---- public API ---------------------------------------------------------- */

int i2c_slave_init(struct i2c_slave *s, const struct i2c_slave_config *cfg)
{
	if (!s || !cfg || !cfg->ops)
		return -22;	/* -EINVAL */

	memset(s, 0, sizeof(*s));
	s->sda	     = cfg->sda;
	s->scl	     = cfg->scl;
	s->addr	     = cfg->addr;
	s->guard_irq = cfg->guard_irq;
	s->ops	     = cfg->ops;
	s->ctx	     = cfg->ctx;
	s->phase     = I2C_PH_IDLE;

	/* Bring both lines up as released open-drain inputs (idle bus). */
	gt_gpio_init_od(&s->sda);
	gt_gpio_init_od(&s->scl);
	return 0;
}

void i2c_slave_stop(struct i2c_slave *s)
{
	if (s)
		s->stop = true;
}

void i2c_slave_get_stats(const struct i2c_slave *s, struct i2c_slave_stats *out)
{
	if (s && out)
		*out = s->stats;
}

void i2c_slave_run(struct i2c_slave *s)
{
	GT_IRQ_FLAGS_T flags = 0;
	bool irq_disabled = false;
	u32 spins = 0;
	u32 idle_ticks = 0;

	/* Idle bus state. */
	sda_release(s);
	s->prev_scl = read_scl(s);
	s->prev_sda = read_sda(s);
	s->phase = I2C_PH_IDLE;

	while (!s->stop) {
		u32 scl, sda;

		/* Sample SCL and SDA atomically (see gt_gpio_get2). */
		gt_gpio_get2(&s->sda, &s->scl, &sda, &scl);

		/*
		 * Guard the sensitive part of a transaction against preemption
		 * and IRQs: enter the critical region the moment we leave
		 * I2C_PH_IDLE and leave it when we return to idle.
		 */
		if (s->guard_irq) {
			if (s->phase != I2C_PH_IDLE && !irq_disabled) {
				GT_IRQ_SAVE(flags);
				irq_disabled = true;
			} else if (s->phase == I2C_PH_IDLE && irq_disabled) {
				GT_IRQ_RESTORE(flags);
				irq_disabled = false;
			}
		}

		fsm_step(s, scl, sda);

		if (s->phase == I2C_PH_IDLE) {
			/*
			 * Bus idle. The critical region is closed here, so this
			 * is the one safe place to service upper-layer work and,
			 * on a !PREEMPT_RCU kernel, to offer a quiescent state
			 * (a SCHED_FIFO busy loop otherwise trips an rcu stall).
			 */
			spins = 0;
			if (s->ops->idle)
				s->ops->idle(s->ctx);

			if ((++idle_ticks & 0xffff) == 0) {
				GT_WATCHDOG_KICK();
				GT_YIELD();
			}
		} else if (++spins > BUS_TIMEOUT_SPINS) {
			/* Stuck mid-transaction: recover so the bus can't wedge us. */
			s->stats.timeouts++;
			if (s->guard_irq && irq_disabled) {
				GT_IRQ_RESTORE(flags);
				irq_disabled = false;
			}
			fsm_stop(s);
			spins = 0;
		}

		GT_CPU_RELAX();
	}

	if (s->guard_irq && irq_disabled)
		GT_IRQ_RESTORE(flags);
	sda_release(s);
}

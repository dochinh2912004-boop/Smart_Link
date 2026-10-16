// SPDX-License-Identifier: GPL-2.0
/*
 * gt9xx - portable Goodix GT9xx touch-controller emulation. See gt9xx.h.
 *
 * Two independent mechanisms run here, exactly as on the real chip:
 *
 *   1. INT-driven touch reporting. When a touch is staged, we write the 8-byte
 *      report at 0x814E and pull INT low; the host wakes, reads the report and
 *      writes "81 4E 00" to clear the status, at which point we release INT.
 *
 *   2. An ESD watchdog the host services on its own ~2 s timer: it reads 0x8040
 *      (we answer 0xFF) and writes 0xAA back. We only have to ACK it - going
 *      silent makes the host declare the chip dead and reset the whole stack.
 *
 * The INT line is driven push-pull: the host's INT net has no pull-up (measured
 * - a released line reads LOW), so an open-drain "release" would sit low and
 * never make the falling edge the host triggers on. We own the line, so we
 * actively drive both rails: idle = HIGH, asserted = LOW. During a host reset we
 * float it (Hi-Z) so the host can drive it for I2C-address selection.
 */

#include "gt9xx.h"
#include "gt_hal.h"

/*
 * Config table dumped from the real chip (CLAUDE.md section 8), 247 bytes.
 * Checksum verified: all bytes sum to 0 (mod 256). Do NOT regenerate - the
 * host recomputes the checksum and rejects a mismatched table. Decoded here:
 * X_OUTPUT_MAX = 0x0500 = 1280, Y_OUTPUT_MAX = 0x0320 = 800.
 */
static const u8 gt9xx_config[GT9XX_CONFIG_LEN] = {
	0x55, 0x00, 0x05, 0x20, 0x03, 0x0A, 0x0D, 0x00, 0x01, 0x08, 0x28, 0x05, 0x5A, 0x3C, 0x0F, 0x00,
	0x00, 0x00, 0xFB, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x90, 0x30, 0xAA, 0x3C, 0x3E,
	0x39, 0x13, 0x00, 0x00, 0x00, 0x9A, 0x23, 0x2C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00,
	0x00, 0x00, 0x02, 0x28, 0x50, 0x94, 0xC0, 0x02, 0x07, 0x00, 0x00, 0x04, 0x94, 0x2A, 0x00, 0x89,
	0x31, 0x00, 0x82, 0x38, 0x00, 0x7D, 0x41, 0x00, 0x7A, 0x4A, 0x00, 0x7A, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09,
	0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x02, 0x04, 0x06, 0x07, 0x08, 0x0A, 0x0C, 0x0D, 0x0E,
	0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x19, 0x1B, 0x1C, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23, 0x24,
	0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xA3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

/* Product id "928\0" + firmware version 0x1060 (CLAUDE.md section 7). */
static const u8 gt9xx_id[GT9XX_ID_LEN] = { 0x39, 0x32, 0x38, 0x00, 0x60, 0x10 };

/* ---- INT line (push-pull toward the host) -------------------------------- */

static void int_assert(struct gt9xx *g)
{
	gt_gpio_set(&g->int_pin, 0);	/* drive LOW: a fresh report is ready */
	g->int_asserted = true;
}

static void int_release(struct gt9xx *g)
{
	gt_gpio_set(&g->int_pin, 1);	/* drive HIGH: idle */
	g->int_asserted = false;
}

/* ---- register file (poll-thread context) --------------------------------- */

static inline bool in_window(u16 addr)
{
	return addr >= GT9XX_WIN_BASE && addr < GT9XX_WIN_BASE + GT9XX_WIN_SIZE;
}

static inline u8 *reg_slot(struct gt9xx *g, u16 addr)
{
	return &g->regfile[addr - GT9XX_WIN_BASE];
}

static u8 reg_read(struct gt9xx *g, u16 addr)
{
	if (addr == GT9XX_REG_ESD) {
		g->stats.reads_esd++;
		return GT9XX_ESD_READ_VAL;	/* always 0xFF, like the real chip */
	}
	if (addr == GT9XX_REG_COORD)
		g->stats.reads_coord++;

	if (in_window(addr))
		return *reg_slot(g, addr);
	return 0xFF;
}

static void reg_write(struct gt9xx *g, u16 addr, u8 val)
{
	if (addr == GT9XX_REG_ESD) {
		/* Host kicks the ESD watchdog (0xAA ~every 2 s). ACK only. */
		g->stats.esd_kicks++;
		return;
	}

	if (in_window(addr))
		*reg_slot(g, addr) = val;

	/*
	 * The host clears the buffer-status byte (writes 0x814E with the
	 * data-ready bit low) once it has consumed our report: drop INT.
	 */
	if (addr == GT9XX_REG_COORD && !(val & GT9XX_STAT_READY))
		int_release(g);
}

/* ---- publish a touch + RST monitoring ------------------------------------ */

static void publish_touch(struct gt9xx *g, const struct gt9xx_touch *t)
{
	u8 *c = reg_slot(g, GT9XX_REG_COORD);

	c[GT9XX_COORD_STATUS]  = GT9XX_STAT_READY | (t->active ? 1 : 0);
	c[GT9XX_COORD_TRACK]   = t->track_id;
	c[GT9XX_COORD_X_LO]    = t->x & 0xFF;
	c[GT9XX_COORD_X_HI]    = t->x >> 8;
	c[GT9XX_COORD_Y_LO]    = t->y & 0xFF;
	c[GT9XX_COORD_Y_HI]    = t->y >> 8;
	c[GT9XX_COORD_SIZE_LO] = t->size & 0xFF;
	c[GT9XX_COORD_SIZE_HI] = t->size >> 8;

	int_assert(g);			/* pull INT low -> host comes to read */
}

/*
 * Watch the host RST line, sampled cheaply (throttled) at bus-idle - the bus is
 * quiet during a host reset, so the idle loop runs freely then. On RST low we
 * tri-state INT so the host can drive it for I2C-address selection; on release
 * we take INT back over, push-pull idle-high, ready to report touches again.
 *
 * Debounced: a level must read the same for DEBOUNCE_SAMPLES consecutive
 * throttled samples before we act on it. A loose FFC connector or line noise
 * can otherwise produce spurious extra edges (rst_edges counting higher than
 * the host's real reset sequence), or a very short glitch counted as a full
 * edge - both of which used to make the touch handshake succeed or fail
 * somewhat unpredictably depending on how many "edges" we happened to see.
 */
#define RST_DEBOUNCE_SAMPLES 3

static void watch_reset(struct gt9xx *g)
{
	u8 rst;

	if ((++g->idle_ticks & 0x0FFFu) != 0)		/* ~once per few 100 us */
		return;

	rst = (u8)gt_gpio_get(&g->rst_pin);
	if (rst == g->rst_prev) {
		g->rst_debounce = 0;
		return;
	}

	if (++g->rst_debounce < RST_DEBOUNCE_SAMPLES)
		return;			/* not stable yet - could be noise/bounce */
	g->rst_debounce = 0;
	g->rst_prev = rst;

	if (!rst) {
		g->in_reset = true;
		gt_gpio_hiz(&g->int_pin);		/* float for addr select */
		g->int_asserted = false;
	} else {
		g->in_reset = false;
		g->stats.rst_edges++;
		gt_gpio_init_pp(&g->int_pin, 1);	/* restore push-pull high */
		g->int_asserted = false;
		/*
		 * The host just reset us: that IS the re-init we were fishing
		 * for, so stop playing dead and answer its handshake.
		 */
		g->muted = false;
	}
}

/*
 * Cold-boot re-sync: stay muted (NACKing) until the host notices and resets us
 * (handled in watch_reset), or until resync_ms elapses as a fallback for hosts
 * that recover without toggling RST.
 */
static void resync_tick(struct gt9xx *g)
{
	if (!g->muted)
		return;
	if ((u32)(gt_now_ms() - g->boot_ms) >= g->resync_ms)
		g->muted = false;
}

/* ---- i2c_slave callbacks ------------------------------------------------- */

static void op_start(void *ctx, bool is_read)
{
	struct gt9xx *g = ctx;

	if (!is_read)
		g->wr_count = 0;	/* a write phase re-arms the 2-byte address */
}

static void op_write_byte(void *ctx, u8 byte)
{
	struct gt9xx *g = ctx;

	if (g->wr_count == 0) {
		g->reg_ptr = (u16)byte << 8;
		g->wr_count = 1;
	} else if (g->wr_count == 1) {
		g->reg_ptr = (g->reg_ptr & 0xFF00) | byte;
		g->wr_count = 2;
		if (g->reg_ptr == GT9XX_REG_COORD)
			g->stats.coord_ptr_sets++;	/* host about to read touch */
		else if (g->reg_ptr == GT9XX_REG_ID ||
			 g->reg_ptr == GT9XX_REG_CONFIG)
			g->stats.init_ptr_sets++;	/* host (re)doing init */
	} else {
		reg_write(g, g->reg_ptr, byte);
		g->reg_ptr++;
	}
}

static void op_read_byte(void *ctx, u8 *byte)
{
	struct gt9xx *g = ctx;

	*byte = reg_read(g, g->reg_ptr);
	g->reg_ptr++;
}

static void op_stop(void *ctx)
{
	(void)ctx;	/* register pointer persists for the next transaction */
}

/* Play dead while re-syncing so the host's ESD watchdog trips and re-inits us. */
static bool op_addressable(void *ctx)
{
	struct gt9xx *g = ctx;

	return !g->muted;
}

static void op_idle(void *ctx)
{
	struct gt9xx *g = ctx;
	struct gt9xx_touch t;

	watch_reset(g);
	resync_tick(g);

	if (g->in_reset)		/* don't fight the host while it resets us */
		return;
	if (g->muted)			/* still playing dead: no touches yet */
		return;
	if (g->fetch && g->fetch(g->fetch_ctx, &t))
		publish_touch(g, &t);
}

static const struct i2c_slave_ops gt9xx_ops = {
	.start		= op_start,
	.write_byte	= op_write_byte,
	.read_byte	= op_read_byte,
	.stop		= op_stop,
	.idle		= op_idle,
	.addressable	= op_addressable,
};

const struct i2c_slave_ops *gt9xx_i2c_ops(void)
{
	return &gt9xx_ops;
}

/* ---- public API ---------------------------------------------------------- */

bool gt9xx_config_ok(void)
{
	u8 sum = 0;
	size_t i;

	for (i = 0; i < GT9XX_CONFIG_LEN; i++)
		sum += gt9xx_config[i];
	return sum == 0;
}

void gt9xx_set_touch_source(struct gt9xx *g, gt9xx_touch_source fetch, void *ctx)
{
	g->fetch = fetch;
	g->fetch_ctx = ctx;
}

void gt9xx_set_resync_ms(struct gt9xx *g, u32 ms)
{
	g->resync_ms = ms;
	g->boot_ms   = gt_now_ms();
	g->muted     = (ms != 0);	/* start playing dead unless disabled */
}

void gt9xx_get_stats(const struct gt9xx *g, struct gt9xx_host_stats *out)
{
	if (g && out)
		*out = g->stats;
}

void gt9xx_init(struct gt9xx *g, const struct gt_gpio *int_pin,
		const struct gt_gpio *rst_pin)
{
	g->int_pin = *int_pin;
	g->rst_pin = *rst_pin;
	g->reg_ptr = 0;
	g->wr_count = 0;
	g->fetch = NULL;
	g->fetch_ctx = NULL;
	memset(&g->stats, 0, sizeof(g->stats));

	/*
	 * Cold-boot re-sync is OFF by default: answer immediately (the proven-good
	 * behaviour). It only helps masters that re-arm their touch IRQ after an
	 * ESD-triggered reset; the head-unit we tested does NOT (it only arms touch
	 * on its own full power-up), so enabling it there just prolongs the outage.
	 * Turn it on via gt9xx_set_resync_ms() for masters that do support it.
	 */
	g->resync_ms = 0;
	g->boot_ms   = gt_now_ms();
	g->muted     = false;

	/* Load the register file: config table + product id, no touch yet. */
	memset(g->regfile, 0, sizeof(g->regfile));
	memcpy(reg_slot(g, GT9XX_REG_CONFIG), gt9xx_config, GT9XX_CONFIG_LEN);
	memcpy(reg_slot(g, GT9XX_REG_ID), gt9xx_id, GT9XX_ID_LEN);
	*reg_slot(g, GT9XX_REG_COORD) = 0x00;

	/*
	 * INT: push-pull, start idle (high). RST: input we only monitor.
	 *
	 * We deliberately start OPTIMISTIC (in_reset = false, rst_prev = 1
	 * "released") instead of latching whatever the very first raw read of
	 * RST happens to be. A single sample taken microseconds after power-on
	 * is exactly the kind of read that can catch transient noise or a
	 * momentarily flaky physical connection (loose FFC pin) as a false
	 * LOW - and because in_reset only ever clears on a later RISING edge
	 * (see watch_reset()), a wrong initial LOW reading with no real reset
	 * pulse behind it would latch touch reporting off for the entire
	 * session, with nothing to ever generate the edge that would release
	 * it (observed symptom: touch dead until the FFC is unplugged and
	 * replugged, which forces a fresh edge).
	 *
	 * Starting optimistic instead means: if RST truly is low right now,
	 * watch_reset()'s very next throttled sample (sub-millisecond later)
	 * will see rst=0 != rst_prev=1 and correctly transition into
	 * in_reset=true - same outcome, just reached via the normal
	 * edge-detection path instead of an unverified one-shot guess. If RST
	 * is high (the overwhelmingly common case - the host finished its own
	 * boot long before we did), we start answering immediately, which is
	 * correct.
	 */
	gt_gpio_init_pp(&g->int_pin, 1);
	g->int_asserted = false;
	gt_gpio_init_in(&g->rst_pin);
	g->rst_prev = 1;
	g->rst_debounce = 0;
	g->in_reset = false;
}
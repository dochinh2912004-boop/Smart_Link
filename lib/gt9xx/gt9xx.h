/* SPDX-License-Identifier: GPL-2.0 */
/*
 * gt9xx - portable emulation of a Goodix GT9xx (GT928-class) capacitive touch
 * controller. Implements the register map, config table, product id, ESD
 * watchdog reply, touch-point report and the INT/RST line protocol.
 *
 * It plugs into the i2c_slave engine (it supplies the i2c_slave_ops) and speaks
 * to the pins only through gt_hal.h, so it is identical on RK3568 and ESP32.
 * Everything platform-specific (threading, the source of injected touches,
 * locking) stays in the platform layer, which feeds touches in through a small
 * fetch hook.
 */
#ifndef _GT9XX_H_
#define _GT9XX_H_

#include "gt_types.h"
#include "gt_hal.h"
#include "gt9xx_regs.h"
#include "i2c_slave.h"

#ifdef __cplusplus
extern "C" {
#endif

/* One touch point in the controller's RAW coordinate space (0..X_MAX/Y_MAX). */
struct gt9xx_touch {
	bool active;	/* true = finger down, false = up / no touch */
	u8   track_id;	/* touch id, usually 0 for a single finger */
	u16  x;		/* raw X */
	u16  y;		/* raw Y */
	u16  size;	/* touch area (may be a fixed value, e.g. 0x20) */
};

/* Upper-layer host-interaction counters (distinct from the bus-level stats). */
struct gt9xx_host_stats {
	u64 reads_coord;	/* host fetched the touch report (0x814E) */
	u64 reads_esd;		/* host polled the ESD register (0x8040) */
	u64 esd_kicks;		/* host wrote 0xAA to keep the watchdog alive */
	u64 coord_ptr_sets;	/* host aimed its pointer at 0x814E (INT worked) */
	u64 init_ptr_sets;	/* host aimed at product-id/config (re-init handshake) */
	u64 rst_edges;		/* host RST low->high cycles (chip resets) */
};

/*
 * Pull the next staged touch. Supplied by the platform; called from the poll
 * thread at bus-idle. Return true and fill *out if a fresh touch is available,
 * false otherwise. The platform owns any locking against its injector.
 */
typedef bool (*gt9xx_touch_source)(void *ctx, struct gt9xx_touch *out);

struct gt9xx {
	/* Lines we own toward the host. */
	struct gt_gpio int_pin;	/* push-pull: low = "touch ready" */
	struct gt_gpio rst_pin;	/* input: host reset, monitored only */

	/* Emulated register space; mutated only from the poll thread. */
	u8 regfile[GT9XX_WIN_SIZE];

	/* Register-pointer machine (poll-thread private). */
	u16 reg_ptr;
	u8  wr_count;		/* address/data bytes seen in the current write */

	bool int_asserted;

	/* RST monitor / re-init tracking (poll-thread private). */
	u8   rst_prev;
	u8   rst_debounce;	/* consecutive stable samples of a pending RST change */
	u32  idle_ticks;	/* throttles RST sampling in the idle handler */
	bool in_reset;		/* host is holding RST low right now */

	/*
	 * Cold-boot re-sync: right after we power up we play dead (NACK our
	 * address) so the host's ESD watchdog fails, resets us over RST and
	 * re-runs its init handshake (re-arming its touch IRQ). We come alive
	 * again on that RST edge, or after resync_ms as a fallback.
	 */
	bool muted;		/* currently NACKing our address */
	u32  boot_ms;		/* gt_now_ms() captured at init */
	u32  resync_ms;		/* mute window; 0 disables the whole mechanism */

	/* Where injected touches come from. */
	gt9xx_touch_source fetch;
	void *fetch_ctx;

	struct gt9xx_host_stats stats;
};

/*
 * Initialise @g: load the config table + product id into the register file,
 * drive INT to idle (push-pull high) and set RST up as an input. @int_pin and
 * @rst_pin describe those two lines for the HAL.
 */
void gt9xx_init(struct gt9xx *g, const struct gt_gpio *int_pin,
		const struct gt_gpio *rst_pin);

/* Register the source of injected touches (see gt9xx_touch_source). */
void gt9xx_set_touch_source(struct gt9xx *g, gt9xx_touch_source fetch, void *ctx);

/*
 * Set the cold-boot re-sync window (ms) and (re)start it. Should be >= one host
 * ESD period (~2 s) so at least one watchdog read fails. 0 disables re-sync
 * (the slave answers immediately). Call before the poll thread starts, or any
 * time to force a fresh re-sync.
 */
void gt9xx_set_resync_ms(struct gt9xx *g, u32 ms);

/* The i2c_slave_ops that implement the Goodix protocol; pass ctx = your gt9xx. */
const struct i2c_slave_ops *gt9xx_i2c_ops(void);

/* Snapshot the host-interaction counters. */
void gt9xx_get_stats(const struct gt9xx *g, struct gt9xx_host_stats *out);

/* True iff the built-in config table is self-consistent (sums to 0 mod 256). */
bool gt9xx_config_ok(void);

#ifdef __cplusplus
}
#endif

#endif /* _GT9XX_H_ */
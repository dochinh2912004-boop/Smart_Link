/* SPDX-License-Identifier: GPL-2.0 */
/*
 * gt9xx_regs.h - Goodix GT9xx (GT928-class) register map and wire formats, as
 * reverse-engineered from a live bus capture of the original head-unit touch
 * link (see CLAUDE.md). Pure constants: shared by the device model (gt9xx.c)
 * and anything that needs to speak the same protocol.
 *
 * The chip presents a 16-bit, big-endian register address space; a master
 * write of two address bytes sets an auto-incrementing pointer, then reads or
 * writes stream from there.
 */
#ifndef _GT9XX_REGS_H_
#define _GT9XX_REGS_H_

/*
 * We only model a small window of the address space. Everything the host
 * touches (0x8040 ESD .. 0x814E+ touch report) fits in 0x8000..0x82FF.
 */
#define GT9XX_WIN_BASE		0x8000
#define GT9XX_WIN_SIZE		0x0300

/* Registers of interest. */
#define GT9XX_REG_ESD		0x8040	/* ESD watchdog / command */
#define GT9XX_REG_CONFIG	0x8047	/* start of the 246-byte config table */
#define GT9XX_REG_ID		0x8140	/* product id (4) + firmware version (2) */
#define GT9XX_REG_COORD		0x814E	/* buffer status + touch point 1 */

/* Buffer-status byte at 0x814E. */
#define GT9XX_STAT_READY	0x80	/* bit7: a fresh report is available */
#define GT9XX_STAT_NTOUCH_MASK	0x0f	/* bits0-3: number of active points */

/* ESD register read-back value observed on the real chip. */
#define GT9XX_ESD_READ_VAL	0xFF

/*
 * Touch point 1 report, 8 bytes starting at 0x814E:
 *   [0] status (see GT9XX_STAT_*)   [1] track id
 *   [2..3] X, little-endian         [4..5] Y, little-endian
 *   [6..7] size, little-endian
 */
#define GT9XX_COORD_STATUS	0
#define GT9XX_COORD_TRACK	1
#define GT9XX_COORD_X_LO	2
#define GT9XX_COORD_X_HI	3
#define GT9XX_COORD_Y_LO	4
#define GT9XX_COORD_Y_HI	5
#define GT9XX_COORD_SIZE_LO	6
#define GT9XX_COORD_SIZE_HI	7
#define GT9XX_COORD_LEN		8

/*
 * Config-table field offsets (relative to GT9XX_REG_CONFIG), little-endian.
 * The decoded panel range is X=1280, Y=800 for this unit.
 */
#define GT9XX_CFG_VERSION	0
#define GT9XX_CFG_X_MAX_LO	1
#define GT9XX_CFG_X_MAX_HI	2
#define GT9XX_CFG_Y_MAX_LO	3
#define GT9XX_CFG_Y_MAX_HI	4

#define GT9XX_CONFIG_LEN	247	/* bytes; whole table sums to 0 (mod 256) */
#define GT9XX_ID_LEN		6	/* "928\0" + version */

#endif /* _GT9XX_REGS_H_ */

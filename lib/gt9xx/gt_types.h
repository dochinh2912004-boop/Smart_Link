/* SPDX-License-Identifier: GPL-2.0 */
/*
 * gt_types.h - fixed-width integer types and the GT_INLINE qualifier used by
 * the portable core (i2c_slave, gt9xx). The core is compiled unchanged both as
 * part of the RK3568 Linux kernel module and, later, on an ESP32 (ESP-IDF).
 * This header is the single place that reconciles the two toolchains.
 *
 * Add a new target here (define GT_PLATFORM_* on its build) and everything in
 * the core keeps compiling.
 */
#ifndef _GT_TYPES_H_
#define _GT_TYPES_H_

#if defined(__KERNEL__)

#include <linux/types.h>
#include <linux/string.h>
#include <linux/kernel.h>
#define GT_INLINE static __always_inline

#else /* bare-metal / RTOS target (e.g. ESP32) */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

#define GT_INLINE static inline

#endif /* __KERNEL__ */

#endif /* _GT_TYPES_H_ */

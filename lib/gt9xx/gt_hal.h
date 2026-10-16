/* SPDX-License-Identifier: GPL-2.0 */
/*
 * gt_hal.h - Hardware Abstraction Layer for the GT9xx touch-controller
 * emulator. This is the ONLY seam between the portable core (i2c_slave, gt9xx)
 * and the silicon. Porting to a new MCU means writing one gt_hal_<mcu>.h that
 * provides the handful of primitives declared below - nothing in the core
 * changes.
 *
 * The contract (each platform header must define all of it):
 *
 *   struct gt_gpio                 opaque per-line descriptor (base+bit, pin#, ...)
 *
 *   Pin I/O (used by the bit-bang engine and the INT/RST logic):
 *     u32  gt_gpio_get(p)          read the live level of a line          -> 0/1
 *     void gt_gpio_get2(a,b,va,vb) sample TWO lines atomically (SDA+SCL)
 *     void gt_gpio_low(p)          open-drain: actively pull the line to 0
 *     void gt_gpio_release(p)      open-drain: let go (Hi-Z, external pull -> 1)
 *     void gt_gpio_set(p,v)        push-pull:  actively drive the line to v (0/1)
 *     void gt_gpio_hiz(p)          tri-state a push-pull line (float it, input)
 *
 *   One-time line setup (called from platform bring-up):
 *     void gt_gpio_init_od(p)      configure as an open-drain line, released
 *     void gt_gpio_init_pp(p,v)    configure as a push-pull output at level v
 *     void gt_gpio_init_in(p)      configure as an input
 *
 *   Time source (for the cold-boot re-sync window):
 *     u32  gt_now_ms()             a free-running millisecond counter (may wrap;
 *                                  only used for short interval subtraction)
 *
 *   Real-time guards (the poll loop lives in the core; the platform decides
 *   what these do - they may be no-ops):
 *     GT_IRQ_FLAGS_T               type of the saved-interrupt cookie
 *     GT_IRQ_SAVE(flags)           enter a timing-critical region
 *     GT_IRQ_RESTORE(flags)        leave it
 *     GT_CPU_RELAX()               spin-wait hint
 *     GT_WATCHDOG_KICK()           pet any hardware/soft watchdog
 *     GT_YIELD()                   hand the scheduler/RCU a quiescent point
 *
 * Design rules for the primitives: O(1), no allocation, no sleeping - they run
 * inside the bit-bang loop where every microsecond counts.
 */
#ifndef _GT_HAL_H_
#define _GT_HAL_H_

#include "gt_types.h"

/*
 * Arduino/ESP32 fork of the original multi-target header: this sketch only
 * ever targets ESP32 (Arduino core), so the platform is hardcoded here
 * instead of relying on a -DGT_PLATFORM_ESP32 build flag (the Arduino IDE
 * has no simple per-sketch place to add that). If you later re-merge this
 * with the RK3568 tree, restore the #if/#elif switch that picks
 * gt_hal_rk3568.h for GT_PLATFORM_RK3568 / __KERNEL__.
 */
#define GT_PLATFORM_ESP32 1
#include "gt_hal_esp32.h"

#endif /* _GT_HAL_H_ */

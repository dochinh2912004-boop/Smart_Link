#include "i2c.h"
#include "i2c_slave.h"
#include "gt9xx.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MHU_X_MIN 10
#define MHU_X_MAX 1270
#define MHU_Y_MIN 7
#define MHU_Y_MAX 793

#define ANDROID_X_MIN 77
#define ANDROID_X_MAX 1862
#define ANDROID_Y_MIN 63
#define ANDROID_Y_MAX 1126

#define POLL_CORE 0

/*
 * RELEASE_TIMEOUT_MS — thời gian tự nhả tay khi không nhận tọa độ mới.
 *
 * V1.0 (len=4): đây là cơ chế nhả tay DUY NHẤT → cần đủ ngắn để tap cảm
 *   giác nhanh, nhưng đủ dài để không nhả giữa chừng. ~200ms là hợp lý.
 *
 * V1.1 (len=5): Android gửi ACTION_UP tường minh → i2c_release_touch() xử
 *   lý ngay lập tức. Timeout ở đây CHỈ là lưới an toàn phòng trường hợp
 *   frame UP bị mất. Đặt cao (500ms) để không bao giờ cắt ngang một cú
 *   swipe đang diễn ra (Android có thể pause >80ms giữa 2 MOVE khi scroll
 *   chậm hoặc CPU bận).
 *
 * ⚠️  80ms cũ quá ngắn: dễ cắt ngang swipe khi vuốt chậm.
 */
#define RELEASE_TIMEOUT_MS 500

portMUX_TYPE gt_esp32_mux = portMUX_INITIALIZER_UNLOCKED;

static struct gt9xx     g_core;
static struct i2c_slave g_slave;

static portMUX_TYPE       stage_mux = portMUX_INITIALIZER_UNLOCKED;
static struct gt9xx_touch stage_pending;
static volatile bool      stage_dirty;
static volatile bool      touch_down;
static volatile uint32_t  last_touch_ms;

static uint16_t scale_x(uint16_t android_x)
{
    if (android_x < ANDROID_X_MIN) android_x = ANDROID_X_MIN;
    if (android_x > ANDROID_X_MAX) android_x = ANDROID_X_MAX;
    return (uint16_t)(MHU_X_MIN +
        (uint32_t)(android_x - ANDROID_X_MIN) * (MHU_X_MAX - MHU_X_MIN) /
        (ANDROID_X_MAX - ANDROID_X_MIN));
}

static uint16_t scale_y(uint16_t android_y)
{
    if (android_y < ANDROID_Y_MIN) android_y = ANDROID_Y_MIN;
    if (android_y > ANDROID_Y_MAX) android_y = ANDROID_Y_MAX;
    return (uint16_t)(MHU_Y_MIN +
        (uint32_t)(android_y - ANDROID_Y_MIN) * (MHU_Y_MAX - MHU_Y_MIN) /
        (ANDROID_Y_MAX - ANDROID_Y_MIN));
}

static void stage(bool active, uint16_t x, uint16_t y)
{
    portENTER_CRITICAL(&stage_mux);
    stage_pending.active   = active;
    stage_pending.track_id = 0;
    stage_pending.x        = x;
    stage_pending.y        = y;
    stage_pending.size     = active ? 0x20 : 0x00;
    stage_dirty = true;
    portEXIT_CRITICAL(&stage_mux);
}

static bool fetch_touch(void *ctx, struct gt9xx_touch *out)
{
    (void)ctx;
    if (!stage_dirty)
        return false;
    portENTER_CRITICAL(&stage_mux);
    if (!stage_dirty) {
        portEXIT_CRITICAL(&stage_mux);
        return false;
    }
    *out = stage_pending;
    stage_dirty = false;
    portEXIT_CRITICAL(&stage_mux);
    return true;
}

static void poll_task(void *arg)
{
    (void)arg;
    i2c_slave_run(&g_slave);
    vTaskDelete(NULL);
}

void i2c_init()
{
    esp_task_wdt_deinit();
#if POLL_CORE == 1
    disableCore1WDT();
#else
    disableCore0WDT();
#endif

    struct gt_gpio int_pin = { .num = (gpio_num_t)I2C_INT_PIN };
    struct gt_gpio rst_pin = { .num = (gpio_num_t)I2C_RST_PIN };
    struct i2c_slave_config cfg = {
        .sda       = { .num = (gpio_num_t)I2C_SDA_PIN },
        .scl       = { .num = (gpio_num_t)I2C_SCL_PIN },
        .addr      = I2C_ADD,
        .guard_irq = true,
        .ops       = gt9xx_i2c_ops(),
        .ctx       = &g_core,
    };

    gt9xx_init(&g_core, &int_pin, &rst_pin);
    gt9xx_set_touch_source(&g_core, fetch_touch, NULL);

    /*
     * Force a ~2.5s cold-boot re-sync window regardless of what the RST
     * line does. Previously touch success depended on the host's RST
     * pin toggling the exact right number of times (fragile - noise,
     * FFC contact bounce, or a missed/extra edge could break the
     * handshake unpredictably). This makes us go quiet (NACK our
     * address) for one full ESD watchdog period on every ESP32 boot, so
     * the host's own probe/retry logic reliably sees "chip appeared"
     * and runs its full init handshake - independent of RST edge count.
     */
    gt9xx_set_resync_ms(&g_core, 2500);

    i2c_slave_init(&g_slave, &cfg);

    xTaskCreatePinnedToCore(poll_task, "gt9xx_poll", 4096, NULL,
                            configMAX_PRIORITIES - 1, NULL, POLL_CORE);
}

void i2c_aux_mode(bool aux_on) { (void)aux_on; }

void i2c_prepare_touch(uint16_t android_x, uint16_t android_y)
{
    stage(true, scale_x(android_x), scale_y(android_y));
    touch_down    = true;
    last_touch_ms = millis();
}

void i2c_release_touch()
{
    /*
     * Gọi ngay khi nhận ACTION_UP từ Android (V1.1).
     * stage(false) → fetch_touch() trả active=false → gt9xx engine assert
     * INT với status=0x80 (0 điểm) → MHU nhận finger-up đúng cách.
     * Không chờ timeout, không delay.
     */
    stage(false, 0, 0);
    touch_down = false;
}

void i2c_touch_tick()
{
    /*
     * Safety net: tự nhả tay nếu không nhận tọa độ mới trong
     * RELEASE_TIMEOUT_MS. Dành cho V1.0 (không có explicit UP) và
     * phòng trường hợp frame V1.1 ACTION_UP bị mất trên đường truyền.
     */
    if (touch_down && (millis() - last_touch_ms) >= RELEASE_TIMEOUT_MS) {
        stage(false, 0, 0);
        touch_down = false;
    }
}
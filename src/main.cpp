#include "main.h"
#include "checksum.h"
#include "can_vf3.h"
#include "uart0.h"
#include "i2c.h"
#include "Headuint.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

uint8_t  gear                  = GEAR_P;
uint8_t  drivemode             = MODE_NORMAL;
uint8_t  regen_brake           = REGEN_OFF;
bool     hand_brake            = false;
bool     seat_belt             = false;
uint8_t  speed                 = 0;
uint8_t  battery               = 100;
uint16_t range                 = 0;
uint8_t  cruise_control_status = CRUISE_OFF;
uint8_t  cruise_control_value  = 0;

bool daylight    = false;
bool lowbeam     = false;
bool highbeam    = false;
bool leftlight   = false;
bool rightlight  = false;
bool camera_turn_left  = false;
bool camera_turn_right = false;

bool front_leftdoor    = false;
bool front_rightdoor   = false;
bool rear_leftdoor     = false;
bool rear_rightdoor    = false;
bool front_trunk       = false;
bool rear_trunk        = false;
bool fuse_charge_cover = false;
bool sunroof           = false;

bool fold_mirror = false;
bool tailgate    = false;
bool AUX_status  = false;

uint8_t steering_position     = 0x80;
uint8_t sensor_rear_left      = 0;
uint8_t sensor_rear_mid_left  = 0;
uint8_t sensor_rear_mid_right = 0;
uint8_t sensor_rear_right     = 0;

static void send_steering_angle_periodic()
{
    static unsigned long last_send = 0;
    static bool send_started = false;
    const unsigned long now = millis();

    // Send the latest CAN angle directly. Only the UART update rate is limited;
    // no interpolation or slew limiting is applied.
    if (!send_started || now - last_send >= 10UL) {
        Send_Steering_Angle(steering_position);
        last_send = now;
        send_started = true;
    }
}

void camera_control_()
{
    enum CameraState : uint8_t {
        CAMERA_STATE_OFF,
        CAMERA_STATE_LEFT,
        CAMERA_STATE_RIGHT
    };

    static CameraState camera_state = CAMERA_STATE_OFF;
    static unsigned long time_off_camera = 0;
    const unsigned long now = millis();

    // Reverse camera always has priority over the two side cameras.
    if (gear == GEAR_R) {
        digitalWrite(cam_back, HIGH);
        if (camera_state != CAMERA_STATE_OFF) {
            SIGNAL_OFF();
            camera_state = CAMERA_STATE_OFF;
        }
        time_off_camera = 0;
        return;
    }

    digitalWrite(cam_back, LOW);

    if (camera_turn_left && !camera_turn_right) {
        time_off_camera = now;
        if (camera_state != CAMERA_STATE_LEFT) {
            SIGNAL_LEFT();
            camera_state = CAMERA_STATE_LEFT;
        }
        return;
    }

    if (camera_turn_right && !camera_turn_left) {
        time_off_camera = now;
        if (camera_state != CAMERA_STATE_RIGHT) {
            SIGNAL_RIGHT();
            camera_state = CAMERA_STATE_RIGHT;
        }
        return;
    }

    // Both indicators active are hazard lights, so no side camera is selected.
    if (camera_turn_left && camera_turn_right) {
        if (camera_state != CAMERA_STATE_OFF) {
            SIGNAL_OFF();
            camera_state = CAMERA_STATE_OFF;
        }
        time_off_camera = 0;
        return;
    }

    if (camera_state != CAMERA_STATE_OFF &&
        now - time_off_camera >= 1500UL) {
        SIGNAL_OFF();
        camera_state = CAMERA_STATE_OFF;
        time_off_camera = 0;
    }
}

/* -------------------------------------------------------------------------
 * UART RX TASK — xử lý cảm ứng ở priority cao, không phụ thuộc loop().
 *
 * Tại sao cần task riêng?
 *   loop() gọi can_read() TRƯỚC uart0_receive(). Nếu can_read() block vài ms
 *   (chờ CAN frame hoặc timeout), touch frame đã nằm sẵn trong UART buffer
 *   nhưng phải đợi. Với swipe 60fps (frame cách nhau 16ms), mỗi ms delay
 *   là 6% latency của 1 frame — cộng dồn làm cảm giác lag rõ rệt.
 *
 * Cách hoạt động:
 *   Task này chạy trên Core 1, priority = configMAX_PRIORITIES-2 (cao hơn
 *   loopTask mặc định priority=1). FreeRTOS preemptive sẽ preempt loopTask
 *   ngay khi uart_rx_task sẵn sàng → touch frame được xử lý trong vòng 1ms
 *   kể từ khi dữ liệu vào buffer, bất kể loop() đang làm gì.
 *
 *   gt9xx_poll (Core 0, MAX-1) > uart_rx_task (Core 1, MAX-2) > loopTask (Core 1, 1)
 *
 * Thread safety:
 *   i2c_prepare_touch() và i2c_release_touch() dùng portENTER_CRITICAL →
 *   an toàn khi gọi từ task này. Serial trên ESP32 Arduino dùng mutex nội
 *   bộ → an toàn khi write từ 2 task khác nhau.
 * ------------------------------------------------------------------------- */
static void uart_rx_task(void *arg)
{
    (void)arg;
    for (;;) {
        /* Chỉ gọi khi có dữ liệu để tránh lock không cần thiết */
        if (Serial.available())
            uart0_receive();

        /* 1ms polling: max latency từ lúc Android gửi đến lúc xử lý = 1ms.
         * Với swipe 60fps (frame mỗi 16ms), độ trễ trung bình là 0.5ms —
         * so với vài ms+ khi phụ thuộc vào loop(). */
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void setup()
{
    
    Serial.begin(115200);
    Serial2.begin(38400, SERIAL_8N1, 17, 16);
    
    i2c_init();   /* tạo gt9xx_poll task bên trong, POLL_CORE=0 */
    
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    pinMode(ACC_PIN, OUTPUT);
    digitalWrite(ACC_PIN, HIGH);

    pinMode(ILL, OUTPUT);
    digitalWrite(ILL, LOW); 

    pinMode(cam_back, OUTPUT);
    digitalWrite(cam_back, LOW);

    pinMode(MIRROR_PIN, OUTPUT);
    pinMode(TRUNK_PIN, OUTPUT);

    can_init();
   

    /* Tạo uart_rx_task SAU Serial.begin() và i2c_init() */
    xTaskCreatePinnedToCore(
        uart_rx_task,
        "uart_rx",
        4096,
        NULL,
        configMAX_PRIORITIES - 2,   /* priority cao, chỉ dưới gt9xx_poll */
        NULL,
        1                            /* Core 1: cùng core với loopTask */
    );
    

}

void loop()
{
    
    // digitalWrite(LED_PIN, HIGH);
    // vTaskDelay(pdMS_TO_TICKS(50));
    // digitalWrite(LED_PIN, LOW);
    // vTaskDelay(pdMS_TO_TICKS(50));
    can_read();
    camera_control_();
    send_steering_angle_periodic();
    i2c_touch_tick();  
    output_tick();

    static unsigned long t_vehicle   = 0;
    static unsigned long t_lightdoor = 0;
    unsigned long now = millis();

    if (now - t_vehicle >= 200) {
        uart0_send_vehicle();
        t_vehicle = now;
    }

    if (now - t_lightdoor >= 200 && now - t_vehicle >= 100) {
        uart0_send_lightdoor();
        t_lightdoor = now;
    }
}

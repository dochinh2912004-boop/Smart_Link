#include "uart0.h"
#include "checksum.h"
#include "main.h"
#include "i2c.h"

/* =========================================================================
 * TOUCH PROTOCOL V1.1 - Tóm tắt:
 *
 *  len=4 (V1.0 cũ): X_H X_L Y_H Y_L          → auto-release theo timeout
 *  len=5 (V1.1 mới): X_H X_L Y_H Y_L State    → explicit up/down
 *
 *  State: 0x01 = DOWN / MOVE (đang nhấn hoặc vuốt)
 *         0x00 = UP (vừa nhấc tay)
 * ========================================================================= */
static bool mirror_flag = false;
static bool trunk_flag  = false;

static uint32_t time1 = 0;   /* mốc thời gian bắt đầu xung MIRROR_PIN */
static uint32_t time2 = 0;   /* mốc thời gian bắt đầu xung TRUNK_PIN */

void uart0_send_vehicle() {
    uint8_t data[7];
    data[0] = (gear & 0x03)
            | ((drivemode   & 0x03) << 2)
            | ((regen_brake & 0x03) << 4)
            | (hand_brake ? (1 << 6) : 0)
            | (seat_belt  ? (1 << 7) : 0);
    data[1] = speed;
    data[2] = battery;
    data[3] = (uint8_t)(range >> 8);
    data[4] = (uint8_t)(range & 0xFF);
    data[5] = cruise_control_status;
    data[6] = cruise_control_value;

    uint8_t frame[11];
    frame[0] = FRAME_HEAD;
    frame[1] = DTYPE_VEHICLE;
    frame[2] = 0x07;
    for (int i = 0; i < 7; i++) frame[3 + i] = data[i];
    frame[10] = calc_checksum(DTYPE_VEHICLE, 7, data);
    Serial.write(frame, 11);
}

void uart0_send_lightdoor() {
    uint8_t data[2];
    data[0] = (daylight    ? (1 << 0) : 0)
            | (lowbeam     ? (1 << 1) : 0)
            | (highbeam    ? (1 << 2) : 0)
            | (leftlight   ? (1 << 3) : 0)
            | (rightlight  ? (1 << 4) : 0);
    data[1] = (front_leftdoor    ? (1 << 0) : 0)
            | (front_rightdoor   ? (1 << 1) : 0)
            | (rear_leftdoor     ? (1 << 2) : 0)
            | (rear_rightdoor    ? (1 << 3) : 0)
            | (front_trunk       ? (1 << 4) : 0)
            | (rear_trunk        ? (1 << 5) : 0)
            | (fuse_charge_cover ? (1 << 6) : 0)
            | (sunroof           ? (1 << 7) : 0);

    uint8_t frame[6];
    frame[0] = FRAME_HEAD;
    frame[1] = DTYPE_LIGHTDOOR;
    frame[2] = 0x02;
    frame[3] = data[0];
    frame[4] = data[1];
    frame[5] = calc_checksum(DTYPE_LIGHTDOOR, 2, data);
    Serial.write(frame, 6);
}

void uart0_receive() {
    static uint8_t state    = 0;
    static uint8_t datatype = 0;
    static uint8_t len      = 0;
    static uint8_t data[16];
    static uint8_t idx      = 0;

    while (Serial.available()) {
        uint8_t b = Serial.read();

        switch (state) {
            case 0:
                if (b == FRAME_HEAD) state = 1;
                break;

            case 1:
                datatype = b;
                state = 2;
                break;

            case 2:
                len = b;
                idx = 0;
                if (len == 0 || len > (uint8_t)sizeof(data)) {
                    // Serial.printf("[ERR] Invalid len=0x%02X for dtype=0x%02X\n", len, datatype);
                    state = 0;
                    break;
                }
                state = 3;
                break;

            case 3:
                data[idx++] = b;
                if (idx >= len) state = 4;
                break;

            case 4: {
                uint8_t expected = calc_checksum(datatype, len, data);
                if (b != expected) {
                    // Serial.printf("[ERR] Checksum fail: got=0x%02X expect=0x%02X dtype=0x%02X\n",
                    //               b, expected, datatype);
                    Serial.write(NACK_CHECKSUM);
                    state = 0;
                    break;
                }

                Serial.write(ACK_OK);

                /* ---- DTYPE_BUTTON 0x80 ---------------------------------- */
                if (datatype == DTYPE_BUTTON) {
                    fold_mirror = (data[0] == 0x01);
                    tailgate    = (data[1] == 0x01);
                    AUX_status  = (data[2] == 0x01);
                    Serial.printf("[BTN] mirror=%d tailgate=%d aux=%d\n",
                                  fold_mirror, tailgate, AUX_status);

                    /*
                     * Android gửi ĐÚNG 1 frame rời rạc mỗi lần bấm thật (đã
                     * xác nhận qua test thực tế), không gửi lặp lại/heartbeat
                     * - nên bản thân việc frame này đến với bit=1 ĐÃ LÀ tín
                     * hiệu bấm, không cần dò "cạnh" gì thêm. Chỉ cần chặn
                     * kích lại trong lúc xung trước còn đang chạy dở
                     * (mirror_flag/trunk_flag), để không bị reset xung giữa
                     * chừng nếu vô tình có frame trùng lặp nào đó.
                     */
                    if (fold_mirror && !mirror_flag) {
                        mirror_flag = true;
                        time1 = millis();
                        digitalWrite(MIRROR_PIN, HIGH);
                    }
                    if (tailgate && !trunk_flag) {
                        trunk_flag = true;
                        time2 = millis();
                        digitalWrite(TRUNK_PIN, HIGH);
                    }

                /* ---- DTYPE_TOUCH 0x81 ----------------------------------- */
                } else if (datatype == DTYPE_TOUCH) {

                    uint16_t tx = ((uint16_t)data[0] << 8) | data[1];
                    uint16_t ty = ((uint16_t)data[2] << 8) | data[3];

                    /* --- V1.1: len=5, có Touch State byte --- */
                    if (len == 5) {
                        uint8_t touch_state = data[4];

                        if (touch_state == 0x01) {
                            /* DOWN hoặc MOVE: gửi tọa độ sang MHU */
                            Serial.printf("[V1.1] DOWN/MOVE X=%d Y=%d\n", tx, ty);
                            i2c_prepare_touch(tx, ty);

                        } else if (touch_state == 0x00) {
                            /* UP: nhả ngón tay ngay lập tức, không chờ timeout */
                            // Serial.printf("[V1.1] UP X=%d Y=%d\n", tx, ty);
                            i2c_release_touch();   /* gọi đúng hàm public, không extern static */

                        } else {
                            Serial.printf("[V1.1] Unknown state=0x%02X X=%d Y=%d\n",
                                          touch_state, tx, ty);
                        }

                    /* --- V1.0: len=4, tương thích ngược, dùng auto-release --- */
                    } else if (len == 4) {
                        Serial.printf("[V1.0] TAP X=%d Y=%d (auto-release)\n", tx, ty);
                        i2c_prepare_touch(tx, ty);

                    } else {
                        // Serial.printf("[ERR] DTYPE_TOUCH bad len=%d\n", len);
                        // Serial.write(NACK_NOT_SUPPORT);
                    }

                /* ---- Unknown -------------------------------------------- */
                } else {
                    // Serial.printf("[ERR] Unknown dtype=0x%02X len=%d\n", datatype, len);
                    // Serial.write(NACK_NOT_SUPPORT);
                }

                state = 0;
                idx   = 0;
                break;
            }

            default:
                state = 0;
                break;
        }
    }
}

void output_tick()
{
    uint32_t now = millis();

    if (mirror_flag && now - time1 >= 1000) {
        mirror_flag = false;
        digitalWrite(MIRROR_PIN, LOW);
    }

    if (trunk_flag && now - time2 >= 5000) {
        trunk_flag = false;
        digitalWrite(TRUNK_PIN, LOW);
    }
}
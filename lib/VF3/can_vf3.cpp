#include "can_vf3.h"
#include "main.h"
#include "driver/twai.h"
#include "Headuint.h"

static uint8_t parking_level(uint8_t raw)
{
    // VF3 reports 5/4/3 from far to near; the head unit expects 1/2/3.
    if (raw == 5) return 1;
    if (raw == 4) return 2;
    if (raw == 3) return 3;
    return 0;
}

static void send_parking_sensors_limited()
{
    static uint8_t last_rear_left = 0;
    static uint8_t last_rear_mid_left = 0;
    static uint8_t last_rear_mid_right = 0;
    static uint8_t last_rear_right = 0;
    static unsigned long last_send = 0;
    static bool send_started = false;

    const unsigned long now = millis();
    const bool changed = !send_started ||
                         sensor_rear_left != last_rear_left ||
                         sensor_rear_mid_left != last_rear_mid_left ||
                         sensor_rear_mid_right != last_rear_mid_right ||
                         sensor_rear_right != last_rear_right;
    const unsigned long elapsed = now - last_send;

    // Changed data is sent at most every 40 ms; unchanged data is refreshed
    // every 200 ms so the head unit does not retain an obsolete indication.
    if (!send_started || (changed && elapsed >= 40UL) || elapsed >= 200UL) {
        Send_Blind_Spot_Sensor_Rear(sensor_rear_left,
                                    sensor_rear_mid_left,
                                    sensor_rear_mid_right,
                                    sensor_rear_right);
        last_rear_left = sensor_rear_left;
        last_rear_mid_left = sensor_rear_mid_left;
        last_rear_mid_right = sensor_rear_mid_right;
        last_rear_right = sensor_rear_right;
        last_send = now;
        send_started = true;
    }
}

void can_init() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
 
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK)
        Serial.println("[CAN] Driver installed");
    else
        Serial.println("[CAN] Driver install FAILED");
 
    if (twai_start() == ESP_OK)
        Serial.println("[CAN] Started");
    else
        Serial.println("[CAN] Start FAILED");
}

void can_read() {
    twai_message_t msg;
    while(twai_receive(&msg, 0) == ESP_OK) {
        if(msg.flags & TWAI_MSG_FLAG_EXTD) continue;

        switch (msg.identifier) {
            case 0x40D: // Rear parking/collision sensors
                if (msg.data_length_code >= 7) {
                    sensor_rear_left      = constrain((msg.data[5] >> 4) & 0x0F, 0, 3);
                    sensor_rear_mid_left  = parking_level(msg.data[5] & 0x0F);
                    sensor_rear_mid_right = parking_level((msg.data[6] >> 4) & 0x0F);
                    sensor_rear_right     = constrain(msg.data[6] & 0x0F, 0, 3);

                    send_parking_sensors_limited();
                }
                break;

            case 0x17E: // Steering angle
                if (msg.data_length_code >= 8) {
                    const int16_t raw_angle =
                        (int16_t)(((uint16_t)msg.data[6] << 8) | msg.data[7]);
                    int16_t sensor_angle =
                        (int16_t)(raw_angle * 0.0238f);
                    int32_t position;

                    // Keep the conversion identical to the reference VF3
                    // decoder: 0x02=left, 0x80=centre, 0xFE=right.
                    if (sensor_angle < 0) {
                        sensor_angle = (int16_t)abs((int32_t)sensor_angle);
                        position = 0x80L -
                            (((777L - sensor_angle) * 126L) / 539L);
                    } else {
                        position = 0x80L +
                            (((777L - sensor_angle) * 126L) / 539L);
                    }

                    steering_position = (uint8_t)constrain(
                        position, 0x02L, 0xFEL);
                }
                break;

            //  case 0x112: // acc
            // {
            //     uint8_t raw = ((msg.data[4] >> 1) & 0x01);
            //     if (raw) digitalWrite(ACC_PIN, HIGH); // bật ACC cấp nguồn cho Màn Android
            //     else     digitalWrite(ACC_PIN, LOW);  // tắt ACC 
            //     break;
            // }

            case 0x2BF: // speed
             speed = (uint8_t)((((msg.data[6] & 0x7F) << 8) | msg.data[7]) * 0.01f);
             break;

            case 0x269: // Gear
            {
                uint8_t raw = (msg.data[1] & 0xE0) >> 5;
                if      (raw == 0) gear = GEAR_P;
                else if (raw == 1) gear = GEAR_R;
                else if (raw == 2) gear = GEAR_N;
                else if (raw == 3) gear = GEAR_D;
                break;
            }
 
            case 0x379: // Drive mode + Battery
            {
                uint8_t raw = (msg.data[0] & 0x38) >> 3;
                if      (raw == 1) drivemode = MODE_ECO;
                else if (raw == 0) drivemode = MODE_NORMAL;
                // SPORT: bổ sung thêm nếu có raw value
                battery = msg.data[7] & 0x7F;
                break;
            }
 
            case 0x106: // Lights
                lowbeam    = (msg.data[5] & 0x03) ? true : false;
                leftlight  = (msg.data[5] & 0x10) ? true : false;
                rightlight = (msg.data[5] & 0x20) ? true : false;
                camera_turn_left  = leftlight;
                camera_turn_right = rightlight;
                highbeam   = (msg.data[3] & 0xC0) ? true : false;
                daylight   = (msg.data[3] & 0x01) ? true : false;
                digitalWrite(ILL, lowbeam ? HIGH : LOW);
                break;
 
            case 0x102: // Hand brake
                hand_brake = ((msg.data[2] & 0x20) >> 5) == 0 ? true : false;
                break;
 
            case 0x105: // Doors
                front_leftdoor  = ((msg.data[2] & 0x0C) >> 2) ? true : false;
                front_rightdoor = ((msg.data[2] & 0x30) >> 4) ? true : false;
                rear_trunk      = (msg.data[2] & 0x03)        ? true : false;
                break;
 
            case 0x381: // Seat belt
                if (((msg.data[3] & 0x0C) >> 2) == 0x00) seat_belt = true;
                else if (((msg.data[3] & 0x03)==0x00)& ((msg.data[2] & 0x03)==0x01)) seat_belt = true;
                // else if ((msg.data[2] & 0x03)==0x00) seat_belt = true;
                // else if ((msg.data[3] & 0xC0)==0x00) seat_belt = true;
                else seat_belt = false;
                break;
 
            case 0x2E9: // Range
                range = (((uint16_t)msg.data[4] << 8) | msg.data[5]) / 10;
                break;
         
        }
    }
}

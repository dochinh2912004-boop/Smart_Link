//-------------------------lz---------------------------
#include "HeadUint.h"
#include "main.h"

void VOL_UP()
{
	
	HeadUnits_Serial.write(0x2E);
  HeadUnits_Serial.write(0x20);
  HeadUnits_Serial.write(0x02);
  HeadUnits_Serial.write(0x01);
  HeadUnits_Serial.write(0x01);
  HeadUnits_Serial.write(0xDB);


}

void Send_Steering_Angle(uint8_t steering_position)
{
	
	const uint8_t distance = (steering_position < 0x80)
		? (uint8_t)(0x80 - steering_position)
		: (uint8_t)(steering_position - 0x80);
	const uint8_t protocol_angle = (uint8_t)map(distance, 0L, 126L, 0L, 50L);
	const uint8_t direction = (protocol_angle == 0)
		? 0x00
		: ((steering_position < 0x80) ? 0x01 : 0x00);
	const uint8_t checksum = (uint8_t)((0x29 + 0x02 + direction + protocol_angle) ^ 0xFF);

	HeadUnits_Serial.write(0x2E);         // Head Code
	HeadUnits_Serial.write(0x29);         // Data Type: Góc EPS
	HeadUnits_Serial.write(0x02);         // Length
	HeadUnits_Serial.write(direction);      // Data0: 0 = right, 1 = left
	HeadUnits_Serial.write(protocol_angle); //
	HeadUnits_Serial.write(checksum);
}


void Send_Blind_Spot_Sensor_Rear(uint8_t sensor_rl, uint8_t sensor_rml,
								 uint8_t sensor_rmr, uint8_t sensor_rr)
{
	// Protocol 0x22: outer sensors use 0..3; middle sensors use 0..4.
	const uint8_t rear_left       = constrain(sensor_rl,  0, 3);
	const uint8_t rear_mid_left   = constrain(sensor_rml, 0, 4);
	const uint8_t rear_mid_right  = constrain(sensor_rmr, 0, 4);
	const uint8_t rear_right      = constrain(sensor_rr,  0, 3);
	const uint8_t checksum = (uint8_t)((0x22 + 0x04 + rear_left + rear_mid_left +
										  rear_mid_right + rear_right) ^ 0xFF);

	HeadUnits_Serial.write(0x2E);         // Head Code
	HeadUnits_Serial.write(0x22);         // Data Type: Cảm biến lùi
	HeadUnits_Serial.write(0x04);         // Length
	HeadUnits_Serial.write(rear_left);      // Data0: rear outer-left, 
	HeadUnits_Serial.write(rear_mid_left);  // Data1: rear middle-left, 
	HeadUnits_Serial.write(rear_mid_right); // Data2: rear middle-right, 
	HeadUnits_Serial.write(rear_right);     // Data3: rear outer-right, 
	HeadUnits_Serial.write(checksum);
}

void CAMERA_OFF() {  //off camera
  HeadUnits_Serial.write(0x2E);
  HeadUnits_Serial.write(0x7D);
  HeadUnits_Serial.write(0x02);
  HeadUnits_Serial.write(0x01);
  HeadUnits_Serial.write(0x00);
  HeadUnits_Serial.write(0x7F);
  // Serial.println("off");
}

void LEFT() {//camera left
  HeadUnits_Serial.write(0x2E);
  HeadUnits_Serial.write(0x7D);
  HeadUnits_Serial.write(0x02);
  HeadUnits_Serial.write(0x01);
  HeadUnits_Serial.write(0x10);
  HeadUnits_Serial.write(0x6F);
  // Serial.println("left");
}

void RIGHT() {//camera right
  HeadUnits_Serial.write(0x2E);
  HeadUnits_Serial.write(0x7D);
  HeadUnits_Serial.write(0x02);
  HeadUnits_Serial.write(0x01);
  HeadUnits_Serial.write(0x08);
  HeadUnits_Serial.write(0x77);
  // Serial.println("right");
}

void CENTER() { //camera center
  HeadUnits_Serial.write(0x2E);
  HeadUnits_Serial.write(0x7D);
  HeadUnits_Serial.write(0x02);
  HeadUnits_Serial.write(0x01);
  HeadUnits_Serial.write(0x18);
  HeadUnits_Serial.write(0x67);
  // Serial.println("center");
}


void SIGNAL_LEFT() {
  byte frame[] = {
    0x2E, // Head Code
    0x35, // Data Type
    0x0D, // Length (13 bytes payload)
    0x10, // Data0: BIT4 = 1 (Xi nhan trái)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xAD  
  };
  
  for(int i = 0; i < sizeof(frame); i++) {
    HeadUnits_Serial.write(frame[i]);
  }
  // Serial.println("Turn Signal: LEFT");
}

// Hàm gửi xi nhan phải để kích hoạt Cam phải
void SIGNAL_RIGHT() {
  byte frame[] = {
    0x2E, // Head Code
    0x35, // Data Type
    0x0D, // Length
    0x20, // Data0: BIT5 = 1 (Xi nhan phải)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Data1 -> Data12 = 0x00
    0x9D  // Checksum: (0x35 + 0x0D + 0x20 + 0) XOR 0xFF = 0x9D
  };
  
  for(int i = 0; i < sizeof(frame); i++) {
    HeadUnits_Serial.write(frame[i]);
  }
  // Serial.println("Turn Signal: RIGHT");
}

// Hàm gửi lệnh tắt xi nhan (để màn hình thu Cam360 về lại nền)
void SIGNAL_OFF() {
  byte frame[] = {
    0x2E, // Head Code
    0x35, // Data Type
    0x0D, // Length
    0x00, // Data0: Tất cả đèn tắt
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Data1 -> Data12 = 0x00
    0xBD  // Checksum: (0x35 + 0x0D + 0x00 + 0) XOR 0xFF = 0xBD
  };
  
  for(int i = 0; i < sizeof(frame); i++) {
    HeadUnits_Serial.write(frame[i]);
  }
  // Serial.println("Turn Signal: OFF");
}

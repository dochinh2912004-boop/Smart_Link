#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>

#define HeadUnits_Serial  Serial2

#define CAN_TX_PIN GPIO_NUM_5
#define CAN_RX_PIN GPIO_NUM_4


#define LED_PIN 2

#define FRAME_HEAD 0x2E

#define DTYPE_VEHICLE 0x10
#define DTYPE_LIGHTDOOR 0x11

#define DTYPE_BUTTON 0x80
#define DTYPE_TOUCH 0x81

#define ACK_OK 0xFF
#define NACK_CHECKSUM 0xF0
#define NACK_NOT_SUPPORT 0xF3
#define NACK_BUSY 0xFC


// Gear
#define GEAR_P              0x00
#define GEAR_N              0x01
#define GEAR_R              0x02
#define GEAR_D              0x03
 
// Drive mode
#define MODE_ECO            0x00
#define MODE_NORMAL         0x01
#define MODE_SPORT          0x02
 
// Regen brake
#define REGEN_OFF           0x00
#define REGEN_LOW           0x01
#define REGEN_HIGH          0x02
 
// Cruise control status
#define CRUISE_OFF          0x00
#define CRUISE_STANDBY      0x01
#define CRUISE_ACTIVE       0x02

#define  ACC_PIN             25
#define  ILL                 26  

#define  cam_back            12

#define MIRROR_PIN           33
#define TRUNK_PIN            15


extern uint8_t  gear;
extern uint8_t  drivemode;
extern uint8_t  regen_brake;
extern bool     hand_brake;
extern bool     seat_belt;
extern uint8_t  speed;
extern uint8_t  battery;
extern uint16_t range;
extern uint8_t  cruise_control_status;
extern uint8_t  cruise_control_value;
 
extern bool daylight;
extern bool lowbeam;
extern bool highbeam;
extern bool leftlight;
extern bool rightlight;


extern bool camera_turn_left;
extern bool camera_turn_right;
 
extern bool front_leftdoor;
extern bool front_rightdoor;
extern bool rear_leftdoor;
extern bool rear_rightdoor;
extern bool front_trunk;
extern bool rear_trunk;
extern bool fuse_charge_cover;
extern bool sunroof;
 
extern bool fold_mirror;
extern bool tailgate;
extern bool AUX_status;

extern uint8_t steering_position;
extern uint8_t sensor_rear_left;
extern uint8_t sensor_rear_mid_left;
extern uint8_t sensor_rear_mid_right;
extern uint8_t sensor_rear_right;

void camera_control_();

 
#endif

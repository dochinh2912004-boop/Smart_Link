#ifndef HEADUINT_H
#define HEADUINT_H

#include <Arduino.h>

void VOL_UP();  
void VOL_DOWN();
void VOL_MUTE();
void VOICE();
void PHONE();
void SOURCE();
void NEXT();
void BACK();
void CAMERA_OFF();
void LEFT();
void RIGHT();
void CENTER();
void Send_Steering_Angle(uint8_t steering_position);
void Send_Blind_Spot_Sensor_Rear(byte sensor_rl, byte sensor_rml, byte sensor_rmr, byte sensor_rr);

void SIGNAL_LEFT();
void SIGNAL_RIGHT();
void SIGNAL_OFF();

#endif // HEADUINT_H

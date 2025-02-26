#ifndef __COMMON_H
#define __COMMON_H

#include <Arduino.h>

extern float roll_converted;  // 横滚
extern float pitch_converted; // 俯仰
extern float yaw_converted;   // 偏航
extern float X_G;             // X加速度 米每秒
extern float Y_G;             // Y加速度
extern float Z_G;             // Z加速度
extern float Acc;             // 合加速度 G
extern int sensor_distance;
extern int GPS_speed;  // 地速
extern int air_speed; // 空速
extern int D_Line;     // 分化线比例
extern int D_Hight;   // 分化线间距
extern int W_Line;    // 分化线比例
extern int W_Hight;   // 分化线间距(像素)
extern int air_Height;
extern int air_Line;
extern int air_short_Line;
extern int alt_Height;
extern int alt_Line;
extern int alt_short_Line;
extern int alt;
extern float climb;
extern float heading;
extern uint16_t throttle;


extern unsigned long int runTime; // 从系统上电开始的运行时间
extern int LOOP_DELAY;           // 每多少毫秒系统更新一次状态

extern int horizon_line_y;

extern int last_roll;
extern int last_pitch;

// Variables for test only
extern int test_roll;
extern int delta;

extern unsigned long redrawTime;

void set_request_datastream(int com_id, int D_T);
void request_datastream(); // 直接请求MAVLINK消息
void MavLink_receive(); // 读取MAVLINK数据

#endif
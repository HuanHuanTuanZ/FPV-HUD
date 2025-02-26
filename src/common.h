#ifndef __COMMON_H
#define __COMMON_H

#include <Arduino.h>

extern float roll_converted;  // ���
extern float pitch_converted; // ����
extern float yaw_converted;   // ƫ��
extern float X_G;             // X���ٶ� ��ÿ��
extern float Y_G;             // Y���ٶ�
extern float Z_G;             // Z���ٶ�
extern float Acc;             // �ϼ��ٶ� G
extern int sensor_distance;
extern int GPS_speed;  // ����
extern int air_speed; // ����
extern int D_Line;     // �ֻ��߱���
extern int D_Hight;   // �ֻ��߼��
extern int W_Line;    // �ֻ��߱���
extern int W_Hight;   // �ֻ��߼��(����)
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


extern unsigned long int runTime; // ��ϵͳ�ϵ翪ʼ������ʱ��
extern int LOOP_DELAY;           // ÿ���ٺ���ϵͳ����һ��״̬

extern int horizon_line_y;

extern int last_roll;
extern int last_pitch;

// Variables for test only
extern int test_roll;
extern int delta;

extern unsigned long redrawTime;

void set_request_datastream(int com_id, int D_T);
void request_datastream(); // ֱ������MAVLINK��Ϣ
void MavLink_receive(); // ��ȡMAVLINK����

#endif
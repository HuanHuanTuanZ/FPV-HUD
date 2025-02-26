#include <Arduino.h>
#include "../.pio/libdeps/esp32dev/c_library_v2-master/all/mavlink.h"
#include "common.h"

// mavlink参数设置
uint8_t _system_id = 255;      // id of computer which is sending the command (ground control software has id of 255)
uint8_t _component_id = 0;     // seems like it can be any # except the number of what Pixhawk sys_id is
uint8_t _target_system = 1;    // Id #
uint8_t _target_component = 0; // Target component, 0 = all (seems to work with 0 or 1
uint16_t _command = MAV_CMD_REQUEST_MESSAGE;
uint16_t _command_1 = MAV_CMD_SET_MESSAGE_INTERVAL;
uint8_t _confirmation = 0;

void set_request_datastream(int com_id, int D_T) // 设置MAVLINK通信速率
{
    mavlink_message_t msg;
    uint8_t *buf;
    uint16_t len = mavlink_msg_command_long_pack(_system_id, _component_id, &msg, _target_system, _target_component, _command_1, _confirmation, com_id, D_T, 0, 0, 0, 0, 0);
    buf = new uint8_t[len];
    mavlink_msg_to_send_buffer(buf, &msg); // 消息写入字符串
    Serial2.write(buf, len);               // 发送至串口
}

void request_datastream() // 直接请求MAVLINK消息
{
    mavlink_message_t msg;
    uint8_t *buf;
    uint16_t len = mavlink_msg_command_long_pack(_system_id, _component_id, &msg, _target_system, _target_component, _command, _confirmation, 30, 0, 0, 0, 0, 0, 0);
    buf = new uint8_t[len];
    mavlink_msg_to_send_buffer(buf, &msg); // Send the message (.write sends as bytes)
    Serial2.write(buf, len);               // Write data to serial port
}

void MavLink_receive() // 读取MAVLINK数据
{
  mavlink_message_t msg;
  mavlink_status_t status;
  while (Serial2.available())
  {
    uint8_t c = Serial2.read();

    // Get new message
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) //逐字解析MAVLINK消息
    {

      // Handle new message from autopilot
      switch (msg.msgid)
      {

      case MAVLINK_MSG_ID_VFR_HUD:
      {
        mavlink_vfr_hud_t packet;
        mavlink_msg_vfr_hud_decode(&msg, &packet);
        air_speed = packet.airspeed;    // 空速
        GPS_speed = packet.groundspeed; // 地速
        alt = packet.alt;               // 高度
      }
      break;

      case MAVLINK_MSG_ID_ATTITUDE:
      {
        mavlink_attitude_t packet;
        mavlink_msg_attitude_decode(&msg, &packet);
        roll_converted = packet.roll * 57.3;   // 横滚
        pitch_converted = packet.pitch * 57.3; // 俯仰
        yaw_converted = packet.yaw * 57.3;     // 偏航
        if (yaw_converted < 0 && yaw_converted > -180)
          yaw_converted = yaw_converted + 360;
      }
      break;

      case MAVLINK_MSG_ID_RAW_IMU:
      {
        __mavlink_raw_imu_t packet;
        mavlink_msg_raw_imu_decode(&msg, &packet);
        X_G = packet.xacc; // 加速度
        Y_G = packet.yacc;
        Z_G = packet.zacc;
        X_G = X_G / 1000;
        Y_G = Y_G / 1000;
        Z_G = Z_G / 1000;
      }
      break;

      case MAVLINK_MSG_ID_DISTANCE_SENSOR:
      {
        __mavlink_distance_sensor_t packet;
        mavlink_msg_distance_sensor_decode(&msg, &packet);
        sensor_distance = packet.current_distance; // 下视传感器读数，CM
      }
      break;
        /*
        case MAVLINK_MSG_ID_HEARTBEAT:  // #0: Heartbeat
            {
            Serial.println("MAVLink working.");
            }
            break;
        */
      }
    }
  }
}
/*ESP32开发板与1.69寸TFT LCD屏的接线顺序
CS D15
DC D2
RES D4
SDA D23
SCL D18
*/

#include <Arduino.h>
#include "../.pio/libdeps/esp32dev/c_library_v2-master/all/mavlink.h"
#include <math.h>
//------------------------------------------------------------------
// #include <TFT_eSPI.h>
//------------------------------------------------------------------
#include <LovyanGFX.hpp>
#include <lgfx_user/LGFX_ESP32_sample.hpp>
#include <LGFX_TFT_eSPI.hpp>
//------------------------------------------------------------------

#include <SPI.h>

//------------------------------------------------------------------
static LGFX tft;
static LGFX_Sprite img(&tft);
static LGFX_Sprite artificical_horizon(&tft);
// #define LGFX_AUTODETECT //自动识别板型
//------------------------------------------------------------------

float roll_converted = 0;  // 横滚
float pitch_converted = 0; // 俯仰
float yaw_converted = 0;   // 偏航
float X_G = 0;             // X加速度 米每秒
float Y_G = 0;             // Y加速度
float Z_G = 0;             // Z加速度
float Acc = 0;             // 合加速度 G
int sensor_distance = 600;
int GPS_speed = 0;  // 地速
int air_speed = -2; // 空速
int D_Line = 5;     // 分化线比例
int D_Hight = 45;   // 分化线间距
int W_Line = 10;    // 分化线比例
int W_Hight = 60;   // 分化线间距(像素)
int air_Height = 50;
int air_Line = 10;
int air_short_Line = 5;
int alt_Height = 50;
int alt_Line = 10;
int alt_short_Line = 5;
int alt = 0;
float climb = 0;
float heading = 0;
uint16_t throttle = 0;
// mavlink参数设置
uint8_t _system_id = 255;      // id of computer which is sending the command (ground control software has id of 255)
uint8_t _component_id = 0;     // seems like it can be any # except the number of what Pixhawk sys_id is
uint8_t _target_system = 1;    // Id #
uint8_t _target_component = 0; // Target component, 0 = all (seems to work with 0 or 1
uint16_t _command = MAV_CMD_REQUEST_MESSAGE;
uint16_t _command_1 = MAV_CMD_SET_MESSAGE_INTERVAL;
uint8_t _confirmation = 0;

unsigned long int runTime = 0; // 从系统上电开始的运行时间
int LOOP_DELAY = 40;           // 每多少毫秒系统更新一次状态

int horizon_line_y = 0;

int last_roll = 0;
int last_pitch = 0;

// Variables for test only
int test_roll = 0;
int delta = 0;

unsigned long redrawTime = 0;

//------------------------------------------------------------------
// TFT_eSPI tft = TFT_eSPI();
// TFT_eSprite artificical_horizon = TFT_eSprite(&tft);
// TFT_eSprite img = TFT_eSprite(&tft);
//------------------------------------------------------------------

void set_request_datastream(int com_id, int D_T)
{
  mavlink_message_t msg;
  uint8_t *buf;
  uint16_t len = mavlink_msg_command_long_pack(_system_id, _component_id, &msg, _target_system, _target_component, _command_1, _confirmation, com_id, D_T, 0, 0, 0, 0, 0);
  buf = new uint8_t[len];
  mavlink_msg_to_send_buffer(buf, &msg); // Send the message (.write sends as bytes)
  Serial2.write(buf, len);               // Write data to serial port
}
void request_datastream()
{
  mavlink_message_t msg;
  uint8_t *buf;
  uint16_t len = mavlink_msg_command_long_pack(_system_id, _component_id, &msg, _target_system, _target_component, _command, _confirmation, 30, 0, 0, 0, 0, 0, 0);
  buf = new uint8_t[len];
  mavlink_msg_to_send_buffer(buf, &msg); // Send the message (.write sends as bytes)
  Serial2.write(buf, len);               // Write data to serial port
}

// function called by arduino to read any MAVlink messages sent by serial communication from flight controller to arduino
void MavLink_receive() // MAVLINK READ
{
  mavlink_message_t msg;
  mavlink_status_t status;
  while (Serial2.available())
  {
    uint8_t c = Serial2.read();

    // Get new message
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
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

void artificial_horizon_function() // 水平仪
{
  artificical_horizon.fillRect(0, 0, 320, 240, TFT_BLACK);                          // 屏幕重置为黑色
  horizon_line_y = pitch_converted * D_Hight / D_Line + 120;                        // 像素偏移距离，地平线初始高度转换
  artificical_horizon.drawLine(80, horizon_line_y, 140, horizon_line_y, TFT_GREEN); // 中心线
  artificical_horizon.drawLine(140, horizon_line_y, 140, horizon_line_y + 4, TFT_GREEN);
  artificical_horizon.drawLine(180, horizon_line_y, 180, horizon_line_y + 4, TFT_GREEN);
  artificical_horizon.drawLine(180, horizon_line_y, 240, horizon_line_y, TFT_GREEN);
  int Maddle = (horizon_line_y - 120) / D_Hight;
  int Top = min(Maddle + 3, 90 / D_Line);
  int Down = max(Maddle - 4, -90 / D_Line);
  artificical_horizon.setTextDatum(middle_center);
  artificical_horizon.setTextColor(TFT_GREEN);
  artificical_horizon.setTextSize(1.5);
  for (int i = Down; i <= Top; i++) // 向上取整
  {
    int horizon_l_y = horizon_line_y - i * D_Hight;
    if (i > 0)
    {
      artificical_horizon.drawLine(106, horizon_l_y, 140, horizon_l_y, TFT_GREEN);
      artificical_horizon.drawLine(140, horizon_l_y, 140, horizon_l_y + 8, TFT_GREEN);
      artificical_horizon.drawLine(180, horizon_l_y, 214, horizon_l_y, TFT_GREEN);
      artificical_horizon.drawLine(180, horizon_l_y, 180, horizon_l_y + 8, TFT_GREEN);
      artificical_horizon.drawNumber(i * D_Line, 93, horizon_l_y);
      artificical_horizon.drawNumber(i * D_Line, 229, horizon_l_y);
    }
    else if (i < 0)
    {
      artificical_horizon.drawLine(106, horizon_l_y, 112, horizon_l_y, TFT_GREEN);
      artificical_horizon.drawLine(118, horizon_l_y, 125, horizon_l_y, TFT_GREEN);
      artificical_horizon.drawLine(132, horizon_l_y, 140, horizon_l_y, TFT_GREEN);
      artificical_horizon.drawLine(140, horizon_l_y, 140, horizon_l_y - 8, TFT_GREEN);
      artificical_horizon.drawLine(180, horizon_l_y, 188, horizon_l_y, TFT_GREEN);
      artificical_horizon.drawLine(195, horizon_l_y, 202, horizon_l_y, TFT_GREEN);
      artificical_horizon.drawLine(208, horizon_l_y, 214, horizon_l_y, TFT_GREEN);
      artificical_horizon.drawLine(180, horizon_l_y, 180, horizon_l_y - 8, TFT_GREEN);
      artificical_horizon.drawNumber(-i * D_Line, 93, horizon_l_y);
      artificical_horizon.drawNumber(-i * D_Line, 229, horizon_l_y);
    }
  }

  artificical_horizon.pushRotated(&img, -roll_converted); // 浮点偏转
}

// 红绿渐变处理函数
void color_gradual_change(float value, int upper_limit, int lower_limit)
{
  if (value > upper_limit)
    img.setTextColor(TFT_GREEN);
  if (value <= upper_limit && value >= (upper_limit + lower_limit) / 2)
  {
    uint16_t airspeed_color = tft.color565(map(value, (upper_limit + lower_limit) / 2, upper_limit, 255, 0), 255, 0);
    img.setTextColor(airspeed_color);
  }
  if (value >= lower_limit && value <= (upper_limit + lower_limit) / 2)
  {
    uint16_t airspeed_color = tft.color565(255, map(value, lower_limit, (upper_limit + lower_limit) / 2, 0, 255), 0);
    img.setTextColor(airspeed_color);
  }
  if (value < lower_limit)
    img.setTextColor(TFT_RED);
}

void air_speed_display(int cruise_speed, int stall_speed)
{
  if (air_speed == -2)
  {
    air_speed = GPS_speed;
  }
  int air_speed_KM = air_speed * 60 * 60 / 1000;
  color_gradual_change(air_speed, cruise_speed, stall_speed); // 空速颜色渐变处理
  img.setTextSize(1.5);
  img.setTextDatum(middle_right);
  img.drawNumber(air_speed_KM, 52, 121);
  img.drawLine(60, 120, 52, 111, TFT_GREEN);
  img.drawLine(60, 120, 52, 128, TFT_GREEN);
  img.drawLine(20, 128, 20, 111, TFT_GREEN);
  img.drawLine(20, 111, 52, 111, TFT_GREEN);
  img.drawLine(20, 128, 52, 128, TFT_GREEN);
  int air_speed_y = air_speed_KM * air_Height / air_Line + 120;
  int air_Meddle = air_speed_KM / air_Line;
  int Down = max(0, air_Meddle - 5);
  for (int i = Down; i < air_Meddle + 3; i++)
  {
    int air_speed_line = air_speed_y - i * air_Height;
    if (air_speed_line > 190)
      continue;
    if (air_speed_line < 50)
      break;
    img.drawLine(60, air_speed_line, 70, air_speed_line);
    img.drawNumber(i * air_Line, 58, air_speed_line);
  }
  for (int i = Down; i < air_Meddle + 3; i++)
  {
    int air_speed_line = air_speed_y - i * air_Height;

    for (int i = 1; i < 5; i++)
    {
      int air_speed_short_line = air_speed_line - i * 10;
      if (air_speed_short_line > 190)
        continue;
      if (air_speed_short_line < 50)
        break;
      img.drawLine(65, air_speed_short_line, 70, air_speed_short_line);
    }
  }
  img.setTextColor(TFT_GREEN);
}

void alt_display()
{
  img.setTextColor(TFT_GREEN);
  img.setTextSize(1.5);
  img.setTextDatum(middle_right);
  img.drawChar('R', 251, 195);
  if (sensor_distance > 540)
  {
    img.drawString("GPS", 290, 201);
  }
  else
  {
    img.drawNumber(sensor_distance, 290, 201);
  }
  img.setTextDatum(middle_left);
  img.drawNumber(alt, 268, 121);
  img.drawLine(260, 120, 269, 111, TFT_GREEN);
  img.drawLine(260, 120, 268, 128, TFT_GREEN);
  img.drawLine(299, 128, 299, 111, TFT_GREEN);
  img.drawLine(299, 111, 269, 111, TFT_GREEN);
  img.drawLine(299, 128, 268, 128, TFT_GREEN);

  img.drawLine(260, 193, 260, 207, TFT_GREEN);
  img.drawLine(260, 193, 290, 193, TFT_GREEN);
  img.drawLine(290, 193, 290, 207, TFT_GREEN);
  img.drawLine(290, 207, 260, 207, TFT_GREEN);
  int alt_speed_y = alt * alt_Height / alt_Line + 120;
  int alt_Meddle = alt / alt_Line;
  int Down = max(0, alt_Meddle - 5);
  for (int i = Down; i < alt_Meddle + 3; i++)
  {
    int alt_speed_line = alt_speed_y - i * alt_Height;
    if (alt_speed_line > 190)
      continue;
    if (alt_speed_line < 50)
      break;
    img.drawLine(250, alt_speed_line, 260, alt_speed_line);
    img.drawNumber(i * alt_Line, 262, alt_speed_line);
  }
  for (int i = Down; i < alt_Meddle + 3; i++)
  {
    int alt_speed_line = alt_speed_y - i * alt_Height;

    for (int i = 1; i < 5; i++)
    {
      int alt_speed_short_line = alt_speed_line - i * 10;
      if (alt_speed_short_line > 190)
        continue;
      if (alt_speed_short_line < 50)
        break;
      img.drawLine(250, alt_speed_short_line, 255, alt_speed_short_line);
    }
  }
}

void information_display() // 固定位置信息显示
{
  int cruise_speed = 20;
  int stall_speed = 10;
  int yaw_y = 25;
  // img.fillRect(0, 0, 60, 240, TFT_BLACK);    // 左区域重置为黑色
  // img.fillRect(300, 0, 300, 240, TFT_BLACK); // 右定区域重置为黑色
  //  img.fillRect(0, 0, 320, 40, TFT_BLACK);    // 上区域重置为黑色
  //  img.fillRect(0, 200, 320, 240, TFT_BLACK); // 下定区域重置为黑色
  //   两侧竖线
  //   X/Y逆
  //  img.drawLine(60, 20, 60, 220, TFT_GREEN);
  //  img.drawLine(260, 20, 260, 220, TFT_GREEN);
  //   中心图形绘制
  img.drawLine(150, 130, 160, 120, TFT_GREEN);
  img.drawLine(170, 130, 160, 120, TFT_GREEN);
  img.drawLine(150, 130, 130, 130, TFT_GREEN);
  img.drawLine(170, 130, 190, 130, TFT_GREEN);

  air_speed_display(cruise_speed, stall_speed);
  alt_display();
  Acc = sqrt(X_G * X_G + Y_G * Y_G + Z_G * Z_G);
  img.setCursor(30, 205);
  img.printf("G %0.1f", Acc);
  // img.print(alt);
  //  img.setCursor(270, 100);//设置光标位置
  //  img.print(heading);

  img.setTextSize(1.2);
  heading = yaw_converted * W_Hight / W_Line - 160; // 偏航屏库坐标
  img.setTextDatum(middle_center);
  int R_Line = yaw_converted / W_Line;
  for (int i = R_Line - 2; i <= R_Line + 2; i++)
  {
    int heading_x = i * W_Hight - heading;
    if (heading_x < 250 && heading_x > 70)
    {
      img.drawLine(heading_x, 7, heading_x, 15, TFT_GREEN);
      if (i < 0)
      {
        img.drawNumber(36 + i % 36, heading_x, 25);
      }
      else
      {
        img.drawNumber(i % 36, heading_x, 25);
      }
    }
  }
  for (int i = R_Line - 2; i <= R_Line + 2; i++)
  {
    int heading_x = i * W_Hight - heading + 30;
    if (heading_x < 250 && heading_x > 70)
    {
      img.drawLine(heading_x, 7, heading_x, 11, TFT_GREEN);
    }
  }
}

void setup()
{
  setCpuFrequencyMhz(240);
  Serial.begin(38400);  // Main serial port for console output
  Serial2.begin(57600); // RXTX (Port TX17,RX16 ESP32)
  // Serial1.begin(115200, SERIAL_8N1, 18, 19)
  set_request_datastream(27, 100000);
  set_request_datastream(30, 40000);
  tft.setRotation(1);
  tft.setPivot(120, 160);
  tft.init();

  tft.fillScreen(TFT_BLACK);

  img.setColorDepth(8);
  img.createSprite(320, 240);
  img.fillScreen(TFT_BLACK);

  artificical_horizon.setColorDepth(8);
  artificical_horizon.createSprite(320, 240);
  artificical_horizon.fillScreen(TFT_BLACK);
}

void loop()
{
  MavLink_receive();
  if (millis() - runTime >= LOOP_DELAY)
  {
    // request_datastream();
    img.fillRect(0, 0, 320, 240, TFT_BLACK); // 屏幕重置为黑色
    artificial_horizon_function();
    information_display();
    img.pushSprite(0, 0);
  }
}

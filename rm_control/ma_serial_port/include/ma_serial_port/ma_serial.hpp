#ifndef __MA_SERIAL_H__
#define __MA_SERIAL_H__

#include <serial/serial.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

namespace ma_serial_packet{

/**
 * SendPacket
 * 
*/
struct SendPacket{
    uint8_t head = 0x5A;
    float linear_x;
    float linear_y;
    float angular_z;
    uint8_t navigating;
    uint16_t checksum = 0;
}__attribute__((packed));


/***
 * @details: Receive odometry msg.
 * @param: head, v_x, v_y, v_z, p_x, p_y, p_z, checksum
*/
struct ReceivePacket{
   uint8_t head = 0x5A;
   uint8_t robot_id;
   uint16_t sentry_HP;
   uint16_t red_outpost_HP;
   uint16_t blue_outpost_HP;
   uint16_t game_time;
   uint16_t checksum = 0;
}__attribute__((packed));

/**
 * Alter vector above into struct and utilize it.
*/
inline ReceivePacket fromVector(const std::vector<uint8_t> & data)
{
  ReceivePacket packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

/**
 * Alter struct object above into vector and utilize it.
*/
inline std::vector<uint8_t> toVector(const SendPacket & data)
{
  std::vector<uint8_t> packet(sizeof(SendPacket));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacket), packet.begin());
  return packet;
}


}

#endif
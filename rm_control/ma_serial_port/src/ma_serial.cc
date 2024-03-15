#include "ma_serial_port/crc.hpp"
#include "ma_serial_port/ma_serial.hpp"

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt8.h>

#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>

#include <string>
#include <thread>

serial::Serial ser;
std_msgs::UInt8 navigating;
std::thread receive_thread_;

using ReceivePacket = ma_serial_packet::ReceivePacket;


void cmd_velCallBack(const geometry_msgs::TwistStamped::ConstPtr& msg){
    
    try{
        ma_serial_packet::SendPacket packet;
        packet.head = 0x5A;
        packet.linear_x = msg->twist.linear.x;
        packet.linear_y = -msg->twist.linear.y;
        packet.angular_z = msg->twist.angular.z;
        packet.navigating = navigating.data;

        crc16::append_crc16_checksum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
        
        ROS_INFO("Current msg: v_x: %f, v_y: %f, a_z: %f, nav: %d", msg->twist.linear.x, msg->twist.linear.y, msg->twist.angular.z, navigating.data);

        if(navigating.data == 0) ROS_INFO_STREAM("\033[1;32m Navigating.\033");
        else ROS_INFO_STREAM("\033[1;33m Waiting for navigating.\033");
        
        uint8_t* packet_ptr = reinterpret_cast<uint8_t*>(&packet);

        ser.write(packet_ptr, sizeof(packet));
        
    }catch(const std::exception& ex){
        ROS_INFO("Error when sending data: %s", ex.what());
    }
}

void nav_callBack(const std_msgs::UInt8::ConstPtr& msg){
    navigating.data = msg->data;
    ROS_INFO("Current state is: %d", navigating.data);
}

void receiveData()
{
  std::vector<uint8_t> header(1);
  std::vector<uint8_t> data;
  tf::TransformBroadcaster tf_broadcaster;


  data.reserve(sizeof(ReceivePacket));

  while (ros::ok()) {
    try {
        ser.read(header.data(), 1);

      if (header[0] == 0x5A) {
        data.resize(sizeof(ReceivePacket) - 1);
        ser.read(data.data(), sizeof(ReceivePacket));

        data.insert(data.begin(), header[0]);
        ReceivePacket packet = ma_serial_packet::fromVector(data);

        bool crc_ok =
          crc16::verify_crc16_checksum(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
        if (crc_ok) {
        //    ROS_INFO("Current hp is: %d", packet.sentry_hp);
        } else {
          ROS_ERROR("CRC error!");
        }
      } else {
        // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "Invalid header: %02X", header[0]);
      }
    } catch (const std::exception & ex) {

    }
  }
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "ma_serial");  // Initialize ROS
    ros::NodeHandle nh("~");  // Node handle

    std::string port;
    std::string topic;
    int baud_rate;
    int time_out;
    bool debug;
    
    if (nh.getParam("/ma_serial/port", port) 
        && nh.getParam("/ma_serial/baud_rate", baud_rate)
        && nh.getParam("/ma_serial/time_out", time_out)
        && nh.getParam("/ma_serial/topic", topic)
        && nh.getParam("/ma_serial/debug", debug)) {
        ROS_INFO("Current port name is %s, baud rate is %d.", port.c_str(), baud_rate);
    } else {
        ROS_ERROR("Failed to retrieve the port parameter.");
        return 1;  // Exit with an error code
    }

    try {
        ser.setPort(port);
        ser.setBaudrate(baud_rate);
        serial::Timeout to = serial::Timeout::simpleTimeout(time_out);
        ser.setTimeout(to);
        ser.open();
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open the serial port: " << e.what());
        return 1;
    }

if (ser.isOpen()) {
        ROS_INFO_STREAM("Serial port initialized.");
        receive_thread_ = std::thread(receiveData);
    }else{
        return -1;
    }

    ros::Publisher pub = nh.advertise<std_msgs::UInt16>("/ma_vel", 50);
    ros::Subscriber sub = nh.subscribe<geometry_msgs::TwistStamped>("/cmd_vel", 100, cmd_velCallBack);
    ros::Subscriber stop_sub = nh.subscribe<std_msgs::Int8>("/stop", 10, nav_callBack);
    tf::TransformBroadcaster tf_broadcaster;

    while(ros::ok())
        ros::spinOnce();

    return 0;
}

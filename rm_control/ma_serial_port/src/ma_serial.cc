#include "ma_serial_port/crc.hpp"
#include "ma_serial_port/ma_serial.hpp"

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/TwistStamped.h>

#include <tf/transform_broadcaster.h>

#include <string>
#include <thread>
#include <vector>

serial::Serial ser;
std_msgs::Int8 navigating;
std::thread receive_thread_;

using ReceivePacket = ma_serial_packet::ReceivePacket;

std_msgs::Int8 robot_id_msg;
std_msgs::UInt16 red_outpost_HP_msg;
std_msgs::UInt16 blue_outpost_HP_msg;
std_msgs::UInt16 game_time_msg;

void cmd_velCallBack(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    try {
        ma_serial_packet::SendPacket packet;
        packet.head = 0x5A;
        packet.linear_x = msg->twist.linear.x;
        packet.linear_y = -msg->twist.linear.y;
        packet.angular_z = msg->twist.angular.z;
        packet.navigating = navigating.data;

        crc16::append_crc16_checksum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
        
        ROS_INFO("Current msg: v_x: %f, v_y: %f, a_z: %f, nav: %d", 
                 msg->twist.linear.x, msg->twist.linear.y, msg->twist.angular.z, navigating.data);

        if(navigating.data == 0) 
            ROS_INFO_STREAM("\033[1;32m Navigating.\033");
        else 
            ROS_INFO_STREAM("\033[1;33m Waiting for navigating.\033");
        
        ser.write(reinterpret_cast<uint8_t*>(&packet), sizeof(packet));
    } catch(const std::exception& ex) {
        ROS_ERROR("Error when sending data: %s", ex.what());
    }
}

void nav_callBack(const std_msgs::Int8::ConstPtr& msg) {
    navigating.data = msg->data;
    // ROS_INFO("Current state is: %d", navigating.data);
}

void receiveData() {
    std::vector<uint8_t> header(1);
    std::vector<uint8_t> data;
    tf::TransformBroadcaster tf_broadcaster;

    data.reserve(sizeof(ReceivePacket));

    while (ros::ok()) {
        try {
            if (ser.read(header.data(), 1) && header[0] == 0x5A) {
                data.resize(sizeof(ReceivePacket) - 1);
                ser.read(data.data(), data.size());
                data.insert(data.begin(), header[0]);

                ReceivePacket* packet = reinterpret_cast<ReceivePacket*>(data.data());

                if (crc16::verify_crc16_checksum(reinterpret_cast<uint8_t*>(packet), sizeof(ReceivePacket))) {
                    // Process packet data
                    robot_id_msg.data = packet->robot_id;
                    red_outpost_HP_msg.data = packet->red_outpost_HP;
                    blue_outpost_HP_msg.data = packet->blue_outpost_HP;
                    game_time_msg.data = packet->game_time;
                    // ROS_INFO("robot_id_msg: %u, red_outpost_HP_msg: %u, blue_outpost_HP_msg: %u, game_time_msg: %u",robot_id_msg.data, red_outpost_HP_msg.data, blue_outpost_HP_msg.data, game_time_msg.data);
                    
                } else {
                    ROS_ERROR("CRC error!");
                }
            }
        } catch (const std::exception &ex) {
            ROS_ERROR("Error in receiving data: %s", ex.what());
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ma_serial");
    ros::NodeHandle nh("~");

    ros::Publisher robot_id_pub = nh.advertise<std_msgs::Int8>("/robot_id",50);
    ros::Publisher red_outpost_HP_pub = nh.advertise<std_msgs::UInt16>("/red_outpost_HP",50);
    ros::Publisher blue_outpost_HP_pub = nh.advertise<std_msgs::UInt16>("/blue_outpost_HP",50);
    ros::Publisher game_time_pub = nh.advertise<std_msgs::UInt16>("/game_time",50);

    std::string port;
    int baud_rate, time_out;
    bool debug;

    navigating.data = 0; // Initialize with a default value

    if (nh.getParam("/ma_serial/port", port) &&
        nh.getParam("/ma_serial/baud_rate", baud_rate) &&
        nh.getParam("/ma_serial/time_out", time_out) &&
        nh.getParam("/ma_serial/debug", debug)) {
        ROS_INFO("Current port name is %s, baud rate is %d.", port.c_str(), baud_rate);
    } else {
        ROS_ERROR("Failed to retrieve the port parameter.");
        return 1;
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
        ROS_INFO("Serial port initialized.");
        receive_thread_ = std::thread(receiveData);
    } else {
        return -1;
    }

    
    ros::Publisher pub = nh.advertise<std_msgs::UInt16>("/ma_vel", 50);
    ros::Subscriber sub = nh.subscribe<geometry_msgs::TwistStamped>("/cmd_vel_filtered", 100, cmd_velCallBack);
    ros::Subscriber stop_sub = nh.subscribe<std_msgs::Int8>("/stop", 10, nav_callBack);
    //ROS_INFO("robot_id_msg: %u, red_outpost_HP_msg: %u, blue_outpost_HP_msg: %u, game_time_msg: %u",robot_id_msg.data, red_outpost_HP_msg.data, blue_outpost_HP_msg.data, game_time_msg.data);
    ros::Rate rate(100);
    while(ros::ok)
    {
        robot_id_pub.publish(robot_id_msg);
        //ROS_INFO("robot_id : %u", robot_id_msg.data);
        red_outpost_HP_pub.publish(red_outpost_HP_msg);
        blue_outpost_HP_pub.publish(blue_outpost_HP_msg);
        game_time_pub.publish(game_time_msg);

        ros::spinOnce();  // 处理一次回调
        rate.sleep();     // 根据Rate对象设定的频率休眠
    }
    


    return 0;
}

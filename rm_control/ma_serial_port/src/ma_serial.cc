#include "ma_serial_port/crc.hpp"
#include "ma_serial_port/ma_serial.hpp"

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/UInt16.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>

#include <string>


serial::Serial ser;

void cmd_velCallBack(const geometry_msgs::Twist::ConstPtr& msg){
    
    try{
        ma_serial_packet::SendPacket packet;
        packet.head = 0x5A;
        packet.linear_x = msg->linear.x;
        packet.linear_y = msg->linear.y;
        packet.angular_z = msg->angular.z;

        crc16::append_crc16_checksum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
        

        ROS_INFO("Current msg: v_x: %f, v_y: %f, a_z: %f", msg->linear.x, msg->linear.y, msg->angular.z);

        // std::vector<uint8_t> data = ma_serial_packet::toVector(packet);
        
        uint8_t* packet_ptr = reinterpret_cast<uint8_t*>(&packet);

        

        ser.write(packet_ptr, sizeof(packet));
        

    }catch(const std::exception& ex){
        ROS_INFO("Error when sending data: %s", ex.what());
        // Reopen port here
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
    }else{
        return -1;
    }

    ros::Publisher pub = nh.advertise<std_msgs::UInt16>("ma_vel", 50);
    ros::Subscriber sub = nh.subscribe("/cmd_vel", 1000, cmd_velCallBack);
    tf::TransformBroadcaster tf_broadcaster;

    ros::spin();

    return 0;
}

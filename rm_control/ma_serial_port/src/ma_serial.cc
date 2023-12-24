#include "ma_serial_port/crc.hpp"
#include "ma_serial_port/ma_serial.hpp"

#include <ros/ros.h>
#include <serial/serial.h>

#include <string>


ma_serial_packet::SendPacket d;

int main(int argc, char** argv) {

    ros::init(argc, argv, "ma_serial");  // Initialize ROS
    ros::NodeHandle nh("~");  // Node handle

    serial::Serial ser;
    std::string port;
    int baud_rate;
    int time_out;


    if (nh.getParam("/ma_serial/port", port) 
        && nh.getParam("/ma_serial/baud_rate", baud_rate)
        && nh.getParam("/ma_serial/time_out", time_out)) {
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


    }


    return 0;
}

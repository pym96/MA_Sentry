Errors     << ma_serial_port:make /home/dan/learn/MA_Sentry/logs/ma_serial_port/build.make.303.log
/home/dan/learn/MA_Sentry/src/rm_control/ma_serial_port/src/ma_serial.cc: In function ‘void receiveData()’:
/home/dan/learn/MA_Sentry/src/rm_control/ma_serial_port/src/ma_serial.cc:71:106: error: cannot call member function ‘ros::Publisher ros::NodeHandle::advertise(const string&, uint32_t, bool) [with M = std_msgs::UInt16_<std::allocator<void> >; std::string = std::__cxx11::basic_string<char>; uint32_t = unsigned int]’ without object
   71 |                     ros::Publisher hp_pub = ros::NodeHandle::advertise<std_msgs::UInt16>("/sentry_hp", 50);
      |                                                                                                          ^
In file included from /opt/ros/noetic/include/ros/serialization.h:37,
                 from /opt/ros/noetic/include/std_msgs/String.h:14,
                 from /home/dan/learn/MA_Sentry/src/rm_control/ma_serial_port/include/ma_serial_port/ma_serial.hpp:5,
                 from /home/dan/learn/MA_Sentry/src/rm_control/ma_serial_port/src/ma_serial.cc:2:
/opt/ros/noetic/include/ros/message_traits.h: In instantiation of ‘static const char* ros::message_traits::MD5Sum<M>::value(const M&) [with M = std::shared_ptr<std_msgs::UInt16_<std::allocator<void> > >]’:
/opt/ros/noetic/include/ros/message_traits.h:254:102:   required from ‘const char* ros::message_traits::md5sum(const M&) [with M = std::shared_ptr<std_msgs::UInt16_<std::allocator<void> > >]’
/opt/ros/noetic/include/ros/publisher.h:117:38:   required from ‘void ros::Publisher::publish(const M&) const [with M = std::shared_ptr<std_msgs::UInt16_<std::allocator<void> > >]’
/home/dan/learn/MA_Sentry/src/rm_control/ma_serial_port/src/ma_serial.cc:73:39:   required from here
/opt/ros/noetic/include/ros/message_traits.h:125:14: error: ‘const class std::shared_ptr<std_msgs::UInt16_<std::allocator<void> > >’ has no member named ‘__getMD5Sum’
  125 |     return m.__getMD5Sum().c_str();
      |            ~~^~~~~~~~~~~
/opt/ros/noetic/include/ros/message_traits.h: In instantiation of ‘static const char* ros::message_traits::DataType<M>::value(const M&) [with M = std::shared_ptr<std_msgs::UInt16_<std::allocator<void> > >]’:
/opt/ros/noetic/include/ros/message_traits.h:263:104:   required from ‘const char* ros::message_traits::datatype(const M&) [with M = std::shared_ptr<std_msgs::UInt16_<std::allocator<void> > >]’
/opt/ros/noetic/include/ros/publisher.h:119:11:   required from ‘void ros::Publisher::publish(const M&) const [with M = std::shared_ptr<std_msgs::UInt16_<std::allocator<void> > >]’
/home/dan/learn/MA_Sentry/src/rm_control/ma_serial_port/src/ma_serial.cc:73:39:   required from here
/opt/ros/noetic/include/ros/message_traits.h:142:14: error: ‘const class std::shared_ptr<std_msgs::UInt16_<std::allocator<void> > >’ has no member named ‘__getDataType’
  142 |     return m.__getDataType().c_str();
      |            ~~^~~~~~~~~~~~~
In file included from /opt/ros/noetic/include/std_msgs/String.h:14,
                 from /home/dan/learn/MA_Sentry/src/rm_control/ma_serial_port/include/ma_serial_port/ma_serial.hpp:5,
                 from /home/dan/learn/MA_Sentry/src/rm_control/ma_serial_port/src/ma_serial.cc:2:
/opt/ros/noetic/include/ros/serialization.h: In instantiation of ‘static uint32_t ros::serialization::Serializer<T>::serializedLength(typename boost::call_traits<T>::param_type) [with T = std::shared_ptr<std_msgs::UInt16_<std::allocator<void> > >; uint32_t = unsigned int; typename boost::call_traits<T>::param_type = const std::shared_ptr<std_msgs::UInt16_<std::allocator<void> > >&]’:
/opt/ros/noetic/include/ros/serialization.h:180:41:   required from ‘uint32_t ros::serialization::serializationLength(const T&) [with T = std::shared_ptr<std_msgs::UInt16_<std::allocator<void> > >; uint32_t = unsigned int]’
/opt/ros/noetic/include/ros/serialization.h:815:37:   required from ‘ros::SerializedMessage ros::serialization::serializeMessage(const M&) [with M = std::shared_ptr<std_msgs::UInt16_<std::allocator<void> > >]’
/opt/ros/noetic/include/ros/publisher.h:126:26:   required from ‘void ros::Publisher::publish(const M&) const [with M = std::shared_ptr<std_msgs::UInt16_<std::allocator<void> > >]’
/home/dan/learn/MA_Sentry/src/rm_control/ma_serial_port/src/ma_serial.cc:73:39:   required from here
/opt/ros/noetic/include/ros/serialization.h:145:14: error: ‘const class std::shared_ptr<std_msgs::UInt16_<std::allocator<void> > >’ has no member named ‘serializationLength’
  145 |     return t.serializationLength();
      |            ~~^~~~~~~~~~~~~~~~~~~
/opt/ros/noetic/include/ros/serialization.h: In instantiation of ‘static void ros::serialization::Serializer<T>::write(Stream&, typename boost::call_traits<T>::param_type) [with Stream = ros::serialization::OStream; T = std::shared_ptr<std_msgs::UInt16_<std::allocator<void> > >; typename boost::call_traits<T>::param_type = const std::shared_ptr<std_msgs::UInt16_<std::allocator<void> > >&]’:
/opt/ros/noetic/include/ros/serialization.h:155:23:   required from ‘void ros::serialization::serialize(Stream&, const T&) [with T = std::shared_ptr<std_msgs::UInt16_<std::allocator<void> > >; Stream = ros::serialization::OStream]’
/opt/ros/noetic/include/ros/serialization.h:822:12:   required from ‘ros::SerializedMessage ros::serialization::serializeMessage(const M&) [with M = std::shared_ptr<std_msgs::UInt16_<std::allocator<void> > >]’
/opt/ros/noetic/include/ros/publisher.h:126:26:   required from ‘void ros::Publisher::publish(const M&) const [with M = std::shared_ptr<std_msgs::UInt16_<std::allocator<void> > >]’
/home/dan/learn/MA_Sentry/src/rm_control/ma_serial_port/src/ma_serial.cc:73:39:   required from here
/opt/ros/noetic/include/ros/serialization.h:128:7: error: ‘const class std::shared_ptr<std_msgs::UInt16_<std::allocator<void> > >’ has no member named ‘serialize’
  128 |     t.serialize(stream.getData(), 0);
      |     ~~^~~~~~~~~
make[2]: *** [CMakeFiles/ma_serial_port_node.dir/build.make:63: CMakeFiles/ma_serial_port_node.dir/src/ma_serial.cc.o] Error 1
make[1]: *** [CMakeFiles/Makefile2:278: CMakeFiles/ma_serial_port_node.dir/all] Error 2
make: *** [Makefile:141: all] Error 2
cd /home/dan/learn/MA_Sentry/build/ma_serial_port; catkin build --get-env ma_serial_port | catkin env -si  /usr/bin/make --jobserver-auth=3,4; cd -


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
                    auto msg = std::make_shared<std_msgs::UInt16>();
                    ros::Publisher hp_pub = ros::NodeHandle::advertise<std_msgs::UInt16>("/sentry_hp", 50);
                    msg->data = packet->sentry_hp; 
                    hp_pub.publish(msg);
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
    ros::Subscriber sub = nh.subscribe<geometry_msgs::TwistStamped>("/cmd_vel", 100, cmd_velCallBack);
    ros::Subscriber stop_sub = nh.subscribe<std_msgs::Int8>("/stop", 10, nav_callBack);

    ros::spin();

    return 0;
}

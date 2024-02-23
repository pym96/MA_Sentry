#include "ma_serial_port/crc.hpp"
#include "ma_serial_port/ma_serial_2d.hpp"

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/UInt16.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>

#include <string>
#include <thread>

serial::Serial ser;
std::thread receive_thread_;

using ma_serial_packet::ReceviePacket = ReceviePacket;

void cmd_velCallBack(const geometry_msgs::TwistStamped::ConstPtr& msg){
    
    try{
        ma_serial_packet::SendPacket packet;
        packet.head = 0x5A;
        packet.linear_x = msg->twist.linear.x;
        packet.linear_y = msg->twist.linear.y;
        packet.angular_z = msg->twist.angular.z;

        crc16::append_crc16_checksum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
        
        ROS_INFO("Current msg: v_x: %f, v_y: %f, a_z: %f", msg->twist.linear.x, msg->twist.linear.y, msg->twist.angular.z);

        uint8_t* packet_ptr = reinterpret_cast<uint8_t*>(&packet);

        ser.write(packet_ptr, sizeof(packet));
        
    }catch(const std::exception& ex){
        ROS_INFO("Error when sending data: %s", ex.what());
    }
}

void receiveData()
{
  std::vector<uint8_t> header(1);
  std::vector<uint8_t> data;

  data.reserve(sizeof(ReceviePacket));

  while (ros::ok()) {
    try {
        ser.read(header.data(), 1);

      if (header[0] == 0x5A) {
        data.resize(sizeof(ReceviePacket) - 1);
        ser.read(data.data(), sizeof(ReceviePacket))

        data.insert(data.begin(), header[0]);
        ReceivePacket packet = fromVector(data);

        bool crc_ok =
          crc16::verify_crc16_checksum(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
        if (crc_ok) {

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

int main(int argc, char** argv){

    ros::init(argc, argv, "ma_serial_receive");  // Initialize ROS
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
        receive_thread_ = std::thread(&receiveData, this);
    }else{
        return -1;
    }

    ros::Publisher pub = nh.advertise<std_msgs::UInt16>("/ma_vel", 50);
    ros::Subscriber sub = nh.subscribe("/msg_pub/cmd_vel", 1000, cmd_velCallBack);
    tf::TransformBroadcaster tf_broadcaster;

    while(ros::ok())
        ros::spinOnce();


    return 0;
}

```
Errors     << ma_serial_port:make /home/dan/learn/MA_Sentry/logs/ma_serial_port/build.make.091.log
/home/dan/learn/MA_Sentry/src/rm_control/ma_serial_port/src/ma_serial_2d_rec.cc:16:38: error: expected ‘;’ before ‘=’ token
   16 | using ma_serial_packet::ReceviePacket = ReceviePacket;
      |                                      ^~
      |                                      ;
/home/dan/learn/MA_Sentry/src/rm_control/ma_serial_port/src/ma_serial_2d_rec.cc:16:39: error: expected unqualified-id before ‘=’ token
   16 | using ma_serial_packet::ReceviePacket = ReceviePacket;
      |                                       ^
/home/dan/learn/MA_Sentry/src/rm_control/ma_serial_port/src/ma_serial_2d_rec.cc: In function ‘void receiveData()’:
/home/dan/learn/MA_Sentry/src/rm_control/ma_serial_port/src/ma_serial_2d_rec.cc:53:53: error: expected ‘;’ before ‘data’
   53 |         ser.read(data.data(), sizeof(ReceviePacket))
      |                                                     ^
      |                                                     ;
   54 | 
   55 |         data.insert(data.begin(), header[0]);
      |         ~~~~                                         
/home/dan/learn/MA_Sentry/src/rm_control/ma_serial_port/src/ma_serial_2d_rec.cc:56:9: error: ‘ReceivePacket’ was not declared in this scope
   56 |         ReceivePacket packet = fromVector(data);
      |         ^~~~~~~~~~~~~
/home/dan/learn/MA_Sentry/src/rm_control/ma_serial_port/src/ma_serial_2d_rec.cc:59:75: error: ‘packet’ was not declared in this scope
   59 |           crc16::verify_crc16_checksum(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
      |                                                                           ^~~~~~
/home/dan/learn/MA_Sentry/src/rm_control/ma_serial_port/src/ma_serial_2d_rec.cc: In function ‘int main(int, char**)’:
/home/dan/learn/MA_Sentry/src/rm_control/ma_serial_port/src/ma_serial_2d_rec.cc:109:53: error: invalid use of ‘this’ in non-member function
  109 |         receive_thread_ = std::thread(&receiveData, this);
      |                                                     ^~~~
make[2]: *** [CMakeFiles/ma_serial_port_receive.dir/build.make:63: CMakeFiles/ma_serial_port_receive.dir/src/ma_serial_2d_rec.cc.o] Error 1
make[1]: *** [CMakeFiles/Makefile2:224: CMakeFiles/ma_serial_port_receive.dir/all] Error 2
make: *** [Makefile:141: all] Error 2
cd /home/dan/learn/MA_Sentry/build/ma_serial_port; catkin build --get-env ma_serial_port | catkin env -si  /usr/bin/make --jobserver-auth=3,4; cd -

..............................................................................................
Failed     << ma_serial_port:make                    [ Exited with code 2 ]                   
Failed    <<< ma_serial_port                     

```

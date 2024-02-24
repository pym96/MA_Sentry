#include "ma_serial_port/crc.hpp"
#include "ma_serial_port/ma_serial_2d.hpp"

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/UInt16.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <string>
#include <thread>

serial::Serial ser;
std::thread receive_thread_;
tf::TransformBroadcaster tf_broadcaster;

ros::Publisher odom_pub;

using ReceivePacket = ma_serial_packet::ReceivePacket;


void cmd_velCallBack(const geometry_msgs::TwistStamped::ConstPtr& msg);

void receiveData();


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
        receive_thread_ = std::thread(receiveData);
    }else{
        return -1;
    }

    ros::Publisher pub = nh.advertise<std_msgs::UInt16>("/ma_vel", 50);
    ros::Subscriber sub = nh.subscribe("/msg_pub/cmd_vel", 1000, cmd_velCallBack);
    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 50);
    // ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 50);

    while(ros::ok())
        ros::spinOnce();


    return 0;
}

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
           ROS_INFO("Current v_x is: %f,"
          " v_y is: %f,"
          " yaw is: %f,"
          " x is: %f,"
          " y is: %f", packet.v_x, packet.v_y, packet.yaw, packet.p_x, packet.p_y);

          nav_msgs::Odometry odom;
          odom.header.stamp = ros::Time::now();
          odom.header.frame_id = "map";
          odom.child_frame_id = "odom";

          odom.twist.twist.linear.x = packet.v_x;
          odom.twist.twist.linear.y = packet.v_y;
          // TODO: Add z angular velocity.

          odom.pose.pose.position.x = packet.p_x;
          odom.pose.pose.position.y = packet.p_y;

          tf::Quaternion q;
          q.setRPY(0, 0, packet.yaw);
          tf::quaternionTFToMsg(q, odom.pose.pose.orientation);

          odom_pub.publish(odom);

          geometry_msgs::TransformStamped odom_trans;
          odom_trans.header.stamp = ros::Time::now();
          odom_trans.header.frame_id = "map";
          odom_trans.child_frame_id = "odom";

          odom_trans.transform.translation.x = packet.p_x;
          odom_trans.transform.translation.y = packet.p_y;  
          odom_trans.transform.translation.z = 0.0;

          tf2::Quaternion q_;
          q_.setRPY(0, 0, packet.yaw);
          odom_trans.transform.rotation.x = q_.x();
          odom_trans.transform.rotation.y = q_.y();
          odom_trans.transform.rotation.z = q_.z();
          odom_trans.transform.rotation.w = q_.w();
          
          // Publish tf transform
          tf_broadcaster.sendTransform(odom_trans);

        } else {
          ROS_ERROR("CRC error!");
        }
      } else {
        // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "Invalid header: %02X", header[0]);
      }
    } catch (const std::exception & ex) {

    }
  }
}process[msg_pub-1]: started with pid [64262]
[FATAL] [1708759957.379808170]: You must call ros::init() before creating the first NodeHandle
[ERROR] [1708759957.388108871]: [registerPublisher] Failed to contact master at [:0].  Retrying...

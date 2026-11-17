#ifndef __RANDOM_EXPLORE_HPP
#define __RANDOM_EXPLORE_HPP

#include <behaviortree_cpp_v3/action_node.h>
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Int8.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h> // 包含Point数据结构的头文件

#include <string>

class RandomExplore : public BT::SyncActionNode {
public:
    RandomExplore(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts() {
        return {};
    }

    BT::NodeStatus tick() override;

private:
    ros::NodeHandle nh_;
    ros::Publisher way_point_pub_;
    ros::Subscriber stop_sub_;    // 订阅停止信号
    ros::Subscriber goal_point_sub_;    // 订阅way_point信息
    ros::Subscriber odom_sub_;

    bool goal_reached_;  // 目标是否已达到的标志
    bool initial_position_set;  // 初始位置是否已设置的标志
    geometry_msgs::Point initial_position;  // 存储初始位置
    geometry_msgs::Point fixed_point_;

    void stop_callback(const std_msgs::Int8::ConstPtr& msg);  // 处理停止信号的回调函数
    void goalpoint_callback(const geometry_msgs::PointStamped::ConstPtr& msg);  // 处理里程计信息的回调函数
    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);

};

#endif // __RANDOM_EXPLORE_HPP
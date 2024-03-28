#include "ros/ros.h"

#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>

#include "rm_decision/final_decision.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "behavior_tree_robot");
    ros::NodeHandle nh;

    // 订阅 /current_hp 以更新血量

    BT::BehaviorTreeFactory factory;

    // 注册自定义节点
    factory.registerNodeType<MoveToPosition>("MoveToPosition");
    factory.registerNodeType<CheckHP>("CheckHP");
    factory.registerNodeType<Wait>("Wait");
    factory.registerNodeType<AlwaysRunning>("AlwaysRunning");

    // 创建行为树
    auto tree = factory.createTreeFromFile("/home/dan/learn/MA_Sentry/src/rm_control/rm_decision/config/final_tree.xml");

    ros::Rate rate(10);
    while (ros::ok()) {
        // 运行行为树
        tree.tickRoot();

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


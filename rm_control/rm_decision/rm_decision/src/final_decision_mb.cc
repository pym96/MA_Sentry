#include "ros/ros.h"

#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>

#include "rm_decision/final_decision_mb.hpp" // 假设这里包含所有自定义节点的定义

int main(int argc, char** argv) {
    ros::init(argc, argv, "behavior_tree_robot");
    ros::NodeHandle nh;

    BT::BehaviorTreeFactory factory;

    // Register custom nodes
    factory.registerNodeType<MoveAndCheckPosition>("MoveAndCheckPosition");
    // factory.registerNodeType<CheckHP>("CheckHP");
    // factory.registerNodeType<Wait>("Wait");
    // factory.registerNodeType<AlwaysRunning>("AlwaysRunning");
    // Assuming CheckReached is also a custom node you have defined, register it as well
    // factory.registerNodeType<CheckReached>("CheckReached");

    // Create the behavior tree from the XML configuration file
    // Make sure to specify the correct path to your XML file
    auto tree = factory.createTreeFromFile("/home/dan/learn/MA_Sentry/src/rm_control/rm_decision/config/final_mb_tree.xml");

    ros::Rate rate(10);
    while (ros::ok()) {
        // Tick the root of the behavior tree
        tree.tickRoot();


        ros::spinOnce();
        rate.sleep();

    }

    return 0;
}

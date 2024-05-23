#include "ros/ros.h"

#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>

#include "rm_decision/final_decision.hpp" // Make sure this includes the CheckReached node definition

int main(int argc, char** argv) {
    ros::init(argc, argv, "behavior_tree_robot");
    ros::NodeHandle nh;

    BT::BehaviorTreeFactory factory;

    // Register custom nodes
    factory.registerNodeType<ma_decision::bt::MoveAndCheckPosition>("MoveAndCheckPosition");
    factory.registerNodeType<ma_decision::bt::last_do>("last_do");

    // Create the behavior tree from the XML configuration file
    auto tree = factory.createTreeFromFile("/home/l/MA/src/rm_control/rm_decision/config/final_tree_2.xml");

    ros::Rate rate(10);
    while (ros::ok()) {
        // Tick the root of the behavior tree
        tree.tickRoot();
/*
        if(is_last)
        {
            return 0;
        }
*/
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
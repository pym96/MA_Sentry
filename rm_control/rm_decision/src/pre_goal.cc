#include "ros/ros.h"

#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>

#include "rm_decision/patrol.hpp"
#include "rm_decision/random_explore.hpp"


int main(int argc, char** argv){

    ros::init(argc, argv, "robot_decision");

    ros::NodeHandle nh;

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<PatrolAction>("RandomExplore");

    auto tree = factory.createTreeFromFile("../config/random_explore.xml");
    
    ros::Rate rate(10);
    while(ros::ok()){
        tree.tickRoot();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
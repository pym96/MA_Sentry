#include "rm_decision/random_explore.hpp"

#include <stdlib.h>
#include <time.h>

RandomExplore::RandomExplore(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config), initial_position_set(false) {

    nh_ = ros::NodeHandle("~");
    way_point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/way_point", 10);
    stop_sub_ = nh_.subscribe("/stop", 10, &RandomExplore::stop_callback, this);
    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/state_estimation", 5, &RandomExplore::odom_callback);
    goal_reached_ = false;
    srand(time(NULL));
}

void RandomExplore::odom_callback(const nav_msgs::Odometry::ConstPtr& msg){
    this->curr_x = msg->pose.pose.position.x;
    this->curr_y = msg->pose.pose.position.y;
}

void RandomExplore::stop_callback(const std_msgs::Int8::ConstPtr& msg){
    goal_reached_ = (msg->data == 1);
}

BT::NodeStatus RandomExplore::tick(){
    if(goal_reached_){
        return BT::NodeStatus::SUCCESS;
    }

    if (!initial_position_set) {
        // Assume current position is (0, 0) for the example, 
        // but you should retrieve the actual robot position here.
        initial_position.x = curr_x;
        initial_position.y = curr_y;
        initial_position_set = true;
    }

    // Generate a random point within a 5-meter radius from the initial position
    float angle = static_cast<float>(rand()) / RAND_MAX * 2 * M_PI;
    float radius = static_cast<float>(rand()) / RAND_MAX * 5;   
    geometry_msgs::PointStamped new_goal;
    new_goal.header.frame_id = "vehicle"; // Assuming the fixed frame is "map"
    new_goal.point.x = initial_position.x + cos(angle) * radius;
    new_goal.point.y = initial_position.y + sin(angle) * radius;
    new_goal.header.stamp = ros::Time::now();
    way_point_pub_.publish(new_goal);

    return BT::NodeStatus::FAILURE;
}

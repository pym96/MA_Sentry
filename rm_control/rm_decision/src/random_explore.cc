#include "rm_decision/random_explore.hpp"

#include <stdlib.h>
#include <time.h>

RandomExplore::RandomExplore(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config){

    way_point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/way_point", 10);
    stop_sub_ = nh_.subscribe("/stop", 10, &RandomExplore::stop_callback, this);
    goal_reached_ = false;
    srand(time(NULL));
}

void RandomExplore::stop_callback(const std_msgs::Int8::ConstPtr& msg){
    goal_reached_ = (msg->data == 1);
}

BT::NodeStatus RandomExplore::tick(){
    if(goal_reached_){
        return BT::NodeStatus::SUCCESS;
    }

    // Generate a random point within a 5-meter radius
    float angle = static_cast<float>(rand()) / RAND_MAX * 2 * M_PI;
    float radius = static_cast<float>(rand()) / RAND_MAX * 5;   
    geometry_msgs::PointStamped new_goal;
    new_goal.header.frame_id = "vehicle";
    new_goal.point.x = cos(angle) * radius;
    new_goal.point.y = sin(angle) * radius;
    new_goal.header.stamp = ros::Time::now();
    way_point_pub_.publish(new_goal);

    return BT::NodeStatus::RUNNING;
}

// void RandomExplore::halt(){
//     // Do something;
// }
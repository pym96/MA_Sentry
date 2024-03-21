#include "rm_decision/random_explore.hpp"

#include <stdlib.h>
#include <time.h>

RandomExplore::RandomExplore(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config), goal_reached_(false), initial_position_set(false) {

    nh_ = ros::NodeHandle("~");
    way_point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/way_point", 10);
    stop_sub_ = nh_.subscribe("/stop", 10, &RandomExplore::stop_callback, this);
    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/state_estimation", 5, &RandomExplore::odom_callback);
    srand(time(NULL));
}

void RandomExplore::odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
    if (!goal_reached_) {
        // 如果机器人还在导航中，不更新初始位置
        return;
    }

    // 如果已经到达目标位置，但初始位置尚未设置，则设置初始位置
    if (goal_reached_ && !initial_position_set) {
        initial_position.x = msg->pose.pose.position.x;
        initial_position.y = msg->pose.pose.position.y;
        initial_position_set = true;
    }
}

void RandomExplore::stop_callback(const std_msgs::Int8::ConstPtr& msg) {
    goal_reached_ = (msg->data == 1);
    if (goal_reached_) {
        // 一旦到达目标位置，重置初始位置设置，以便于下一次导航结束时更新位置
        initial_position_set = false;
    }
}

BT::NodeStatus RandomExplore::tick() {
    if (!goal_reached_) {
        // 如果还在导航中，直接返回失败，不执行随机探索
        return BT::NodeStatus::FAILURE;
    }

    if (!initial_position_set) {
        // 如果初始位置未设置，返回失败，不执行随机探索
        return BT::NodeStatus::FAILURE;
    }

    // 生成一个在初始位置周围5米半径内的随机点
    float angle = static_cast<float>(rand()) / RAND_MAX * 2 * M_PI;
    float radius = static_cast<float>(rand()) / RAND_MAX * 5;
    geometry_msgs::PointStamped new_goal;
    new_goal.header.frame_id = "vehicle";  // 使用"vehicle"作为参考坐标系
    new_goal.point.x = cos(angle) * radius;
    new_goal.point.y = sin(angle) * radius;
    new_goal.header.stamp = ros::Time::now();
    way_point_pub_.publish(new_goal);

    return BT::NodeStatus::RUNNING;  // 返回RUNNING状态，表示随机探索正在进行
}

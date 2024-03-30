#ifndef __FiNAL_DECISION_HPP__
#define __FiNAL_DECISION_HPP__

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt16.h>

class MoveToPosition : public BT::SyncActionNode
{
public:
    MoveToPosition(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {
            way_point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/way_point", 10);
        }

BT::NodeStatus tick() override {
    geometry_msgs::PointStamped target_position;
    float target_x;
    float target_y;
    float target_z;
    if (!getInput<float>("target_x", target_x) || 
        !getInput<float>("target_y", target_y) ||
        !getInput<float>("target_z", target_z)) {
        ROS_ERROR("Missing target_position input");
        return BT::NodeStatus::FAILURE;
    }

    target_position.header.stamp = ros::Time::now();
    target_position.header.frame_id = "map";
    target_position.point.x = target_x;
    target_position.point.y = target_y;
    target_position.point.z = target_z;

    ROS_INFO("Moving to target position - X: %f, Y: %f, Z: %f", target_x, target_y, target_z);
    
    way_point_pub_.publish(target_position);
        
    return BT::NodeStatus::SUCCESS;
}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<float>("target_x"),
                 BT::InputPort<float>("target_y"),
                 BT::InputPort<float>("target_z")};
    }

private:
    ros::Publisher way_point_pub_;
    ros::NodeHandle nh_;


};

class CheckHP : public BT::ConditionNode
{
public:
    CheckHP(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config), global_current_hp(600) // 初始化血量
    {
        // 在构造函数中订阅，确保只订阅一次
        hp_sub_ = nh_.subscribe("/sentry_hp", 5, &CheckHP::hp_callback, this);
    }

    BT::NodeStatus tick() override {
        uint16_t current_hp_copy;
        {
            std::lock_guard<std::mutex> lock(hp_mutex_);
            current_hp_copy = global_current_hp;
        }
        return current_hp_copy < 400 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    static BT::PortsList providedPorts() {
        return {};
    }

private:
    uint16_t global_current_hp;
    ros::NodeHandle nh_;
    ros::Subscriber hp_sub_;
    std::mutex hp_mutex_; // 用于保护 global_current_hp 的互斥锁

    void hp_callback(const std_msgs::UInt16::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(hp_mutex_);
        global_current_hp = msg->data;
    }
};


class Wait : public BT::SyncActionNode { public: Wait(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config) {}

BT::NodeStatus tick() override {
    int duration;
    // 获取 duration 输入
    getInput("duration", duration);
    ros::Duration(duration).sleep(); // 根据 duration 等待
    return BT::NodeStatus::SUCCESS;
}

static BT::PortsList providedPorts() {
    // 声明 duration 端口
    return { BT::InputPort<int>("duration") };
}

};

class AlwaysRunning : public BT::SyncActionNode { public: AlwaysRunning(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config) {}

BT::NodeStatus tick() override {
    return BT::NodeStatus::RUNNING; // 始终返回 RUNNING
}

static BT::PortsList providedPorts() {
    return {}; // 没有输入或输出端口
}

};

class CheckReached : public BT::ConditionNode
{
public:
    CheckReached(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config), has_reached(true) // assuming it starts as reached
    {
        goal_sub_ = nh_.subscribe<geometry_msgs::PointStamped>("/way_point", 5, &CheckReached::goal_handler, this);
        odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/state_estimation", 5, &CheckReached::odom_handler, this);
    }

    BT::NodeStatus tick() override
    {
        if (has_reached) {
            return BT::NodeStatus::SUCCESS;
        } else {
            return BT::NodeStatus::FAILURE;
        }
    }

    static BT::PortsList providedPorts() {
        return {};
    }

private:
    bool has_reached;
    double goal_x, goal_y, goal_z;
    double curr_x, curr_y, curr_z;
    ros::NodeHandle nh_;
    ros::Subscriber goal_sub_, odom_sub_;

    void goal_handler(const geometry_msgs::PointStamped::ConstPtr& msg)
    {
        goal_x = msg->point.x;
        goal_y = msg->point.y;
        goal_z = msg->point.z;
        has_reached = false; // Reset on new goal
    }

    void odom_handler(const nav_msgs::Odometry::ConstPtr& odom)
    {
        curr_x = odom->pose.pose.position.x;
        curr_y = odom->pose.pose.position.y;
        curr_z = odom->pose.pose.position.z;

        double disX = curr_x - goal_x;
        double disY = curr_y - goal_y;
        double dis = sqrt(disX * disX + disY * disY);

        if (dis < 0.52) { // Threshold for reaching the goal
            has_reached = true;
        }
    }
};



#endif 

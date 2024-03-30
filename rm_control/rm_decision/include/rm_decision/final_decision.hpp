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


class CheckReachedAndStop : public BT::SyncActionNode
{
public:
    CheckReachedAndStop(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config), nh_("~"), has_reached_(true)
    {
        stop_pub_ = nh_.advertise<std_msgs::Int8>("/stop", 10);
        odom_sub_ = nh_.subscribe("/state_estimation", 5, &CheckReachedAndStop::odomHandler, this);
        // Not subscribing to a target here; assuming target is passed via BT blackboard
    }

    BT::NodeStatus tick() override
    {
        double target_x, target_y, target_z;
        // Getting inputs
        if (!getInput<double>("target_x", target_x) ||
            !getInput<double>("target_y", target_y) ||
            !getInput<double>("target_z", target_z))
        {
            ROS_ERROR("CheckReachedAndStop: Missing required input [target_x, target_y, target_z]");
            return BT::NodeStatus::FAILURE;
        }

        // Check if reached
        if (!has_reached_) {
            double disX = curr_x_ - target_x;
            double disY = curr_y_ - target_y;
            double dis = sqrt(disX * disX + disY * disY);

            if (dis < waypoint_xy_radius_) {
                ROS_INFO("CheckReachedAndStop: Target reached.");
                has_reached_ = true;
                publishStopSignal();
                return BT::NodeStatus::SUCCESS;
            }
        }

        // Target not reached yet, or no target set
        return BT::NodeStatus::RUNNING;
    }

    static BT::PortsList providedPorts() {
        return {BT::InputPort<double>("target_x"),
                BT::InputPort<double>("target_y"),
                BT::InputPort<double>("target_z")};
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher stop_pub_;
    ros::Subscriber odom_sub_;
    double curr_x_ = 0.0, curr_y_ = 0.0, curr_z_ = 0.0;
    bool has_reached_;
    const double waypoint_xy_radius_ = 0.52;

    void odomHandler(const nav_msgs::Odometry::ConstPtr& msg) {
        curr_x_ = msg->pose.pose.position.x;
        curr_y_ = msg->pose.pose.position.y;
        curr_z_ = msg->pose.pose.position.z;
        has_reached_ = false; // Reset reached status on new odom message
    }

    void publishStopSignal() {
        std_msgs::Int8 reached_msg;
        reached_msg.data = 1; // Indicate stop
        stop_pub_.publish(reached_msg);
    }
};

#endif 

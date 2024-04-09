#ifndef __FiNAL_DECISION_HPP__
#define __FiNAL_DECISION_HPP__

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int8.h>


class MoveAndCheckPosition : public BT::SyncActionNode
{
public:
    MoveAndCheckPosition(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config), has_reached_(false), nh_("~")
    {
        way_point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/way_point", 10);
        stop_sub_ = nh_.subscribe("/stop", 10, &MoveAndCheckPosition::stopCallback, this);
    }

    BT::NodeStatus tick() override {
        if (!has_reached_) {
            geometry_msgs::PointStamped target_position;
            float target_x, target_y, target_z;

            if (!getInput<float>("target_x", target_x) || 
                !getInput<float>("target_y", target_y) ||
                !getInput<float>("target_z", target_z)) {
                ROS_ERROR("MoveAndCheckPosition: Missing target_position input");
                return BT::NodeStatus::FAILURE;
            }

            ROS_INFO("Current position x:%f, y:%f", target_x, target_y);
            target_position.header.stamp = ros::Time::now();
            target_position.header.frame_id = "map";
            target_position.point.x = target_x;
            target_position.point.y = target_y;
            target_position.point.z = target_z;

            way_point_pub_.publish(target_position);

            // Wait for the stop signal or timeout
            ros::Time start_time = ros::Time::now();
            ros::Rate rate(100); // 100 Hz
            const double timeout = 10.0; // Timeout after 10 seconds

            while (ros::ok()) {
                ros::spinOnce();
                double elapsed = (ros::Time::now() - start_time).toSec();

                if (has_reached_) {
                    ROS_INFO("MoveAndCheckPosition: Target reached.");
                    return BT::NodeStatus::SUCCESS;
                }

                if (elapsed > timeout) {
                    ROS_ERROR("MoveAndCheckPosition: Timeout waiting for stop signal.");
                    return BT::NodeStatus::SUCCESS;
                }

                rate.sleep();
            }
        }
        // Reset for next tick call
        has_reached_ = false;
        return BT::NodeStatus::SUCCESS;
    }

    static BT::PortsList providedPorts() {
        return {BT::InputPort<float>("target_x"),
                BT::InputPort<float>("target_y"),
                BT::InputPort<float>("target_z")};
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher way_point_pub_;
    ros::Subscriber stop_sub_;
    bool has_reached_;

    void stopCallback(const std_msgs::Int8::ConstPtr& msg) {
        if (msg->data == 1) {
            has_reached_ = true;
        } else {
            has_reached_ = false;
        }
    }
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


#endif
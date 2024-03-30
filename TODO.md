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
            : BT::ConditionNode(name, config) {}

        BT::NodeStatus tick() override {
            // 使用全局或通过某种机制传递的血量变量
            hp_sub_ = nh_.subscribe<std_msgs::UInt16>("/sentry_hp", 5, &CheckHP::hp_callback, this);
            return global_current_hp < 400 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        }

        static BT::PortsList providedPorts() {
            return {};
        }
    private:
        uint16_t global_current_hp = 600;
        ros::NodeHandle nh_;
        ros::Subscriber hp_sub_;
        
        void hp_callback(const std_msgs::UInt16& msg){
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

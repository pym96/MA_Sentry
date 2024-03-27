terminate called after throwing an instance of 'BT::RuntimeError'
  what():  Error at line 17: -> Node not recognized: AlwaysRunning
Aborted (core dumped)

terminate called after throwing an instance of 'BT::RuntimeError'
  what():  Possible typo? In the XML, you tried to remap port "duration" in node [Wait / Wait], but the manifest of this node does not contain a port with this name.
Aborted (core dumped)


#ifndef __FiNAL_DECISION_HPP__
#define __FiNAL_DECISION_HPP__

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Int32.h>

class MoveToPosition : public BT::SyncActionNode
{
public:
    MoveToPosition(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {
            way_point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/way_point", 10);
        }

    BT::NodeStatus tick() override {
        geometry_msgs::PointStamped target_position;
        if (!getInput<geometry_msgs::PointStamped>("target_position", target_position)) {
            ROS_ERROR("Missing target_position input");
            return BT::NodeStatus::FAILURE;
        }

        way_point_pub_.publish(target_position);
            
        return BT::NodeStatus::SUCCESS;
    }

    static BT::PortsList providedPorts() {
        return { BT::InputPort<geometry_msgs::PointStamped>("target_position") };
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
            return global_current_hp < 400 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        }

        static BT::PortsList providedPorts() {
            return {};
        }
    private:
        int global_current_hp = 1000;

};

class Wait : public BT::SyncActionNode
{
public:
    Wait(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}

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

class AlwaysRunning : public BT::SyncActionNode
{
public:
    AlwaysRunning(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}

    BT::NodeStatus tick() override {
        return BT::NodeStatus::RUNNING; // 始终返回 RUNNING
    }

    static BT::PortsList providedPorts() {
        return {}; // 没有输入或输出端口
    }
};



#include "ros/ros.h"

#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>

#include "rm_decision/final_decision.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "behavior_tree_robot");
    ros::NodeHandle nh;

    // 订阅 /current_hp 以更新血量

    BT::BehaviorTreeFactory factory;

    // 注册自定义节点
    factory.registerNodeType<MoveToPosition>("MoveToPosition");
    factory.registerNodeType<CheckHP>("CheckHP");
    factory.registerNodeType<Wait>("Wait");
    factory.registerNodeType<AlwaysRunning>("AlwaysRunning");


    // 创建行为树
    auto tree = factory.createTreeFromFile("/home/dan/learn/MA_Sentry/src/rm_control/rm_decision/config/final_tree.xml");

    ros::Rate rate(10);
    while (ros::ok()) {
        // 运行行为树
        tree.tickRoot();

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}




#endif 

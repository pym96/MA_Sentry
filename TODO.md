```
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
    private:
        int global_current_hp = 1000;

};


// Wait 实现
class Wait : public BT::SyncActionNode
{
public:
    Wait(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}

    BT::NodeStatus tick() override {
        ros::Duration(5).sleep(); // 等待5秒
        return BT::NodeStatus::SUCCESS;
    }
};
```

#endif 

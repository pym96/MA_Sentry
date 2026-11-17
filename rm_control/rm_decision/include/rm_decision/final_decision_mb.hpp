#ifndef __FINAL_DECISION_HPP__
#define __FINAL_DECISION_HPP__

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseActionGoal.h> // Include for move_base
#include <actionlib_msgs/GoalStatusArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int8.h>
#include <mutex>


class MoveAndCheckPosition : public BT::SyncActionNode
{
public:
    MoveAndCheckPosition(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config), has_reached_(false), nh_("~")
    {
        way_point_pub_ = nh_.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 10);
        status_sub_ = nh_.subscribe("/move_base/status", 10, &MoveAndCheckPosition::statusCallback, this);
        // stop_sub_ = nh_.subscribe("/stop", 10, &MoveAndCheckPosition::stopCallback, this);
    }

    BT::NodeStatus tick() override {
        if (!has_reached_) {
            move_base_msgs::MoveBaseActionGoal target_goal;
            float target_x, target_y;

            if (!getInput<float>("target_x", target_x) || 
                !getInput<float>("target_y", target_y)) {
                ROS_ERROR("MoveAndCheckPosition: Missing target_position input");
                return BT::NodeStatus::FAILURE;
            }

            ROS_INFO("Sending goal x:%f, y:%f", target_x, target_y);
            // target_goal.goal.target_pose.header.stamp = ros::Time::now();
            target_goal.goal.target_pose.header.frame_id = "map";
            target_goal.goal.target_pose.pose.position.x = target_x;
            target_goal.goal.target_pose.pose.position.y = target_y;
            target_goal.goal.target_pose.pose.orientation.w = 1.0;

            way_point_pub_.publish(target_goal);

            return BT::NodeStatus::SUCCESS;
        }

        has_reached_ = false; // Reset for the next call
        return BT::NodeStatus::SUCCESS;
    }

    static BT::PortsList providedPorts() {
        return {BT::InputPort<float>("target_x"),
                BT::InputPort<float>("target_y")};
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher way_point_pub_;
    ros::Subscriber status_sub_;
    ros::Subscriber stop_sub_;
    bool has_reached_;

    void stopCallback(const std_msgs::Int8::ConstPtr& msg) {
        if (msg->data == 1) {
            has_reached_ = true;
        } else {
            has_reached_ = false;
        }
    }


    void statusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& status){
        if (!status->status_list.empty()) {
            // Check the status of the last goal sent to move_base
            actionlib_msgs::GoalStatus goalStatus = status->status_list.back();
            
            switch(goalStatus.status) {
                case actionlib_msgs::GoalStatus::SUCCEEDED:
                    ROS_INFO("Goal reached");
                    has_reached_ = true;
                    break;
                case actionlib_msgs::GoalStatus::ABORTED:
                    ROS_WARN("Goal aborted");
                    has_reached_ = false;
                    break;
                case actionlib_msgs::GoalStatus::REJECTED:
                    ROS_WARN("Goal rejected");
                    has_reached_ = false;
                    break;
                default:
                    has_reached_ = false;
            }
        }
    }
};

#endif

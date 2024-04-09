#ifndef __MOVE_AND_CHECK_POSITION_HPP__
#define __MOVE_AND_CHECK_POSITION_HPP__

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Int8.h>
#include <cmath>

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

            target_position.header.stamp = ros::Time::now();
            target_position.header.frame_id = "map";
            target_position.point.x = target_x;
            target_position.point.y = target_y;
            target_position.point.z = target_z;

            way_point_pub_.publish(target_position);

            // Wait for the stop signal or timeout
            ros::Time start_time = ros::Time::now();
            ros::Rate rate(10); // 10 Hz
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
                    return BT::NodeStatus::FAILURE;
                }

                rate.sleep();
            }
        }
        // Reset for next tick call
        has_reached_ = false;
        return BT::NodeStatus::FAILURE;
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

#endif // __MOVE_AND_CHECK_POSITION_HPP__

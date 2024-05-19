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
#include <nav_msgs/Odometry.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <vector>
#include <random>

namespace ma_decision{

namespace utils{
    bool in_patrol(const double& x, const double& y, const double& len, const double& width) {
        if (-len < x && x < len && -width < y && y < width) {
            return true;
        } else {
            return false;
        }
    }

    double random_double(double min, double max) {
        static std::default_random_engine e;
        std::uniform_real_distribution<double> dist(min, max);
        return dist(e);
    }
}


namespace bt{
class MoveAndCheckPosition : public BT::SyncActionNode
{
public:
    MoveAndCheckPosition(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config), has_reached_(false), nh_("~")
    {   
        way_point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/way_point", 10);
        stop_sub_ = nh_.subscribe("/stop", 10, &MoveAndCheckPosition::stopCallback, this);
        odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/state_estimation", 5, &MoveAndCheckPosition::odom_callback, this);
    }

    BT::NodeStatus tick() override {
        
        // TODO: Getting current position and use in_patrol()
        // if yes,  then random use and return FAILURE, publish random way point
        // else, then do nothing.
    if(utils::in_patrol(vehicleX, vehicleY, 1 , 1)){
            // Publishing random point. But how?
            // Generate a random point within bounds (-1, 1) for x, and (-2, 2) for y
            ros::Time start_time = ros::Time::now();
            double randX = utils::random_double(-1.5, 1.5);
            double randY = utils::random_double(-1.5, 1.5);

            // Create and publish the random waypoint
            geometry_msgs::PointStamped random_point;
            random_point.header.stamp = ros::Time::now();
            random_point.header.frame_id = "map"; // or any other relevant frame
            random_point.point.x = randX;
            random_point.point.y = randY;
            ROS_INFO("Current random point is x:%f, y:%f", randX, randY);
            
            // if(!has_reached_){
            //     return BT::NodeStatus::FAILURE;
            // }

            way_point_pub_.publish(random_point);
            
            while(ros::ok()){
                double elapsed = (ros::Time::now() - start_time).toSec();
                
                if (elapsed > 1.0) 
                  break;
                else   
                    continue;

            }


            return BT::NodeStatus::FAILURE;
        }
        else if (!has_reached_) {
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
            const double timeout = 20.0; // Timeout after 10 seconds

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
    ros::Subscriber odom_sub_;
    std::vector<std::pair<int, int>> partrol_point_;
    bool has_reached_;
    double odomTime;
    double vehicleX, vehicleY, vehicleZ;

    void stopCallback(const std_msgs::Int8::ConstPtr& msg) {
        if (msg->data == 1) {
            has_reached_ = true;
        } else {
            has_reached_ = false;
        }
    }

   void odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
        odomTime = msg->header.stamp.toSec();

        double roll, pitch, yaw;
        geometry_msgs::Quaternion geoQuat = msg->pose.pose.orientation;
        tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

        vehicleX = msg->pose.pose.position.x;
        vehicleY = msg->pose.pose.position.y;
        vehicleZ = msg->pose.pose.position.z;
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
}
}

#endif
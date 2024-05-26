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
#include <string>

using namespace std;

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

        robot_id_sub = nh_.subscribe("/robot_id", 10, &MoveAndCheckPosition::robot_id_callback, this);
        red_outpost_HP_sub = nh_.subscribe("/red_outpost_HP", 10, &MoveAndCheckPosition::red_outpost_HP_callback, this);
        blue_outpost_HP_sub = nh_.subscribe("/blue_outpost_HP", 10, &MoveAndCheckPosition::blue_outpost_HP_callback, this);
        game_time_sub = nh_.subscribe("/game_time", 10, &MoveAndCheckPosition::game_time_callback, this);
    }

    BT::NodeStatus tick() override 
    {
        
        // TODO: Getting current position and use in_patrol()
        // if yes,  then random use and return FAILURE, publish random way point
        // else, then do nothing.
    if(!has_reached_)
    {
        geometry_msgs::PointStamped target_position;
        float target_x, target_y, target_z;
        string action;
        if (!getInput<float>("target_x", target_x) || 
            !getInput<float>("target_y", target_y) ||
            !getInput<float>("target_z", target_z) ||
            !getInput<string>("action",action)) 
        {
            ROS_ERROR("MoveAndCheckPosition: Missing target_position input");
            return BT::NodeStatus::FAILURE;
        }
        if(action == "patrol")//最好在前边加个靠近巡逻区的中间点
        {
            while(ros::ok)
            {
                ros::Time start_time = ros::Time::now();
                double randX = utils::random_double(-1.5, 1.5);
                double randY = utils::random_double(-1.5, 1.5);
                geometry_msgs::PointStamped random_point;
                random_point.header.stamp = ros::Time::now();
                random_point.header.frame_id = "map"; 
                random_point.point.x = randX;
                random_point.point.y = randY;
                ROS_INFO("Current random point is x:%f, y:%f", randX, randY);
                way_point_pub_.publish(random_point);
                while(ros::ok())
                {
                    double elapsed = (ros::Time::now() - start_time).toSec();
                    if (elapsed > 3.0) 
                        break;
                    else   
                        continue;
                    
                }
            }
            
        }
        else if(action == "nothing")
        {
            ROS_INFO("Current position x:%f, y:%f", target_x, target_y);
            target_position.header.stamp = ros::Time::now();
            target_position.header.frame_id = "map";
            target_position.point.x = target_x;
            target_position.point.y = target_y;
            target_position.point.z = target_z;

            way_point_pub_.publish(target_position);

            ros::Time start_time = ros::Time::now();
            ros::Rate rate(100); // 100 Hz
            const double timeout_1 = 20.0; // Timeout after 10 seconds


            while (ros::ok()) 
            {
                ros::spinOnce();
                double elapsed_1 = (ros::Time::now() - start_time).toSec();

                //ROS_INFO("robot_id : %u", robot_id_.data);

                if (has_reached_) 
                {
                    ROS_INFO("has_reach------nothing");
                    return BT::NodeStatus::SUCCESS;
                }
                if (elapsed_1 > timeout_1) 
                {
                    ROS_ERROR("MoveAndCheckPosition: Timeout waiting for stop signal!!!!.");
                    return BT::NodeStatus::SUCCESS;
                }

            }
        }
        else if(action == "fight")
        {
            ROS_INFO("Current position x:%f, y:%f", target_x, target_y);
            target_position.header.stamp = ros::Time::now();
            target_position.header.frame_id = "map";
            target_position.point.x = target_x;
            target_position.point.y = target_y;
            target_position.point.z = target_z;

            way_point_pub_.publish(target_position);

            ros::Time start_time = ros::Time::now();
            ros::Rate rate(100); // 100 Hz
            const double timeout_2 = 10.0; // Timeout after 10 seconds

            ROS_INFO("FIGHT position x:%f, y:%f", target_x, target_y);
            while (ros::ok()) //10秒
            {
                ros::spinOnce();
                double elapsed_2 = (ros::Time::now() - start_time).toSec();

                if (elapsed_2 > timeout_2) //没到前哨战，超时了
                {
                    ROS_ERROR("FIGHT : Timeout waiting for stop signal!!!!.");
                    return BT::NodeStatus::SUCCESS;
                }
                ROS_INFO("FIGHT WAIT x:%f, y:%f", target_x, target_y);

                if (has_reached_) //10秒之内到了
                {
                    ROS_INFO("FIGHT reach");
               
                        if(robot_id_.data == 7)
                        {
                            if(red_outpost_HP_.data <= 200 || blue_outpost_HP_.data == 0 || game_time_.data == 4)//直到
                            {
                                return BT::NodeStatus::SUCCESS;
                            }
                            else
                            {
                                ROS_INFO("FIGHT!!!");
                            }
                        }
                        else if(robot_id_.data == 107)
                        {
                            if(blue_outpost_HP_.data <= 200 || red_outpost_HP_.data == 0 || game_time_.data == 4)
                            {
                                return BT::NodeStatus::SUCCESS;
                            }
                            else
                            {
                                ROS_INFO("FIGHT!!!");
                            }
                        }else{
                             ROS_INFO("We are attacking outpost, retreating is needless!");
                             return BT::NodeStatus::FAILURE;
                        }

                }
            }
        }

        return BT::NodeStatus::FAILURE;
    }
        return BT::NodeStatus::SUCCESS;
}

    static BT::PortsList providedPorts() 
    {
        return {BT::InputPort<float>("target_x"),
                BT::InputPort<float>("target_y"),
                BT::InputPort<float>("target_z"),
                BT::InputPort<string>("action")};
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher way_point_pub_;
    ros::Subscriber stop_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber robot_id_sub;
    ros::Subscriber red_outpost_HP_sub;
    ros::Subscriber blue_outpost_HP_sub;
    ros::Subscriber game_time_sub;
    std::vector<std::pair<int, int>> partrol_point_;
    bool has_reached_;
    double odomTime;
    double vehicleX, vehicleY, vehicleZ;
    std_msgs::Int8 robot_id_;
    std_msgs::UInt16 red_outpost_HP_;
    std_msgs::UInt16 blue_outpost_HP_;
    std_msgs::UInt16 game_time_;

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

    void robot_id_callback(const std_msgs::Int8::ConstPtr& msg)
    {
        //ROS_INFO("callback");
        robot_id_.data = msg->data;
        ROS_INFO("robot_id : %u" , robot_id_.data);
    }

    void red_outpost_HP_callback(const std_msgs::UInt16::ConstPtr& msg)
    {
        red_outpost_HP_.data = msg->data;
    }

    void blue_outpost_HP_callback(const std_msgs::UInt16::ConstPtr& msg)
    {
        blue_outpost_HP_.data = msg->data;
    }

    void game_time_callback(const std_msgs::UInt16::ConstPtr& msg)
    {
        game_time_.data = msg->data;
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
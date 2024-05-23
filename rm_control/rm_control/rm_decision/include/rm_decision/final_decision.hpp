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
        : BT::SyncActionNode(name, config), has_reached_(false), red_outpost_current_hp(1500),blue_outpost_current_hp(1500),nh_("~")
    {   
        way_point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/way_point", 10);
        stop_sub_ = nh_.subscribe("/stop", 10, &MoveAndCheckPosition::stopCallback, this);
        odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/state_estimation", 10, &MoveAndCheckPosition::odom_callback, this);
        red_outpost_HP_sub = nh_.subscribe("/red_outpost_HP", 10, &MoveAndCheckPosition::red_hp_callback, this);
        blue_outpost_HP_sub = nh_.subscribe("/blue_outpost_HP", 10, &MoveAndCheckPosition::blue_hp_callback, this);
        robot_id_sub = nh_.subscribe("/robot_id", 10, &MoveAndCheckPosition::id_callback, this);
        game_time_sub =nh_.subscribe("/game_time", 10, &MoveAndCheckPosition::time_callback, this);
    }

    BT::NodeStatus tick() override {
        
        // TODO: Getting current position and use in_patrol()
        // if yes,  then random use and return FAILURE, publish random way point
        // else, then do nothing.
       
        if (!has_reached_) 
        {
            geometry_msgs::PointStamped target_position;
            float target_x, target_y, target_z ;

            if (!getInput<float>("target_x", target_x) || 
                !getInput<float>("target_y", target_y) ||
                !getInput<float>("target_z", target_z) ||
                !getInput<string>("action", action) ) {
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

            while (ros::ok()) 
            {
                double elapsed = (ros::Time::now() - start_time).toSec();
                ros::spinOnce();

                if (checkPosition(vehicleX, vehicleY, target_x, target_y)) 
                {
                    center_x = target_x;
                    center_y = target_y;
                    has_reached_ = true; 
                }

                if (has_reached_) 
                {
                    ROS_INFO("MoveAndCheckPosition: Target reached.");
                    if(action == "patrol")
                    {
                        double randX = utils::random_double(center_x - 3 , center_x + 3);
                        double randY = utils::random_double(center_y - 3 , center_y + 3);

                        geometry_msgs::PointStamped random_point;
                        random_point.header.stamp = ros::Time::now();
                        random_point.header.frame_id = "map"; // or any other relevant frame
                        random_point.point.x = randX;
                        random_point.point.y = randY;
                        ROS_INFO("Current random point is x:%f, y:%f", randX, randY);

                        way_point_pub_.publish(random_point);
                        
                        while(ros::ok())
                        {
                            double elapsed = (ros::Time::now() - start_time).toSec();
                
                            if (elapsed > 1.0) 
                            break;
                            else   
                            continue;

                        }
                        return BT::NodeStatus::FAILURE;
                    }
                    else if(action == "fight")
                    {
                        /*ROS_INFO("FIGHT!!!----1 min!!!.");
                        ros::Duration(10).sleep(); */
                        if(robo_id == 7)
                        {
                            if(red_outpost_current_hp < 200 || blue_outpost_current_hp == 0 || game_time == 4)
                            {
                                return BT::NodeStatus::SUCCESS;
                            }
                            else
                            {

                            }
                        }
                        else if(robo_id == 107)
                        {
                            if(blue_outpost_current_hp < 200 || red_outpost_current_hp == 0 || game_time == 4)
                            {
                                return BT::NodeStatus::SUCCESS;
                            }
                            else
                            {

                            }
                        }
                        has_reached_ = false;
                    }
                    else if(action == "nothing")
                    {
                        has_reached_ = false;
                        return BT::NodeStatus::SUCCESS;
                    }
                }
                if (elapsed > timeout) 
                {
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
                BT::InputPort<float>("target_z"),
                BT::InputPort<string>("action")};
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher way_point_pub_;
    ros::Subscriber stop_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber red_outpost_HP_sub;
    ros::Subscriber blue_outpost_HP_sub;
    ros::Subscriber robot_id_sub;
    ros::Subscriber game_time_sub;
    std::vector<std::pair<int, int>> partrol_point_;
    bool has_reached_;
    double odomTime;
    double vehicleX, vehicleY, vehicleZ,center_x ,center_y;
    std::string action;
    uint16_t red_outpost_current_hp;
    uint16_t blue_outpost_current_hp;
    uint8_t robo_id;
    uint16_t game_time;

    std::mutex hp_mutex_; // 用于保护 global_current_hp 的互斥锁

    void red_hp_callback(const std_msgs::UInt16::ConstPtr& msg) 
    {
        std::lock_guard<std::mutex> lock(hp_mutex_);
        red_outpost_current_hp = msg->data;
    }

    void blue_hp_callback(const std_msgs::UInt16::ConstPtr& msg) 
    {
        std::lock_guard<std::mutex> lock(hp_mutex_);
        blue_outpost_current_hp = msg->data;
    }

    void id_callback(const std_msgs::Int8::ConstPtr& msg)
    {
        robo_id = msg->data;
    }

    void time_callback(const std_msgs::UInt16::ConstPtr& msg)
    {
        game_time = msg->data;
    }

    void stopCallback(const std_msgs::Int8::ConstPtr& msg) 
    {
        if (msg->data == 1) 
        {
            has_reached_ = true;
        } 
        else 
        {
            has_reached_ = false;
        }
    }

   void odom_callback(const nav_msgs::Odometry::ConstPtr& msg) 
   {
        odomTime = msg->header.stamp.toSec();

        double roll, pitch, yaw;
        geometry_msgs::Quaternion geoQuat = msg->pose.pose.orientation;
        tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

        vehicleX = msg->pose.pose.position.x;
        vehicleY = msg->pose.pose.position.y;
        vehicleZ = msg->pose.pose.position.z;
    }

    bool checkPosition(double current_x, double current_y, double target_x, double target_y, double threshold = 0.6) 
    {
        double distance = std::sqrt(std::pow(current_x - target_x, 2) + std::pow(current_y - target_y, 2));
        return distance <= threshold;
    }
};

class last_do : public BT::AsyncActionNode
{
public:
    last_do(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config), nh_("~"), is_running_(false)
    {
        way_point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/way_point", 10);
        stop_sub_ = nh_.subscribe("/stop", 10, &last_do::stopCallback, this);
        odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/state_estimation", 5, &last_do::odom_callback, this);
    }

    BT::NodeStatus tick() override
    {
        if (!is_running_)
        {
            start_thread();
            is_running_ = true;
        }
        return BT::NodeStatus::RUNNING;
    }

    void halt() override
    {
        if (is_running_)
        {
            stop_thread();
            is_running_ = false;
        }
    }

    static BT::PortsList providedPorts() 
    {
        return {}; // 没有输入或输出端口
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher way_point_pub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber stop_sub_;
    std::thread worker_thread_;
    bool is_running_ , first_random = true;
    double vehicleX, vehicleY, vehicleZ, centerX ,centerY;

    void start_thread()
    {
        worker_thread_ = std::thread([this]() {
            run_loop();
        });
    }

    void stop_thread()
    {
        if (worker_thread_.joinable())
        {
            worker_thread_.join();
        }
    }

    void run_loop()
    {
        if(first_random)
        {
            centerX = vehicleX;
            centerY = vehicleY;
            first_random = false;
        }
        ros::Rate rate(1); // Adjust the rate as needed
        while (ros::ok() && !has_reached_)
        {
            double randX = utils::random_double(centerX - 3, centerX + 3);
            double randY = utils::random_double(centerY - 3, centerY + 3);

            geometry_msgs::PointStamped random_point;
            random_point.header.stamp = ros::Time::now();
            random_point.header.frame_id = "map";
            random_point.point.x = randX;
            random_point.point.y = randY;
            ROS_INFO("Current random point is x:%f, y:%f", randX, randY);

            way_point_pub_.publish(random_point);
            rate.sleep();
        }
    }

    void stopCallback(const std_msgs::Int8::ConstPtr& msg)
    {
        if (msg->data == 1)
        {
            has_reached_ = true;
        }
    }

    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        vehicleX = msg->pose.pose.position.x;
        vehicleY = msg->pose.pose.position.y;
        vehicleZ = msg->pose.pose.position.z;
    }

    

    bool has_reached_ = false;
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

    BT::NodeStatus tick() override 
    {
        uint16_t current_hp_copy;
        {
            std::lock_guard<std::mutex> lock(hp_mutex_);
            current_hp_copy = global_current_hp;
        }
        return current_hp_copy < 100 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    static BT::PortsList providedPorts() 
    {
        return {};
    }

private:
    uint16_t global_current_hp;
    ros::NodeHandle nh_;
    ros::Subscriber hp_sub_;
    std::mutex hp_mutex_; // 用于保护 global_current_hp 的互斥锁

    void hp_callback(const std_msgs::UInt16::ConstPtr& msg) 
    {
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

class AlwaysRunning : public BT::SyncActionNode 
{ 
public: 
    AlwaysRunning(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config) {}

BT::NodeStatus tick() override 
{
    return BT::NodeStatus::RUNNING; // 始终返回 RUNNING
}

static BT::PortsList providedPorts() 
{
    return {}; // 没有输入或输出端口
}

};

}
}

#endif
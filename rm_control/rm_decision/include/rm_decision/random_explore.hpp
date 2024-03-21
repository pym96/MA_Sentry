#ifndef __RANDOM_EXPLORE_HPP
#define __RANDOM_EXPLORE_HPP

#include <behaviortree_cpp_v3/action_node.h>
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Point.h> // Include for Point data structure

#include <string>

class RandomExplore: public BT::SyncActionNode
{
public:
    RandomExplore(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts(){
        return {};
    }

    BT::NodeStatus tick() override;

    // void halt() override;

private:
    ros::NodeHandle nh_;
    ros::Publisher way_point_pub_;
    ros::Subscriber stop_sub_;
    bool goal_reached_;
    void stop_callback(const std_msgs::Int8::ConstPtr& msg);

    bool initial_position_set; // Flag to check if the initial position has been set
    geometry_msgs::Point initial_position; // Variable to store the initial position
};

#endif // __RANDOM_EXPLORE_HPP

#ifndef __PRE_GOAL_HPP__
#define __PRE_GOAL_HPP__

#include "ros/ros.h"

#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>

class PreGoal
{
    public:
        explicit PreGoal(){}
        

    private:
        geometry_msgs::PointStamped goal_;
        

};


#endif 
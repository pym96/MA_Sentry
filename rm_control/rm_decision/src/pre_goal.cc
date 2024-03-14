#include "ros/ros.h"

#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>

class WayPointFollower{

    public:
        WayPointFollower(){

            ros::NodeHandle nh;

            waypoint_sub_ = nh.subscribe<geometry_msgs::PointStamped>("/way_point", 10, &WayPointFollower::waypoint_callback, this);
            odom_sub_ = nh.subscribe<nav_msgs::Odometry>("/odometry", 10, &WayPointFollower::odom_callback, this);
            
            cmd_vel_pub_ = nh.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 10);


        }

        void waypoint_callback(const geometry_msgs::PointStamped::ConstPtr& msg){
            target_point_ = msg->point;
            has_reached_ = false;
        }

        void odom_callback(const nav_msgs::Odometry::ConstPtr& msg){
            current_position_ = msg->pose.pose.position;
            if(!has_reached_){
                move_towards_target();
            }
        }

        void move_towards_target(){
            double distance = std::sqrt(std::pow(target_point_.x - current_position_.x, 2) +
                                        std::pow(target_point_.y - current_position_.y,2) +
                                        std::pow(target_point_.z - current_position_.z, 2));
            
            if(distance <= distance_threshold_){
                has_reached_ = true;
                stop_moving();
            }else{
                // TODO: keep moving
            }

        }

        void stop_moving(){
            geometry_msgs::TwistStamped cmd_vel;
            cmd_vel.twist.linear.x = 0.0;
            cmd_vel.twist.linear.z = 0.0;
            cmd_vel_pub_.publish(cmd_vel);
        }

    private:
        ros::Subscriber waypoint_sub_;
        ros::Subscriber odom_sub_;
        ros::Publisher cmd_vel_pub_;

        geometry_msgs::Point target_point_;
        geometry_msgs::Point current_position_;

        bool has_reached_;
        double distance_threshold_;
};


int main(int argc, char** argv){

    ros::init(argc, argv, "robot_decision");

    
    


    return 0;
}
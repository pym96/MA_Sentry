#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

double goal_x = 0.0;
double goal_y = 0.0;
double goal_z = 0.0;

double curr_x = 0.0;
double curr_y = 0.0;
double curr_z = 0.0;

double odom_time;

double curr_roll;
double curr_pitch;
double curr_yaw;

double waypoint_xy_radius = 0.52;
double waypoint_z_bound = 5.0;
double dis;

bool has_reached = false;

void has_reached_callback() {
    // 只有在还未到达目标点时才检查是否已到达
    if (!has_reached) {
        float disX = curr_x - goal_x;
        float disY = curr_y - goal_y;
        float disZ = curr_z - goal_z;

        dis = sqrt(disX * disX + disY * disY);
        

        if (dis < waypoint_xy_radius) {
            ROS_INFO("Current dis is: %f", dis);
            has_reached = true;
        }
    }
}


void goal_handler(const geometry_msgs::PointStamped::ConstPtr& msg) {
    goal_x = msg->point.x;
    goal_y = msg->point.y;
    goal_z = msg->point.z;

    // 收到新目标点时重置 has_reached 状态
    has_reached = false;
}



void odom_handler(const nav_msgs::Odometry::ConstPtr& odomIn) {
    odom_time = odomIn->header.stamp.toSec();

    geometry_msgs::Quaternion geoQuat = odomIn->pose.pose.orientation;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(geoQuat, quat);
    tf::Matrix3x3(quat).getRPY(curr_roll, curr_pitch, curr_yaw);

    curr_x = odomIn->pose.pose.position.x;
    curr_y = odomIn->pose.pose.position.y;
    curr_z = odomIn->pose.pose.position.z;

    has_reached_callback();
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_stop");
    ros::NodeHandle nh;
    
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/state_estimation", 5, odom_handler);
    ros::Subscriber goal_sub = nh.subscribe<geometry_msgs::PointStamped>("/way_point", 5, goal_handler);
    ros::Publisher stop_pub = nh.advertise<std_msgs::Int8>("/stop", 10);

    ros::Rate rate(100); // 100 Hz

    while (ros::ok()) {
        std_msgs::Int8 reached;
        reached.data = has_reached ? 1 : 0;
        
        stop_pub.publish(reached);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>


double goal_x;
double goal_y;
double goal_z;

double curr_x;
double curr_y;
double curr_z;

double odom_time;

double curr_roll;
double curr_pitch;
double curr_yaw;

double waypoint_xy_radius = 0.5;
double waypoint_z_bound = 5.0;

bool has_reached = false;

void has_reached_callback(){

    float disX = curr_x - goal_x;
    float disY = curr_y - goal_y;
    float disZ = curr_z - goal_z;

      // start waiting if the current waypoint is reached
    if (sqrt(disX * disX + disY * disY) < waypoint_xy_radius && fabs(disZ) < waypoint_z_bound && !has_reached) {
      has_reached = true;
    }else{
      has_reached = false;
    }
    
}

void odom_handler(const nav_msgs::Odometry::ConstPtr& odomIn)
{
  odom_time = odomIn->header.stamp.toSec();

  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odomIn->pose.pose.orientation;

  curr_roll = roll;
  curr_pitch = pitch;
  curr_yaw = yaw;
  curr_x = odomIn->pose.pose.position.x;
  curr_y = odomIn->pose.pose.position.y;
  curr_z = odomIn->pose.pose.position.z;
}

void goal_handler(geometry_msgs::PointStamped::ConstPtr& msg){
  goal_x = msg->point.x;
  goal_y = msg->point.y;
}

int main(int argc, char** argv){

    ros::init(argc, argv, "robot_stop");
    ros::NodeHandle nh;
    
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/state_estimation", 5, odom_handler);
    ros::Subscriber goal_sub = nh.subscribe<geometry_msgs::PointStamped>("/way_point",5, goal_handler);
    ros::Publisher stop_pub = nh.advertise<std_msgs::Int8>("/stop", 10);

    bool status = ros::ok();

    std_msgs::Int8 reached = 0;

    while(status){

        // TODO: 
        if(!has_reached){
            reached = 1;
        }else{
            reached = 0;
        }
        
        stop_pub.publish(reached);

    }


    return 0;
}
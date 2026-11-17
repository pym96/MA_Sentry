#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

ros::Publisher corrected_odom_pub;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // Create a copy of the received Odometry message
    nav_msgs::Odometry corrected_msg = *msg;

    // Reverse the sign of the Y component of the position
    corrected_msg.pose.pose.position.y = -msg->pose.pose.position.y;

    // Reverse the sign of the Y component of the orientation quaternion
    corrected_msg.pose.pose.orientation.y = -msg->pose.pose.orientation.y;

    // Publish the corrected Odometry message
    corrected_odom_pub.publish(corrected_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "reverse_odom_y");
    ros::NodeHandle nh;

    ros::Subscriber odom_sub = nh.subscribe("/Odometry", 1, odomCallback);
    corrected_odom_pub = nh.advertise<nav_msgs::Odometry>("/Odometry_1", 1);

    ros::spin();

    return 0;
}

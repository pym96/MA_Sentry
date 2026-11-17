#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cmd_vel_publisher");
    ros::NodeHandle nh;
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Rate rate(10);  // 10 Hz

    while (ros::ok())
    {
        // Create a Twist message with zero linear and angular velocities
        geometry_msgs::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_msg.linear.y = 0.0;
        cmd_vel_msg.linear.z = 0.0;
        cmd_vel_msg.angular.x = 0.0;
        cmd_vel_msg.angular.y = 0.0;
        cmd_vel_msg.angular.z = 0.0;

        // Publish the Twist message
        cmd_vel_pub.publish(cmd_vel_msg);

        // Sleep to maintain the desired rate
        rate.sleep();
    }

    return 0;
}

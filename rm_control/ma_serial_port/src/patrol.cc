#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <math.h>
#include <chrono>

int main(int argc, char** argv) {

    ros::init(argc, argv, "ma_sentry_patrol");  // Initialize ROS
    ros::NodeHandle nh;  // Node handle

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/msg_pub/cmd_vel", 10);

    ros::Rate loop_rate(10); // 10 hz

    double y_speed = 2.0;
    double angular_speed = -M_PI / 6;

    auto start_time = std::chrono::steady_clock::now();

    bool increasing = true;

   
    while(ros::ok()){
        geometry_msgs::TwistStamped msg;

        // Time stamp
        msg.header.stamp = ros::Time::now();

        auto current_time = std::chrono::steady_clock::now();

        if(std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count() >=3 ){
            y_speed = -y_speed;
            start_time = current_time;
        }

        if(increasing){
            angular_speed += 0.1;
            if(angular_speed > M_PI / 6){
                increasing = false;
            }
        }else{
            angular_speed -= 0.1;
            if(angular_speed < -M_PI / 6){
                increasing = true;
            }
        }

        msg.twist.linear.y = y_speed;
        msg.twist.angular.z = angular_speed;
        
        vel_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

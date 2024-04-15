#ifndef VELOCITY_SMOOTHER_EMA_H
#define VELOCITY_SMOOTHER_EMA_H


#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h> // 确保包含了TwistStamped类型的定义


class VelocitySmootherEma
{
    public:
        VelocitySmootherEma(ros::NodeHandle* nh);
        ~VelocitySmootherEma();
        void twist_callback(const geometry_msgs::TwistStamped::ConstPtr& msg);
        void update(const ros::TimerEvent&);

    private:
        ros::NodeHandle nh_;
        ros::Subscriber velocity_sub_;
        ros::Publisher velocity_pub_;
        ros::Timer timer;


        double alpha_v, alpha_w;
        double cmd_rate;
        double max_accel;
        std::string raw_cmd_topic = "raw_cmd_topic";
        std::string cmd_topic = "cmd_topic";
        double previous_x_vel, previous_y_vel, previous_w_vel;
        double x_vel, y_vel, w_vel;
        double smoothed_x_vel, smoothed_y_vel, smoothed_w_vel;
        double last_smoothed_x_vel, last_smoothed_y_vel;
        geometry_msgs::TwistStamped cmd_vel_msg_;
        int stop_counter = 3;
};

#endif

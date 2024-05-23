#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

class VelocitySmootherEma
{
public:
    VelocitySmootherEma(ros::NodeHandle* nh) : nh_(*nh)
    {
        nh_.param<double>("/alpha_v", alpha_v_, 0.4);
        nh_.param<double>("/alpha_w", alpha_w_, 0.4);
        nh_.param<std::string>("/cmd_vel_topic", cmd_vel_topic_, "cmd_vel");
        nh_.param<std::string>("/smoothed_vel_topic", smoothed_vel_topic_, "smoothed_vel");
        nh_.param<double>("/cmd_rate", cmd_rate_, 30.0);
        nh_.param<int>("/stop_counter", stop_counter_, 3);

        cmd_vel_sub_ = nh_.subscribe(cmd_vel_topic_, 10, &VelocitySmootherEma::cmdVelCallback, this);
        smoothed_vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(smoothed_vel_topic_, 10, true);
        timer_ = nh_.createTimer(ros::Duration(1.0 / cmd_rate_), &VelocitySmootherEma::update, this);

        previous_x_vel_ = 0.0;
        previous_y_vel_ = 0.0;
        previous_w_vel_ = 0.0;
    }

    ~VelocitySmootherEma()
    {
        ros::shutdown();
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber cmd_vel_sub_;
    ros::Publisher smoothed_vel_pub_;
    ros::Timer timer_;

    std::string cmd_vel_topic_;
    std::string smoothed_vel_topic_;
    double alpha_v_;
    double alpha_w_;
    double cmd_rate_;
    int stop_counter_;

    double previous_x_vel_, previous_y_vel_, previous_w_vel_;

    geometry_msgs::TwistStamped current_cmd_;

    void cmdVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
    {
        current_cmd_ = *msg;
        stop_counter_ = 10; // Reset stop counter every time a new message is received
    }

    void update(const ros::TimerEvent&)
    {
        if (stop_counter_-- <= 0)
        {
            stop_counter_ = 0;
            current_cmd_.twist.linear.x = 0.0;
            current_cmd_.twist.linear.y = 0.0;
            current_cmd_.twist.angular.z = 0.0;
        }

        double x_vel = current_cmd_.twist.linear.x;
        double y_vel = current_cmd_.twist.linear.y;
        double w_vel = current_cmd_.twist.angular.z;

        double smoothed_x_vel = alpha_v_ * x_vel + (1 - alpha_v_) * previous_x_vel_;
        double smoothed_y_vel = alpha_v_ * y_vel + (1 - alpha_v_) * previous_y_vel_;
        double smoothed_w_vel = alpha_w_ * w_vel + (1 - alpha_w_) * previous_w_vel_;

        geometry_msgs::TwistStamped smoothed_msg;
        smoothed_msg.header.stamp = ros::Time::now();
        smoothed_msg.twist.linear.x = smoothed_x_vel;
        smoothed_msg.twist.linear.y = smoothed_y_vel;
        smoothed_msg.twist.angular.z = smoothed_w_vel;

        ROS_INFO("Publishing smoothed velocities: vx: %f, vy: %f, wz: %f", smoothed_x_vel, smoothed_y_vel, smoothed_w_vel);

        smoothed_vel_pub_.publish(smoothed_msg);

        previous_x_vel_ = smoothed_x_vel;
        previous_y_vel_ = smoothed_y_vel;
        previous_w_vel_ = smoothed_w_vel;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "velocity_smoother_ema");
    ros::NodeHandle nh;
    VelocitySmootherEma vse(&nh);
    ros::spin();
    return 0;
}

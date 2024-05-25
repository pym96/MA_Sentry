#include <velocity_smoother_ema/velocity_smoother_ema.hpp>

VelocitySmootherEma::VelocitySmootherEma(ros::NodeHandle* nh):nh_(*nh)
{
    nh_.param<double>("/alpha_v", alpha_v, 0.4);
    nh_.param<double>("/alpha_w", alpha_w, 0.4);
    nh_.param<std::string>("/raw_cmd_topic", raw_cmd_topic, "raw_cmd_vel");
    nh_.param<std::string>("/cmd_topic", cmd_topic, "cmd_vel");
    nh_.param<double>("/cmd_rate", cmd_rate, 30.0);
    nh_.param<int>("/stop_counter", stop_counter, 3);
    nh_.param<double>("maxAccel", max_accel, 0.7);


    velocity_sub_ = nh_.subscribe(raw_cmd_topic, 10, &VelocitySmootherEma::twist_callback, this);
    // velocity_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(cmd_topic, 10, true);
    velocity_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(cmd_topic, 10, true);
    timer = nh_.createTimer(ros::Duration(1.0 / cmd_rate), &VelocitySmootherEma::update, this);

    previous_x_vel = 0.0;
    previous_y_vel = 0.0;
    previous_w_vel = 0.0;
    x_vel = 0.0;
    y_vel = 0.0;
    w_vel = 0.0;
    smoothed_x_vel = 0.0;
    smoothed_y_vel = 0.0;
    smoothed_w_vel = 0.0;

    last_smoothed_x_vel = 0.0;
    last_smoothed_y_vel = 0.0;
}

VelocitySmootherEma::~VelocitySmootherEma()
{
    ros::shutdown();
}

void VelocitySmootherEma::twist_callback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    cmd_vel_msg_ = *msg;
    stop_counter = 10;
}


void VelocitySmootherEma::update(const ros::TimerEvent&)
{
    if (stop_counter-- <= 0)
    {
        stop_counter = 0;
        cmd_vel_msg_.twist.linear.x = 0.0;
        cmd_vel_msg_.twist.linear.y = 0.0;
        cmd_vel_msg_.twist.angular.z = 0.0;
        // ROS_INFO("THE CMD IS ZERO");
    }
    // ROS_INFO("THE CMD IS : %d", stop_counter);
    y_vel = cmd_vel_msg_.twist.linear.y;
    w_vel = cmd_vel_msg_.twist.angular.z;
    x_vel = cmd_vel_msg_.twist.linear.x;

    double temp_smoothed_x_vel = alpha_v * x_vel + (1 - alpha_v) * previous_x_vel;
    double temp_smoothed_y_vel = alpha_v * y_vel + (1 - alpha_v) * previous_y_vel;
    double temp_smoothed_w_vel = alpha_w * w_vel + (1 - alpha_w) * previous_w_vel;

    double accel_x = (temp_smoothed_x_vel - last_smoothed_x_vel) / (1.0 / cmd_rate);
    double accel_y = (temp_smoothed_y_vel - last_smoothed_y_vel) / (1.0 / cmd_rate);

    double smoothed_accel_x = std::min(std::max(accel_x, -max_accel), max_accel);
    double smoothed_accel_y = std::min(std::max(accel_y, -max_accel), max_accel);

    smoothed_x_vel += smoothed_accel_x * (1.0 / cmd_rate);
    smoothed_y_vel += smoothed_accel_y * (1.0 / cmd_rate);

    cmd_vel_msg_.twist.linear.x = smoothed_x_vel;
    cmd_vel_msg_.twist.linear.y = smoothed_y_vel;


    ROS_INFO("Current v_x: %f, v_y: %f, a_z: %f", smoothed_x_vel, smoothed_y_vel, smoothed_w_vel);

    previous_x_vel = smoothed_x_vel;
    previous_y_vel = smoothed_y_vel;
    
    last_smoothed_x_vel = smoothed_x_vel;
    last_smoothed_y_vel = smoothed_y_vel;



    velocity_pub_.publish(cmd_vel_msg_);
    // ROS_INFO("PUBLISHING TWIST MESSAGE!");
    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "velocity_smoother_ema");

    ros::NodeHandle nh;

    VelocitySmootherEma vse(&nh);

    ros::spin();

    return 0;
}
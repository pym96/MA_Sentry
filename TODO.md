```
#include <velocity_smoother_ema/velocity_smoother_ema.hpp>
#include <algorithm> // 为了使用 std::min 和 std::max

VelocitySmootherEma::VelocitySmootherEma(ros::NodeHandle* nh):nh_(*nh)
{
    nh_.param<double>("/alpha_v", alpha_v, 0.4);
    nh_.param<double>("/alpha_w", alpha_w, 0.4);
    nh_.param<std::string>("/raw_cmd_topic", raw_cmd_topic, "raw_cmd_vel");
    nh_.param<std::string>("/cmd_topic", cmd_topic, "cmd_vel");
    nh_.param<double>("/cmd_rate", cmd_rate, 30.0);
    nh_.param<int>("/stop_counter", stop_counter, 3);

    // 新增PID参数
    nh_.param<double>("/kp_v", kp_v, 0.1);
    nh_.param<double>("/ki_v", ki_v, 0.01);
    nh_.param<double>("/kd_v", kd_v, 0.01);
    nh_.param<double>("/kp_w", kp_w, 0.1);
    nh_.param<double>("/ki_w", ki_w, 0.01);
    nh_.param<double>("/kd_w", kd_w, 0.01);

    // 新增积分饱和限制参数
    nh_.param<double>("/max_integral_v", max_integral_v, 1.0);
    nh_.param<double>("/min_integral_v", min_integral_v, -1.0);
    nh_.param<double>("/max_integral_w", max_integral_w, 1.0);
    nh_.param<double>("/min_integral_w", min_integral_w, -1.0);

    velocity_sub_ = nh_.subscribe(raw_cmd_topic, 10, &VelocitySmootherEma::twist_callback, this);
    velocity_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(cmd_topic, 10, true);
    timer = nh_.createTimer(ros::Duration(1.0 / cmd_rate), &VelocitySmootherEma::update, this);

    // 初始化PID控制器和滑动平均滤波器的变量
    previous_x_vel = 0.0;
    previous_y_vel = 0.0;
    previous_w_vel = 0.0;
    x_vel = 0.0;
    y_vel = 0.0;
    w_vel = 0.0;
    smoothed_x_vel = 0.0;
    smoothed_y_vel = 0.0;
    smoothed_w_vel = 0.0;

    // 初始化积分项和上一次误差
    i_error_x = 0.0; i_error_y = 0.0; i_error_w = 0.0;
    prev_error_x = 0.0; prev_error_y = 0.0; prev_error_w = 0.0;
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
    }
    y_vel = cmd_vel_msg_.twist.linear.y;
    w_vel = cmd_vel_msg_.twist.angular.z;
    x_vel = cmd_vel_msg_.twist.linear.x;

    smoothed_x_vel = alpha_v * x_vel + (1 - alpha_v) * previous_x_vel;
    smoothed_y_vel = alpha_v * y_vel + (1 - alpha_v) * previous_y_vel;
    smoothed_w_vel = alpha_w * w_vel + (1 - alpha_w) * previous_w_vel;

    // PID 控制逻辑
    double error_x = x_vel - smoothed_x_vel;
    double error_y = y_vel - smoothed_y_vel;
    double error_w = w_vel - smoothed_w_vel;

    i_error_x = std::max(std::min(i_error_x + error_x, max_integral_v), min_integral_v);
    i_error_y = std::max(std::min(i_error_y + error_y, max_integral_v), min_integral_v);
    i_error_w = std::max(std::min(i_error_w + error_w, max_integral_w), min_integral_w);

    double d_error_x = error_x - prev_error_x;
    double d_error_y = error_y - prev_error_y;
    double d_error_w = error_w - prev_error_w;

    smoothed_x_vel += kp_v * error_x + ki_v * i_error_x + kd_v * d_error_x;
    smoothed_y_vel += kp_v * error_y + ki_v * i_error_y + kd_v * d_error_y;
    smoothed_w_vel += kp_w * error_w + ki_w * i_error_w + kd_w * d_error_w;

    prev_error_x = error_x;
    prev_error_y = error_y;
    prev_error_w = error_w;

    cmd_vel_msg_.twist.linear.x = smoothed_x_vel;
    cmd_vel_msg_.twist.linear.y = smoothed_y_vel;
    cmd_vel_msg_.twist.angular.z = smoothed_w_vel;

    velocity_pub_.publish(cmd_vel_msg_);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "velocity_smoother_ema");

    ros::NodeHandle nh;

    VelocitySmootherEma vse(&nh);

    ros::spin();

    return 0;
}
```

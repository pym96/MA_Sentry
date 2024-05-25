```c++
if(utils::in_patrol(vehicleX, vehicleY, 1 , 1)){
    ros::Time start_time = ros::Time::now();
    double randX = utils::random_double(-1.5, 1.5);
    double randY = utils::random_double(-1.5, 1.5);

    // 创建并发布 sensor_msgs::Joy 消息
    sensor_msgs::Joy joy_message;
    joy_message.header.stamp = ros::Time::now();  // 确保时间戳一致
    joy_message.header.frame_id = "waypoint_tool";
    joy_message.axes.resize(8, 0);  // 根据实际情况调整数组大小和默认值
    joy_message.buttons.resize(11, 0);  // 同上
    joy_message.buttons[7] = 1;  // 模拟按下一个按钮
    pub_joy_.publish(joy_message);

    // 创建并发布随机 waypoint
    geometry_msgs::PointStamped random_point;
    random_point.header.stamp = ros::Time::now();
    random_point.header.frame_id = "map";
    random_point.point.x = randX;
    random_point.point.y = randY;
    ROS_INFO("Current random point is x:%f, y:%f", randX, randY);
    way_point_pub_.publish(random_point);

    usleep(10000); // 短暂等待

    return BT::NodeStatus::FAILURE;
}


```

```c++
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

class VelocitySmootherEma
{
    // 已有的成员变量和函数...

public:
    void applySCurveAdjustment(double& vel, double a, double J, const std::vector<double>& T, double Tf);

private:
    ros::Time start_time; // 记录开始加速的时间
};

void VelocitySmootherEma::applySCurveAdjustment(double& vel, double a, double J, const std::vector<double>& T, double Tf)
{
    // 如果是第一次调用，初始化start_time
    if (start_time == ros::Time(0))
    {
        start_time = ros::Time::now();
    }

    // 计算当前时间与开始加速的时间差，作为S曲线缩放函数的输入
    double t = (ros::Time::now() - start_time).toSec();

    // 计算S曲线缩放因子
    double s = SCurveScaling(t, vel, a, J, T, Tf);

    // 根据S曲线缩放因子调整速度
    vel *= s;

    // 如果达到了S曲线的总时间，重置start_time以便下一次调用
    if (t >= Tf)
    {
        start_time = ros::Time(0);
    }
}

```

```c++
double SCurveScaling(double t, double V, double A, double J, const std::vector<double>& T, double Tf) {
    double s = 0.0; // 初始化缩放因子为0

    // 根据t的值计算s
    if (t >= 0 && t <= T[0]) {
        s = 1/6.0 * J * std::pow(t, 3);
    } else if (t > T[0] && t <= T[0] + T[1]) {
        double dt = t - T[0];
        s = 1/2.0 * A * std::pow(dt, 2) + A*A/(2.0*J) * dt + A*A*A/(6.0*J*J);
    } else if (t > T[0] + T[1] && t <= T[0] + T[1] + T[2]) {
        double dt = t - T[0] - T[1];
        s = -1/6.0*J*std::pow(dt, 3) + 1/2.0*A*std::pow(dt, 2) + (A*T[1] + A*A/(2.0*J))*dt
            + 1/2.0*A*std::pow(T[1], 2) + A*A/(2.0*J)*T[1] + A*A*A/(6.0*J*J);
    } else if (t > T[0] + T[1] + T[2] && t <= T[0] + T[1] + T[2] + T[3]) {
        double dt = t - T[0] - T[1] - T[2];
        s = V*dt + (-1/6.0*J*std::pow(T[2], 3)) + 1/2.0*A*std::pow(T[2], 2) 
            + (A*T[1] + A*A/(2.0*J))*T[2] + 1/2.0*A*std::pow(T[1], 2) 
            + A*A/(2.0*J)*T[1] + A*A*A/(6.0*J*J);
    } else if (t > T[0] + T[1] + T[2] + T[3] && t <= T[0] + T[1] + T[2] + T[3] + T[4]) {
        double t_temp = Tf - t;
        double dt = t_temp - T[0] - T[1];
        s = -1/6.0*J*std::pow(dt, 3) + 1/2.0*A*std::pow(dt, 2) + (A*T[1] + A*A/(2.0*J))*dt
            + 1/2.0*A*std::pow(T[1], 2) + A*A/(2.0*J)*T[1] + A*A*A/(6.0*J*J);
        s = 1 - s;
    } else if (t > T[0] + T[1] + T[2] + T[3] + T[4] && t <= T[0] + T[1] + T[2] + T[3] + T[4] + T[5]) {
        double t_temp = Tf - t;
        double dt = t_temp - T[0];
        s = 1/2.0 * A * std::pow(dt, 2) + A*A/(2.0*J) * dt + A*A*A/(6.0*J*J);
        s = 1 - s;
    } else if (t > T[0] + T[1] + T[2] + T[3] + T[4] + T[5] && t <= Tf) {
        double t_temp = Tf - t;
        s = 1/6.0 * J * std::pow(t_temp, 3);
        s = 1 - s;
    }

    return s;
}

```

```c++
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <cmath>
#include <vector>

class VelocitySmootherEma
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber velocity_sub_;
    ros::Publisher velocity_pub_;
    ros::Timer timer;
    geometry_msgs::TwistStamped cmd_vel_msg_;

    double alpha_v;
    double alpha_w;
    std::string raw_cmd_topic;
    std::string cmd_topic;
    double cmd_rate;
    int stop_counter;

    double previous_x_vel = 0.0;
    double previous_y_vel = 0.0;
    double previous_w_vel = 0.0;
    double smoothed_x_vel = 0.0;
    double smoothed_y_vel = 0.0;
    double smoothed_w_vel = 0.0;

    // S曲线参数
    double a_x, a_y, a_w; // 加速度
    double J_x, J_y, J_w; // 加加速度
    std::vector<double> T_x, T_y, T_w; // 各阶段时间
    double Tf_x, Tf_y, Tf_w; // 总时间

    ros::Time start_time_x, start_time_y, start_time_w;

public:
    VelocitySmootherEma(ros::NodeHandle* nh): nh_(*nh) {
        nh_.param<double>("/alpha_v", alpha_v, 0.4);
        nh_.param<double>("/alpha_w", alpha_w, 0.4);
        nh_.param<std::string>("/raw_cmd_topic", raw_cmd_topic, "raw_cmd_vel");
        nh_.param<std::string>("/cmd_topic", cmd_topic, "cmd_vel");
        nh_.param<double>("/cmd_rate", cmd_rate, 30.0);
        nh_.param<int>("/stop_counter", stop_counter, 3);

        velocity_sub_ = nh_.subscribe(raw_cmd_topic, 10, &VelocitySmootherEma::twist_callback, this);
        velocity_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(cmd_topic, 10);
        timer = nh_.createTimer(ros::Duration(1.0 / cmd_rate), &VelocitySmootherEma::update, this);
    }

    ~VelocitySmootherEma() {
        ros::shutdown();
    }

    void twist_callback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        cmd_vel_msg_ = *msg;
        stop_counter = 10;
        // 重置S曲线开始时间
        start_time_x = ros::Time::now();
        start_time_y = ros::Time::now();
        start_time_w = ros::Time::now();
        // 假设这里设置了S型曲线的参数
    }

    void update(const ros::TimerEvent&) {
        if (stop_counter-- <= 0) {
            stop_counter = 0;
            cmd_vel_msg_.twist.linear.x = 0.0;
            cmd_vel_msg_.twist.linear.y = 0.0;
            cmd_vel_msg_.twist.angular.z = 0.0;
        }

        double x_vel = cmd_vel_msg_.twist.linear.x;
        double y_vel = cmd_vel_msg_.twist.linear.y;
        double w_vel = cmd_vel_msg_.twist.angular.z;

        // EMA滤波
        smoothed_x_vel = alpha_v * x_vel + (1 - alpha_v) * previous_x_vel;
        smoothed_y_vel = alpha_v * y_vel + (1 - alpha_v) * previous_y_vel;
        smoothed_w_vel = alpha_w * w_vel + (1 - alpha_w) * previous_w_vel;

        // 应用S型曲线调整
        applySCurveAdjustment(smoothed_x_vel, start_time_x, a_x, J_x, T_x, Tf_x);
        applySCurveAdjustment(smoothed_y_vel, start_time_y, a_y, J_y, T_y, Tf_y);
        applySCurveAdjustment(smoothed_w_vel, start_time_w, a_w, J_w, T_w, Tf_w);

        // 发布速度
        cmd_vel_msg_.twist.linear.x = smoothed_x_vel;
        cmd_vel_msg_.twist.linear.y = smoothed_y_vel;
        cmd_vel_msg_.twist.angular.z = smoothed_w_vel;
        velocity_pub_.publish(cmd_vel_msg_);

        previous_x_vel = smoothed_x_vel;
        previous_y_vel = smoothed_y_vel;
        previous_w_vel = smoothed_w_vel;
    }

    void applySCurveAdjustment(double& vel, ros::Time& start_time, double a, double J, const std::vector<double>& T, double Tf) {
        if (start_time == ros::Time(0)) return; // 如果没有初始化，直接返回
        double t = (ros::Time::now() - start_time).toSec();
        double s = SCurveScaling(t, vel, a, J, T, Tf);
        vel *= s;
        if (t >= Tf) start_time = ros::Time(0); // 重置时间
    }

    double SCurveScaling(double t, double V, double A, double J, const std::vector<double>& T, double Tf) {
        // 此处应实现SCurveScaling函数的C++版本，代码略
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "velocity_smoother_ema");
    ros::NodeHandle nh;
    VelocitySmootherEma vse(&nh);
    ros::spin();
    return 0;
}

```

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

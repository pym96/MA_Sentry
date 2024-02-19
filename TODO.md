```
int main(int argc, char **argv) {
    ros::init(argc, argv, "odom_to_goal_rotation");
    ros::NodeHandle nh;

    tf::TransformListener listener;
    tf::StampedTransform transform;
    try {
        // 假设odom的frame_id是 "odom"，机器人的frame_id是 "base_link"
        listener.waitForTransform("odom", "base_link", ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform("odom", "base_link", ros::Time(0), transform);
    } catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        return -1;
    }

    // 假设目标点的坐标是 (goal_x, goal_y)
    double goal_x = 10.0;
    double goal_y = 10.0;

    // 获取odom的位置和方向
    double odom_x = transform.getOrigin().x();
    double odom_y = transform.getOrigin().y();
    tf::Quaternion q = transform.getRotation();
    Eigen::Quaterniond quaternion(q.w(), q.x(), q.y(), q.z());
    Eigen::Matrix3d rotation_matrix = quaternion.toRotationMatrix();

    // 计算从odom到目标点的方向向量
    Eigen::Vector3d direction_to_goal(goal_x -
```

```
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <cmath>
#include <array>
#include <memory>

using std::placeholders::_1;
using namespace std::chrono_literals;

std::array<double, 3> axes{0.0, 0.0, 0.0};

class Commander : public rclcpp::Node
{
public:
    Commander()
    : Node("commander"), L(0.125), Rw(0.03)
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_velocity_controller/commands", 10);
        timer_ = this->create_wall_timer(5ms, std::bind(&Commander::timer_callback, this));
    }

private:
    void timer_callback()
    {
        std::array<double, 4> wheel_vel;

        double vel_x = axes[0];
        double vel_y = axes[1];
        double vel_w = axes[2];

        // Compute wheel velocities
        wheel_vel[0] = (vel_x * std::sin(M_PI / 4) + vel_y * std::cos(M_PI / 4) + L * vel_w) / Rw;
        wheel_vel[1] = (vel_x * std::sin(M_PI / 4 + M_PI / 2) + vel_y * std::cos(M_PI / 4 + M_PI / 2) + L * vel_w) / Rw;
        wheel_vel[2] = (vel_x * std::sin(M_PI / 4 - M_PI) + vel_y * std::cos(M_PI / 4 - M_PI) + L * vel_w) / Rw;
        wheel_vel[3] = (vel_x * std::sin(M_PI / 4 - M_PI / 2) + vel_y * std::cos(M_PI / 4 - M_PI / 2) + L * vel_w) / Rw;

        // Publish wheel velocities
        auto message = std_msgs::msg::Float64MultiArray();
        message.data = {wheel_vel.begin(), wheel_vel.end()};
        publisher_->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double L;
    double Rw;
};

class JoySubscriber : public rclcpp::Node
{
public:
    JoySubscriber()
    : Node("cmd_vel_subscriber")
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel_nav", 10, std::bind(&JoySubscriber::listener_callback, this, _1));
    }

private:
    void listener_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        axes[0] = -msg->linear.y;
        axes[1] = -msg->linear.x;
        axes[2] = -msg->angular.z;
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto commander = std::make_shared<Commander>();
    auto joy_subscriber = std::make_shared<JoySubscriber>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(commander);
    executor.add_node(joy_subscriber);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}
```

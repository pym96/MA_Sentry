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


Path follower
```
#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Joy.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/Dense>

using namespace std;

const double PI = 3.1415926;

double sensorOffsetX = 0;
double sensorOffsetY = 0;
int pubSkipNum = 1;
int pubSkipCount = 0;
bool twoWayDrive = true;
double lookAheadDis = 0.5;
double yawRateGain = 7.5;
double stopYawRateGain = 7.5;
double maxYawRate = 45.0;
double maxSpeed = 1.0;
double maxAccel = 1.0;
double switchTimeThre = 1.0;
double dirDiffThre = 0.1;
double stopDisThre = 0.2;
double slowDwnDisThre = 1.0;
bool useInclRateToSlow = false;
double inclRateThre = 120.0;
double slowRate1 = 0.25;
double slowRate2 = 0.5;
double slowTime1 = 2.0;
double slowTime2 = 2.0;
bool useInclToStop = false;
double inclThre = 45.0;
double stopTime = 5.0;
bool noRotAtStop = false;
bool noRotAtGoal = true;
bool autonomyMode = false;
double autonomySpeed = 1.0;
double joyToSpeedDelay = 2.0;
double currAngle = 0.0;
double vel_x = 0.0;
double vel_y = 0.0;

float joySpeed = 0;
float joySpeedRaw = 0;
float joyYaw = 0;
int safetyStop = 0;

float vehicleX = 0;
float vehicleY = 0;
float vehicleZ = 0;
float vehicleRoll = 0;
float vehiclePitch = 0;
float vehicleYaw = 0;

float vehicleXRec = 0;
float vehicleYRec = 0;
float vehicleZRec = 0;
float vehicleRollRec = 0;
float vehiclePitchRec = 0;
float vehicleYawRec = 0;

float vehicleYawRate = 0;
float vehicleSpeed = 0;

double odomTime = 0;
double joyTime = 0;
double slowInitTime = 0;
double stopInitTime = false;
int pathPointID = 0;
bool pathInit = false;
bool navFwd = true;
bool isReach = false;
double switchTime = 0;
double goalPointX = 0.0;
double goalPointY = 0.0;


// Calculate the rotation matrix
Eigen::Matrix2d goal2map;
Eigen::Matrix2d map2odom_rotation;
Eigen::Matrix2d goal2odom;
Eigen::Matrix2cd odom2goal;
tf::StampedTransform map2odom;

nav_msgs::Path path;
geometry_msgs::Point goal_point;  // Declare goal_point globally

bool hasReached(double x, double y, const geometry_msgs::Point& goal) {
    double distance = pow(x - goal.x, 2) + pow(y - goal.y, 2);
    if (distance <= stopDisThre) return true;
    return false;
}

void odomHandler(const nav_msgs::Odometry::ConstPtr& odomIn)
{
  odomTime = odomIn->header.stamp.toSec();

  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odomIn->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  vehicleRoll = roll;
  vehiclePitch = pitch;
  vehicleYaw = yaw;
  vehicleX = odomIn->pose.pose.position.x - cos(yaw) * sensorOffsetX + sin(yaw) * sensorOffsetY;
  vehicleY = odomIn->pose.pose.position.y - sin(yaw) * sensorOffsetX - cos(yaw) * sensorOffsetY;
  vehicleZ = odomIn->pose.pose.position.z;

  //rad, too steep to move
  if ((fabs(roll) > inclThre * PI / 180.0 || fabs(pitch) > inclThre * PI / 180.0) && useInclToStop) {
    stopInitTime = odomIn->header.stamp.toSec();
  }

  if ((fabs(odomIn->twist.twist.angular.x) > inclRateThre * PI / 180.0 || fabs(odomIn->twist.twist.angular.y) > inclRateThre * PI / 180.0) && useInclRateToSlow) {
    slowInitTime = odomIn->header.stamp.toSec();
  }
}

void pathHandler(const nav_msgs::Path::ConstPtr& pathIn)
{
  int pathSize = pathIn->poses.size();
  path.poses.resize(pathSize);
  for (int i = 0; i < pathSize; i++) {
    path.poses[i].pose.position.x = pathIn->poses[i].pose.position.x;
    path.poses[i].pose.position.y = pathIn->poses[i].pose.position.y;
    path.poses[i].pose.position.z = pathIn->poses[i].pose.position.z;
  }

  vehicleXRec = vehicleX;
  vehicleYRec = vehicleY;
  vehicleZRec = vehicleZ;
  vehicleRollRec = vehicleRoll;
  vehiclePitchRec = vehiclePitch;
  vehicleYawRec = vehicleYaw;

  pathPointID = 0;
  pathInit = true;
}

void joystickHandler(const sensor_msgs::Joy::ConstPtr& joy)
{
  joyTime = ros::Time::now().toSec();

  joySpeedRaw = sqrt(joy->axes[3] * joy->axes[3] + joy->axes[4] * joy->axes[4]);
  joySpeed = joySpeedRaw;
  if (joySpeed > 1.0) joySpeed = 1.0;
  if (joy->axes[4] == 0) joySpeed = 0;
  joyYaw = joy->axes[3];
  if (joySpeed == 0 && noRotAtStop) joyYaw = 0;

  if (joy->axes[4] < 0 && !twoWayDrive) {
    joySpeed = 0;
    joyYaw = 0;
  }

  if (joy->axes[2] > -0.1) {
    autonomyMode = false;
  } else {
    autonomyMode = true;
  }
}

void speedHandler(const std_msgs::Float32::ConstPtr& speed)
{
  double speedTime = ros::Time::now().toSec();

  if (autonomyMode && speedTime - joyTime > joyToSpeedDelay && joySpeedRaw == 0) {
    joySpeed = speed->data / maxSpeed;

    if (joySpeed < 0) joySpeed = 0;
    else if (joySpeed > 1.0) joySpeed = 1.0;
  }
}

void stopHandler(const std_msgs::Int8::ConstPtr& stop)
{
  safetyStop = stop->data;
}


void navHandler(const geometry_msgs::Point::ConstPtr& msg){
    goal_point = *msg;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pathFollower");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("sensorOffsetX", sensorOffsetX);
  nhPrivate.getParam("sensorOffsetY", sensorOffsetY);
  nhPrivate.getParam("pubSkipNum", pubSkipNum);
  nhPrivate.getParam("twoWayDrive", twoWayDrive);
  nhPrivate.getParam("lookAheadDis", lookAheadDis);
  nhPrivate.getParam("yawRateGain", yawRateGain);
  nhPrivate.getParam("stopYawRateGain", stopYawRateGain);
  nhPrivate.getParam("maxYawRate", maxYawRate);
  nhPrivate.getParam("maxSpeed", maxSpeed);
  nhPrivate.getParam("maxAccel", maxAccel);
  nhPrivate.getParam("switchTimeThre", switchTimeThre);
  nhPrivate.getParam("dirDiffThre", dirDiffThre);
  nhPrivate.getParam("stopDisThre", stopDisThre);
  nhPrivate.getParam("slowDwnDisThre", slowDwnDisThre);
  nhPrivate.getParam("useInclRateToSlow", useInclRateToSlow);
  nhPrivate.getParam("inclRateThre", inclRateThre);
  nhPrivate.getParam("slowRate1", slowRate1);
  nhPrivate.getParam("slowRate2", slowRate2);
  nhPrivate.getParam("slowTime1", slowTime1);
  nhPrivate.getParam("slowTime2", slowTime2);
  nhPrivate.getParam("useInclToStop", useInclToStop);
  nhPrivate.getParam("inclThre", inclThre);
  nhPrivate.getParam("stopTime", stopTime);
  nhPrivate.getParam("noRotAtStop", noRotAtStop);
  nhPrivate.getParam("noRotAtGoal", noRotAtGoal);
  nhPrivate.getParam("autonomyMode", autonomyMode);
  nhPrivate.getParam("autonomySpeed", autonomySpeed);
  nhPrivate.getParam("joyToSpeedDelay", joyToSpeedDelay);

  ros::Subscriber subOdom = nh.subscribe<nav_msgs::Odometry> ("/state_estimation", 5, odomHandler);

  ros::Subscriber subPath = nh.subscribe<nav_msgs::Path> ("/path", 5, pathHandler);

  ros::Subscriber subJoystick = nh.subscribe<sensor_msgs::Joy> ("/joy", 5, joystickHandler);

  ros::Subscriber subSpeed = nh.subscribe<std_msgs::Float32> ("/speed", 5, speedHandler);

  ros::Subscriber subStop = nh.subscribe<std_msgs::Int8> ("/stop", 5, stopHandler);

  ros::Subscriber goalPoint = nh.subscribe<geometry_msgs::Point>("/way_point", 5, navHandler);

  ros::Publisher pubSpeed = nh.advertise<geometry_msgs::TwistStamped> ("/cmd_vel", 5);
  geometry_msgs::TwistStamped cmd_vel;

  tf::TransformListener listener(ros::Duration(60)); // Increase the buffer size if needed
  cmd_vel.header.frame_id = "vehicle";

  if (autonomyMode) {
    joySpeed = autonomySpeed / maxSpeed;

    if (joySpeed < 0) joySpeed = 0;
    else if (joySpeed > 1.0) joySpeed = 1.0;
  }

  ros::Rate rate(100);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();

    if (pathInit) {
      float vehicleXRel = cos(vehicleYawRec) * (vehicleX - vehicleXRec) 
                        + sin(vehicleYawRec) * (vehicleY - vehicleYRec);
      float vehicleYRel = -sin(vehicleYawRec) * (vehicleX - vehicleXRec) 
                        + cos(vehicleYawRec) * (vehicleY - vehicleYRec);

      int pathSize = path.poses.size();
      float endDisX = path.poses[pathSize - 1].pose.position.x - vehicleXRel;
      float endDisY = path.poses[pathSize - 1].pose.position.y - vehicleYRel;
      float endDis = sqrt(endDisX * endDisX + endDisY * endDisY);

      isReach = hasReached(vehicleXRel, vehicleYRel, goal_point);

      // if (isReach) {
      //     cmd_vel.twist.linear.x = 0;
      //     cmd_vel.twist.linear.y = 0;
      //     pubSpeed.publish(cmd_vel); 
      //     pathInit = false;
      //     continue;
      // }

      float disX, disY, dis;
      while (pathPointID < pathSize - 1) {
        disX = path.poses[pathPointID].pose.position.x - vehicleXRel;
        disY = path.poses[pathPointID].pose.position.y - vehicleYRel;
        dis = sqrt(disX * disX + disY * disY);
        if (dis < lookAheadDis) {
          pathPointID++;
        } else {
          break;
        }
      }

      // Get segmented moving direction
      float pathDir = atan2(disY, disX);

      // Set rotation matrix
      // TODO: give it to velocity
      // goal2map << cos(pathDir), -sin(pathDir),
      //             sin(pathDir), cos(pathDir);


    try {
        // The code where you perform the transform lookup causing the exception
        listener.lookupTransform("map", "vehicle", ros::Time(odomTime), map2odom);
        
      } catch (tf::TransformException& ex) {
        // Handle the exception
        ROS_ERROR("Transform lookup failed: %s", ex.what());
      }


      #if 1

        // tf::Quaternion q = map2odom.getRotation();

        // Eigen::Quaterniond quaternion(q.w(), q.x(), q.y(), q.z());
        // Eigen::Matrix3d roatation_matrix = quaternion.toRotationMatrix();

        // Dir vector
        Eigen::Vector3d direction_to_goal(disX, disY, 0);

        // Normalize goal dir vector
        Eigen::Vector3d unit_direction_to_goal = direction_to_goal.normalized();

        double angle = atan2(unit_direction_to_goal.y(), unit_direction_to_goal.x());

        // 计算当前机器人朝向（基于vehicleYaw）
        Eigen::Vector3d current_direction(cos(vehicleYaw), sin(vehicleYaw), 0);

            // 计算朝向与目标方向之间的角度差
        double angle_diff = atan2(current_direction.y(), current_direction.x()) 
                         - atan2(unit_direction_to_goal.y(), unit_direction_to_goal.x());

        Eigen::Matrix3d nor_rotation_matrix;
        nor_rotation_matrix = Eigen::AngleAxisd(angle_diff, Eigen::Vector3d::UnitZ());

      #endif


      // Extract the rotation matrix
      tf::Matrix3x3 rotation_matrix = map2odom.getBasis();

      double roll, pitch, yaw;
      
      rotation_matrix.getRPY(roll, pitch, yaw);

      yaw = yaw * M_PI / 180.0;
            

      map2odom_rotation << cos(yaw), -sin(yaw),
                           sin(yaw), cos(yaw); 

      goal2odom = goal2map * map2odom_rotation;

      // TODO: publish velocity directly, using rotation matrix

      float dirDiff = vehicleYaw - vehicleYawRec - pathDir;
      if (dirDiff > PI) dirDiff -= 2 * PI;
      else if (dirDiff < -PI) dirDiff += 2 * PI;
      if (dirDiff > PI) dirDiff -= 2 * PI;
      else if (dirDiff < -PI) dirDiff += 2 * PI;

      if (twoWayDrive) {
        double time = ros::Time::now().toSec();
        if (fabs(dirDiff) > PI / 2 && navFwd && time - switchTime > switchTimeThre) {
          navFwd = false;
          switchTime = time;
        } else if (fabs(dirDiff) < PI / 2 && !navFwd && time - switchTime > switchTimeThre) {
          navFwd = true;
          switchTime = time;
        }
      }

      float joySpeed2 = maxSpeed * joySpeed;


      if (pathSize <= 1 || (dis < stopDisThre && noRotAtGoal)) {
        // Rotating
        vehicleYawRate = 0;
      }

      if (pathSize <= 1) {
        joySpeed2 = 0;
      } else if (endDis / slowDwnDisThre < joySpeed) {
        joySpeed2 *= endDis / slowDwnDisThre;
      }

      float joySpeed3 = joySpeed2;
      if (odomTime < slowInitTime + slowTime1 && slowInitTime > 0) joySpeed3 *= slowRate1;
      else if (odomTime < slowInitTime + slowTime1 + slowTime2 && slowInitTime > 0) joySpeed3 *= slowRate2;


      if (fabs(vehicleSpeed) < maxSpeed) vehicleSpeed += maxAccel / 100.0;
      else vehicleSpeed = vehicleSpeed;  

      
      if (odomTime < stopInitTime + stopTime && stopInitTime > 0) {
        vehicleSpeed = 0;
        vehicleYawRate = 0;
      }

      pubSkipCount--;
      // ROS_INFO("Current pub skit count is: %d", pubSkipCount);
      if (pubSkipCount < 0) {
        cmd_vel.header.stamp = ros::Time(odomTime);

        odom2goal = goal2odom.transpose();

        Eigen::Vector3d vel(vehicleSpeed, 0, 0);

        Eigen::Vector3d rotated_velocity = nor_rotation_matrix * vel;

        cmd_vel.twist.linear.x = rotated_velocity.x();
        cmd_vel.twist.linear.y = rotated_velocity.y();    

        pubSpeed.publish(cmd_vel);

        pubSkipCount = pubSkipNum;
      }
    }

    status = ros::ok();
    rate.sleep();
  }

  return 0;
}
```

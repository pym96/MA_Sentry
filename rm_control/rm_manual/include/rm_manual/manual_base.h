//
// Created by peter on 2020/12/3.
//

#pragma once

#include <queue>
#include <iostream>
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/Bool.h>
#include <tf2_ros/buffer.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <controller_manager_msgs/SwitchController.h>

#include <rm_common/ori_tool.h>
#include <rm_common/ros_utilities.h>
#include <rm_common/decision/command_sender.h>
#include <rm_common/decision/controller_manager.h>
#include <rm_common/decision/calibration_queue.h>

#include <rm_msgs/DbusData.h>
#include <rm_msgs/TrackData.h>
#include <rm_msgs/GameStatus.h>
#include <rm_msgs/GameRobotHp.h>
#include <rm_msgs/BalanceState.h>
#include <rm_msgs/PowerHeatData.h>
#include <rm_msgs/ActuatorState.h>
#include <rm_msgs/GimbalDesError.h>
#include <rm_msgs/GameRobotStatus.h>
#include <rm_msgs/ManualToReferee.h>
#include <rm_msgs/PowerManagementSampleAndStatusData.h>
#include "rm_manual/input_event.h"

namespace rm_manual
{
class ManualBase
{
public:
  ManualBase(ros::NodeHandle& nh, ros::NodeHandle& nh_referee);
  enum
  {
    PASSIVE,
    IDLE,
    RC,
    PC
  };
  virtual void run();

protected:
  virtual void checkReferee();
  virtual void checkKeyboard(const rm_msgs::DbusData::ConstPtr& data){};
  virtual void updateRc(const rm_msgs::DbusData::ConstPtr& dbus_data);
  virtual void updatePc(const rm_msgs::DbusData::ConstPtr& dbus_data);
  virtual void sendCommand(const ros::Time& time) = 0;
  virtual void updateActuatorStamp(const rm_msgs::ActuatorState::ConstPtr& data, std::vector<std::string> act_vector,
                                   ros::Time& last_get_stamp);

  virtual void jointStateCallback(const sensor_msgs::JointState::ConstPtr& data);
  virtual void dbusDataCallback(const rm_msgs::DbusData::ConstPtr& data);
  virtual void trackCallback(const rm_msgs::TrackData::ConstPtr& data);
  virtual void gameRobotStatusCallback(const rm_msgs::GameRobotStatus::ConstPtr& data);
  virtual void powerHeatDataCallback(const rm_msgs::PowerHeatData::ConstPtr& data);
  virtual void capacityDataCallback(const rm_msgs::PowerManagementSampleAndStatusData::ConstPtr& data)
  {
  }
  virtual void gimbalDesErrorCallback(const rm_msgs::GimbalDesError::ConstPtr& data)
  {
  }
  virtual void odomCallback(const nav_msgs::Odometry::ConstPtr& data)
  {
  }
  virtual void actuatorStateCallback(const rm_msgs::ActuatorState::ConstPtr& data);
  virtual void gameRobotHpCallback(const rm_msgs::GameRobotHp::ConstPtr& data)
  {
  }
  virtual void gameStatusCallback(const rm_msgs::GameStatus::ConstPtr& data)
  {
  }
  virtual void suggestFireCallback(const std_msgs::Bool::ConstPtr& data)
  {
  }

  // Referee
  virtual void chassisOutputOn()
  {
    ROS_INFO("Chassis output ON");
  }
  virtual void gimbalOutputOn()
  {
    ROS_INFO("Gimbal output ON");
  }
  virtual void shooterOutputOn()
  {
    ROS_INFO("Shooter output ON");
  }
  virtual void robotDie();
  virtual void robotRevive();

  // Remote Controller
  virtual void remoteControlTurnOff();
  virtual void remoteControlTurnOn();
  virtual void leftSwitchDownRise(){};
  virtual void leftSwitchMidRise(){};
  virtual void leftSwitchMidFall(){};
  virtual void leftSwitchUpRise(){};
  virtual void rightSwitchDownRise()
  {
    state_ = IDLE;
  }
  virtual void rightSwitchDownOn()
  {
    state_ = IDLE;
  }
  virtual void rightSwitchMidRise()
  {
    state_ = RC;
  }
  virtual void rightSwitchMidOn()
  {
    state_ = RC;
  }
  virtual void rightSwitchUpRise()
  {
    state_ = PC;
  }
  virtual void rightSwitchUpOn()
  {
    state_ = PC;
  }

  ros::Publisher manual_to_referee_pub_;

  ros::Subscriber odom_sub_, dbus_sub_, track_sub_, referee_sub_, capacity_sub_, game_status_sub_, joint_state_sub_,
      game_robot_hp_sub_, actuator_state_sub_, power_heat_data_sub_, gimbal_des_error_sub_, game_robot_status_sub_,
      suggest_fire_sub_;

  sensor_msgs::JointState joint_state_;
  rm_msgs::TrackData track_data_;
  rm_msgs::ManualToReferee manual_to_referee_pub_data_;

  rm_common::ControllerManager controller_manager_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::NodeHandle nh_;

  ros::Time referee_last_get_stamp_;
  bool remote_is_open_{}, referee_is_online_ = false;
  int state_ = PASSIVE;
  int robot_id_, chassis_power_;
  int chassis_output_on_ = 0, gimbal_output_on_ = 0, shooter_output_on_ = 0;
  InputEvent robot_hp_event_, right_switch_down_event_, right_switch_mid_event_, right_switch_up_event_,
      left_switch_down_event_, left_switch_mid_event_, left_switch_up_event_;

  InputEvent chassis_power_on_event_, gimbal_power_on_event_, shooter_power_on_event_;
  ros::Time chassis_actuator_last_get_stamp_, gimbal_actuator_last_get_stamp_, shooter_actuator_last_get_stamp_;
  std::vector<std::string> chassis_mount_motor_, gimbal_mount_motor_, shooter_mount_motor_;
};

}  // namespace rm_manual

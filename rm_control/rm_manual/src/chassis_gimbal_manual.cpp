//
// Created by peter on 2021/7/22.
//

#include "rm_manual/chassis_gimbal_manual.h"

namespace rm_manual
{
ChassisGimbalManual::ChassisGimbalManual(ros::NodeHandle& nh, ros::NodeHandle& nh_referee) : ManualBase(nh, nh_referee)
{
  ros::NodeHandle chassis_nh(nh, "chassis");
  chassis_cmd_sender_ = new rm_common::ChassisCommandSender(chassis_nh);
  if (!chassis_nh.getParam("speed_change_scale", speed_change_scale_))
    speed_change_scale_ = 1.;
  ros::NodeHandle vel_nh(nh, "vel");
  vel_cmd_sender_ = new rm_common::Vel2DCommandSender(vel_nh);
  if (!vel_nh.getParam("gyro_move_reduction", gyro_move_reduction_))
    ROS_ERROR("Gyro move reduction no defined (namespace: %s)", nh.getNamespace().c_str());
  if (!vel_nh.getParam("gyro_rotate_reduction", gyro_rotate_reduction_))
    ROS_ERROR("Gyro rotate reduction no defined (namespace: %s)", nh.getNamespace().c_str());
  ros::NodeHandle gimbal_nh(nh, "gimbal");
  gimbal_cmd_sender_ = new rm_common::GimbalCommandSender(gimbal_nh);
  gimbal_scale_ = getParam(gimbal_nh, "gimbal_scale", 1.0);
  if (!gimbal_nh.getParam("finish_turning_threshold", finish_turning_threshold_))
    ROS_ERROR("Finish turning threshold no defined (namespace: %s)", nh.getNamespace().c_str());

  chassis_power_on_event_.setRising(boost::bind(&ChassisGimbalManual::chassisOutputOn, this));
  gimbal_power_on_event_.setRising(boost::bind(&ChassisGimbalManual::gimbalOutputOn, this));
  w_event_.setEdge(boost::bind(&ChassisGimbalManual::wPress, this), boost::bind(&ChassisGimbalManual::wRelease, this));
  w_event_.setActiveHigh(boost::bind(&ChassisGimbalManual::wPressing, this));
  s_event_.setEdge(boost::bind(&ChassisGimbalManual::sPress, this), boost::bind(&ChassisGimbalManual::sRelease, this));
  s_event_.setActiveHigh(boost::bind(&ChassisGimbalManual::sPressing, this));
  a_event_.setEdge(boost::bind(&ChassisGimbalManual::aPress, this), boost::bind(&ChassisGimbalManual::aRelease, this));
  a_event_.setActiveHigh(boost::bind(&ChassisGimbalManual::aPressing, this));
  d_event_.setEdge(boost::bind(&ChassisGimbalManual::dPress, this), boost::bind(&ChassisGimbalManual::dRelease, this));
  d_event_.setActiveHigh(boost::bind(&ChassisGimbalManual::dPressing, this));
}

void ChassisGimbalManual::sendCommand(const ros::Time& time)
{
  chassis_cmd_sender_->sendChassisCommand(time, is_gyro_);
  vel_cmd_sender_->sendCommand(time);
  gimbal_cmd_sender_->sendCommand(time);
}

void ChassisGimbalManual::updateRc(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ManualBase::updateRc(dbus_data);
  gimbal_cmd_sender_->setRate(-dbus_data->ch_l_x, -dbus_data->ch_l_y);
}
void ChassisGimbalManual::updatePc(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ManualBase::updatePc(dbus_data);
  gimbal_cmd_sender_->setRate(-dbus_data->m_x * gimbal_scale_, dbus_data->m_y * gimbal_scale_);
}

void ChassisGimbalManual::checkReferee()
{
  ManualBase::checkReferee();
}

void ChassisGimbalManual::checkKeyboard(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ManualBase::checkKeyboard(dbus_data);
  if (robot_id_ == rm_msgs::GameRobotStatus::RED_ENGINEER || robot_id_ == rm_msgs::GameRobotStatus::BLUE_ENGINEER)
  {
    w_event_.update((!dbus_data->key_ctrl) && (!dbus_data->key_shift) && dbus_data->key_w);
    s_event_.update((!dbus_data->key_ctrl) && (!dbus_data->key_shift) && dbus_data->key_s);
    a_event_.update((!dbus_data->key_ctrl) && (!dbus_data->key_shift) && dbus_data->key_a);
    d_event_.update((!dbus_data->key_ctrl) && (!dbus_data->key_shift) && dbus_data->key_d);
  }
  else
  {
    w_event_.update(dbus_data->key_w);
    s_event_.update(dbus_data->key_s);
    a_event_.update(dbus_data->key_a);
    d_event_.update(dbus_data->key_d);
  }
  if (dbus_data->m_z != 0)
  {
    mouseMidRise(dbus_data->m_z);
  }
}

void ChassisGimbalManual::gameStatusCallback(const rm_msgs::GameStatus::ConstPtr& data)
{
  ManualBase::gameStatusCallback(data);
  chassis_cmd_sender_->updateGameStatus(*data);
}

void ChassisGimbalManual::gameRobotStatusCallback(const rm_msgs::GameRobotStatus::ConstPtr& data)
{
  ManualBase::gameRobotStatusCallback(data);
  chassis_cmd_sender_->updateGameRobotStatus(*data);
}

void ChassisGimbalManual::powerHeatDataCallback(const rm_msgs::PowerHeatData::ConstPtr& data)
{
  ManualBase::powerHeatDataCallback(data);
  chassis_cmd_sender_->updatePowerHeatData(*data);
}

void ChassisGimbalManual::dbusDataCallback(const rm_msgs::DbusData::ConstPtr& data)
{
  ManualBase::dbusDataCallback(data);
  chassis_cmd_sender_->updateRefereeStatus(referee_is_online_);
}

void ChassisGimbalManual::capacityDataCallback(const rm_msgs::PowerManagementSampleAndStatusData ::ConstPtr& data)
{
  ManualBase::capacityDataCallback(data);
  chassis_cmd_sender_->updateCapacityData(*data);
}

void ChassisGimbalManual::trackCallback(const rm_msgs::TrackData::ConstPtr& data)
{
  ManualBase::trackCallback(data);
}

void ChassisGimbalManual::remoteControlTurnOff()
{
  ManualBase::remoteControlTurnOff();
  vel_cmd_sender_->setZero();
  chassis_cmd_sender_->setZero();
  gimbal_cmd_sender_->setZero();
}

void ChassisGimbalManual::rightSwitchDownRise()
{
  ManualBase::rightSwitchDownRise();
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
  vel_cmd_sender_->setZero();
  gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
  gimbal_cmd_sender_->setZero();
}

void ChassisGimbalManual::rightSwitchMidRise()
{
  ManualBase::rightSwitchMidRise();
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
  gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
}

void ChassisGimbalManual::rightSwitchUpRise()
{
  ManualBase::rightSwitchUpRise();
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
  vel_cmd_sender_->setZero();
  gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
}

void ChassisGimbalManual::leftSwitchDownRise()
{
  ManualBase::leftSwitchDownRise();
  gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
}

void ChassisGimbalManual::wPressing()
{
  double final_x_scale = x_scale_ * speed_change_scale_;
  vel_cmd_sender_->setLinearXVel(is_gyro_ ? final_x_scale * gyro_move_reduction_ : final_x_scale);
}

void ChassisGimbalManual::aPressing()
{
  double final_y_scale = y_scale_ * speed_change_scale_;
  vel_cmd_sender_->setLinearYVel(is_gyro_ ? final_y_scale * gyro_move_reduction_ : final_y_scale);
}

void ChassisGimbalManual::sPressing()
{
  double final_x_scale = x_scale_ * speed_change_scale_;
  vel_cmd_sender_->setLinearXVel(is_gyro_ ? final_x_scale * gyro_move_reduction_ : final_x_scale);
}

void ChassisGimbalManual::dPressing()
{
  double final_y_scale = y_scale_ * speed_change_scale_;
  vel_cmd_sender_->setLinearYVel(is_gyro_ ? final_y_scale * gyro_move_reduction_ : final_y_scale);
}

void ChassisGimbalManual::wRelease()
{
  x_scale_ = x_scale_ <= -1.0 ? -1.0 : x_scale_ - 1.0;
  vel_cmd_sender_->setLinearXVel(x_scale_);
}

void ChassisGimbalManual::sRelease()
{
  x_scale_ = x_scale_ >= 1.0 ? 1.0 : x_scale_ + 1.0;
  vel_cmd_sender_->setLinearXVel(x_scale_);
}

void ChassisGimbalManual::aRelease()
{
  y_scale_ = y_scale_ <= -1.0 ? -1.0 : y_scale_ - 1.0;
  vel_cmd_sender_->setLinearYVel(y_scale_);
}

void ChassisGimbalManual::dRelease()
{
  y_scale_ = y_scale_ >= 1.0 ? 1.0 : y_scale_ + 1.0;
  vel_cmd_sender_->setLinearYVel(y_scale_);
}

void ChassisGimbalManual::mouseMidRise(double m_z)
{
  if (gimbal_scale_ >= 1. && gimbal_scale_ <= 30.)
  {
    if (gimbal_scale_ + 1. <= 30. && m_z > 0.)
      gimbal_scale_ += 1.;
    else if (gimbal_scale_ - 1. >= 1. && m_z < 0.)
      gimbal_scale_ -= 1.;
  }
}

}  // namespace rm_manual

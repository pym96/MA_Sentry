//
// Created by qiayuan on 5/22/21.
//

#pragma once

#include "rm_manual/manual_base.h"

namespace rm_manual
{
class ChassisGimbalManual : public ManualBase
{
public:
  ChassisGimbalManual(ros::NodeHandle& nh, ros::NodeHandle& nh_referee);

protected:
  void sendCommand(const ros::Time& time) override;
  void updateRc(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void updatePc(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void checkReferee() override;
  void checkKeyboard(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void remoteControlTurnOff() override;
  void rightSwitchDownRise() override;
  void rightSwitchMidRise() override;
  void rightSwitchUpRise() override;
  void leftSwitchDownRise() override;
  void gameStatusCallback(const rm_msgs::GameStatus::ConstPtr& data) override;
  void gameRobotStatusCallback(const rm_msgs::GameRobotStatus::ConstPtr& data) override;
  void powerHeatDataCallback(const rm_msgs::PowerHeatData::ConstPtr& data) override;
  void dbusDataCallback(const rm_msgs::DbusData::ConstPtr& data) override;
  void capacityDataCallback(const rm_msgs::PowerManagementSampleAndStatusData ::ConstPtr& data) override;
  void trackCallback(const rm_msgs::TrackData::ConstPtr& data) override;
  virtual void wPress()
  {
    x_scale_ = x_scale_ >= 1.0 ? 1.0 : x_scale_ + 1.0;
  }
  virtual void wRelease();
  virtual void sPress()
  {
    x_scale_ = x_scale_ <= -1.0 ? -1.0 : x_scale_ - 1.0;
  }
  virtual void sRelease();
  virtual void aPress()
  {
    y_scale_ = y_scale_ >= 1.0 ? 1.0 : y_scale_ + 1.0;
  }
  virtual void aRelease();
  virtual void dPress()
  {
    y_scale_ = y_scale_ <= -1.0 ? -1.0 : y_scale_ - 1.0;
  }
  virtual void dRelease();
  virtual void wPressing();
  virtual void aPressing();
  virtual void sPressing();
  virtual void dPressing();
  void mouseMidRise(double m_z);

  rm_common::Vel2DCommandSender* vel_cmd_sender_{};
  rm_common::GimbalCommandSender* gimbal_cmd_sender_{};
  rm_common::ChassisCommandSender* chassis_cmd_sender_{};

  double x_scale_{}, y_scale_{};
  bool is_gyro_{ 0 };
  double speed_change_scale_{ 1. };
  double gimbal_scale_{ 1. };
  double gyro_move_reduction_{ 1. };
  double gyro_rotate_reduction_{ 1. };
  double finish_turning_threshold_{};

  InputEvent w_event_, s_event_, a_event_, d_event_;
};
}  // namespace rm_manual

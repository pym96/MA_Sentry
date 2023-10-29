//
// Created by qiayuan on 5/22/21.
//

#pragma once

#include "rm_manual/chassis_gimbal_manual.h"
#include <rm_common/decision/calibration_queue.h>
#include <angles/angles.h>

namespace rm_manual
{
class ChassisGimbalShooterManual : public ChassisGimbalManual
{
public:
  ChassisGimbalShooterManual(ros::NodeHandle& nh, ros::NodeHandle& nh_referee);
  void run() override;

protected:
  void checkReferee() override;
  void checkKeyboard(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void updateRc(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void updatePc(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void sendCommand(const ros::Time& time) override;
  void chassisOutputOn() override;
  void shooterOutputOn() override;
  void gimbalOutputOn() override;
  void selfInspectionStart()
  {
    shooter_calibration_->reset();
  };
  void gameStart()
  {
    shooter_calibration_->reset();
  };
  void remoteControlTurnOff() override;
  void remoteControlTurnOn() override;
  void robotDie() override;
  void rightSwitchDownRise() override;
  void rightSwitchMidRise() override;
  void rightSwitchUpRise() override;
  void leftSwitchDownRise() override;
  void leftSwitchMidRise() override;
  void leftSwitchMidOn(ros::Duration duration);
  void leftSwitchUpRise() override;
  void gameRobotStatusCallback(const rm_msgs::GameRobotStatus::ConstPtr& data) override;
  void powerHeatDataCallback(const rm_msgs::PowerHeatData::ConstPtr& data) override;
  void dbusDataCallback(const rm_msgs::DbusData::ConstPtr& data) override;
  void gameStatusCallback(const rm_msgs::GameStatus::ConstPtr& data) override;
  void gimbalDesErrorCallback(const rm_msgs::GimbalDesError::ConstPtr& data) override;
  void suggestFireCallback(const std_msgs::Bool::ConstPtr& data) override;
  void trackCallback(const rm_msgs::TrackData::ConstPtr& data) override;
  void leftSwitchUpOn(ros::Duration duration);
  void mouseLeftPress();
  void mouseLeftRelease()
  {
    shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
    prepare_shoot_ = true;
  }
  void mouseRightPress();
  void mouseRightRelease()
  {
    gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
    if (shooter_cmd_sender_->getMsg()->mode == rm_msgs::ShootCmd::PUSH)
      shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
  }
  void wPress() override;
  void aPress() override;
  void sPress() override;
  void dPress() override;
  void wPressing() override;
  void aPressing() override;
  void sPressing() override;
  void dPressing() override;
  void wRelease() override;
  void aRelease() override;
  void sRelease() override;
  void dRelease() override;
  virtual void gPress();
  virtual void vPress();
  virtual void xPress();
  virtual void ePress();
  virtual void eRelease();
  virtual void cPress();
  virtual void bPress();
  virtual void rPress();
  virtual void xReleasing();
  virtual void shiftPress();
  virtual void shiftRelease();
  void qPress()
  {
    shooter_cmd_sender_->setShootFrequency(rm_common::HeatLimit::BURST);
  }
  void qRelease()
  {
    shooter_cmd_sender_->setShootFrequency(rm_common::HeatLimit::LOW);
  }
  void fPress()
  {
    shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
  }
  void ctrlVPress();
  void ctrlBPress();
  void ctrlRPress();
  virtual void ctrlQPress();

  InputEvent self_inspection_event_, game_start_event_, e_event_, c_event_, g_event_, q_event_, f_event_, b_event_,
      x_event_, r_event_, v_event_, ctrl_v_event_, ctrl_b_event_, ctrl_q_event_, ctrl_r_event_, shift_event_,
      ctrl_shift_b_event_, mouse_left_event_, mouse_right_event_;
  rm_common::ShooterCommandSender* shooter_cmd_sender_{};
  rm_common::CameraSwitchCommandSender* camera_switch_cmd_sender_{};
  rm_common::JointPositionBinaryCommandSender* scope_cmd_sender_{};
  rm_common::JointPositionBinaryCommandSender* image_transmission_cmd_sender_{};
  rm_common::SwitchDetectionCaller* switch_detection_srv_{};
  rm_common::SwitchDetectionCaller* switch_armor_target_srv_{};
  rm_common::CalibrationQueue* shooter_calibration_;
  rm_common::CalibrationQueue* gimbal_calibration_;

  geometry_msgs::PointStamped point_out_;

  bool prepare_shoot_ = false, turn_flag_ = false, is_balance_ = false, use_scope_ = false,
       adjust_image_transmission_ = false;
  double yaw_current_{};
};
}  // namespace rm_manual

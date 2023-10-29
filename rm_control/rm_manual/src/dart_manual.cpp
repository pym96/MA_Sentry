//
// Created by luotinkai on 2022/7/15.
//

#include "rm_manual/dart_manual.h"

namespace rm_manual
{
DartManual::DartManual(ros::NodeHandle& nh, ros::NodeHandle& nh_referee) : ManualBase(nh, nh_referee)
{
  XmlRpc::XmlRpcValue dart_list, targets, launch_id, trigger_position;
  nh.getParam("launch_id", launch_id);
  nh.getParam("trigger_position", trigger_position);
  nh.getParam("dart_list", dart_list);
  nh.getParam("targets", targets);
  getList(dart_list, targets, launch_id, trigger_position);
  qd_ = dart_list_[0].outpost_qd_;
  dart_list_[4].outpost_qd_ = dart_list_[3].outpost_qd_;
  dart_list_[4].base_qd_ = dart_list_[3].base_qd_;
  ros::NodeHandle nh_yaw = ros::NodeHandle(nh, "yaw");
  ros::NodeHandle nh_left_pitch = ros::NodeHandle(nh, "left_pitch");
  yaw_sender_ = new rm_common::JointPointCommandSender(nh_yaw, joint_state_);
  pitch_sender_ = new rm_common::JointPointCommandSender(nh_left_pitch, joint_state_);
  ros::NodeHandle nh_trigger = ros::NodeHandle(nh, "trigger");
  ros::NodeHandle nh_friction_left = ros::NodeHandle(nh, "friction_left");
  ros::NodeHandle nh_friction_right = ros::NodeHandle(nh, "friction_right");
  scale_ = getParam(nh_left_pitch, "scale", 0.);
  scale_micro_ = getParam(nh_left_pitch, "scale_micro", 0.);
  upward_vel_ = getParam(nh_trigger, "upward_vel", 0.);
  trigger_sender_ = new rm_common::JointPointCommandSender(nh_trigger, joint_state_);
  friction_left_sender_ = new rm_common::JointPointCommandSender(nh_friction_left, joint_state_);
  friction_right_sender_ = new rm_common::JointPointCommandSender(nh_friction_right, joint_state_);
  XmlRpc::XmlRpcValue trigger_rpc_value, gimbal_rpc_value;
  nh.getParam("trigger_calibration", trigger_rpc_value);
  trigger_calibration_ = new rm_common::CalibrationQueue(trigger_rpc_value, nh, controller_manager_);
  nh.getParam("gimbal_calibration", gimbal_rpc_value);
  gimbal_calibration_ = new rm_common::CalibrationQueue(gimbal_rpc_value, nh, controller_manager_);
  left_switch_up_event_.setActiveHigh(boost::bind(&DartManual::leftSwitchUpOn, this));
  left_switch_mid_event_.setActiveHigh(boost::bind(&DartManual::leftSwitchMidOn, this));
  left_switch_down_event_.setActiveHigh(boost::bind(&DartManual::leftSwitchDownOn, this));
  right_switch_down_event_.setActiveHigh(boost::bind(&DartManual::rightSwitchDownOn, this));
  right_switch_mid_event_.setRising(boost::bind(&DartManual::rightSwitchMidRise, this));
  right_switch_up_event_.setRising(boost::bind(&DartManual::rightSwitchUpRise, this));
  wheel_clockwise_event_.setRising(boost::bind(&DartManual::wheelClockwise, this));
  wheel_anticlockwise_event_.setRising(boost::bind(&DartManual::wheelAntiClockwise, this));
  dbus_sub_ = nh.subscribe<rm_msgs::DbusData>("/dbus_data", 10, &DartManual::dbusDataCallback, this);
  dart_client_cmd_sub_ = nh_referee.subscribe<rm_msgs::DartClientCmd>("dart_client_cmd_data", 10,
                                                                      &DartManual::dartClientCmdCallback, this);
  game_robot_hp_sub_ =
      nh_referee.subscribe<rm_msgs::GameRobotHp>("game_robot_hp", 10, &DartManual::gameRobotHpCallback, this);
  game_status_sub_ =
      nh_referee.subscribe<rm_msgs::GameStatus>("game_status", 10, &DartManual::gameStatusCallback, this);
}

void DartManual::getList(const XmlRpc::XmlRpcValue& darts, const XmlRpc::XmlRpcValue& targets,
                         const XmlRpc::XmlRpcValue& launch_id, const XmlRpc::XmlRpcValue& trigger_position)
{
  for (const auto& dart : darts)
  {
    ROS_ASSERT(dart.second.hasMember("param") and dart.second.hasMember("id"));
    ROS_ASSERT(dart.second["param"].getType() == XmlRpc::XmlRpcValue::TypeArray and
               dart.second["id"].getType() == XmlRpc::XmlRpcValue::TypeInt);
    for (int i = 0; i < 4; ++i)
    {
      if (dart.second["id"] == launch_id[i])
      {
        Dart dart_info;
        dart_info.outpost_offset_ = static_cast<double>(dart.second["param"][0]);
        dart_info.outpost_qd_ = static_cast<double>(dart.second["param"][1]);
        dart_info.base_offset_ = static_cast<double>(dart.second["param"][2]);
        dart_info.base_qd_ = static_cast<double>(dart.second["param"][3]);
        dart_info.trigger_position_ = static_cast<double>(trigger_position[i]);
        dart_list_.insert(std::make_pair(i, dart_info));
      }
    }
  }
  for (const auto& target : targets)
  {
    ROS_ASSERT(target.second.hasMember("position"));
    ROS_ASSERT(target.second["position"].getType() == XmlRpc::XmlRpcValue::TypeArray);
    std::vector<double> position(2);
    position[0] = static_cast<double>(target.second["position"][0]);
    position[1] = static_cast<double>(target.second["position"][1]);
    target_position_.insert(std::make_pair(target.first, position));
  }
}

void DartManual::run()
{
  ManualBase::run();
  trigger_calibration_->update(ros::Time::now());
  gimbal_calibration_->update(ros::Time::now());
}

void DartManual::gameRobotStatusCallback(const rm_msgs::GameRobotStatus::ConstPtr& data)
{
  ManualBase::gameRobotStatusCallback(data);
  robot_id_ = data->robot_id;
}

void DartManual::gameStatusCallback(const rm_msgs::GameStatus::ConstPtr& data)
{
  ManualBase::gameStatusCallback(data);
  game_progress_ = data->game_progress;
}

void DartManual::sendCommand(const ros::Time& time)
{
  friction_left_sender_->sendCommand(time);
  friction_right_sender_->sendCommand(time);
  trigger_sender_->sendCommand(time);
  pitch_sender_->sendCommand(time);
  yaw_sender_->sendCommand(time);
}

void DartManual::updateRc(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ManualBase::updateRc(dbus_data);
  move(pitch_sender_, dbus_data->ch_r_y);
  move(yaw_sender_, dbus_data->ch_l_x);
}

void DartManual::updatePc(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ManualBase::updatePc(dbus_data);
  double velocity_threshold = 0.001;
  if (pitch_velocity_ < velocity_threshold && yaw_velocity_ < velocity_threshold)
    move_state_ = STOP;
  else
    move_state_ = MOVING;
  getDartFiredNum();
  if (game_progress_ == rm_msgs::GameStatus::IN_BATTLE)
  {
    switch (auto_state_)
    {
      case OUTPOST:
        pitch_sender_->setPoint(pitch_outpost_);
        yaw_sender_->setPoint(yaw_outpost_ + dart_list_[dart_fired_num_].outpost_offset_);
        qd_ = dart_list_[dart_fired_num_].outpost_qd_;
        break;
      case BASE:
        pitch_sender_->setPoint(pitch_base_);
        yaw_sender_->setPoint(yaw_base_ + dart_list_[dart_fired_num_].base_offset_);
        qd_ = dart_list_[dart_fired_num_].base_qd_;
        break;
    }
    friction_right_sender_->setPoint(qd_);
    friction_left_sender_->setPoint(qd_);
    if (last_dart_door_status_ - dart_launch_opening_status_ ==
        rm_msgs::DartClientCmd::OPENING_OR_CLOSING - rm_msgs::DartClientCmd::OPENED)
    {
      dart_door_open_times_++;
      initial_dart_fired_num_ = dart_fired_num_;
      has_stopped = false;
    }
    if (move_state_ == STOP)
      launch_state_ = AIMED;
    else
      launch_state_ = NONE;
    if (dart_launch_opening_status_ == rm_msgs::DartClientCmd::OPENED)
    {
      switch (launch_state_)
      {
        case NONE:
          trigger_sender_->setPoint(0.);
          break;
        case AIMED:
          launchTwoDart();
          break;
      }
    }
    else
      trigger_sender_->setPoint(0.);
    last_dart_door_status_ = dart_launch_opening_status_;
  }
  else
  {
    friction_right_sender_->setPoint(0.);
    friction_left_sender_->setPoint(0.);
  }
}

void DartManual::checkReferee()
{
  ManualBase::checkReferee();
}

void DartManual::remoteControlTurnOn()
{
  ManualBase::remoteControlTurnOn();
  gimbal_calibration_->stopController();
  trigger_calibration_->stopController();
  gimbal_calibration_->reset();
  trigger_calibration_->reset();
}

void DartManual::leftSwitchDownOn()
{
  friction_right_sender_->setPoint(0.);
  friction_left_sender_->setPoint(0.);
  trigger_sender_->setPoint(-upward_vel_);
  triggerComeBackProtect();
}

void DartManual::leftSwitchMidOn()
{
  getDartFiredNum();
  initial_dart_fired_num_ = dart_fired_num_;
  has_stopped = false;
  friction_right_sender_->setPoint(qd_);
  friction_left_sender_->setPoint(qd_);
  trigger_sender_->setPoint(0.);
}

void DartManual::leftSwitchUpOn()
{
  getDartFiredNum();
  launchTwoDart();
  switch (manual_state_)
  {
    case OUTPOST:
      qd_ = dart_list_[dart_fired_num_].outpost_qd_;
      yaw_sender_->setPoint(yaw_outpost_ + dart_list_[dart_fired_num_].outpost_offset_);
      break;
    case BASE:
      qd_ = dart_list_[dart_fired_num_].base_qd_;
      yaw_sender_->setPoint(yaw_base_ + dart_list_[dart_fired_num_].base_offset_);
      break;
  }
  friction_right_sender_->setPoint(qd_);
  friction_left_sender_->setPoint(qd_);
}

void DartManual::rightSwitchDownOn()
{
  recordPosition(dbus_data_);
  if (dbus_data_.ch_l_y == 1.)
  {
    pitch_sender_->setPoint(pitch_outpost_);
    yaw_sender_->setPoint(yaw_outpost_);
  }
  if (dbus_data_.ch_l_y == -1.)
  {
    pitch_sender_->setPoint(pitch_base_);
    yaw_sender_->setPoint(yaw_base_);
  }
  if (dbus_data_.ch_l_x == 1.)
  {
    pitch_sender_->setPoint(target_position_["outpost"][0]);
    yaw_sender_->setPoint(target_position_["outpost"][1]);
  }
  if (dbus_data_.ch_l_x == -1.)
  {
    pitch_sender_->setPoint(target_position_["base"][0]);
    yaw_sender_->setPoint(target_position_["base"][1]);
  }
}

void DartManual::rightSwitchMidRise()
{
  ManualBase::rightSwitchMidRise();
  dart_door_open_times_ = 0;
  initial_dart_fired_num_ = 0;
  move_state_ = NORMAL;
}

void DartManual::rightSwitchUpRise()
{
  ManualBase::rightSwitchUpRise();
  pitch_sender_->setPoint(pitch_outpost_);
  yaw_sender_->setPoint(yaw_outpost_);
}

void DartManual::move(rm_common::JointPointCommandSender* joint, double ch)
{
  if (!joint_state_.position.empty())
  {
    double position = joint_state_.position[joint->getIndex()];
    if (ch != 0.)
    {
      joint->setPoint(position + ch * scale_);
      if_stop_ = true;
    }
    if (ch == 0. && if_stop_)
    {
      joint->setPoint(joint_state_.position[joint->getIndex()]);
      if_stop_ = false;
    }
  }
}

void DartManual::triggerComeBackProtect()
{
  if (trigger_position_ < 0.0003)
    trigger_sender_->setPoint(0.);
}

void DartManual::waitAfterLaunch(double time)
{
  if (!has_stopped)
    stop_time_ = ros::Time::now();
  if (ros::Time::now() - stop_time_ < ros::Duration(time))
  {
    trigger_sender_->setPoint(0.);
    has_stopped = true;
  }
  else
    trigger_sender_->setPoint(upward_vel_);
}

void DartManual::launchTwoDart()
{
  if (dart_fired_num_ < 4)
  {
    if (dart_fired_num_ - initial_dart_fired_num_ < 2)
    {
      if (dart_fired_num_ - initial_dart_fired_num_ == 1)
        waitAfterLaunch(2.0);
      else
        trigger_sender_->setPoint(upward_vel_);
    }
    else
      trigger_sender_->setPoint(0.);
  }
  else
    trigger_sender_->setPoint(0.);
}

void DartManual::getDartFiredNum()
{
  if (trigger_position_ < dart_list_[0].trigger_position_)
    dart_fired_num_ = 0;
  if (trigger_position_ > dart_list_[0].trigger_position_)
    dart_fired_num_ = 1;
  if (trigger_position_ > dart_list_[1].trigger_position_)
    dart_fired_num_ = 2;
  if (trigger_position_ > dart_list_[2].trigger_position_)
    dart_fired_num_ = 3;
  if (trigger_position_ > dart_list_[3].trigger_position_)
    dart_fired_num_ = 4;
}
void DartManual::recordPosition(const rm_msgs::DbusData dbus_data)
{
  if (dbus_data.ch_r_y == 1.)
  {
    pitch_outpost_ = joint_state_.position[pitch_sender_->getIndex()];
    yaw_outpost_ = joint_state_.position[yaw_sender_->getIndex()];
    ROS_INFO("Recorded outpost position.");
  }
  if (dbus_data.ch_r_y == -1.)
  {
    pitch_base_ = joint_state_.position[pitch_sender_->getIndex()];
    yaw_base_ = joint_state_.position[yaw_sender_->getIndex()];
    ROS_INFO("Recorded base position.");
  }
}
void DartManual::dbusDataCallback(const rm_msgs::DbusData::ConstPtr& data)
{
  ManualBase::dbusDataCallback(data);
  if (!joint_state_.name.empty())
  {
    trigger_position_ = std::abs(joint_state_.position[trigger_sender_->getIndex()]);
    pitch_velocity_ = std::abs(joint_state_.velocity[pitch_sender_->getIndex()]);
    yaw_velocity_ = std::abs(joint_state_.velocity[yaw_sender_->getIndex()]);
  }
  wheel_clockwise_event_.update(data->wheel == 1.0);
  wheel_anticlockwise_event_.update(data->wheel == -1.0);
  dbus_data_ = *data;
}

void DartManual::dartClientCmdCallback(const rm_msgs::DartClientCmd::ConstPtr& data)
{
  dart_launch_opening_status_ = data->dart_launch_opening_status;
}

void DartManual::gameRobotHpCallback(const rm_msgs::GameRobotHp::ConstPtr& data)
{
  switch (robot_id_)
  {
    case rm_msgs::GameRobotStatus::RED_DART:
      outpost_hp_ = data->blue_outpost_hp;
      break;
    case rm_msgs::GameRobotStatus::BLUE_DART:
      outpost_hp_ = data->red_outpost_hp;
      break;
  }
  if (outpost_hp_ != 0)
    auto_state_ = OUTPOST;
  else
    auto_state_ = BASE;
}

void DartManual::wheelClockwise()
{
  switch (move_state_)
  {
    case NORMAL:
      scale_ = scale_micro_;
      move_state_ = MICRO;
      ROS_INFO("Pitch and yaw : MICRO_MOVE_MODE");
      break;
    case MICRO:
      scale_ = 0.04;
      move_state_ = NORMAL;
      ROS_INFO("Pitch and yaw : NORMAL_MOVE_MODE");
      break;
  }
}

void DartManual::wheelAntiClockwise()
{
  switch (manual_state_)
  {
    case OUTPOST:
      manual_state_ = BASE;
      qd_ = dart_list_[dart_fired_num_].base_qd_;
      ROS_INFO("Friction wheels : BASE_MODE");
      pitch_sender_->setPoint(pitch_base_);
      yaw_sender_->setPoint(yaw_base_);
      break;
    case BASE:
      manual_state_ = OUTPOST;
      qd_ = dart_list_[dart_fired_num_].outpost_qd_;
      ROS_INFO("Friction wheels : OUTPOST_MODE");
      pitch_sender_->setPoint(pitch_outpost_);
      yaw_sender_->setPoint(yaw_outpost_);
      break;
  }
}
}  // namespace rm_manual

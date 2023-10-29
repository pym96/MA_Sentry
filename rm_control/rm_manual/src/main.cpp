//
// Created by luohx on 7/27/20.
//

#include "rm_manual/manual_base.h"
#include "rm_manual/chassis_gimbal_shooter_cover_manual.h"
#include "rm_manual/engineer_manual.h"
#include "rm_manual/dart_manual.h"
#include "rm_manual/balance_manual.h"

int main(int argc, char** argv)
{
  std::string robot;
  rm_manual::ManualBase* manual_control;
  ros::init(argc, argv, "rm_manual");
  ros::NodeHandle nh("~");
  ros::NodeHandle nh_referee("rm_referee");
  robot = getParam(nh, "robot_type", (std::string) "error");
  if (robot == "standard")
    manual_control = new rm_manual::ChassisGimbalShooterCoverManual(nh, nh_referee);
  else if ((robot == "hero") || (robot == "drone"))
    manual_control = new rm_manual::ChassisGimbalShooterManual(nh, nh_referee);
  else if (robot == "engineer")
    manual_control = new rm_manual::EngineerManual(nh, nh_referee);
  else if (robot == "dart")
    manual_control = new rm_manual::DartManual(nh, nh_referee);
  else if (robot == "balance")
    manual_control = new rm_manual::BalanceManual(nh, nh_referee);
  else
  {
    ROS_ERROR("no robot type ");
    return 0;
  }
  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    ros::spinOnce();
    manual_control->run();
    loop_rate.sleep();
  }
  return 0;
}

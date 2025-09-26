/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/GlobalPlugin.h>
#include "LpfThreshold.h"
#include <mc_rbdyn/BodySensor.h>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <string>
#include <mc_tvm/Robot.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>
#include <RBDyn/FA.h>
#include <state-observation/tools/rigid-body-kinematics.hpp>

namespace mc_plugin
{

struct CollisionDetectionJerk : public mc_control::GlobalPlugin
{
  void init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config) override;

  void reset(mc_control::MCGlobalController & controller) override;

  void before(mc_control::MCGlobalController & controller) override;

  void after(mc_control::MCGlobalController & controller) override;

  void addGui(mc_control::MCGlobalController & controller);
  void addLog(mc_control::MCGlobalController & controller);
  void addPlot(mc_control::MCGlobalController & controller);

  void jerkEstimationInit(mc_control::MCGlobalController & ctl);
  void jerkEstimation(mc_control::MCGlobalController & ctl);
  void jerkEstimationWithLinearVelocity(mc_control::MCGlobalController & ctl);

  mc_control::GlobalPlugin::GlobalPluginConfiguration configuration() override;

  ~CollisionDetectionJerk() override;

private:

  // GUI
  double dt_;
  double counter_;

  int jointNumber_;
  int axis_shown_ = 0; // x, y, z
  bool activate_plot_ = false;
  bool plot_added_ = false;
  bool collision_stop_activated_ = false;
  bool obstacle_detected_ = false;
  bool obstacle_detection_base_ = false;
  bool obstacle_detection_vel_ = false;
  bool activate_verbose_ = true;

  bool velocity_flag_ = false;

  // Threshold base method 
  LpfThreshold lpf_threshold_base_;
  Eigen::Vector3d threshold_offset_base_;
  double threshold_filtering_base_;
  Eigen::Vector3d threshold_high_base_;
  Eigen::Vector3d threshold_low_base_;
  // Threshold velocity method 
  LpfThreshold lpf_threshold_vel_;
  Eigen::Vector3d threshold_offset_vel_;
  double threshold_filtering_vel_;
  Eigen::Vector3d threshold_high_vel_;
  Eigen::Vector3d threshold_low_vel_;

  // Algorithm
  std::string imuBodyName_; // Name of the IMU sensor
  std::string robotBodyName_;
  bool imu_not_yet_initialized_;

  // Gains
  double k_vel_; // Gyro bias correction gain including the linear velocity
  double alpha_rot_vel_; // Complementary filter gain for rotation matrix estimation from the linear velocity estimation
  double alpha_v_; // Complementary filter gain for the linear velocity part of the rotation matrix estimation
  double alpha_acc_; // Complementary filter gain for acceleration estimation
  double alpha_jerk_vel_; // Complementary filter gain for jerk estimation including the linear velocity

  // IMU measurements
  Eigen::Vector3d accelero_;
  Eigen::Vector3d accelero_dot_; // Derivative of the accelerometer measurements
  Eigen::Vector3d accelero_dot_dot_; // Second derivative of the accelerometer measurements
  Eigen::Vector3d prev_accelero_;
  Eigen::Vector3d prev_accelero_dot_;
  Eigen::Vector3d gyro_;
  Eigen::Vector3d prev_gyro_;
  Eigen::Vector3d gyro_dot_;

  // Rotation matrix of the robot (model based)
  Eigen::Matrix3d R_rob_;
  Eigen::Quaternion<double> quat_R_rob_;

  // Linear velocity from the robot model
  Eigen::Vector3d v_encoders_;

  // No model estimation
  Eigen::Vector3d jerk_withoutModel_;
  Eigen::Vector3d jerk_diff_; // without model - with velocity
    
  // Estimation including the linear velocity
  Eigen::Vector3d bias_gyro_vel_; // Gyro bias derivative including the linear velocity
  Eigen::Vector3d bias_gyro_dot_vel_; // Gyro bias derivative including the linear velocity
  Eigen::Matrix3d R_vel_; // Estimated rotation matrix including the linear velocity
  Eigen::Quaternion<double> quat_tilde_vel_;
  Eigen::Quaternion<double> quat_R_vel_;
  Eigen::Vector3d omega_acceleroAndEncVel_;
  Eigen::Vector3d omega_vel_; // Angular velocity including the linear velocity
  Eigen::Vector3d jerk_vel_; // Estimated jerk including the linear velocity
  Eigen::Vector3d jerk_dot_vel_; // Estimated jerk derivative including the linear velocity
  Eigen::Vector3d v_vel_;
  Eigen::Vector3d v_dot_vel_;
  Eigen::Vector3d v_dot_accelero_; // Linear acceleration from the accelerometer measurements (Gravity removed)
  Eigen::Vector3d acc_vel_; // Linear acceleration including the linear velocity
  Eigen::Vector3d acc_dot_vel_; // Derivative of the linear acceleration including the linear velocity

};

} // namespace mc_plugin

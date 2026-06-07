/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/GlobalPlugin.h>
#include <mc_rbdyn/BodySensor.h>
#include <mc_tvm/Robot.h>
#include "LpfThreshold.h"
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

  mc_control::GlobalPlugin::GlobalPluginConfiguration configuration() override;

  ~CollisionDetectionJerk() override;

private:
  // GUI
  double counter_ = 0.0;
  int axis_shown_ = 0; // x, y, z
  bool activate_plot_ = false;
  bool plot_added_ = false;
  bool collision_stop_activated_ = false;
  bool obstacle_detected_ = false;
  bool activate_verbose_ = true;

  // Threshold base method
  LpfThreshold lpf_threshold_base_;
  Eigen::Vector3d threshold_offset_base_;
  double threshold_filtering_base_;
  Eigen::Vector3d threshold_high_base_;
  Eigen::Vector3d threshold_low_base_;

  // Algorithm
  double dt_;
  std::string imuBodyName_; // Name of the IMU sensor
  bool imu_not_yet_initialized_;

  Eigen::Vector3d accelero_;
  Eigen::Vector3d accelero_dot_; // Derivative of the accelerometer measurements
  Eigen::Vector3d prev_accelero_;
  Eigen::Vector3d gyro_;

  // No model estimation
  Eigen::Vector3d jerk_;
};

} // namespace mc_plugin

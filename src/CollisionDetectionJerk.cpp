#include "CollisionDetectionJerk.h"
#include <mc_control/GlobalPluginMacros.h>

namespace mc_plugin
{

CollisionDetectionJerk::~CollisionDetectionJerk() = default;

void CollisionDetectionJerk::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
{
  auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);
  auto & robot = ctl.robot();

  // Make sure to have obstacle detection datastore entry
  if(!ctl.controller().datastore().has("Obstacle detected"))
  {
    ctl.controller().datastore().make<bool>("Obstacle detected", false);
  }

  // Get plugin configuration
  auto plugin_config = config("collision_detection_jerk");
  imuBodyName_ = "Accelerometer";
  if(config.has("imuBodyName"))
  {
    imuBodyName_.assign(config("imuBodyName"));
  }
  threshold_filtering_base_ = plugin_config("threshold_filtering_base", 0.05);
  threshold_offset_base_ = plugin_config("threshold_offset_base");
  if(threshold_offset_base_.size() != 3)
  {
    threshold_offset_base_ = Eigen::Vector3d::Constant(3, 1.0);
  }
  lpf_threshold_base_.setValues(threshold_offset_base_, threshold_filtering_base_, 3);

  // Check if the robot has the specified IMU sensor
  if(!robot.hasBodySensor(imuBodyName_))
  {
    mc_rtc::log::error("[CollisionDetectionJerk] The IMU sensor {} does not exist in the robot", imuBodyName_);
    return;
  }

  jerkEstimationInit(ctl);

  addGui(ctl);
  addLog(ctl);
  mc_rtc::log::info("CollisionDetectionJerk::init called with configuration:\n{}", config.dump(true, true));
}

void CollisionDetectionJerk::reset(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("CollisionDetectionJerk::reset called");
}

void CollisionDetectionJerk::before(mc_control::MCGlobalController & controller)
{
  auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);
  counter_ += dt_;
  if(activate_plot_ & !plot_added_)
  {
    addPlot(ctl);
    plot_added_ = true;
  }

  jerkEstimation(ctl);
  threshold_high_base_ = lpf_threshold_base_.adaptiveThreshold(jerk_withoutModel_, true);
  threshold_low_base_ = lpf_threshold_base_.adaptiveThreshold(jerk_withoutModel_, false);

  obstacle_detected_ = false;
  // For each axis
  for(int i = 0; i < 3; i++)
  {
    if(jerk_withoutModel_(i) > threshold_high_base_(i) || jerk_withoutModel_(i) < threshold_low_base_(i))
    {
      obstacle_detected_ = true;
      if(activate_verbose_)
        mc_rtc::log::info("[CollisionDetectionJerk] Obstacle detected without Robot Model method on axis {}", i);
      // break;
    }
  }
  if(collision_stop_activated_)
  {
    ctl.controller().datastore().get<bool>("Obstacle detected") = obstacle_detected_;
  }
  // mc_rtc::log::info("CollisionDetectionJerk::before");
}

void CollisionDetectionJerk::after(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("CollisionDetectionJerk::after");
}

mc_control::GlobalPlugin::GlobalPluginConfiguration CollisionDetectionJerk::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after = false;
  out.should_always_run = true;
  return out;
}

void CollisionDetectionJerk::addGui(mc_control::MCGlobalController & controller)
{
  auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);

  ctl.controller().gui()->addElement(
      {"Plugins", "CollisionDetectionJerk"},
      mc_rtc::gui::NumberInput(
          "threshold_filtering_base", [this]() { return threshold_filtering_base_; },
          [this](double threshold_filtering_base)
          {
            this->threshold_filtering_base_ = threshold_filtering_base;
            lpf_threshold_base_.setFiltering(threshold_filtering_base_);
          }),
      mc_rtc::gui::ArrayInput(
          "threshold_offset_base", {"x", "y", "z"}, [this]() { return this->threshold_offset_base_; },
          [this](const Eigen::VectorXd & offset)
          {
            threshold_offset_base_ = offset;
            lpf_threshold_base_.setOffset(threshold_offset_base_);
          }),
      mc_rtc::gui::IntegerInput(
          "Axis shown", [this]() { return axis_shown_; }, [this](int axis) { this->axis_shown_ = axis; }),
      mc_rtc::gui::Checkbox("Collision stop", collision_stop_activated_),
      mc_rtc::gui::Checkbox("Verbose", activate_verbose_),
      mc_rtc::gui::Button("Add plot", [this]() { return activate_plot_ = true; }));
}

void CollisionDetectionJerk::addLog(mc_control::MCGlobalController & controller)
{
  auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);
  ctl.controller().logger().addLogEntry("CollisionDetectionJerk_jerk_withoutModel",
                                        [this]() { return jerk_withoutModel_; });
  ctl.controller().logger().addLogEntry("CollisionDetectionJerk_jerk_withoutModel_norm",
                                        [this]() { return jerk_withoutModel_.norm(); });
}

void CollisionDetectionJerk::addPlot(mc_control::MCGlobalController & ctl)
{
  auto & gui = *ctl.controller().gui();
  gui.addPlot(
      "jerk without model", mc_rtc::gui::plot::X("t", [this]() { return counter_; }),
      mc_rtc::gui::plot::Y(
          "jerk", [this]() { return jerk_withoutModel_[axis_shown_]; }, mc_rtc::gui::Color::Red),
      mc_rtc::gui::plot::Y(
          "threshold_high_base", [this]() { return threshold_high_base_[axis_shown_]; }, mc_rtc::gui::Color::Gray),
      mc_rtc::gui::plot::Y(
          "threshold_low_base", [this]() { return threshold_low_base_[axis_shown_]; }, mc_rtc::gui::Color::Gray));
}

void CollisionDetectionJerk::jerkEstimationInit(mc_control::MCGlobalController & ctl)
{
  // Initializing variables
  imu_not_yet_initialized_ = true;
  dt_ = ctl.timestep();
  gyro_ = Eigen::Vector3d::Zero();
  accelero_ = Eigen::Vector3d::Zero();
  prev_accelero_ = Eigen::Vector3d::Zero();
  accelero_dot_ = Eigen::Vector3d::Zero();
}

void CollisionDetectionJerk::jerkEstimation(mc_control::MCGlobalController & ctl)
{
  auto & robot = ctl.robot();
  auto & realRobot = ctl.realRobot();
  if(!robot.hasBodySensor(imuBodyName_))
  {
    mc_rtc::log::error("[CollisionDetectionJerk] Body sensor {} does not exist in the robot.", imuBodyName_);
    return;
  }
  const mc_rbdyn::BodySensor & imu = robot.bodySensor(imuBodyName_);
  if(imu.linearAcceleration().norm() != 0.0 && imu.angularVelocity().norm() != 0.0 && imu_not_yet_initialized_)
  {
    imu_not_yet_initialized_ = false;
    accelero_ = imu.linearAcceleration();
    gyro_ = imu.angularVelocity();
  }

  if(imu_not_yet_initialized_)
  {
    return;
  }

  // IMU measurements
  gyro_ = imu.angularVelocity();
  prev_accelero_ = accelero_;
  accelero_ = imu.linearAcceleration();
  accelero_dot_ = (accelero_ - prev_accelero_) / dt_;
  jerk_withoutModel_ = stateObservation::kine::skewSymmetric(gyro_) * accelero_ + accelero_dot_;
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("CollisionDetectionJerk", mc_plugin::CollisionDetectionJerk)

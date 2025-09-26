#include "CollisionDetectionJerk.h"
#include <mc_control/GlobalPluginMacros.h>

namespace mc_plugin
{

CollisionDetectionJerk::~CollisionDetectionJerk() = default;

void CollisionDetectionJerk::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
{
  auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);
  auto & robot = ctl.robot();
  auto & realRobot = ctl.realRobot();

  jointNumber_ = ctl.robot(ctl.robots()[0].name()).refJointOrder().size();
  imu_not_yet_initialized_ = true;
  dt_ = ctl.timestep();
  counter_ = 0.0;

  if(!ctl.controller().datastore().has("Obstacle detected"))
  {
    ctl.controller().datastore().make<bool>("Obstacle detected", false);
  }

  auto plugin_config = config("collision_detection_jerk");

  k_vel_ = plugin_config("k_vel", 1.0);
  alpha_rot_vel_ = plugin_config("alpha_rot_vel", 1.0);
  alpha_v_ = plugin_config("alpha_v", 1.0);
  alpha_acc_ = plugin_config("alpha_acc", 1.0);
  alpha_jerk_vel_ = plugin_config("alpha_jerk_vel", 1.0);
  imuBodyName_ = "Accelerometer";
  robotBodyName_ = "FT_sensor_mounting";
  if(config.has("imuBodyName"))
  {
    imuBodyName_.assign(config("imuBodyName"));
  }
  if(config.has("robotBodyName"))
  {
    robotBodyName_.assign(config("robotBodyName"));
  }

  threshold_filtering_base_ = plugin_config("threshold_filtering_base", 0.05);
  threshold_offset_base_ = plugin_config("threshold_offset_base");
  if(threshold_offset_base_.size() != 3)
  {
    threshold_offset_base_ = Eigen::Vector3d::Constant(3, 1.0);
  }
  lpf_threshold_base_.setValues(threshold_offset_base_, threshold_filtering_base_, 3);

  threshold_filtering_vel_ = plugin_config("threshold_filtering_vel", 0.05);
  threshold_offset_vel_ = plugin_config("threshold_offset_vel");
  if(threshold_offset_vel_.size() != 3)
  {
    threshold_offset_vel_ = Eigen::Vector3d::Constant(3, 1.0);
  }
  lpf_threshold_vel_.setValues(threshold_offset_vel_, threshold_filtering_vel_, 3);

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

  auto & robot = ctl.robot();
  auto & realRobot = ctl.realRobot();
  if(!robot.hasBodySensor(imuBodyName_))
  {
    mc_rtc::log::error("[CollisionDetectionJerk] Body sensor {} does not exist in the robot.", imuBodyName_);
    return;
  }
  const mc_rbdyn::BodySensor & imu = robot.bodySensor(imuBodyName_);

  rbd::forwardKinematics(realRobot.mb(), realRobot.mbc());
  rbd::forwardVelocity(realRobot.mb(), realRobot.mbc());
  rbd::forwardAcceleration(realRobot.mb(), realRobot.mbc());

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
  prev_gyro_ = gyro_; 
  gyro_ = imu.angularVelocity();
  const Eigen::Vector3d gyro_dot = (gyro_ - prev_gyro_)/dt_;
  prev_accelero_ = accelero_;
  accelero_ = imu.linearAcceleration();
  prev_accelero_dot_ = accelero_dot_;
  accelero_dot_ = (accelero_- prev_accelero_)/dt_;
  accelero_dot_dot_ = (accelero_dot_ - prev_accelero_dot_)/dt_;


  // Rotation matrix model
  R_rob_ = realRobot.bodyPosW(robotBodyName_).rotation().transpose();
  quat_R_rob_ = stateObservation::kine::rotationVectorToQuaternion(stateObservation::kine::rotationMatrixToRotationVector(R_rob_));

  // Velocity from the robot model
  v_encoders_ = realRobot.bodyVelW(robotBodyName_).linear();

  jerkEstimation(ctl);
  jerkEstimationWithLinearVelocity(ctl);
  jerk_diff_ = jerk_withoutModel_ - jerk_vel_;
  threshold_high_base_ = lpf_threshold_base_.adaptiveThreshold(jerk_withoutModel_, true);
  threshold_low_base_ = lpf_threshold_base_.adaptiveThreshold(jerk_withoutModel_, false);
  threshold_high_vel_ = lpf_threshold_vel_.adaptiveThreshold(jerk_vel_, true);
  threshold_low_vel_ = lpf_threshold_vel_.adaptiveThreshold(jerk_vel_, false);

  obstacle_detected_ = false;
  obstacle_detection_base_ = false;
  obstacle_detection_vel_ = false;
  // For each axis
  for(int i = 0; i < 3; i++)
  {
    if(jerk_withoutModel_(i) > threshold_high_base_(i) || jerk_withoutModel_(i) < threshold_low_base_(i))
    {
      obstacle_detection_base_ = true;
      if(!velocity_flag_) obstacle_detected_ = true;
      if(activate_verbose_) mc_rtc::log::info("[CollisionDetectionJerk] Obstacle detected without Robot Model method on axis {}", i);
      // break;
    }
    if(jerk_vel_(i) > threshold_high_vel_(i) || jerk_vel_(i) < threshold_low_vel_(i))
    {
      obstacle_detection_vel_ = true;
      if(velocity_flag_) obstacle_detected_ = true;
      if(activate_verbose_) mc_rtc::log::info("[CollisionDetectionJerk] Obstacle detected with Velocity method on axis {}", i);
      // break;
    }
  }
  if (collision_stop_activated_)
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

  ctl.controller().gui()->addElement({"Plugins", "CollisionDetectionJerk"},
    mc_rtc::gui::NumberInput("k_vel", [this]() { return k_vel_; },
      [this](double k_vel) 
      {
        this->k_vel_ = k_vel;
      }),
    mc_rtc::gui::NumberInput("alpha_rot_vel", [this]() { return alpha_rot_vel_; },
      [this](double alpha_rot_vel) 
      {
        this->alpha_rot_vel_ = alpha_rot_vel;
      }),
    mc_rtc::gui::NumberInput("alpha_v", [this]() { return alpha_v_; },
      [this](double alpha_v) 
      {
        this->alpha_v_ = alpha_v;
      }),
    mc_rtc::gui::NumberInput("alpha_acc", [this]() { return alpha_acc_; },
      [this](double alpha_acc) 
      {
        this->alpha_acc_ = alpha_acc;
      }),
    mc_rtc::gui::NumberInput("alpha_jerk_vel", [this]() { return alpha_jerk_vel_; },
      [this](double alpha_jerk_vel) 
      {
        this->alpha_jerk_vel_ = alpha_jerk_vel;
      }),
    mc_rtc::gui::NumberInput("threshold_filtering_base", [this]() { return threshold_filtering_base_; },
      [this](double threshold_filtering_base) 
      {
        this->threshold_filtering_base_ = threshold_filtering_base;
        lpf_threshold_base_.setFiltering(threshold_filtering_base_);
      }),
    mc_rtc::gui::ArrayInput("threshold_offset_base", {"x", "y", "z"}, 
      [this](){return this->threshold_offset_base_;},
        [this](const Eigen::VectorXd & offset)
      { 
        threshold_offset_base_ = offset;
        lpf_threshold_base_.setOffset(threshold_offset_base_); 
      }),
    mc_rtc::gui::NumberInput("threshold_filtering_vel", [this]() { return threshold_filtering_vel_; },
      [this](double threshold_filtering_vel) 
      {
        this->threshold_filtering_vel_ = threshold_filtering_vel;
        lpf_threshold_vel_.setFiltering(threshold_filtering_vel_);
      }),
    mc_rtc::gui::ArrayInput("threshold_offset_vel", {"x", "y", "z"},
      [this](){return this->threshold_offset_vel_;},
        [this](const Eigen::VectorXd & offset)
      { 
        threshold_offset_vel_ = offset;
        lpf_threshold_vel_.setOffset(threshold_offset_vel_); 
      }),
    mc_rtc::gui::IntegerInput("Axis shown", [this]() { return axis_shown_; },
      [this](int axis) 
      {
        this->axis_shown_ = axis;
      }),
    mc_rtc::gui::Checkbox("Use velocity for collision", velocity_flag_),
    mc_rtc::gui::Checkbox("Collision stop", collision_stop_activated_),
    mc_rtc::gui::Checkbox("Verbose", activate_verbose_),
    mc_rtc::gui::Button("Add plot", [this]() { return activate_plot_ = true; })
    );

}

void CollisionDetectionJerk::addLog(mc_control::MCGlobalController & controller)
{
  auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);
  ctl.controller().logger().addLogEntry("CollisionDetectionJerk_accelero", [this]() { return accelero_; });
  ctl.controller().logger().addLogEntry("CollisionDetectionJerk_accelero_norm", [this]() { return accelero_.norm(); });
  ctl.controller().logger().addLogEntry("CollisionDetectionJerk_accelero_dot", [this]() { return accelero_dot_; });
  ctl.controller().logger().addLogEntry("CollisionDetectionJerk_accelero_dot_norm", [this]() { return accelero_dot_.norm(); });
  ctl.controller().logger().addLogEntry("CollisionDetectionJerk_accelero_dot_dot", [this]() { return accelero_dot_dot_; });
  ctl.controller().logger().addLogEntry("CollisionDetectionJerk_accelero_dot_dot_norm", [this]() { return accelero_dot_dot_.norm(); });
  ctl.controller().logger().addLogEntry("CollisionDetectionJerk_gyro", [this]() { return gyro_; });
  ctl.controller().logger().addLogEntry("CollisionDetectionJerk_gyro_norm", [this]() { return gyro_.norm(); });
  ctl.controller().logger().addLogEntry("CollisionDetectionJerk_gyro_dot", [this]() { return gyro_dot_; });
  ctl.controller().logger().addLogEntry("CollisionDetectionJerk_gyro_dot_norm", [this]() { return gyro_dot_.norm(); });

  ctl.controller().logger().addLogEntry("CollisionDetectionJerk_R_robot", [this]() { return R_rob_.norm(); });
  ctl.controller().logger().addLogEntry("CollisionDetectionJerk_R_robot_quaternion", [this]() { return quat_R_rob_; });

  ctl.controller().logger().addLogEntry("CollisionDetectionJerk_v_encoders", [this]() { return v_encoders_; });
  ctl.controller().logger().addLogEntry("CollisionDetectionJerk_v_encoders_norm", [this]() { return v_encoders_.norm(); });

  ctl.controller().logger().addLogEntry("CollisionDetectionJerk_jerk_withoutModel", [this]() { return jerk_withoutModel_; });
  ctl.controller().logger().addLogEntry("CollisionDetectionJerk_jerk_withoutModel_norm", [this]() { return jerk_withoutModel_.norm(); });
  ctl.controller().logger().addLogEntry("CollisionDetectionJerk_jerk_withoutModel_dot", [this]() { return jerk_diff_; });
  ctl.controller().logger().addLogEntry("CollisionDetectionJerk_jerk_withoutModel_dot_norm", [this]() { return jerk_diff_.norm(); });

  ctl.controller().logger().addLogEntry("CollisionDetectionJerk_biasGyro_vel", [this]() { return bias_gyro_vel_; });
  ctl.controller().logger().addLogEntry("CollisionDetectionJerk_biasGyro_vel_norm", [this]() { return bias_gyro_vel_.norm(); });
  ctl.controller().logger().addLogEntry("CollisionDetectionJerk_biasGyro_vel_dot", [this]() { return bias_gyro_dot_vel_; });
  ctl.controller().logger().addLogEntry("CollisionDetectionJerk_biasGyro_vel_dot_norm", [this]() { return bias_gyro_dot_vel_.norm(); });
  ctl.controller().logger().addLogEntry("CollisionDetectionJerk_R_vel", [this]() { return R_vel_.norm(); });
  ctl.controller().logger().addLogEntry("CollisionDetectionJerk_R_vel_quaternion_error", [this]() { return quat_tilde_vel_; });
  ctl.controller().logger().addLogEntry("CollisionDetectionJerk_R_vel_quaternion", [this]() { return quat_R_vel_; });
  ctl.controller().logger().addLogEntry("CollisionDetectionJerk_omega_vel", [this]() { return omega_vel_; });
  ctl.controller().logger().addLogEntry("CollisionDetectionJerk_omega_vel_norm", [this]() { return omega_vel_.norm(); });
  ctl.controller().logger().addLogEntry("CollisionDetectionJerk_jerk_vel", [this]() { return jerk_vel_; });
  ctl.controller().logger().addLogEntry("CollisionDetectionJerk_jerk_vel_norm", [this]() { return jerk_vel_.norm(); });
  ctl.controller().logger().addLogEntry("CollisionDetectionJerk_jerk_vel_dot", [this]() { return jerk_dot_vel_; });
  ctl.controller().logger().addLogEntry("CollisionDetectionJerk_jerk_vel_dot_norm", [this]() { return jerk_dot_vel_.norm(); });
  ctl.controller().logger().addLogEntry("CollisionDetectionJerk_v_vel", [this]() { return v_vel_; });
  ctl.controller().logger().addLogEntry("CollisionDetectionJerk_v_vel_norm", [this]() { return v_vel_.norm(); });
  ctl.controller().logger().addLogEntry("CollisionDetectionJerk_v_vel_dot", [this]() { return v_dot_vel_; });
  ctl.controller().logger().addLogEntry("CollisionDetectionJerk_v_vel_dot_norm", [this]() { return v_dot_vel_.norm(); });
  ctl.controller().logger().addLogEntry("CollisionDetectionJerk_v_dot_accelero", [this]() { return v_dot_accelero_; });
  ctl.controller().logger().addLogEntry("CollisionDetectionJerk_v_dot_accelero_norm", [this]() { return v_dot_accelero_.norm(); });
  ctl.controller().logger().addLogEntry("CollisionDetectionJerk_acc_vel", [this]() { return acc_vel_; });
  ctl.controller().logger().addLogEntry("CollisionDetectionJerk_acc_vel_norm", [this]() { return acc_vel_.norm(); });
  ctl.controller().logger().addLogEntry("CollisionDetectionJerk_acc_vel_dot", [this]() { return acc_dot_vel_; });
  ctl.controller().logger().addLogEntry("CollisionDetectionJerk_acc_vel_dot_norm", [this]() { return acc_dot_vel_.norm(); });
  ctl.controller().logger().addLogEntry("CollisionDetectionJerk_obstacleDetected_", [this]() { return obstacle_detected_; });
  ctl.controller().logger().addLogEntry("CollisionDetectionJerk_obstacleDetected_base_", [this]() { return obstacle_detection_base_; });
  ctl.controller().logger().addLogEntry("CollisionDetectionJerk_obstacleDetected_vel_", [this]() { return obstacle_detection_vel_; });
  ctl.controller().logger().addLogEntry("CollisionDetectionJerk_threshold_high_base_", [this]() { return threshold_high_base_; });
  ctl.controller().logger().addLogEntry("CollisionDetectionJerk_threshold_low_base_", [this]() { return threshold_low_base_; });
  ctl.controller().logger().addLogEntry("CollisionDetectionJerk_threshold_high_vel_", [this]() { return threshold_high_vel_; });
  ctl.controller().logger().addLogEntry("CollisionDetectionJerk_threshold_low_vel_", [this]() { return threshold_low_vel_; });
}

void CollisionDetectionJerk::addPlot(mc_control::MCGlobalController & ctl)
{
  auto & gui = *ctl.controller().gui();
  gui.addPlot(
      "jerk without model",
      mc_rtc::gui::plot::X(
          "t", [this]() { return counter_; }),
      mc_rtc::gui::plot::Y(
      "jerk", [this]() { return jerk_withoutModel_[axis_shown_]; }, mc_rtc::gui::Color::Red),
      mc_rtc::gui::plot::Y(
        "threshold_high_base", [this]() { return threshold_high_base_[axis_shown_]; }, mc_rtc::gui::Color::Gray),
      mc_rtc::gui::plot::Y(
        "threshold_low_base", [this]() { return threshold_low_base_[axis_shown_]; }, mc_rtc::gui::Color::Gray)
      );

  gui.addPlot(
    "jerk with velocity",
    mc_rtc::gui::plot::X(
        "t", [this]() { return counter_; }),
    mc_rtc::gui::plot::Y(
        "jerk_vel(t)", [this]() { return jerk_vel_[axis_shown_]; }, mc_rtc::gui::Color::Red),
    mc_rtc::gui::plot::Y(
        "threshold_high_vel", [this]() { return threshold_high_vel_[axis_shown_]; }, mc_rtc::gui::Color::Gray),
    mc_rtc::gui::plot::Y(
        "threshold_low_vel", [this]() { return threshold_low_vel_[axis_shown_]; }, mc_rtc::gui::Color::Gray)
    );

  gui.addPlot(
    "Both jerk method",
    mc_rtc::gui::plot::X(
        "t", [this]() { return counter_; }),
    mc_rtc::gui::plot::Y(
      "jerk without model", [this]() { return jerk_withoutModel_[axis_shown_]; }, mc_rtc::gui::Color::Blue),
    mc_rtc::gui::plot::Y(
      "jerk_vel(t)", [this]() { return jerk_vel_[axis_shown_]; }, mc_rtc::gui::Color::Red)
    );

    gui.addPlot(
      "jerk diff",
      mc_rtc::gui::plot::X(
          "t", [this]() { return counter_; }),
      mc_rtc::gui::plot::Y(
      "jerk without model - vel", [this]() { return jerk_diff_[axis_shown_]; }, mc_rtc::gui::Color::Red)
      );

  //omega
  gui.addPlot(
    "omega",
    mc_rtc::gui::plot::X(
        "t", [this]() { return counter_; }),
    mc_rtc::gui::plot::Y(
    "gyro(t)", [this]() { return gyro_.norm(); }, mc_rtc::gui::Color::Red),
    mc_rtc::gui::plot::Y(
        "omega_vel(t)", [this]() { return omega_vel_.norm(); }, mc_rtc::gui::Color::Green),
    mc_rtc::gui::plot::Y(
        "omega_acceleroAndEncVel(t)", [this]() { return omega_acceleroAndEncVel_.norm(); }, mc_rtc::gui::Color::Cyan)
    );

  //accelero
  gui.addPlot(
    "accelero",
    mc_rtc::gui::plot::X(
        "t", [this]() { return counter_; }),
    mc_rtc::gui::plot::Y(
        "accelero(t)", [this]() { return accelero_.norm(); }, mc_rtc::gui::Color::Red)
    );

  //Rotation matrix
  gui.addPlot(
    "R",
    mc_rtc::gui::plot::X(
        "t", [this]() { return counter_; }),
    mc_rtc::gui::plot::Y(
        "R_rob(t)", [this]() { return R_rob_.norm(); }, mc_rtc::gui::Color::Red),
    mc_rtc::gui::plot::Y(
        "R_vel(t)", [this]() { return R_vel_.norm(); }, mc_rtc::gui::Color::Green)
    );

  //Velocity
  gui.addPlot(
    "velocity",
    mc_rtc::gui::plot::X(
        "t", [this]() { return counter_; }),
    mc_rtc::gui::plot::Y(
        "v_rob(t)", [this]() { return v_encoders_.norm(); }, mc_rtc::gui::Color::Red),
    mc_rtc::gui::plot::Y(
        "v(t)", [this]() { return v_vel_.norm(); }, mc_rtc::gui::Color::Green)
    );

  //Acceleration
  gui.addPlot(
    "acceleration",
    mc_rtc::gui::plot::X(
        "t", [this]() { return counter_; }),
    mc_rtc::gui::plot::Y(
        "v_dot", [this]() { return v_dot_vel_.norm(); }, mc_rtc::gui::Color::Green),
    mc_rtc::gui::plot::Y(
        "v_dot_accelero(t)", [this]() { return v_dot_accelero_.norm(); }, mc_rtc::gui::Color::Red),
    mc_rtc::gui::plot::Y(
        "acc_vel(t)", [this]() { return acc_vel_.norm(); }, mc_rtc::gui::Color::Cyan)
    );

}

void CollisionDetectionJerk::jerkEstimationInit(mc_control::MCGlobalController & ctl)
{
  auto & robot = ctl.robot();
  auto & realRobot = ctl.realRobot();

  const auto & imu = robot.bodySensor(imuBodyName_);
  rbd::forwardKinematics(realRobot.mb(), realRobot.mbc());
  rbd::forwardVelocity(realRobot.mb(), realRobot.mbc());
  rbd::forwardAcceleration(realRobot.mb(), realRobot.mbc());

  // Initializing variables
  accelero_dot_ = Eigen::Vector3d::Zero();
  accelero_dot_dot_ = Eigen::Vector3d::Zero();
  prev_accelero_dot_ = Eigen::Vector3d::Zero();
  gyro_dot_ = Eigen::Vector3d::Zero();

  R_rob_ = realRobot.bodyPosW(robotBodyName_).rotation().transpose();
  quat_R_rob_ = stateObservation::kine::rotationVectorToQuaternion(stateObservation::kine::rotationMatrixToRotationVector(R_rob_));

  v_encoders_ = realRobot.bodyVelW(robotBodyName_).linear();

  // Initialize the estimation without the model
  jerk_withoutModel_ = Eigen::Vector3d::Zero();

  // Initialize the estimation including the linear velocity
  bias_gyro_vel_ = Eigen::Vector3d::Zero();
  bias_gyro_dot_vel_ = Eigen::Vector3d::Zero();
  R_vel_ = R_rob_;
  quat_tilde_vel_ = stateObservation::kine::rotationVectorToQuaternion(stateObservation::kine::rotationMatrixToRotationVector(R_rob_*R_vel_.transpose()));
  quat_R_vel_ = stateObservation::kine::rotationVectorToQuaternion(stateObservation::kine::rotationMatrixToRotationVector(R_vel_));
  omega_acceleroAndEncVel_ = Eigen::Vector3d::Zero();
  omega_vel_ = Eigen::Vector3d::Zero();
  jerk_vel_ = Eigen::Vector3d::Zero();
  jerk_dot_vel_ = Eigen::Vector3d::Zero();
  v_vel_ = Eigen::Vector3d::Zero();
  v_dot_vel_ = Eigen::Vector3d::Zero();
  v_dot_accelero_ = Eigen::Vector3d::Zero();
  acc_vel_ = Eigen::Vector3d::Zero();
  acc_dot_vel_ = Eigen::Vector3d::Zero();

}

void CollisionDetectionJerk::jerkEstimation(mc_control::MCGlobalController & ctl)
{
  jerk_withoutModel_ = stateObservation::kine::skewSymmetric(gyro_) * accelero_ + accelero_dot_;
}

void CollisionDetectionJerk::jerkEstimationWithLinearVelocity(mc_control::MCGlobalController & ctl)
{
  // Estimate the gyro bias
  const Eigen::Matrix3d rot_error = (R_rob_ * R_vel_.transpose() - R_rob_.transpose() * R_vel_)/2;
  const Eigen::Vector3d rot_error_vec = stateObservation::kine::skewSymmetricToRotationVector(rot_error);
  bias_gyro_dot_vel_ = -k_vel_ * rot_error_vec;
  bias_gyro_vel_ += bias_gyro_dot_vel_*dt_;

  // Estimate the angular velocity: gyro with bias correction
  const Eigen::Vector3d omega_prev_vel = omega_vel_;
  omega_vel_ = gyro_ - bias_gyro_vel_;
  const Eigen::Vector3d omega_dot_vel = (omega_vel_ - omega_prev_vel)/dt_;

  // Estimate the rotation matrix
  omega_acceleroAndEncVel_ = acc_vel_.cross(R_vel_.transpose()*(v_encoders_-v_vel_));
  const Eigen::Vector3d filtered_omega_vel = omega_vel_ + alpha_rot_vel_ * rot_error_vec + alpha_v_*omega_acceleroAndEncVel_;
  R_vel_ = R_vel_ * stateObservation::kine::rotationVectorToRotationMatrix(filtered_omega_vel*dt_);

  // Estimate the acceleration from the velocity
  v_dot_accelero_ = Eigen::Vector3d(0,0,-9.81)+R_vel_*accelero_;
  v_dot_vel_= v_dot_accelero_ + alpha_acc_*(v_encoders_ - v_vel_);
  v_vel_ += v_dot_vel_*dt_;

  // Estimate the jerk
  Eigen::Vector3d X = stateObservation::kine::skewSymmetric(omega_vel_) * accelero_ + accelero_dot_;
  acc_dot_vel_ = R_vel_*X + alpha_jerk_vel_*(v_dot_vel_ - acc_vel_);
  acc_vel_ += acc_dot_vel_*dt_;
  jerk_vel_ = R_vel_.transpose()*acc_dot_vel_;

  // Need to be equal to identity matrix to make sure the rotation matrix is correctly estimated
  quat_tilde_vel_ = stateObservation::kine::rotationVectorToQuaternion(stateObservation::kine::rotationMatrixToRotationVector(R_rob_*R_vel_.transpose()));
  // Quaternion of the rotation matrix for logs
  quat_R_vel_ = stateObservation::kine::rotationVectorToQuaternion(stateObservation::kine::rotationMatrixToRotationVector(R_vel_));
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("CollisionDetectionJerk", mc_plugin::CollisionDetectionJerk)

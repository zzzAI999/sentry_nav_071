// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "fake_vel_transform/fake_vel_transform.hpp"

#include <algorithm>
#include <cmath>

#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace fake_vel_transform
{

constexpr double EPSILON = 1e-5;

FakeVelTransform::FakeVelTransform(const rclcpp::NodeOptions & options)
: Node("fake_vel_transform", options)
{
  RCLCPP_INFO(get_logger(), "Start FakeVelTransform!");
  current_robot_base_angle_ = 0.0;
  last_linear_command_time_ = this->get_clock()->now();
  last_heading_command_time_ = this->get_clock()->now();
  last_teleop_command_time_ = this->get_clock()->now();

  this->declare_parameter<std::string>("robot_base_frame", "gimbal_link");
  this->declare_parameter<std::string>("fake_robot_base_frame", "gimbal_link_fake");
  this->declare_parameter<std::string>("odom_topic", "odom");
  this->declare_parameter<std::string>("local_plan_topic", "local_plan");
  this->declare_parameter<std::string>("cmd_spin_topic", "cmd_spin");
  this->declare_parameter<std::string>("input_cmd_vel_topic", "");
  this->declare_parameter<std::string>("output_cmd_vel_topic", "");
  this->declare_parameter<std::string>("teleop_cmd_vel_topic", "/cmd_vel_teleop");
  this->declare_parameter<std::string>("teleop_active_topic", "/app/teleop_active");
  this->declare_parameter<double>("teleop_watchdog_timeout_sec", 0.5);
  this->declare_parameter<float>("init_spin_speed", 0.0);
  this->declare_parameter<bool>("forward_only_enabled", false);
  this->declare_parameter<bool>("forward_debug_enabled", false);
  this->declare_parameter<double>("forward_angular_command_sign", -1.0);
  this->declare_parameter<double>("forward_heading_kp", 1.4);
  this->declare_parameter<double>("forward_heading_deadband_rad", 0.03);
  this->declare_parameter<double>("forward_heading_correction_enter_rad", 0.12);
  this->declare_parameter<double>("forward_rotate_enter_angle_rad", 0.35);
  this->declare_parameter<double>("forward_rotate_exit_angle_rad", 0.12);
  this->declare_parameter<double>("forward_max_angular_speed", 0.6);
  this->declare_parameter<double>("forward_max_angular_accel", 1.2);
  this->declare_parameter<double>("forward_max_angular_decel", 3.0);
  this->declare_parameter<double>("forward_max_linear_accel", 0.8);
  this->declare_parameter<double>("forward_max_linear_decel", 1.2);
  this->declare_parameter<double>("forward_settle_angular_speed", 0.08);
  this->declare_parameter<double>("forward_min_command_speed", 0.02);
  this->declare_parameter<double>("forward_min_linear_speed", 0.3);
  this->declare_parameter<double>("forward_min_angular_speed", 0.1);

  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("fake_robot_base_frame", fake_robot_base_frame_);
  this->get_parameter("odom_topic", odom_topic_);
  this->get_parameter("cmd_spin_topic", cmd_spin_topic_);
  this->get_parameter("input_cmd_vel_topic", input_cmd_vel_topic_);
  this->get_parameter("output_cmd_vel_topic", output_cmd_vel_topic_);
  this->get_parameter("teleop_cmd_vel_topic", teleop_cmd_vel_topic_);
  this->get_parameter("teleop_active_topic", teleop_active_topic_);
  this->get_parameter("teleop_watchdog_timeout_sec", teleop_watchdog_timeout_sec_);
  this->get_parameter("init_spin_speed", spin_speed_);
  this->get_parameter("forward_only_enabled", forward_only_enabled_);
  this->get_parameter("forward_angular_command_sign", forward_angular_command_sign_);
  this->get_parameter("forward_heading_kp", forward_heading_kp_);
  this->get_parameter("forward_heading_deadband_rad", forward_heading_deadband_rad_);
  this->get_parameter(
    "forward_heading_correction_enter_rad", forward_heading_correction_enter_rad_);
  this->get_parameter("forward_rotate_enter_angle_rad", forward_rotate_enter_angle_rad_);
  this->get_parameter("forward_rotate_exit_angle_rad", forward_rotate_exit_angle_rad_);
  this->get_parameter("forward_max_angular_speed", forward_max_angular_speed_);
  this->get_parameter("forward_max_angular_accel", forward_max_angular_accel_);
  this->get_parameter("forward_max_angular_decel", forward_max_angular_decel_);
  this->get_parameter("forward_max_linear_accel", forward_max_linear_accel_);
  this->get_parameter("forward_max_linear_decel", forward_max_linear_decel_);
  this->get_parameter("forward_settle_angular_speed", forward_settle_angular_speed_);
  this->get_parameter("forward_min_command_speed", forward_min_command_speed_);
  this->get_parameter("forward_min_linear_speed", forward_min_linear_speed_);
  this->get_parameter("forward_min_angular_speed", forward_min_angular_speed_);

  forward_heading_deadband_rad_ = std::max(0.0, forward_heading_deadband_rad_);
  forward_heading_correction_enter_rad_ =
    std::max(forward_heading_deadband_rad_, forward_heading_correction_enter_rad_);
  forward_angular_command_sign_ = forward_angular_command_sign_ < 0.0 ? -1.0 : 1.0;
  forward_rotate_exit_angle_rad_ =
    std::max(forward_heading_deadband_rad_, forward_rotate_exit_angle_rad_);
  forward_rotate_enter_angle_rad_ =
    std::max(forward_rotate_exit_angle_rad_, forward_rotate_enter_angle_rad_);
  forward_max_angular_speed_ = std::max(0.0, forward_max_angular_speed_);
  forward_max_angular_accel_ = std::max(0.0, forward_max_angular_accel_);
  forward_max_angular_decel_ = std::max(0.0, forward_max_angular_decel_);
  forward_max_linear_accel_ = std::max(0.0, forward_max_linear_accel_);
  forward_max_linear_decel_ = std::max(0.0, forward_max_linear_decel_);
  forward_settle_angular_speed_ = std::max(0.0, forward_settle_angular_speed_);
  forward_min_command_speed_ = std::max(EPSILON, forward_min_command_speed_);
  forward_min_linear_speed_ = std::max(0.0, forward_min_linear_speed_);
  forward_min_angular_speed_ = std::clamp(
    forward_min_angular_speed_, 0.0, forward_max_angular_speed_);
  teleop_watchdog_timeout_sec_ = std::clamp(teleop_watchdog_timeout_sec_, 0.05, 0.5);

  RCLCPP_INFO(
    get_logger(),
    "Forward-only chassis mode: %s, heading correction enter/exit=%.3f/%.3f rad, "
    "rotate enter/exit=%.3f/%.3f rad, "
    "linear/wz effective minimum=%.3f/%.3f, max wz=%.3f rad/s, "
    "linear accel/decel=%.2f/%.2f, angular accel/decel=%.2f/%.2f, angular sign=%.0f",
    forward_only_enabled_ ? "enabled" : "disabled", forward_heading_correction_enter_rad_,
    forward_heading_deadband_rad_, forward_rotate_enter_angle_rad_,
    forward_rotate_exit_angle_rad_, forward_min_linear_speed_,
    forward_min_angular_speed_, forward_max_angular_speed_,
    forward_max_linear_accel_, forward_max_linear_decel_,
    forward_max_angular_accel_, forward_max_angular_decel_,
    forward_angular_command_sign_);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  cmd_vel_chassis_pub_ =
    this->create_publisher<geometry_msgs::msg::Twist>(output_cmd_vel_topic_, 100);

  cmd_spin_sub_ = this->create_subscription<example_interfaces::msg::Float32>(
    cmd_spin_topic_, 1, std::bind(&FakeVelTransform::cmdSpinCallback, this, std::placeholders::_1));
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    input_cmd_vel_topic_, 10,
    std::bind(&FakeVelTransform::cmdVelCallback, this, std::placeholders::_1));
  teleop_cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    teleop_cmd_vel_topic_, 10,
    std::bind(&FakeVelTransform::teleopCmdVelCallback, this, std::placeholders::_1));
  teleop_active_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    teleop_active_topic_, 10,
    std::bind(&FakeVelTransform::teleopActiveCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, 50,
    std::bind(&FakeVelTransform::odometryCallback, this, std::placeholders::_1));

  // 50Hz Timer to send transform from `robot_base_frame` to `fake_robot_base_frame`
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20), std::bind(&FakeVelTransform::publishTransform, this));
}

void FakeVelTransform::cmdSpinCallback(const example_interfaces::msg::Float32::SharedPtr msg)
{
  spin_speed_ = msg->data;
}

void FakeVelTransform::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  // Keep the chassis heading live while Nav2 is active. Gating this update on
  // local-plan activity freezes the heading whenever ApproximateTime cannot
  // pair an odometry/path message, so rotate-to-heading never exits.
  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  current_robot_base_angle_ = tf2::getYaw(msg->pose.pose.orientation);
}

void FakeVelTransform::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  if (teleop_active_) {
    return;
  }
  // Publish exactly once per Nav2 command using the latest odometry heading.
  auto aft_tf_vel = transformVelocity(msg, current_robot_base_angle_);
  cmd_vel_chassis_pub_->publish(aft_tf_vel);
}

void FakeVelTransform::teleopCmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  if (!teleop_active_) {
    return;
  }

  if (
    !std::isfinite(msg->linear.x) || !std::isfinite(msg->linear.y) ||
    !std::isfinite(msg->angular.z))
  {
    RCLCPP_WARN(get_logger(), "Rejected non-finite teleoperation velocity");
    return;
  }

  last_teleop_command_time_ = this->get_clock()->now();
  cmd_vel_chassis_pub_->publish(*msg);
}

void FakeVelTransform::teleopActiveCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  bool publish_zero = false;
  {
    std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
    if (msg->data) {
      teleop_active_ = true;
      last_teleop_command_time_ = this->get_clock()->now();
      rotating_to_heading_ = false;
      heading_correction_active_ = false;
      last_forward_linear_velocity_ = 0.0;
      last_heading_angular_velocity_ = 0.0;
      last_linear_command_time_ = this->get_clock()->now();
      last_heading_command_time_ = last_linear_command_time_;
    } else if (teleop_active_) {
      teleop_active_ = false;
      publish_zero = true;
    }
  }
  if (publish_zero) {
    publishZeroVelocity();
  }
}

void FakeVelTransform::publishZeroVelocity()
{
  cmd_vel_chassis_pub_->publish(geometry_msgs::msg::Twist());
}

void FakeVelTransform::enforceTeleopWatchdog()
{
  bool timed_out = false;
  {
    std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
    if (
      teleop_active_ &&
      (this->get_clock()->now() - last_teleop_command_time_).seconds() >
      teleop_watchdog_timeout_sec_)
    {
      teleop_active_ = false;
      timed_out = true;
    }
  }
  if (timed_out) {
    publishZeroVelocity();
    RCLCPP_WARN(get_logger(), "Teleoperation watchdog expired; chassis stopped");
  }
}

void FakeVelTransform::publishTransform()
{
  enforceTeleopWatchdog();
  double current_robot_base_angle;
  {
    std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
    current_robot_base_angle = current_robot_base_angle_;
  }

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = robot_base_frame_;
  t.child_frame_id = fake_robot_base_frame_;
  tf2::Quaternion q;
  q.setRPY(0, 0, -current_robot_base_angle);
  t.transform.rotation = tf2::toMsg(q);
  tf_broadcaster_->sendTransform(t);
}

geometry_msgs::msg::Twist FakeVelTransform::transformVelocity(
  const geometry_msgs::msg::Twist::SharedPtr & twist, float yaw_diff)
{
  geometry_msgs::msg::Twist aft_tf_vel;
  const double chassis_x =
    twist->linear.x * std::cos(yaw_diff) + twist->linear.y * std::sin(yaw_diff);
  const double chassis_y =
    -twist->linear.x * std::sin(yaw_diff) + twist->linear.y * std::cos(yaw_diff);

  if (!forward_only_enabled_) {
    aft_tf_vel.angular.z = twist->angular.z + spin_speed_;
    aft_tf_vel.linear.x = chassis_x;
    aft_tf_vel.linear.y = chassis_y;
    return aft_tf_vel;
  }

  const double requested_speed = std::hypot(chassis_x, chassis_y);
  const double external_angular_velocity =
    forward_angular_command_sign_ * twist->angular.z + spin_speed_;
  aft_tf_vel.linear.y = 0.0;

  if (requested_speed < forward_min_command_speed_) {
    rotating_to_heading_ = false;
    heading_correction_active_ = false;
    last_forward_linear_velocity_ = 0.0;
    last_linear_command_time_ = this->get_clock()->now();
    double external_wz = std::clamp(
      external_angular_velocity, -forward_max_angular_speed_, forward_max_angular_speed_);
    if (std::abs(external_wz) <= EPSILON) {
      last_heading_angular_velocity_ = 0.0;
      last_heading_command_time_ = this->get_clock()->now();
      return aft_tf_vel;
    }
    external_wz = limitHeadingAngularVelocity(external_wz);
    if (
      std::abs(external_wz) > EPSILON &&
      std::abs(external_wz) < forward_min_angular_speed_)
    {
      external_wz = std::copysign(forward_min_angular_speed_, external_wz);
    }
    aft_tf_vel.angular.z = external_wz;
    return aft_tf_vel;
  }

  // Always command the shortest turn. Unwrapping this error against the
  // previous sample can turn a small overshoot into a nearly 2*pi error and
  // make the chassis continue in the old direction for a full revolution.
  const double raw_heading_error = std::atan2(chassis_y, chassis_x);
  const double heading_error =
    std::atan2(std::sin(raw_heading_error), std::cos(raw_heading_error));

  const double abs_heading_error = std::abs(heading_error);
  if (!rotating_to_heading_ && abs_heading_error >= forward_rotate_enter_angle_rad_) {
    rotating_to_heading_ = true;
  }
  if (rotating_to_heading_ && abs_heading_error <= forward_rotate_exit_angle_rad_) {
    rotating_to_heading_ = false;
  }

  if (
    !heading_correction_active_ &&
    abs_heading_error >= forward_heading_correction_enter_rad_)
  {
    heading_correction_active_ = true;
  }
  if (
    heading_correction_active_ &&
    abs_heading_error <= forward_heading_deadband_rad_)
  {
    heading_correction_active_ = false;
  }

  double heading_angular_velocity = 0.0;
  if (heading_correction_active_) {
    heading_angular_velocity =
      forward_angular_command_sign_ * forward_heading_kp_ * heading_error;
  }
  heading_angular_velocity = std::clamp(
    heading_angular_velocity, -forward_max_angular_speed_, forward_max_angular_speed_);
  double limited_wz = limitHeadingAngularVelocity(heading_angular_velocity);
  if (std::abs(limited_wz) > EPSILON) {
    if (std::abs(limited_wz) < forward_min_angular_speed_) {
      // Preserve the limiter's current sign while braking. Using the target
      // sign here caused an instantaneous +wz to -wz jump on every overshoot.
      limited_wz = std::copysign(forward_min_angular_speed_, limited_wz);
    }
    aft_tf_vel.angular.z = limited_wz;
  }

  // While translating, the velocity vector defines the chassis heading. Mixing
  // in Nav2's independent yaw command can turn the chassis away from its travel
  // direction. Keep that command only for the zero-translation final-yaw phase.
  if (!rotating_to_heading_) {
    const double scale_range = std::max(
      EPSILON, forward_rotate_enter_angle_rad_ - forward_heading_deadband_rad_);
    const double heading_scale = std::clamp(
      (forward_rotate_enter_angle_rad_ - abs_heading_error) / scale_range, 0.0, 1.0);
    double target_linear_velocity = requested_speed * heading_scale;
    if (target_linear_velocity > EPSILON) {
      target_linear_velocity =
        std::max(target_linear_velocity, forward_min_linear_speed_);
    }
    double limited_linear_velocity =
      limitForwardLinearVelocity(target_linear_velocity);
    if (
      limited_linear_velocity > EPSILON &&
      limited_linear_velocity < forward_min_linear_speed_)
    {
      limited_linear_velocity = forward_min_linear_speed_;
    }
    aft_tf_vel.linear.x = limited_linear_velocity;
  } else {
    // Do not roll forward while a large heading error requires an in-place
    // rotation. This is a safety transition and intentionally bypasses ramping.
    last_forward_linear_velocity_ = 0.0;
    last_linear_command_time_ = this->get_clock()->now();
  }

  if (get_parameter("forward_debug_enabled").as_bool()) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 250,
      "forward_debug yaw=%.3f input=(%.3f,%.3f,%.3f) chassis=(%.3f,%.3f) "
      "error=%.3f correction=%d rotating=%d output=(%.3f,%.3f)",
      yaw_diff, twist->linear.x, twist->linear.y, twist->angular.z, chassis_x, chassis_y,
      heading_error, heading_correction_active_ ? 1 : 0,
      rotating_to_heading_ ? 1 : 0, aft_tf_vel.linear.x,
      aft_tf_vel.angular.z);
  }

  return aft_tf_vel;
}

double FakeVelTransform::limitForwardLinearVelocity(double target_linear_velocity)
{
  const auto now = this->get_clock()->now();
  double dt = (now - last_linear_command_time_).seconds();
  last_linear_command_time_ = now;
  if (dt <= 0.0 || dt > 0.25) {
    dt = 0.05;
  }
  if (dt < 0.005) {
    return last_forward_linear_velocity_;
  }

  const bool slowing_down =
    target_linear_velocity < last_forward_linear_velocity_;
  const double rate_limit =
    slowing_down ? forward_max_linear_decel_ : forward_max_linear_accel_;
  const double max_delta = rate_limit * dt;
  const double delta = std::clamp(
    target_linear_velocity - last_forward_linear_velocity_, -max_delta, max_delta);
  last_forward_linear_velocity_ += delta;
  if (last_forward_linear_velocity_ < EPSILON) {
    last_forward_linear_velocity_ = 0.0;
  }
  return last_forward_linear_velocity_;
}

double FakeVelTransform::limitHeadingAngularVelocity(double target_angular_velocity)
{
  const auto now = this->get_clock()->now();
  double dt = (now - last_heading_command_time_).seconds();
  last_heading_command_time_ = now;

  if (dt <= 0.0 || dt > 0.25) {
    dt = 0.05;
  }
  if (dt < 0.005) {
    return last_heading_angular_velocity_;
  }

  // On a sign reversal, brake to zero first. This preserves shortest-turn
  // behavior without commanding an abrupt full deadzone jump in the opposite
  // direction.
  if (target_angular_velocity * last_heading_angular_velocity_ < 0.0) {
    const double max_delta = forward_max_angular_decel_ * dt;
    if (std::abs(last_heading_angular_velocity_) <= max_delta) {
      last_heading_angular_velocity_ = 0.0;
    } else {
      last_heading_angular_velocity_ -=
        std::copysign(max_delta, last_heading_angular_velocity_);
    }
    return last_heading_angular_velocity_;
  }

  const bool slowing_down =
    std::abs(target_angular_velocity) < std::abs(last_heading_angular_velocity_);
  const double rate_limit =
    slowing_down ? forward_max_angular_decel_ : forward_max_angular_accel_;
  const double max_delta = rate_limit * dt;
  const double delta = std::clamp(
    target_angular_velocity - last_heading_angular_velocity_, -max_delta, max_delta);
  last_heading_angular_velocity_ += delta;
  if (std::abs(last_heading_angular_velocity_) < EPSILON) {
    last_heading_angular_velocity_ = 0.0;
  }
  return last_heading_angular_velocity_;
}

}  // namespace fake_vel_transform

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fake_vel_transform::FakeVelTransform)

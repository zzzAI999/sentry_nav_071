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

#ifndef FAKE_VEL_TRANSFORM__FAKE_VEL_TRANSFORM_HPP_
#define FAKE_VEL_TRANSFORM__FAKE_VEL_TRANSFORM_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "example_interfaces/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace fake_vel_transform
{
class FakeVelTransform : public rclcpp::Node
{
public:
  explicit FakeVelTransform(const rclcpp::NodeOptions & options);

private:
  void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void teleopCmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void teleopActiveCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void cmdSpinCallback(example_interfaces::msg::Float32::SharedPtr msg);
  void publishTransform();
  void enforceTeleopWatchdog();
  void publishZeroVelocity();
  geometry_msgs::msg::Twist transformVelocity(
    const geometry_msgs::msg::Twist::SharedPtr & twist, float yaw_diff);
  double limitForwardLinearVelocity(double target_linear_velocity);
  double limitHeadingAngularVelocity(double target_angular_velocity);

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr teleop_active_sub_;
  rclcpp::Subscription<example_interfaces::msg::Float32>::SharedPtr cmd_spin_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_chassis_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::string robot_base_frame_;
  std::string fake_robot_base_frame_;
  std::string odom_topic_;
  std::string cmd_spin_topic_;
  std::string input_cmd_vel_topic_;
  std::string output_cmd_vel_topic_;
  std::string teleop_cmd_vel_topic_;
  std::string teleop_active_topic_;
  float spin_speed_;
  bool forward_only_enabled_;
  double forward_angular_command_sign_;
  double forward_heading_kp_;
  double forward_heading_deadband_rad_;
  double forward_heading_correction_enter_rad_;
  double forward_rotate_enter_angle_rad_;
  double forward_rotate_exit_angle_rad_;
  double forward_max_angular_speed_;
  double forward_max_angular_accel_;
  double forward_max_angular_decel_;
  double forward_max_linear_accel_;
  double forward_max_linear_decel_;
  double forward_settle_angular_speed_;
  double forward_min_command_speed_;
  double forward_min_linear_speed_;
  double forward_min_angular_speed_;
  double teleop_watchdog_timeout_sec_;

  std::mutex cmd_vel_mutex_;
  double current_robot_base_angle_;
  rclcpp::Time last_linear_command_time_;
  rclcpp::Time last_heading_command_time_;
  rclcpp::Time last_teleop_command_time_;
  bool teleop_active_{false};
  double last_forward_linear_velocity_{0.0};
  double last_heading_angular_velocity_{0.0};
  bool heading_correction_active_{false};
  bool rotating_to_heading_{false};
};

}  // namespace fake_vel_transform

#endif  // FAKE_VEL_TRANSFORM__FAKE_VEL_TRANSFORM_HPP_

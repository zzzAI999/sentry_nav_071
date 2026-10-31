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

#include "small_gicp_relocalization/small_gicp_relocalization.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "pcl/common/transforms.h"
#include "pcl_conversions/pcl_conversions.h"
#include "small_gicp/pcl/pcl_registration.hpp"
#include "small_gicp/util/downsampling_omp.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

namespace small_gicp_relocalization
{

SmallGicpRelocalizationNode::SmallGicpRelocalizationNode(const rclcpp::NodeOptions & options)
: Node("small_gicp_relocalization", options),
  has_accumulation_start_time_(false),
  coarse_relocalization_pending_(true),
  trusted_initial_pose_received_(false),
  has_pending_map_to_odom_seed_(false),
  consecutive_reject_count_(0),
  global_search_grid_ready_(false),
  map_frame_correction_(Eigen::Isometry3d::Identity()),
  pending_map_to_odom_seed_(Eigen::Isometry3d::Identity()),
  result_t_(Eigen::Isometry3d::Identity()),
  previous_result_t_(Eigen::Isometry3d::Identity())
{
  this->declare_parameter("num_threads", 4);
  this->declare_parameter("num_neighbors", 20);
  this->declare_parameter("global_leaf_size", 0.25);
  this->declare_parameter("registered_leaf_size", 0.25);
  this->declare_parameter("max_dist_sq", 1.0);
  this->declare_parameter("min_source_points", 100);
  this->declare_parameter("min_inlier_ratio", 0.35);
  this->declare_parameter("max_mean_error", 0.08);
  this->declare_parameter("max_translation_jump", 0.35);
  this->declare_parameter("max_yaw_jump_rad", 0.35);
  this->declare_parameter("result_smoothing_alpha", 0.25);
  this->declare_parameter("max_accumulated_points", 120000);
  this->declare_parameter("max_accumulation_time_sec", 2.0);
  this->declare_parameter("coarse_relocalization_enabled", true);
  this->declare_parameter("coarse_trigger_failed_attempts", 3);
  this->declare_parameter("coarse_leaf_size", 0.25);
  this->declare_parameter("coarse_xy_search_radius", 1.5);
  this->declare_parameter("coarse_xy_resolution", 0.5);
  this->declare_parameter("coarse_yaw_search_range_rad", 1.5708);
  this->declare_parameter("coarse_yaw_resolution_rad", 0.7854);
  this->declare_parameter("coarse_max_candidates", 160);
  this->declare_parameter("coarse_max_dist_sq", 2.25);
  this->declare_parameter("coarse_max_iterations", 5);
  this->declare_parameter("coarse_min_inlier_ratio", 0.20);
  this->declare_parameter("coarse_max_mean_error", 0.30);
  this->declare_parameter("coarse_max_translation_jump", 3.0);
  this->declare_parameter("coarse_max_yaw_jump_rad", 3.1416);
  this->declare_parameter("coarse_direct_accept_enabled", true);
  this->declare_parameter("coarse_direct_max_translation_jump", 1.2);
  this->declare_parameter("coarse_direct_max_yaw_jump_rad", 1.2);
  this->declare_parameter("coarse_direct_accept_cooldown_sec", 3.0);
  this->declare_parameter("coarse_score_translation_penalty", 0.08);
  this->declare_parameter("coarse_score_yaw_penalty", 0.06);
  this->declare_parameter("fine_after_coarse_max_dist_sq", 1.2);
  this->declare_parameter("fine_after_coarse_max_iterations", 20);
  this->declare_parameter("fine_max_iterations", 30);
  this->declare_parameter("fine_translation_epsilon", 0.01);
  this->declare_parameter("fine_rotation_epsilon_rad", 0.01);
  this->declare_parameter("coarse_translation_epsilon", 0.03);
  this->declare_parameter("coarse_rotation_epsilon_rad", 0.03);
  this->declare_parameter("coarse_refine_hypotheses", 4);
  this->declare_parameter("coarse_hypothesis_translation_separation", 0.25);
  this->declare_parameter("coarse_hypothesis_yaw_separation_rad", 0.20);
  this->declare_parameter("global_search_enabled", true);
  this->declare_parameter("global_search_voxel_resolution", 0.30);
  this->declare_parameter("global_search_xy_resolution", 0.60);
  this->declare_parameter("global_search_yaw_resolution_rad", 0.3926991);
  this->declare_parameter("global_search_z_range", 0.30);
  this->declare_parameter("global_search_z_resolution", 0.30);
  this->declare_parameter("global_search_top_candidates", 24);
  this->declare_parameter("global_search_max_source_points", 1200);
  this->declare_parameter("global_search_min_source_points", 800);
  this->declare_parameter("global_search_dilation_voxels", 1);
  this->declare_parameter("global_search_min_occupancy_ratio", 0.18);
  this->declare_parameter("global_search_score_weight", 0.05);
  this->declare_parameter("relocalization_consensus_required", 3);
  this->declare_parameter("relocalization_consensus_window", 30);
  this->declare_parameter("relocalization_consensus_translation_tolerance", 0.18);
  this->declare_parameter("relocalization_consensus_yaw_tolerance_rad", 0.15);
  this->declare_parameter("relocalization_consensus_min_inlier_ratio", 0.88);
  this->declare_parameter("relocalization_consensus_max_mean_error", 0.08);
  this->declare_parameter("trusted_recovery_enabled", true);
  this->declare_parameter("trusted_recovery_trigger_rejects", 3);
  this->declare_parameter("trusted_recovery_consensus_required", 5);
  this->declare_parameter("trusted_recovery_max_translation_jump", 1.50);
  this->declare_parameter("trusted_recovery_max_yaw_jump_rad", 1.20);
  this->declare_parameter("trusted_recovery_min_inlier_ratio", 0.90);
  this->declare_parameter("trusted_recovery_max_mean_error", 0.05);
  this->declare_parameter("trusted_recovery_alpha", 0.50);
  this->declare_parameter("tf_future_offset_sec", 0.15);
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("odom_frame", "odom");
  this->declare_parameter("base_frame", "");
  this->declare_parameter("robot_base_frame", "");
  this->declare_parameter("lidar_frame", "");
  this->declare_parameter("prior_pcd_file", "");
  this->declare_parameter("init_pose", std::vector<double>{0., 0., 0., 0., 0., 0.});
  this->declare_parameter("input_cloud_topic", "registered_scan");

  this->get_parameter("num_threads", num_threads_);
  this->get_parameter("num_neighbors", num_neighbors_);
  this->get_parameter("global_leaf_size", global_leaf_size_);
  this->get_parameter("registered_leaf_size", registered_leaf_size_);
  this->get_parameter("max_dist_sq", max_dist_sq_);
  this->get_parameter("min_source_points", min_source_points_);
  this->get_parameter("min_inlier_ratio", min_inlier_ratio_);
  this->get_parameter("max_mean_error", max_mean_error_);
  this->get_parameter("max_translation_jump", max_translation_jump_);
  this->get_parameter("max_yaw_jump_rad", max_yaw_jump_rad_);
  this->get_parameter("result_smoothing_alpha", result_smoothing_alpha_);
  this->get_parameter("max_accumulated_points", max_accumulated_points_);
  this->get_parameter("max_accumulation_time_sec", max_accumulation_time_sec_);
  this->get_parameter("coarse_relocalization_enabled", coarse_relocalization_enabled_);
  this->get_parameter("coarse_trigger_failed_attempts", coarse_trigger_failed_attempts_);
  this->get_parameter("coarse_leaf_size", coarse_leaf_size_);
  this->get_parameter("coarse_xy_search_radius", coarse_xy_search_radius_);
  this->get_parameter("coarse_xy_resolution", coarse_xy_resolution_);
  this->get_parameter("coarse_yaw_search_range_rad", coarse_yaw_search_range_rad_);
  this->get_parameter("coarse_yaw_resolution_rad", coarse_yaw_resolution_rad_);
  this->get_parameter("coarse_max_candidates", coarse_max_candidates_);
  this->get_parameter("coarse_max_dist_sq", coarse_max_dist_sq_);
  this->get_parameter("coarse_max_iterations", coarse_max_iterations_);
  this->get_parameter("coarse_min_inlier_ratio", coarse_min_inlier_ratio_);
  this->get_parameter("coarse_max_mean_error", coarse_max_mean_error_);
  this->get_parameter("coarse_max_translation_jump", coarse_max_translation_jump_);
  this->get_parameter("coarse_max_yaw_jump_rad", coarse_max_yaw_jump_rad_);
  this->get_parameter("coarse_direct_accept_enabled", coarse_direct_accept_enabled_);
  this->get_parameter("coarse_direct_max_translation_jump", coarse_direct_max_translation_jump_);
  this->get_parameter("coarse_direct_max_yaw_jump_rad", coarse_direct_max_yaw_jump_rad_);
  this->get_parameter("coarse_direct_accept_cooldown_sec", coarse_direct_accept_cooldown_sec_);
  this->get_parameter("coarse_score_translation_penalty", coarse_score_translation_penalty_);
  this->get_parameter("coarse_score_yaw_penalty", coarse_score_yaw_penalty_);
  this->get_parameter("fine_after_coarse_max_dist_sq", fine_after_coarse_max_dist_sq_);
  this->get_parameter("fine_after_coarse_max_iterations", fine_after_coarse_max_iterations_);
  this->get_parameter("fine_max_iterations", fine_max_iterations_);
  this->get_parameter("fine_translation_epsilon", fine_translation_epsilon_);
  this->get_parameter("fine_rotation_epsilon_rad", fine_rotation_epsilon_rad_);
  this->get_parameter("coarse_translation_epsilon", coarse_translation_epsilon_);
  this->get_parameter("coarse_rotation_epsilon_rad", coarse_rotation_epsilon_rad_);
  this->get_parameter("coarse_refine_hypotheses", coarse_refine_hypotheses_);
  this->get_parameter(
    "coarse_hypothesis_translation_separation",
    coarse_hypothesis_translation_separation_);
  this->get_parameter(
    "coarse_hypothesis_yaw_separation_rad", coarse_hypothesis_yaw_separation_rad_);
  this->get_parameter("global_search_enabled", global_search_enabled_);
  this->get_parameter("global_search_voxel_resolution", global_search_voxel_resolution_);
  this->get_parameter("global_search_xy_resolution", global_search_xy_resolution_);
  this->get_parameter(
    "global_search_yaw_resolution_rad", global_search_yaw_resolution_rad_);
  this->get_parameter("global_search_z_range", global_search_z_range_);
  this->get_parameter("global_search_z_resolution", global_search_z_resolution_);
  this->get_parameter("global_search_top_candidates", global_search_top_candidates_);
  this->get_parameter("global_search_max_source_points", global_search_max_source_points_);
  this->get_parameter("global_search_min_source_points", global_search_min_source_points_);
  this->get_parameter("global_search_dilation_voxels", global_search_dilation_voxels_);
  this->get_parameter(
    "global_search_min_occupancy_ratio", global_search_min_occupancy_ratio_);
  this->get_parameter("global_search_score_weight", global_search_score_weight_);
  this->get_parameter("relocalization_consensus_required", relocalization_consensus_required_);
  this->get_parameter("relocalization_consensus_window", relocalization_consensus_window_);
  this->get_parameter(
    "relocalization_consensus_translation_tolerance",
    relocalization_consensus_translation_tolerance_);
  this->get_parameter(
    "relocalization_consensus_yaw_tolerance_rad",
    relocalization_consensus_yaw_tolerance_rad_);
  this->get_parameter(
    "relocalization_consensus_min_inlier_ratio",
    relocalization_consensus_min_inlier_ratio_);
  this->get_parameter(
    "relocalization_consensus_max_mean_error", relocalization_consensus_max_mean_error_);
  this->get_parameter("trusted_recovery_enabled", trusted_recovery_enabled_);
  this->get_parameter(
    "trusted_recovery_trigger_rejects", trusted_recovery_trigger_rejects_);
  this->get_parameter(
    "trusted_recovery_consensus_required", trusted_recovery_consensus_required_);
  this->get_parameter(
    "trusted_recovery_max_translation_jump", trusted_recovery_max_translation_jump_);
  this->get_parameter(
    "trusted_recovery_max_yaw_jump_rad", trusted_recovery_max_yaw_jump_rad_);
  this->get_parameter(
    "trusted_recovery_min_inlier_ratio", trusted_recovery_min_inlier_ratio_);
  this->get_parameter(
    "trusted_recovery_max_mean_error", trusted_recovery_max_mean_error_);
  this->get_parameter("trusted_recovery_alpha", trusted_recovery_alpha_);
  this->get_parameter("tf_future_offset_sec", tf_future_offset_sec_);
  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("prior_pcd_file", prior_pcd_file_);
  this->get_parameter("init_pose", init_pose_);
  this->get_parameter("input_cloud_topic", input_cloud_topic_);

  // [x, y, z, roll, pitch, yaw] - init_pose parameters
  if (!init_pose_.empty() && init_pose_.size() >= 6) {
    result_t_.translation() << init_pose_[0], init_pose_[1], init_pose_[2];
    result_t_.linear() =
      Eigen::AngleAxisd(init_pose_[5], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(init_pose_[4], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(init_pose_[3], Eigen::Vector3d::UnitX()).toRotationMatrix();
  }
  previous_result_t_ = result_t_;

  accumulated_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  register_ = std::make_shared<
    small_gicp::Registration<
      small_gicp::RobustFactor<small_gicp::Huber, small_gicp::GICPFactor>,
      small_gicp::ParallelReductionOMP>>();
  coarse_register_ = std::make_shared<
    small_gicp::Registration<
      small_gicp::RobustFactor<small_gicp::Huber, small_gicp::GICPFactor>,
      small_gicp::ParallelReductionOMP>>();

  register_->criteria.translation_eps =
    std::max(1e-4, static_cast<double>(fine_translation_epsilon_));
  register_->criteria.rotation_eps =
    std::max(1e-4, static_cast<double>(fine_rotation_epsilon_rad_));
  coarse_register_->criteria.translation_eps =
    std::max(1e-4, static_cast<double>(coarse_translation_epsilon_));
  coarse_register_->criteria.rotation_eps =
    std::max(1e-4, static_cast<double>(coarse_rotation_epsilon_rad_));

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  localization_ready_pub_ = this->create_publisher<std_msgs::msg::Bool>(
    "localization_ready", rclcpp::QoS(1).reliable().transient_local());

  loadGlobalMap(prior_pcd_file_);

  // Downsample points and convert them into pcl::PointCloud<pcl::PointCovariance>
  target_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *global_map_, global_leaf_size_);

  // Estimate covariances of points
  small_gicp::estimate_covariances_omp(*target_, num_neighbors_, num_threads_);

  // Create KdTree for target
  target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    target_, small_gicp::KdTreeBuilderOMP(num_threads_));

  if (coarse_relocalization_enabled_) {
    coarse_target_ = small_gicp::voxelgrid_sampling_omp<
      pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
      *global_map_, coarse_leaf_size_);
    small_gicp::estimate_covariances_omp(*coarse_target_, num_neighbors_, num_threads_);
    coarse_target_tree_ =
      std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
      coarse_target_, small_gicp::KdTreeBuilderOMP(num_threads_));
  }

  pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_cloud_topic_, 10,
    std::bind(&SmallGicpRelocalizationNode::registeredPcdCallback, this, std::placeholders::_1));

  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 10,
    std::bind(&SmallGicpRelocalizationNode::initialPoseCallback, this, std::placeholders::_1));

  register_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000),  // 1 Hz
    std::bind(&SmallGicpRelocalizationNode::performRegistration, this));

  transform_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),  // 20 Hz
    std::bind(&SmallGicpRelocalizationNode::publishTransform, this));
}

void SmallGicpRelocalizationNode::loadGlobalMap(const std::string & file_name)
{
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *global_map_) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file: %s", file_name.c_str());
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Loaded global map with %zu points", global_map_->points.size());

  // NOTE: Transform global pcd_map (based on `lidar_odom` frame) to the `odom` frame
  Eigen::Affine3d odom_to_lidar_odom;
  while (true) {
    try {
      auto tf_stamped = tf_buffer_->lookupTransform(
        base_frame_, lidar_frame_, this->now(), rclcpp::Duration::from_seconds(1.0));
      odom_to_lidar_odom = tf2::transformToEigen(tf_stamped.transform);
      RCLCPP_INFO_STREAM(
        this->get_logger(), "odom_to_lidar_odom: translation = "
                              << odom_to_lidar_odom.translation().transpose() << ", rpy = "
                              << odom_to_lidar_odom.rotation().eulerAngles(0, 1, 2).transpose());
      break;
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s Retrying...", ex.what());
      rclcpp::sleep_for(std::chrono::seconds(1));
    }
  }
  pcl::transformPointCloud(*global_map_, *global_map_, odom_to_lidar_odom);
  buildGlobalSearchGrid();
}

size_t SmallGicpRelocalizationNode::globalGridOffset(int x, int y, int z) const
{
  return static_cast<size_t>(x) +
         static_cast<size_t>(global_grid_dims_[0]) *
         (static_cast<size_t>(y) +
         static_cast<size_t>(global_grid_dims_[1]) * static_cast<size_t>(z));
}

void SmallGicpRelocalizationNode::buildGlobalSearchGrid()
{
  global_search_grid_ready_ = false;
  global_occupancy_grid_.clear();
  if (!global_search_enabled_ || global_map_->empty()) {
    return;
  }

  const double resolution = std::max(
    0.05, static_cast<double>(global_search_voxel_resolution_));
  global_map_min_ = Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
  global_map_max_ = Eigen::Vector3d::Constant(std::numeric_limits<double>::lowest());
  for (const auto & point : global_map_->points) {
    const Eigen::Vector3d p(point.x, point.y, point.z);
    if (!p.allFinite()) {
      continue;
    }
    global_map_min_ = global_map_min_.cwiseMin(p);
    global_map_max_ = global_map_max_.cwiseMax(p);
  }
  if (!global_map_min_.allFinite() || !global_map_max_.allFinite()) {
    return;
  }

  const int dilation = std::max(0, global_search_dilation_voxels_);
  const double margin = static_cast<double>(dilation + 1) * resolution;
  global_grid_min_ =
    (global_map_min_.array() / resolution).floor().matrix() * resolution -
    Eigen::Vector3d::Constant(margin);
  const Eigen::Vector3d grid_max =
    (global_map_max_.array() / resolution).ceil().matrix() * resolution +
    Eigen::Vector3d::Constant(margin);
  for (int axis = 0; axis < 3; ++axis) {
    global_grid_dims_[axis] = std::max(
      1, static_cast<int>(std::ceil(
        (grid_max[axis] - global_grid_min_[axis]) / resolution)) + 1);
  }

  const size_t cell_count =
    static_cast<size_t>(global_grid_dims_[0]) *
    static_cast<size_t>(global_grid_dims_[1]) *
    static_cast<size_t>(global_grid_dims_[2]);
  if (cell_count == 0 || cell_count > 100000000U) {
    RCLCPP_ERROR(
      this->get_logger(), "Global search grid has invalid size: %zu cells", cell_count);
    return;
  }

  std::vector<std::uint8_t> occupied(cell_count, 0U);
  for (const auto & point : global_map_->points) {
    const Eigen::Vector3d p(point.x, point.y, point.z);
    if (!p.allFinite()) {
      continue;
    }
    const Eigen::Array3i index =
      ((p - global_grid_min_) / resolution).array().floor().cast<int>();
    if (
      (index >= 0).all() && index[0] < global_grid_dims_[0] &&
      index[1] < global_grid_dims_[1] && index[2] < global_grid_dims_[2])
    {
      occupied[globalGridOffset(index[0], index[1], index[2])] = 1U;
    }
  }

  global_occupancy_grid_.assign(cell_count, 0U);
  for (int z = 0; z < global_grid_dims_[2]; ++z) {
    for (int y = 0; y < global_grid_dims_[1]; ++y) {
      for (int x = 0; x < global_grid_dims_[0]; ++x) {
        if (occupied[globalGridOffset(x, y, z)] == 0U) {
          continue;
        }
        for (int dz = -dilation; dz <= dilation; ++dz) {
          for (int dy = -dilation; dy <= dilation; ++dy) {
            for (int dx = -dilation; dx <= dilation; ++dx) {
              const int nx = x + dx;
              const int ny = y + dy;
              const int nz = z + dz;
              if (
                nx >= 0 && ny >= 0 && nz >= 0 && nx < global_grid_dims_[0] &&
                ny < global_grid_dims_[1] && nz < global_grid_dims_[2])
              {
                global_occupancy_grid_[globalGridOffset(nx, ny, nz)] = 1U;
              }
            }
          }
        }
      }
    }
  }

  global_search_grid_ready_ = true;
  RCLCPP_INFO(
    this->get_logger(),
    "Global search grid ready. bounds=[%.2f %.2f]x[%.2f %.2f]x[%.2f %.2f] "
    "dims=%dx%dx%d resolution=%.2f",
    global_map_min_.x(), global_map_max_.x(), global_map_min_.y(), global_map_max_.y(),
    global_map_min_.z(), global_map_max_.z(), global_grid_dims_[0], global_grid_dims_[1],
    global_grid_dims_[2], resolution);
}

double SmallGicpRelocalizationNode::scoreGlobalCandidate(
  const pcl::PointCloud<pcl::PointCovariance> & source,
  const Eigen::Isometry3d & candidate) const
{
  if (!global_search_grid_ready_ || source.empty()) {
    return 0.0;
  }

  const double resolution = std::max(
    0.05, static_cast<double>(global_search_voxel_resolution_));
  const size_t max_points = static_cast<size_t>(std::max(1, global_search_max_source_points_));
  const size_t stride = std::max<size_t>(1, (source.size() + max_points - 1) / max_points);
  size_t tested = 0;
  size_t occupied = 0;
  for (size_t i = 0; i < source.size(); i += stride) {
    const auto & point = source.points[i];
    const Eigen::Vector3d p = candidate * Eigen::Vector3d(point.x, point.y, point.z);
    if (!p.allFinite()) {
      continue;
    }
    ++tested;
    const Eigen::Array3i index =
      ((p - global_grid_min_) / resolution).array().floor().cast<int>();
    if (
      (index >= 0).all() && index[0] < global_grid_dims_[0] &&
      index[1] < global_grid_dims_[1] && index[2] < global_grid_dims_[2] &&
      global_occupancy_grid_[globalGridOffset(index[0], index[1], index[2])] != 0U)
    {
      ++occupied;
    }
  }
  return tested > 0 ? static_cast<double>(occupied) / static_cast<double>(tested) : 0.0;
}

bool SmallGicpRelocalizationNode::findGlobalInitialGuesses(
  const pcl::PointCloud<pcl::PointCovariance> & source,
  std::vector<std::pair<double, Eigen::Isometry3d>> * candidates) const
{
  if (!candidates || !global_search_grid_ready_) {
    return false;
  }

  const double xy_resolution = std::max(
    0.10, static_cast<double>(global_search_xy_resolution_));
  const double requested_yaw_resolution = std::max(
    0.05, static_cast<double>(global_search_yaw_resolution_rad_));
  constexpr double kPi = 3.14159265358979323846;
  const int yaw_steps = std::max(
    4, static_cast<int>(std::ceil(2.0 * kPi / requested_yaw_resolution)));
  const double yaw_resolution = 2.0 * kPi / static_cast<double>(yaw_steps);
  const double z_range = std::max(0.0, static_cast<double>(global_search_z_range_));
  const double z_resolution = std::max(
    0.05, static_cast<double>(global_search_z_resolution_));
  const int z_steps = static_cast<int>(std::ceil(z_range / z_resolution));

  std::vector<Eigen::Isometry3d> search_poses;
  for (
    double x = global_map_min_.x();
    x <= global_map_max_.x() + 0.5 * xy_resolution; x += xy_resolution)
  {
    for (
      double y = global_map_min_.y();
      y <= global_map_max_.y() + 0.5 * xy_resolution; y += xy_resolution)
    {
      for (int iz = -z_steps; iz <= z_steps; ++iz) {
        const double z = previous_result_t_.translation().z() + iz * z_resolution;
        for (int iyaw = 0; iyaw < yaw_steps; ++iyaw) {
          const double yaw = -kPi + iyaw * yaw_resolution;
          Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
          pose.translation() << x, y, z;
          pose.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
          search_poses.push_back(pose);
        }
      }
    }
  }

  std::vector<double> scores(search_poses.size(), 0.0);
#pragma omp parallel for num_threads(num_threads_) schedule(static)
  for (long i = 0; i < static_cast<long>(search_poses.size()); ++i) {
    scores[static_cast<size_t>(i)] = scoreGlobalCandidate(source, search_poses[static_cast<size_t>(i)]);
  }

  std::vector<size_t> order(search_poses.size());
  for (size_t i = 0; i < order.size(); ++i) {
    order[i] = i;
  }
  std::sort(
    order.begin(), order.end(),
    [&scores](size_t lhs, size_t rhs) { return scores[lhs] > scores[rhs]; });

  candidates->clear();
  const size_t max_candidates = static_cast<size_t>(std::max(1, global_search_top_candidates_));
  const double min_score = std::clamp(
    static_cast<double>(global_search_min_occupancy_ratio_), 0.0, 1.0);
  for (const size_t index : order) {
    if (scores[index] < min_score) {
      break;
    }
    const auto & pose = search_poses[index];
    bool distinct = true;
    for (const auto & selected : *candidates) {
      const Eigen::Isometry3d delta = selected.second.inverse() * pose;
      const double delta_yaw = std::abs(normalizeAngle(
        std::atan2(delta.rotation()(1, 0), delta.rotation()(0, 0))));
      if (
        delta.translation().head<2>().norm() < 0.75 * xy_resolution &&
        delta_yaw < 0.75 * yaw_resolution)
      {
        distinct = false;
        break;
      }
    }
    if (distinct) {
      candidates->emplace_back(scores[index], pose);
      if (candidates->size() >= max_candidates) {
        break;
      }
    }
  }

  if (candidates->empty()) {
    RCLCPP_WARN(
      this->get_logger(), "Global search found no candidate above occupancy ratio %.3f",
      min_score);
    return false;
  }
  RCLCPP_INFO(
    this->get_logger(),
    "Global search seeds ready. evaluated=%zu selected=%zu best_occupancy=%.3f",
    search_poses.size(), candidates->size(), candidates->front().first);
  return true;
}

void SmallGicpRelocalizationNode::registeredPcdCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  last_scan_time_ = msg->header.stamp;
  current_scan_frame_id_ = msg->header.frame_id;

  if (!has_accumulation_start_time_) {
    accumulation_start_time_ = msg->header.stamp;
    has_accumulation_start_time_ = true;
  } else if (max_accumulation_time_sec_ > 0.0) {
    const double accumulation_age_sec = (rclcpp::Time(msg->header.stamp) - accumulation_start_time_).seconds();
    if (accumulation_age_sec > static_cast<double>(max_accumulation_time_sec_)) {
      RCLCPP_WARN(
        this->get_logger(),
        "Accumulated cloud timeout (%.3fs > %.3fs). Reset accumulation.",
        accumulation_age_sec, max_accumulation_time_sec_);
      accumulated_cloud_->clear();
      accumulation_start_time_ = msg->header.stamp;
    }
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *scan);

  if (max_accumulated_points_ > 0) {
    const size_t max_points = static_cast<size_t>(max_accumulated_points_);
    if (accumulated_cloud_->size() >= max_points) {
      RCLCPP_WARN(
        this->get_logger(),
        "Accumulated cloud reached point cap (%zu). Reset accumulation.",
        accumulated_cloud_->size());
      accumulated_cloud_->clear();
      accumulation_start_time_ = msg->header.stamp;
    }
  }

  *accumulated_cloud_ += *scan;

  if (max_accumulated_points_ > 0) {
    const size_t max_points = static_cast<size_t>(max_accumulated_points_);
    if (accumulated_cloud_->size() > max_points) {
      const size_t overflow = accumulated_cloud_->size() - max_points;
      accumulated_cloud_->points.erase(
        accumulated_cloud_->points.begin(), accumulated_cloud_->points.begin() + overflow);
      accumulated_cloud_->width = accumulated_cloud_->points.size();
      accumulated_cloud_->height = 1;
      accumulated_cloud_->is_dense = false;
      RCLCPP_WARN(
        this->get_logger(),
        "Trim accumulated cloud by %zu points to stay within cap (%zu).",
        overflow, max_points);
    }
  }
}

void SmallGicpRelocalizationNode::performRegistration()
{
  if (accumulated_cloud_->empty()) {
    RCLCPP_WARN(this->get_logger(), "No accumulated points to process.");
    return;
  }

  source_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *accumulated_cloud_, registered_leaf_size_);

  if (!source_ || source_->empty()) {
    return;
  }
  if (
    global_search_enabled_ && coarse_relocalization_pending_ &&
    source_->size() < static_cast<size_t>(std::max(1, global_search_min_source_points_)))
  {
    RCLCPP_INFO(
      this->get_logger(), "Waiting for a denser initial cloud. source=%zu required=%d",
      source_->size(), global_search_min_source_points_);
    return;
  }

  small_gicp::estimate_covariances_omp(*source_, num_neighbors_, num_threads_);

  source_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    source_, small_gicp::KdTreeBuilderOMP(num_threads_));

  if (!source_tree_) {
    return;
  }

  Eigen::Isometry3d initial_guess = previous_result_t_;
  const bool should_run_coarse =
    coarse_relocalization_enabled_ && !trusted_initial_pose_received_ &&
    (coarse_relocalization_pending_ ||
    consecutive_reject_count_ >= std::max(1, coarse_trigger_failed_attempts_));
  const bool coarse_found = should_run_coarse && findCoarseInitialGuess(&initial_guess);
  if (should_run_coarse && !coarse_found) {
    RCLCPP_WARN(this->get_logger(), "Coarse relocalization did not find an acceptable seed.");
  }

  register_->reduction.num_threads = num_threads_;
  register_->rejector.max_dist_sq =
    coarse_found ? fine_after_coarse_max_dist_sq_ : max_dist_sq_;
  register_->optimizer.max_iterations =
    coarse_found ? std::max(1, fine_after_coarse_max_iterations_) :
    std::max(1, fine_max_iterations_);

  small_gicp::RegistrationResult result;
  if (coarse_found && !coarse_hypotheses_.empty()) {
    double best_fine_score = std::numeric_limits<double>::max();
    bool found_fine_result = false;
    for (const auto & hypothesis : coarse_hypotheses_) {
      auto candidate_result = register_->align(*target_, *source_, *target_tree_, hypothesis);
      const double candidate_ratio = source_->empty() ? 0.0 :
        static_cast<double>(candidate_result.num_inliers) / static_cast<double>(source_->size());
      const double candidate_mean_error = candidate_result.error /
        static_cast<double>(std::max<size_t>(1, candidate_result.num_inliers));
      if (
        !candidate_result.T_target_source.matrix().allFinite() ||
        !std::isfinite(candidate_ratio) || !std::isfinite(candidate_mean_error))
      {
        continue;
      }
      const double fine_score =
        candidate_mean_error / std::max(0.05, candidate_ratio) +
        (candidate_result.converged ? 0.0 : 0.25);
      if (fine_score < best_fine_score) {
        best_fine_score = fine_score;
        result = candidate_result;
        found_fine_result = true;
      }
    }
    if (!found_fine_result) {
      result = register_->align(*target_, *source_, *target_tree_, initial_guess);
    }
    RCLCPP_INFO(
      this->get_logger(),
      "Fine relocalization selected from %zu hypotheses. converged=%d score=%.4f",
      coarse_hypotheses_.size(), static_cast<int>(result.converged), best_fine_score);
  } else {
    result = register_->align(*target_, *source_, *target_tree_, initial_guess);
  }

  double inlier_ratio = 0.0;
  double mean_error = 0.0;
  double delta_translation = 0.0;
  double delta_yaw = 0.0;
  const double translation_gate =
    coarse_found ? coarse_max_translation_jump_ : max_translation_jump_;
  const double yaw_gate = coarse_found ? coarse_max_yaw_jump_rad_ : max_yaw_jump_rad_;
  const bool acceptable = isRegistrationAcceptable(
    result, source_->size(), &inlier_ratio, &mean_error, &delta_translation, &delta_yaw,
    translation_gate, yaw_gate);

  bool accepted = false;
  if (!trusted_initial_pose_received_)
  {
    // Initial fine-only results need the same consensus before the map seed
    // can be applied; otherwise they can bypass and disable initial consensus.
    const bool consensus_quality_ok =
      (coarse_found || acceptable) && result.converged &&
      result.T_target_source.matrix().allFinite() &&
      inlier_ratio >= static_cast<double>(relocalization_consensus_min_inlier_ratio_) &&
      mean_error <= static_cast<double>(relocalization_consensus_max_mean_error_);
    if (consensus_quality_ok) {
      Eigen::Isometry3d consensus_pose = Eigen::Isometry3d::Identity();
      size_t consensus_support = 0;
      if (updateRelocalizationConsensus(
          result.T_target_source, &consensus_pose, &consensus_support))
      {
        acceptRegistrationPose(consensus_pose, 1.0);
        trusted_initial_pose_received_ = true;
        if (has_pending_map_to_odom_seed_) {
          applyMapFrameCorrection(pending_map_to_odom_seed_);
        }
        accepted = true;
        const Eigen::Vector3d t = consensus_pose.translation();
        const Eigen::Matrix3d r = consensus_pose.rotation();
        const double yaw = normalizeAngle(std::atan2(r(1, 0), r(0, 0)));
        RCLCPP_WARN(
          this->get_logger(),
          "Relocalization consensus accepted. support=%zu pose=(%.3f,%.3f,%.3f,%.3f) "
          "ratio=%.3f mean_err=%.4f",
          consensus_support, t.x(), t.y(), t.z(), yaw, inlier_ratio, mean_error);
      }
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "Relocalization candidate failed quality gate. converged=%d ratio=%.3f/%.3f "
        "mean_err=%.4f/%.4f",
        static_cast<int>(result.converged), inlier_ratio,
        static_cast<double>(relocalization_consensus_min_inlier_ratio_), mean_error,
        static_cast<double>(relocalization_consensus_max_mean_error_));
    }
  } else if (acceptable) {
    const double alpha =
      coarse_found ? 1.0 : std::clamp(static_cast<double>(result_smoothing_alpha_), 0.0, 1.0);
    acceptRegistrationPose(result.T_target_source, alpha);
    accepted = true;
  } else if (
    trusted_recovery_enabled_ && trusted_initial_pose_received_ &&
    consecutive_reject_count_ + 1 >= std::max(1, trusted_recovery_trigger_rejects_) &&
    result.converged && result.T_target_source.matrix().allFinite() &&
    inlier_ratio >= static_cast<double>(trusted_recovery_min_inlier_ratio_) &&
    mean_error <= static_cast<double>(trusted_recovery_max_mean_error_) &&
    delta_translation <= static_cast<double>(trusted_recovery_max_translation_jump_) &&
    delta_yaw <= static_cast<double>(trusted_recovery_max_yaw_jump_rad_))
  {
    Eigen::Isometry3d consensus_pose = Eigen::Isometry3d::Identity();
    size_t consensus_support = 0;
    const size_t required =
      static_cast<size_t>(std::max(2, trusted_recovery_consensus_required_));
    if (updateRelocalizationConsensus(
        result.T_target_source, &consensus_pose, &consensus_support, required))
    {
      acceptRegistrationPose(
        consensus_pose, std::clamp(static_cast<double>(trusted_recovery_alpha_), 0.0, 1.0));
      accepted = true;
      const Eigen::Vector3d t = consensus_pose.translation();
      const Eigen::Matrix3d r = consensus_pose.rotation();
      const double yaw = normalizeAngle(std::atan2(r(1, 0), r(0, 0)));
      RCLCPP_WARN(
        this->get_logger(),
        "Trusted-pose recovery consensus accepted. support=%zu pose=(%.3f,%.3f,%.3f,%.3f) "
        "ratio=%.3f mean_err=%.4f",
        consensus_support, t.x(), t.y(), t.z(), yaw, inlier_ratio, mean_error);
    }
  }

  if (!accepted) {
    const Eigen::Vector3d candidate_translation = result.T_target_source.translation();
    const Eigen::Matrix3d candidate_rotation = result.T_target_source.rotation();
    const double candidate_yaw = normalizeAngle(
      std::atan2(candidate_rotation(1, 0), candidate_rotation(0, 0)));
    consecutive_reject_count_++;
    coarse_relocalization_pending_ =
      coarse_relocalization_enabled_ && !trusted_initial_pose_received_ &&
      (coarse_found ||
      consecutive_reject_count_ >= std::max(1, coarse_trigger_failed_attempts_));
    RCLCPP_WARN(
      this->get_logger(),
      "Reject relocalization update. converged=%d iterations=%zu source=%zu inliers=%zu ratio=%.3f "
      "mean_err=%.4f delta_trans=%.3f delta_yaw=%.3f candidate=(%.3f,%.3f,%.3f,%.3f)",
      static_cast<int>(result.converged), result.iterations, source_->size(), result.num_inliers,
      inlier_ratio, mean_error, delta_translation, delta_yaw, candidate_translation.x(),
      candidate_translation.y(), candidate_translation.z(), candidate_yaw);
  }

  accumulated_cloud_->clear();
  has_accumulation_start_time_ = false;
}

void SmallGicpRelocalizationNode::acceptRegistrationPose(
  const Eigen::Isometry3d & pose, double alpha)
{
  alpha = std::clamp(alpha, 0.0, 1.0);
  const Eigen::Vector3d smoothed_translation =
    alpha * pose.translation() + (1.0 - alpha) * previous_result_t_.translation();
  const Eigen::Quaterniond prev_q(previous_result_t_.rotation());
  const Eigen::Quaterniond new_q(pose.rotation());
  const Eigen::Quaterniond smoothed_q = prev_q.slerp(alpha, new_q);

  result_t_ = Eigen::Isometry3d::Identity();
  result_t_.translation() = smoothed_translation;
  result_t_.linear() = smoothed_q.normalized().toRotationMatrix();
  previous_result_t_ = result_t_;
  registration_accepted_ = true;
  last_accepted_registration_time_ = this->now();
  consecutive_reject_count_ = 0;
  coarse_relocalization_pending_ = false;
  relocalization_candidates_.clear();
}

void SmallGicpRelocalizationNode::applyMapFrameCorrection(
  const Eigen::Isometry3d & desired_map_to_odom)
{
  map_frame_correction_ = desired_map_to_odom * result_t_.inverse();
  has_pending_map_to_odom_seed_ = false;
  map_frame_calibrated_ = true;
  last_map_alignment_time_ = this->now();

  const Eigen::Vector3d translation = map_frame_correction_.translation();
  const Eigen::Matrix3d rotation = map_frame_correction_.rotation();
  const double yaw = normalizeAngle(std::atan2(rotation(1, 0), rotation(0, 0)));
  constexpr double kRadiansToDegrees = 57.29577951308232;
  RCLCPP_WARN(
    this->get_logger(),
    "PGM-to-PCD frame calibration applied: translation=(%.3f,%.3f,%.3f), yaw=%.3f rad "
    "(%.2f deg).",
    translation.x(), translation.y(), translation.z(), yaw, yaw * kRadiansToDegrees);
}

bool SmallGicpRelocalizationNode::updateRelocalizationConsensus(
  const Eigen::Isometry3d & candidate, Eigen::Isometry3d * consensus_pose,
  size_t * consensus_support, size_t required_override)
{
  if (!consensus_pose || !consensus_support || !candidate.matrix().allFinite()) {
    return false;
  }

  relocalization_candidates_.push_back(candidate);
  const size_t max_window = static_cast<size_t>(std::max(1, relocalization_consensus_window_));
  while (relocalization_candidates_.size() > max_window) {
    relocalization_candidates_.pop_front();
  }

  const double translation_tolerance = std::max(
    0.01, static_cast<double>(relocalization_consensus_translation_tolerance_));
  const double yaw_tolerance = std::max(
    0.01, static_cast<double>(relocalization_consensus_yaw_tolerance_rad_));
  std::vector<size_t> best_cluster;

  for (size_t i = 0; i < relocalization_candidates_.size(); ++i) {
    std::vector<size_t> cluster;
    for (size_t j = 0; j < relocalization_candidates_.size(); ++j) {
      const Eigen::Isometry3d delta =
        relocalization_candidates_[i].inverse() * relocalization_candidates_[j];
      const Eigen::Matrix3d delta_r = delta.rotation();
      const double delta_yaw = std::abs(normalizeAngle(
        std::atan2(delta_r(1, 0), delta_r(0, 0))));
      if (delta.translation().norm() <= translation_tolerance && delta_yaw <= yaw_tolerance) {
        cluster.push_back(j);
      }
    }
    if (cluster.size() > best_cluster.size()) {
      best_cluster = std::move(cluster);
    }
  }

  *consensus_support = best_cluster.size();
  const size_t required = required_override > 0 ?
    required_override : static_cast<size_t>(std::max(2, relocalization_consensus_required_));
  if (best_cluster.size() < required) {
    RCLCPP_INFO(
      this->get_logger(), "Relocalization consensus pending. support=%zu/%zu window=%zu",
      best_cluster.size(), required, relocalization_candidates_.size());
    return false;
  }

  Eigen::Vector3d translation_sum = Eigen::Vector3d::Zero();
  Eigen::Vector4d quaternion_sum = Eigen::Vector4d::Zero();
  const Eigen::Quaterniond reference(
    relocalization_candidates_[best_cluster.front()].rotation());
  for (const size_t index : best_cluster) {
    translation_sum += relocalization_candidates_[index].translation();
    Eigen::Quaterniond q(relocalization_candidates_[index].rotation());
    if (reference.dot(q) < 0.0) {
      q.coeffs() *= -1.0;
    }
    quaternion_sum += q.coeffs();
  }

  consensus_pose->setIdentity();
  consensus_pose->translation() = translation_sum / static_cast<double>(best_cluster.size());
  Eigen::Quaterniond average_q;
  average_q.coeffs() = quaternion_sum.normalized();
  consensus_pose->linear() = average_q.normalized().toRotationMatrix();
  return true;
}

bool SmallGicpRelocalizationNode::findCoarseInitialGuess(Eigen::Isometry3d * coarse_guess)
{
  if (!coarse_guess || !coarse_target_ || !coarse_target_tree_) {
    return false;
  }

  auto coarse_source = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *accumulated_cloud_, coarse_leaf_size_);
  if (!coarse_source || coarse_source->size() < static_cast<size_t>(std::max(1, min_source_points_))) {
    return false;
  }

  small_gicp::estimate_covariances_omp(*coarse_source, num_neighbors_, num_threads_);
  coarse_register_->reduction.num_threads = num_threads_;
  coarse_register_->rejector.max_dist_sq = coarse_max_dist_sq_;
  coarse_register_->optimizer.max_iterations = std::max(1, coarse_max_iterations_);

  std::vector<std::pair<double, Eigen::Isometry3d>> candidates;
  const bool using_global_search =
    global_search_enabled_ && coarse_relocalization_pending_ &&
    !trusted_initial_pose_received_ &&
    findGlobalInitialGuesses(*coarse_source, &candidates);
  if (!using_global_search) {
    const double xy_resolution = std::max(0.01, static_cast<double>(coarse_xy_resolution_));
    const double xy_radius = std::max(0.0, static_cast<double>(coarse_xy_search_radius_));
    const int xy_steps = static_cast<int>(std::ceil(xy_radius / xy_resolution));
    const double yaw_resolution = std::max(
      0.01, static_cast<double>(coarse_yaw_resolution_rad_));
    const double yaw_range = std::max(
      0.0, static_cast<double>(coarse_yaw_search_range_rad_));
    const int yaw_steps = static_cast<int>(std::ceil(yaw_range / yaw_resolution));

    for (int ix = -xy_steps; ix <= xy_steps; ++ix) {
      const double dx = ix * xy_resolution;
      for (int iy = -xy_steps; iy <= xy_steps; ++iy) {
        const double dy = iy * xy_resolution;
        if (std::hypot(dx, dy) > xy_radius + 1e-6) {
          continue;
        }
        for (int iyaw = -yaw_steps; iyaw <= yaw_steps; ++iyaw) {
          const double yaw = iyaw * yaw_resolution;
          if (std::abs(yaw) > yaw_range + 1e-6) {
            continue;
          }
          Eigen::Isometry3d candidate = previous_result_t_;
          candidate.translation().x() += dx;
          candidate.translation().y() += dy;
          candidate.linear() =
            Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
            candidate.rotation();
          const double distance_score =
            dx * dx + dy * dy + 0.25 * std::abs(normalizeAngle(yaw));
          candidates.emplace_back(distance_score, candidate);
        }
      }
    }

    std::sort(
      candidates.begin(), candidates.end(),
      [](const auto & lhs, const auto & rhs) { return lhs.first < rhs.first; });
    if (
      coarse_max_candidates_ > 0 &&
      candidates.size() > static_cast<size_t>(coarse_max_candidates_))
    {
      candidates.resize(static_cast<size_t>(coarse_max_candidates_));
    }
  }

  struct ScoredHypothesis
  {
    double score;
    double ratio;
    double mean_error;
    Eigen::Isometry3d pose;
  };
  std::vector<ScoredHypothesis> scored_hypotheses;

  for (const auto & candidate : candidates) {
    const auto result =
      coarse_register_->align(*coarse_target_, *coarse_source, *coarse_target_tree_, candidate.second);
    if (!result.converged) {
      continue;
    }

    const double inlier_ratio =
      static_cast<double>(result.num_inliers) / static_cast<double>(coarse_source->size());
    const double mean_error =
      result.error / static_cast<double>(std::max<size_t>(1, result.num_inliers));
    if (
      inlier_ratio < static_cast<double>(coarse_min_inlier_ratio_) ||
      mean_error > static_cast<double>(coarse_max_mean_error_))
    {
      continue;
    }

    const Eigen::Isometry3d pose_delta = previous_result_t_.inverse() * result.T_target_source;
    const double delta_translation = pose_delta.translation().norm();
    const Eigen::Matrix3d delta_r = pose_delta.rotation();
    const double raw_delta_yaw = std::atan2(delta_r(1, 0), delta_r(0, 0));
    const double delta_yaw = std::abs(normalizeAngle(raw_delta_yaw));
    const double score = using_global_search ?
      mean_error / std::max(0.05, inlier_ratio) -
      static_cast<double>(global_search_score_weight_) * candidate.first :
      mean_error / std::max(0.05, inlier_ratio) +
      static_cast<double>(coarse_score_translation_penalty_) * delta_translation +
      static_cast<double>(coarse_score_yaw_penalty_) * delta_yaw;
    scored_hypotheses.push_back({score, inlier_ratio, mean_error, result.T_target_source});
  }

  if (scored_hypotheses.empty()) {
    return false;
  }

  std::sort(
    scored_hypotheses.begin(), scored_hypotheses.end(),
    [](const ScoredHypothesis & lhs, const ScoredHypothesis & rhs) {
      return lhs.score < rhs.score;
    });

  coarse_hypotheses_.clear();
  const size_t max_hypotheses = static_cast<size_t>(std::max(1, coarse_refine_hypotheses_));
  const double translation_separation = std::max(
    0.01, static_cast<double>(coarse_hypothesis_translation_separation_));
  const double yaw_separation = std::max(
    0.01, static_cast<double>(coarse_hypothesis_yaw_separation_rad_));
  for (const auto & scored : scored_hypotheses) {
    bool distinct = true;
    for (const auto & selected : coarse_hypotheses_) {
      const Eigen::Isometry3d delta = selected.inverse() * scored.pose;
      const Eigen::Matrix3d delta_r = delta.rotation();
      const double delta_yaw = std::abs(normalizeAngle(
        std::atan2(delta_r(1, 0), delta_r(0, 0))));
      if (delta.translation().norm() < translation_separation && delta_yaw < yaw_separation) {
        distinct = false;
        break;
      }
    }
    if (distinct) {
      coarse_hypotheses_.push_back(scored.pose);
      if (coarse_hypotheses_.size() >= max_hypotheses) {
        break;
      }
    }
  }

  *coarse_guess = coarse_hypotheses_.front();
  const auto & best = scored_hypotheses.front();
  RCLCPP_INFO(
    this->get_logger(),
    "Coarse relocalization seeds accepted. candidates=%zu hypotheses=%zu ratio=%.3f "
    "mean_err=%.4f",
    candidates.size(), coarse_hypotheses_.size(), best.ratio, best.mean_error);
  return true;
}

bool SmallGicpRelocalizationNode::isRegistrationAcceptable(
  const small_gicp::RegistrationResult & result, size_t source_size, double * inlier_ratio,
  double * mean_error, double * delta_translation, double * delta_yaw)
{
  return isRegistrationAcceptable(
    result, source_size, inlier_ratio, mean_error, delta_translation, delta_yaw,
    max_translation_jump_, max_yaw_jump_rad_);
}

bool SmallGicpRelocalizationNode::isRegistrationAcceptable(
  const small_gicp::RegistrationResult & result, size_t source_size, double * inlier_ratio,
  double * mean_error, double * delta_translation, double * delta_yaw,
  double max_translation_jump, double max_yaw_jump)
{
  if (source_size < static_cast<size_t>(std::max(1, min_source_points_))) {
    return false;
  }

  *inlier_ratio =
    source_size > 0 ? static_cast<double>(result.num_inliers) / static_cast<double>(source_size)
                    : 0.0;
  const double inlier_count = std::max<size_t>(1, result.num_inliers);
  *mean_error = result.error / inlier_count;

  const Eigen::Isometry3d delta = previous_result_t_.inverse() * result.T_target_source;
  *delta_translation = delta.translation().norm();
  const Eigen::Matrix3d delta_r = delta.rotation();
  const double raw_yaw = std::atan2(delta_r(1, 0), delta_r(0, 0));
  *delta_yaw = std::abs(normalizeAngle(raw_yaw));

  if (
    !result.T_target_source.matrix().allFinite() || !std::isfinite(*inlier_ratio) ||
    !std::isfinite(*mean_error) || !std::isfinite(*delta_translation) ||
    !std::isfinite(*delta_yaw) || !result.converged)
  {
    return false;
  }

  if (*inlier_ratio < static_cast<double>(min_inlier_ratio_)) {
    return false;
  }
  if (*mean_error > static_cast<double>(max_mean_error_)) {
    return false;
  }
  if (*delta_translation > max_translation_jump) {
    return false;
  }
  if (*delta_yaw > max_yaw_jump) {
    return false;
  }

  return true;
}

double SmallGicpRelocalizationNode::normalizeAngle(double angle)
{
  constexpr double kPi = 3.14159265358979323846;
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

void SmallGicpRelocalizationNode::publishTransform()
{
  if (result_t_.matrix().isZero()) {
    return;
  }

  geometry_msgs::msg::TransformStamped transform_stamped;
  rclcpp::Time transform_stamp = this->now();
  const double tf_future_offset = std::max(0.0, static_cast<double>(tf_future_offset_sec_));
  if (tf_future_offset > 0.0) {
    transform_stamp = transform_stamp + rclcpp::Duration::from_seconds(tf_future_offset);
  }
  transform_stamped.header.stamp = transform_stamp;
  transform_stamped.header.frame_id = map_frame_;
  transform_stamped.child_frame_id = odom_frame_;

  const Eigen::Isometry3d published_result = map_frame_correction_ * result_t_;
  const Eigen::Vector3d translation = published_result.translation();
  const Eigen::Quaterniond rotation(published_result.rotation());

  transform_stamped.transform.translation.x = translation.x();
  transform_stamped.transform.translation.y = translation.y();
  transform_stamped.transform.translation.z = translation.z();
  transform_stamped.transform.rotation.x = rotation.x();
  transform_stamped.transform.rotation.y = rotation.y();
  transform_stamped.transform.rotation.z = rotation.z();
  transform_stamped.transform.rotation.w = rotation.w();

  tf_broadcaster_->sendTransform(transform_stamped);

  // Initial identity TF is also broadcast before registration. Automatic
  // missions must wait for accepted registration and a settled map-frame seed.
  std_msgs::msg::Bool ready;
  if (registration_accepted_ && map_frame_calibrated_) {
    const auto now = this->now();
    // Do not refresh a stale correction's readiness merely by publishing TF.
    const double scan_age = (now - last_scan_time_).seconds();
    const double registration_age = (now - last_accepted_registration_time_).seconds();
    ready.data = (now - last_map_alignment_time_).seconds() >= 1.0 &&
      scan_age >= -0.2 && scan_age <= 2.0 &&
      registration_age >= 0.0 && registration_age <= 10.0;
  }
  localization_ready_pub_->publish(ready);
}

void SmallGicpRelocalizationNode::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  RCLCPP_INFO(
    this->get_logger(), "Received initial pose: [x: %f, y: %f, z: %f]", msg->pose.pose.position.x,
    msg->pose.pose.position.y, msg->pose.pose.position.z);

  Eigen::Isometry3d map_to_robot_base = Eigen::Isometry3d::Identity();
  map_to_robot_base.translation() << msg->pose.pose.position.x, msg->pose.pose.position.y,
    msg->pose.pose.position.z;
  map_to_robot_base.linear() = Eigen::Quaterniond(
                                 msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                 msg->pose.pose.orientation.y, msg->pose.pose.orientation.z)
                                 .toRotationMatrix();

  try {
    auto transform =
      tf_buffer_->lookupTransform(robot_base_frame_, current_scan_frame_id_, tf2::TimePointZero);
    Eigen::Isometry3d robot_base_to_odom = tf2::transformToEigen(transform.transform);
    Eigen::Isometry3d map_to_odom = map_to_robot_base * robot_base_to_odom;

    pending_map_to_odom_seed_ = map_to_odom;
    has_pending_map_to_odom_seed_ = true;
    if (trusted_initial_pose_received_) {
      applyMapFrameCorrection(map_to_odom);
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "PGM map-frame seed stored; waiting for PCD registration consensus.");
    }
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(), "Could not transform initial pose from %s to %s: %s",
      robot_base_frame_.c_str(), current_scan_frame_id_.c_str(), ex.what());
  }
}

}  // namespace small_gicp_relocalization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(small_gicp_relocalization::SmallGicpRelocalizationNode)

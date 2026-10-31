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

#ifndef SMALL_GICP_RELOCALIZATION__SMALL_GICP_RELOCALIZATION_HPP_
#define SMALL_GICP_RELOCALIZATION__SMALL_GICP_RELOCALIZATION_HPP_

#include <array>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "Eigen/Geometry"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "pcl/io/pcd_io.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "small_gicp/ann/kdtree_omp.hpp"
#include "small_gicp/factors/robust_kernel.hpp"
#include "small_gicp/factors/gicp_factor.hpp"
#include "small_gicp/pcl/pcl_point.hpp"
#include "small_gicp/registration/reduction_omp.hpp"
#include "small_gicp/registration/registration.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace small_gicp_relocalization
{

class SmallGicpRelocalizationNode : public rclcpp::Node
{
public:
  explicit SmallGicpRelocalizationNode(const rclcpp::NodeOptions & options);

private:
  void registeredPcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void loadGlobalMap(const std::string & file_name);
  void performRegistration();
  void publishTransform();
  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  bool findCoarseInitialGuess(Eigen::Isometry3d * coarse_guess);
  void buildGlobalSearchGrid();
  bool findGlobalInitialGuesses(
    const pcl::PointCloud<pcl::PointCovariance> & source,
    std::vector<std::pair<double, Eigen::Isometry3d>> * candidates) const;
  double scoreGlobalCandidate(
    const pcl::PointCloud<pcl::PointCovariance> & source,
    const Eigen::Isometry3d & candidate) const;
  size_t globalGridOffset(int x, int y, int z) const;
  bool updateRelocalizationConsensus(
    const Eigen::Isometry3d & candidate, Eigen::Isometry3d * consensus_pose,
    size_t * consensus_support, size_t required_override = 0);
  void acceptRegistrationPose(const Eigen::Isometry3d & pose, double alpha);
  void applyMapFrameCorrection(const Eigen::Isometry3d & desired_map_to_odom);
  bool isRegistrationAcceptable(
    const small_gicp::RegistrationResult & result, size_t source_size, double * inlier_ratio,
    double * mean_error, double * delta_translation, double * delta_yaw);
  bool isRegistrationAcceptable(
    const small_gicp::RegistrationResult & result, size_t source_size, double * inlier_ratio,
    double * mean_error, double * delta_translation, double * delta_yaw,
    double max_translation_jump, double max_yaw_jump);
  static double normalizeAngle(double angle);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;

  int num_threads_;
  int num_neighbors_;
  float global_leaf_size_;
  float registered_leaf_size_;
  float max_dist_sq_;
  float min_inlier_ratio_;
  float max_mean_error_;
  float max_translation_jump_;
  float max_yaw_jump_rad_;
  int min_source_points_;
  float result_smoothing_alpha_;
  int max_accumulated_points_;
  float max_accumulation_time_sec_;
  bool coarse_relocalization_enabled_;
  int coarse_trigger_failed_attempts_;
  int coarse_max_candidates_;
  int coarse_max_iterations_;
  float coarse_leaf_size_;
  float coarse_xy_search_radius_;
  float coarse_xy_resolution_;
  float coarse_yaw_search_range_rad_;
  float coarse_yaw_resolution_rad_;
  float coarse_max_dist_sq_;
  float coarse_min_inlier_ratio_;
  float coarse_max_mean_error_;
  float coarse_max_translation_jump_;
  float coarse_max_yaw_jump_rad_;
  bool coarse_direct_accept_enabled_;
  float coarse_direct_max_translation_jump_;
  float coarse_direct_max_yaw_jump_rad_;
  float coarse_direct_accept_cooldown_sec_;
  float coarse_score_translation_penalty_;
  float coarse_score_yaw_penalty_;
  float fine_after_coarse_max_dist_sq_;
  int fine_after_coarse_max_iterations_;
  int fine_max_iterations_;
  float fine_translation_epsilon_;
  float fine_rotation_epsilon_rad_;
  float coarse_translation_epsilon_;
  float coarse_rotation_epsilon_rad_;
  int coarse_refine_hypotheses_;
  float coarse_hypothesis_translation_separation_;
  float coarse_hypothesis_yaw_separation_rad_;
  bool global_search_enabled_;
  float global_search_voxel_resolution_;
  float global_search_xy_resolution_;
  float global_search_yaw_resolution_rad_;
  float global_search_z_range_;
  float global_search_z_resolution_;
  int global_search_top_candidates_;
  int global_search_max_source_points_;
  int global_search_min_source_points_;
  int global_search_dilation_voxels_;
  float global_search_min_occupancy_ratio_;
  float global_search_score_weight_;
  int relocalization_consensus_required_;
  int relocalization_consensus_window_;
  float relocalization_consensus_translation_tolerance_;
  float relocalization_consensus_yaw_tolerance_rad_;
  float relocalization_consensus_min_inlier_ratio_;
  float relocalization_consensus_max_mean_error_;
  bool trusted_recovery_enabled_;
  int trusted_recovery_trigger_rejects_;
  int trusted_recovery_consensus_required_;
  float trusted_recovery_max_translation_jump_;
  float trusted_recovery_max_yaw_jump_rad_;
  float trusted_recovery_min_inlier_ratio_;
  float trusted_recovery_max_mean_error_;
  float trusted_recovery_alpha_;
  float tf_future_offset_sec_;
  std::vector<double> init_pose_;

  std::string map_frame_;
  std::string odom_frame_;
  std::string prior_pcd_file_;
  std::string base_frame_;
  std::string robot_base_frame_;
  std::string lidar_frame_;
  std::string current_scan_frame_id_;
  std::string input_cloud_topic_;
  rclcpp::Time last_scan_time_;
  rclcpp::Time accumulation_start_time_;
  bool has_accumulation_start_time_;
  bool coarse_relocalization_pending_;
  bool trusted_initial_pose_received_;
  bool has_pending_map_to_odom_seed_;
  int consecutive_reject_count_;
  rclcpp::Time last_coarse_direct_accept_time_;
  std::deque<Eigen::Isometry3d> relocalization_candidates_;
  std::vector<Eigen::Isometry3d> coarse_hypotheses_;
  Eigen::Vector3d global_map_min_;
  Eigen::Vector3d global_map_max_;
  Eigen::Vector3d global_grid_min_;
  std::array<int, 3> global_grid_dims_;
  std::vector<std::uint8_t> global_occupancy_grid_;
  bool global_search_grid_ready_;
  Eigen::Isometry3d map_frame_correction_;
  Eigen::Isometry3d pending_map_to_odom_seed_;
  Eigen::Isometry3d result_t_;
  Eigen::Isometry3d previous_result_t_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr registered_scan_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud_;
  pcl::PointCloud<pcl::PointCovariance>::Ptr target_;
  pcl::PointCloud<pcl::PointCovariance>::Ptr source_;
  pcl::PointCloud<pcl::PointCovariance>::Ptr coarse_target_;

  std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> target_tree_;
  std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> source_tree_;
  std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> coarse_target_tree_;
  std::shared_ptr<
    small_gicp::Registration<
      small_gicp::RobustFactor<small_gicp::Huber, small_gicp::GICPFactor>,
      small_gicp::ParallelReductionOMP>>
    register_;
  std::shared_ptr<
    small_gicp::Registration<
      small_gicp::RobustFactor<small_gicp::Huber, small_gicp::GICPFactor>,
      small_gicp::ParallelReductionOMP>>
    coarse_register_;

  rclcpp::TimerBase::SharedPtr transform_timer_;
  rclcpp::TimerBase::SharedPtr register_timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace small_gicp_relocalization

#endif  // SMALL_GICP_RELOCALIZATION__SMALL_GICP_RELOCALIZATION_HPP_

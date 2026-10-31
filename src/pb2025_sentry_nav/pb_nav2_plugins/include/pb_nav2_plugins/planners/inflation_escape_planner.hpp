// Copyright 2026 RM071 project contributors
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

#ifndef PB_NAV2_PLUGINS__PLANNERS__INFLATION_ESCAPE_PLANNER_HPP_
#define PB_NAV2_PLUGINS__PLANNERS__INFLATION_ESCAPE_PLANNER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_theta_star_planner/theta_star_planner.hpp"
#include "nav_msgs/msg/path.hpp"

namespace pb_nav2_plugins
{

class InflationEscapePlanner : public nav2_core::GlobalPlanner
{
public:
  InflationEscapePlanner() = default;
  ~InflationEscapePlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  struct Cell
  {
    unsigned int x;
    unsigned int y;
  };

  bool isSafe(const Cell & cell, unsigned char max_cost) const;
  bool isTraversable(const Cell & cell) const;

  std::vector<Cell> findShortestExit(
    const Cell & start, unsigned char safe_cost, double max_radius_m) const;

  std::vector<Cell> findReachableSafeProjection(
    const Cell & reachable_start, const Cell & requested_goal, unsigned char safe_cost,
    double max_radius_m) const;

  geometry_msgs::msg::PoseStamped cellPose(
    const Cell & cell, const geometry_msgs::msg::PoseStamped & reference) const;

  nav_msgs::msg::Path prependEscapePath(
    const geometry_msgs::msg::PoseStamped & original_start, const std::vector<Cell> & escape_cells,
    nav_msgs::msg::Path main_path) const;

  void cacheValidPath(
    const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & requested_goal);

  bool cachedPathIsCurrentlyTraversable(const nav_msgs::msg::Path & path) const;

  nav_msgs::msg::Path reuseCachedPath(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & requested_goal) const;

  std::unique_ptr<nav2_theta_star_planner::ThetaStarPlanner> theta_star_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("InflationEscapePlanner")};
  std::string name_;

  int safe_cost_{98};
  double escape_search_radius_m_{2.0};
  double goal_projection_radius_m_{1.5};
  double escape_cost_weight_{8.0};
  bool project_unsafe_goal_{true};
  bool reuse_last_valid_path_{true};
  double cached_path_goal_tolerance_m_{0.35};
  double cached_path_max_start_distance_m_{1.0};
  int cached_path_max_cost_{252};

  mutable std::mutex cached_path_mutex_;
  nav_msgs::msg::Path cached_path_;
  geometry_msgs::msg::PoseStamped cached_requested_goal_;
  bool has_cached_path_{false};
};

}  // namespace pb_nav2_plugins

#endif  // PB_NAV2_PLUGINS__PLANNERS__INFLATION_ESCAPE_PLANNER_HPP_

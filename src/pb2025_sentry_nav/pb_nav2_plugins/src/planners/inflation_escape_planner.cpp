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

#include "pb_nav2_plugins/planners/inflation_escape_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <exception>
#include <limits>
#include <mutex>
#include <queue>
#include <utility>

#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace pb_nav2_plugins
{

namespace
{

struct QueueEntry
{
  double distance;
  unsigned int index;

  bool operator>(const QueueEntry & other) const { return distance > other.distance; }
};

}  // namespace

void InflationEscapePlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw nav2_core::PlannerException("Unable to lock planner lifecycle node");
  }

  node_ = parent;
  name_ = std::move(name);
  costmap_ros_ = std::move(costmap_ros);
  costmap_ = costmap_ros_->getCostmap();
  clock_ = node->get_clock();
  logger_ = node->get_logger();

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".safe_cost", rclcpp::ParameterValue(98));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".escape_search_radius", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".goal_projection_radius", rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".escape_cost_weight", rclcpp::ParameterValue(8.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".project_unsafe_goal", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".reuse_last_valid_path", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".cached_path_goal_tolerance", rclcpp::ParameterValue(0.35));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".cached_path_max_start_distance", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".cached_path_max_cost", rclcpp::ParameterValue(252));

  node->get_parameter(name_ + ".safe_cost", safe_cost_);
  node->get_parameter(name_ + ".escape_search_radius", escape_search_radius_m_);
  node->get_parameter(name_ + ".goal_projection_radius", goal_projection_radius_m_);
  node->get_parameter(name_ + ".escape_cost_weight", escape_cost_weight_);
  node->get_parameter(name_ + ".project_unsafe_goal", project_unsafe_goal_);
  node->get_parameter(name_ + ".reuse_last_valid_path", reuse_last_valid_path_);
  node->get_parameter(name_ + ".cached_path_goal_tolerance", cached_path_goal_tolerance_m_);
  node->get_parameter(name_ + ".cached_path_max_start_distance", cached_path_max_start_distance_m_);
  node->get_parameter(name_ + ".cached_path_max_cost", cached_path_max_cost_);

  safe_cost_ =
    std::clamp(safe_cost_, 0, static_cast<int>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) - 1);
  escape_search_radius_m_ = std::max(0.1, escape_search_radius_m_);
  goal_projection_radius_m_ = std::max(0.1, goal_projection_radius_m_);
  escape_cost_weight_ = std::max(0.0, escape_cost_weight_);
  cached_path_goal_tolerance_m_ = std::max(0.0, cached_path_goal_tolerance_m_);
  cached_path_max_start_distance_m_ = std::max(0.1, cached_path_max_start_distance_m_);
  cached_path_max_cost_ = std::clamp(
    cached_path_max_cost_, 0, static_cast<int>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) - 1);

  theta_star_ = std::make_unique<nav2_theta_star_planner::ThetaStarPlanner>();
  theta_star_->configure(parent, name_, std::move(tf), costmap_ros_);

  RCLCPP_INFO(
    logger_,
    "Inflation escape planner configured: safe_cost=%d, escape_radius=%.2f m, "
    "goal_projection_radius=%.2f m, cost_weight=%.2f, project_goal=%s, "
    "reuse_path=%s (goal_tol=%.2f m, start_tol=%.2f m, max_cost=%d)",
    safe_cost_, escape_search_radius_m_, goal_projection_radius_m_, escape_cost_weight_,
    project_unsafe_goal_ ? "true" : "false", reuse_last_valid_path_ ? "true" : "false",
    cached_path_goal_tolerance_m_, cached_path_max_start_distance_m_, cached_path_max_cost_);
}

void InflationEscapePlanner::cleanup()
{
  {
    std::lock_guard<std::mutex> lock(cached_path_mutex_);
    cached_path_ = nav_msgs::msg::Path();
    cached_requested_goal_ = geometry_msgs::msg::PoseStamped();
    has_cached_path_ = false;
  }
  if (theta_star_) {
    theta_star_->cleanup();
    theta_star_.reset();
  }
  costmap_ = nullptr;
  costmap_ros_.reset();
}

void InflationEscapePlanner::activate()
{
  if (theta_star_) {
    theta_star_->activate();
  }
}

void InflationEscapePlanner::deactivate()
{
  if (theta_star_) {
    theta_star_->deactivate();
  }
}

bool InflationEscapePlanner::isSafe(const Cell & cell, unsigned char max_cost) const
{
  const auto cost = costmap_->getCost(cell.x, cell.y);
  return cost != nav2_costmap_2d::NO_INFORMATION && cost <= max_cost;
}

bool InflationEscapePlanner::isTraversable(const Cell & cell) const
{
  const auto cost = costmap_->getCost(cell.x, cell.y);
  return cost != nav2_costmap_2d::NO_INFORMATION && cost < nav2_costmap_2d::LETHAL_OBSTACLE;
}

std::vector<InflationEscapePlanner::Cell> InflationEscapePlanner::findShortestExit(
  const Cell & start, unsigned char safe_cost, double max_radius_m) const
{
  if (isSafe(start, safe_cost)) {
    return {start};
  }

  const unsigned int width = costmap_->getSizeInCellsX();
  const unsigned int height = costmap_->getSizeInCellsY();
  const unsigned int cell_count = width * height;
  const double resolution = costmap_->getResolution();
  const double max_radius_cells = max_radius_m / resolution;

  const auto index_of = [width](unsigned int x, unsigned int y) { return y * width + x; };
  const auto cell_of = [width](unsigned int index) { return Cell{index % width, index / width}; };

  const unsigned int start_index = index_of(start.x, start.y);
  std::vector<double> distances(cell_count, std::numeric_limits<double>::infinity());
  std::vector<int64_t> parents(cell_count, -1);
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> open;

  distances[start_index] = 0.0;
  parents[start_index] = static_cast<int64_t>(start_index);
  open.push({0.0, start_index});

  static constexpr int K_DX[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  static constexpr int K_DY[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  static constexpr double K_STEP[8] = {1.0,           1.0,           1.0,           1.0,
                                       1.41421356237, 1.41421356237, 1.41421356237, 1.41421356237};

  int64_t safe_index = -1;
  int64_t lowest_cost_index = static_cast<int64_t>(start_index);
  unsigned char lowest_cost = costmap_->getCost(start.x, start.y);
  while (!open.empty()) {
    const QueueEntry current = open.top();
    open.pop();
    if (current.distance > distances[current.index]) {
      continue;
    }

    const Cell cell = cell_of(current.index);
    const unsigned char cell_cost = costmap_->getCost(cell.x, cell.y);
    if (cell_cost < lowest_cost) {
      lowest_cost = cell_cost;
      lowest_cost_index = static_cast<int64_t>(current.index);
    }
    if (current.index != start_index && isSafe(cell, safe_cost)) {
      safe_index = static_cast<int64_t>(current.index);
      break;
    }

    for (size_t i = 0; i < 8; ++i) {
      const int nx = static_cast<int>(cell.x) + K_DX[i];
      const int ny = static_cast<int>(cell.y) + K_DY[i];
      if (nx < 0 || ny < 0 || nx >= static_cast<int>(width) || ny >= static_cast<int>(height)) {
        continue;
      }

      const double dx = static_cast<double>(nx) - static_cast<double>(start.x);
      const double dy = static_cast<double>(ny) - static_cast<double>(start.y);
      if (std::hypot(dx, dy) > max_radius_cells) {
        continue;
      }

      const Cell next{static_cast<unsigned int>(nx), static_cast<unsigned int>(ny)};
      if (!isTraversable(next)) {
        continue;
      }
      if (K_DX[i] != 0 && K_DY[i] != 0) {
        const Cell horizontal{
          static_cast<unsigned int>(static_cast<int>(cell.x) + K_DX[i]), cell.y};
        const Cell vertical{cell.x, static_cast<unsigned int>(static_cast<int>(cell.y) + K_DY[i])};
        if (!isTraversable(horizontal) || !isTraversable(vertical)) {
          continue;
        }
      }

      const unsigned int next_index = index_of(next.x, next.y);
      const double max_inflation_cost =
        static_cast<double>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
      const double current_cost =
        std::min(static_cast<double>(costmap_->getCost(cell.x, cell.y)), max_inflation_cost);
      const double next_cost =
        std::min(static_cast<double>(costmap_->getCost(next.x, next.y)), max_inflation_cost);
      const double normalized_cost = next_cost / max_inflation_cost;
      const double uphill_cost = std::max(0.0, next_cost - current_cost) / max_inflation_cost;
      const double edge_cost =
        K_STEP[i] * (1.0 + escape_cost_weight_ * normalized_cost * normalized_cost +
                     2.0 * escape_cost_weight_ * uphill_cost);
      const double next_distance = current.distance + edge_cost;
      if (next_distance < distances[next_index]) {
        distances[next_index] = next_distance;
        parents[next_index] = static_cast<int64_t>(current.index);
        open.push({next_distance, next_index});
      }
    }
  }

  if (safe_index < 0) {
    // A narrow but traversable corridor may contain no cell below the absolute
    // safe-cost threshold. In that case move toward the locally lowest cost
    // (largest clearance) cell instead of rejecting the corridor.
    if (lowest_cost_index == static_cast<int64_t>(start_index) && !isTraversable(start)) {
      return {};
    }
    safe_index = lowest_cost_index;
  }

  std::vector<Cell> path;
  for (int64_t index = safe_index;; index = parents[static_cast<size_t>(index)]) {
    path.push_back(cell_of(static_cast<unsigned int>(index)));
    if (index == static_cast<int64_t>(start_index)) {
      break;
    }
    if (index < 0 || parents[static_cast<size_t>(index)] < 0) {
      return {};
    }
  }
  std::reverse(path.begin(), path.end());
  return path;
}

std::vector<InflationEscapePlanner::Cell> InflationEscapePlanner::findReachableSafeProjection(
  const Cell & reachable_start, const Cell & requested_goal, unsigned char safe_cost,
  double max_radius_m) const
{
  const unsigned int width = costmap_->getSizeInCellsX();
  const unsigned int height = costmap_->getSizeInCellsY();
  const unsigned int cell_count = width * height;
  const double max_radius_cells = max_radius_m / costmap_->getResolution();

  const auto index_of = [width](unsigned int x, unsigned int y) { return y * width + x; };
  const auto cell_of = [width](unsigned int index) { return Cell{index % width, index / width}; };
  const auto is_collision_free = [this](const Cell & cell) {
    const auto cost = costmap_->getCost(cell.x, cell.y);
    return cost != nav2_costmap_2d::NO_INFORMATION &&
           cost < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
  };

  if (!is_collision_free(reachable_start)) {
    return {};
  }

  std::vector<std::uint8_t> reachable(cell_count, 0);
  std::queue<unsigned int> open;
  const unsigned int start_index = index_of(reachable_start.x, reachable_start.y);
  reachable[start_index] = 1;
  open.push(start_index);

  static constexpr int K_DX[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  static constexpr int K_DY[8] = {0, 0, 1, -1, 1, -1, 1, -1};

  while (!open.empty()) {
    const Cell cell = cell_of(open.front());
    open.pop();
    for (size_t i = 0; i < 8; ++i) {
      const int nx = static_cast<int>(cell.x) + K_DX[i];
      const int ny = static_cast<int>(cell.y) + K_DY[i];
      if (nx < 0 || ny < 0 || nx >= static_cast<int>(width) || ny >= static_cast<int>(height)) {
        continue;
      }

      const Cell next{static_cast<unsigned int>(nx), static_cast<unsigned int>(ny)};
      const unsigned int next_index = index_of(next.x, next.y);
      if (reachable[next_index] != 0 || !is_collision_free(next)) {
        continue;
      }
      if (K_DX[i] != 0 && K_DY[i] != 0) {
        const Cell horizontal{
          static_cast<unsigned int>(static_cast<int>(cell.x) + K_DX[i]), cell.y};
        const Cell vertical{cell.x, static_cast<unsigned int>(static_cast<int>(cell.y) + K_DY[i])};
        if (!is_collision_free(horizontal) || !is_collision_free(vertical)) {
          continue;
        }
      }

      reachable[next_index] = 1;
      open.push(next_index);
    }
  }

  const int radius_cells = static_cast<int>(std::ceil(max_radius_cells));
  const int goal_x = static_cast<int>(requested_goal.x);
  const int goal_y = static_cast<int>(requested_goal.y);
  double best_distance = std::numeric_limits<double>::infinity();
  unsigned char best_cost = nav2_costmap_2d::NO_INFORMATION;
  Cell best_cell{};
  bool found = false;

  for (int y = std::max(0, goal_y - radius_cells);
       y <= std::min(static_cast<int>(height) - 1, goal_y + radius_cells); ++y) {
    for (int x = std::max(0, goal_x - radius_cells);
         x <= std::min(static_cast<int>(width) - 1, goal_x + radius_cells); ++x) {
      const double distance =
        std::hypot(static_cast<double>(x - goal_x), static_cast<double>(y - goal_y));
      if (distance > max_radius_cells) {
        continue;
      }

      const Cell candidate{static_cast<unsigned int>(x), static_cast<unsigned int>(y)};
      const unsigned int candidate_index = index_of(candidate.x, candidate.y);
      if (reachable[candidate_index] == 0 || !isSafe(candidate, safe_cost)) {
        continue;
      }

      const unsigned char candidate_cost = costmap_->getCost(candidate.x, candidate.y);
      if (
        !found || distance < best_distance ||
        (std::abs(distance - best_distance) < 1e-6 && candidate_cost < best_cost)) {
        found = true;
        best_distance = distance;
        best_cost = candidate_cost;
        best_cell = candidate;
      }
    }
  }

  if (!found) {
    return {};
  }
  return {best_cell};
}

geometry_msgs::msg::PoseStamped InflationEscapePlanner::cellPose(
  const Cell & cell, const geometry_msgs::msg::PoseStamped & reference) const
{
  geometry_msgs::msg::PoseStamped pose = reference;
  costmap_->mapToWorld(cell.x, cell.y, pose.pose.position.x, pose.pose.position.y);
  pose.pose.position.z = 0.0;
  return pose;
}

nav_msgs::msg::Path InflationEscapePlanner::prependEscapePath(
  const geometry_msgs::msg::PoseStamped & original_start, const std::vector<Cell> & escape_cells,
  nav_msgs::msg::Path main_path) const
{
  if (escape_cells.size() < 2) {
    return main_path;
  }

  nav_msgs::msg::Path result;
  result.header = main_path.header;
  geometry_msgs::msg::PoseStamped exact_start = original_start;
  exact_start.header = result.header;
  result.poses.push_back(exact_start);

  for (size_t i = 1; i < escape_cells.size(); ++i) {
    auto pose = cellPose(escape_cells[i], original_start);
    pose.header = result.header;

    const auto & previous = result.poses.back().pose.position;
    const double yaw =
      std::atan2(pose.pose.position.y - previous.y, pose.pose.position.x - previous.x);
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, yaw);
    pose.pose.orientation = tf2::toMsg(orientation);
    result.poses.push_back(pose);
  }

  if (!main_path.poses.empty()) {
    const auto & escape_end = result.poses.back().pose.position;
    const auto & main_start = main_path.poses.front().pose.position;
    const double duplicate_tolerance = costmap_->getResolution() * 0.75;
    if (
      std::hypot(escape_end.x - main_start.x, escape_end.y - main_start.y) < duplicate_tolerance) {
      main_path.poses.erase(main_path.poses.begin());
    }
  }

  result.poses.insert(result.poses.end(), main_path.poses.begin(), main_path.poses.end());
  return result;
}

void InflationEscapePlanner::cacheValidPath(
  const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & requested_goal)
{
  if (path.poses.empty()) {
    return;
  }

  std::lock_guard<std::mutex> lock(cached_path_mutex_);
  cached_path_ = path;
  cached_requested_goal_ = requested_goal;
  has_cached_path_ = true;
}

bool InflationEscapePlanner::cachedPathIsCurrentlyTraversable(
  const nav_msgs::msg::Path & path) const
{
  if (!costmap_ || path.poses.empty()) {
    return false;
  }

  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
  const double sample_step = std::max(0.01, costmap_->getResolution() * 0.5);

  const auto point_is_traversable = [this](double x, double y) {
    unsigned int mx;
    unsigned int my;
    if (!costmap_->worldToMap(x, y, mx, my)) {
      return false;
    }
    const unsigned char cost = costmap_->getCost(mx, my);
    // Unknown space is expected during exploration. A newly observed
    // inscribed or lethal cell, however, invalidates the old route.
    return cost == nav2_costmap_2d::NO_INFORMATION ||
           cost <= static_cast<unsigned char>(cached_path_max_cost_);
  };

  const auto & first = path.poses.front().pose.position;
  if (!point_is_traversable(first.x, first.y)) {
    return false;
  }

  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & from = path.poses[i - 1].pose.position;
    const auto & to = path.poses[i].pose.position;
    const double segment_length = std::hypot(to.x - from.x, to.y - from.y);
    const int steps = std::max(1, static_cast<int>(std::ceil(segment_length / sample_step)));
    for (int step = 1; step <= steps; ++step) {
      const double ratio = static_cast<double>(step) / static_cast<double>(steps);
      const double x = from.x + ratio * (to.x - from.x);
      const double y = from.y + ratio * (to.y - from.y);
      if (!point_is_traversable(x, y)) {
        return false;
      }
    }
  }

  return true;
}

nav_msgs::msg::Path InflationEscapePlanner::reuseCachedPath(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & requested_goal) const
{
  nav_msgs::msg::Path cached_path;
  geometry_msgs::msg::PoseStamped cached_goal;
  {
    std::lock_guard<std::mutex> lock(cached_path_mutex_);
    if (!reuse_last_valid_path_ || !has_cached_path_ || cached_path_.poses.empty()) {
      return nav_msgs::msg::Path();
    }
    cached_path = cached_path_;
    cached_goal = cached_requested_goal_;
  }

  if (
    !cached_goal.header.frame_id.empty() && !requested_goal.header.frame_id.empty() &&
    cached_goal.header.frame_id != requested_goal.header.frame_id) {
    return nav_msgs::msg::Path();
  }

  const double goal_distance = std::hypot(
    cached_goal.pose.position.x - requested_goal.pose.position.x,
    cached_goal.pose.position.y - requested_goal.pose.position.y);
  if (goal_distance > cached_path_goal_tolerance_m_) {
    return nav_msgs::msg::Path();
  }

  size_t closest_index = 0;
  double closest_distance = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < cached_path.poses.size(); ++i) {
    const auto & point = cached_path.poses[i].pose.position;
    const double distance =
      std::hypot(point.x - start.pose.position.x, point.y - start.pose.position.y);
    if (distance < closest_distance) {
      closest_distance = distance;
      closest_index = i;
    }
  }
  if (closest_distance > cached_path_max_start_distance_m_) {
    return nav_msgs::msg::Path();
  }

  nav_msgs::msg::Path remaining_path;
  remaining_path.header = cached_path.header;
  remaining_path.header.stamp = start.header.stamp;

  auto exact_start = start;
  exact_start.header = remaining_path.header;
  remaining_path.poses.push_back(exact_start);

  const size_t first_remaining_index = std::min(closest_index + 1, cached_path.poses.size() - 1);
  for (size_t i = first_remaining_index; i < cached_path.poses.size(); ++i) {
    auto pose = cached_path.poses[i];
    pose.header = remaining_path.header;
    remaining_path.poses.push_back(std::move(pose));
  }

  if (!cachedPathIsCurrentlyTraversable(remaining_path)) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "Discarding cached path because the current costmap contains a newly "
      "observed inscribed or lethal obstacle on the remaining route");
    return nav_msgs::msg::Path();
  }

  return remaining_path;
}

nav_msgs::msg::Path InflationEscapePlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal)
{
  if (!theta_star_ || !costmap_) {
    throw nav2_core::PlannerException("Inflation escape planner is not configured");
  }

  try {
    unsigned int start_x;
    unsigned int start_y;
    unsigned int goal_x;
    unsigned int goal_y;
    if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y)) {
      throw nav2_core::PlannerException("Start pose is outside the global costmap");
    }
    if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y)) {
      throw nav2_core::PlannerException("Goal pose is outside the global costmap");
    }

    std::vector<Cell> escape_cells;
    std::vector<Cell> goal_projection_cells;
    unsigned char start_cost;
    unsigned char goal_cost;
    {
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
      start_cost = costmap_->getCost(start_x, start_y);
      goal_cost = costmap_->getCost(goal_x, goal_y);

      escape_cells = findShortestExit(
        Cell{start_x, start_y}, static_cast<unsigned char>(safe_cost_), escape_search_radius_m_);
      if (escape_cells.empty()) {
        throw nav2_core::PlannerException(
          "Robot is in an unsafe costmap area and no non-lethal exit was found");
      }

      if (goal_cost > static_cast<unsigned char>(safe_cost_)) {
        if (!project_unsafe_goal_) {
          throw nav2_core::PlannerException(
            "Goal is in the inflation zone and goal projection is disabled");
        }
        goal_projection_cells = findReachableSafeProjection(
          escape_cells.back(), Cell{goal_x, goal_y}, static_cast<unsigned char>(safe_cost_),
          goal_projection_radius_m_);
        if (goal_projection_cells.empty()) {
          throw nav2_core::PlannerException(
            "Goal is in an unsafe costmap area and no safe projection was found in the robot's "
            "reachable component");
        }
      }
    }

    geometry_msgs::msg::PoseStamped safe_start = start;
    if (escape_cells.size() > 1) {
      safe_start = cellPose(escape_cells.back(), start);
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 2000,
        "Start cost %u is unsafe; planning %.2f m outward to safe cost %d before the main route",
        static_cast<unsigned int>(start_cost),
        std::hypot(
          safe_start.pose.position.x - start.pose.position.x,
          safe_start.pose.position.y - start.pose.position.y),
        safe_cost_);
    }

    geometry_msgs::msg::PoseStamped safe_goal = goal;
    if (!goal_projection_cells.empty()) {
      safe_goal = cellPose(goal_projection_cells.back(), goal);
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 2000,
        "Goal cost %u is unsafe; projected goal within the robot's reachable component "
        "from (%.2f, %.2f) to (%.2f, %.2f)",
        static_cast<unsigned int>(goal_cost), goal.pose.position.x, goal.pose.position.y,
        safe_goal.pose.position.x, safe_goal.pose.position.y);
    }

    auto main_path = theta_star_->createPlan(safe_start, safe_goal);
    if (main_path.poses.empty()) {
      throw nav2_core::PlannerException("Theta* returned an empty path");
    }

    auto result = prependEscapePath(start, escape_cells, std::move(main_path));
    cacheValidPath(result, goal);
    return result;
  } catch (const std::exception & error) {
    auto cached_path = reuseCachedPath(start, goal);
    if (!cached_path.poses.empty()) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 2000,
        "Fresh replanning failed (%s); continuing the cached path with %zu remaining poses",
        error.what(), cached_path.poses.size());
      return cached_path;
    }
    throw;
  }
}

}  // namespace pb_nav2_plugins

PLUGINLIB_EXPORT_CLASS(pb_nav2_plugins::InflationEscapePlanner, nav2_core::GlobalPlanner)

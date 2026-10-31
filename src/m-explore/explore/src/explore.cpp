/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  Copyright (c) 2021, Carlos Alvarez, Juan Galvis.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <explore/explore.h>

#include <algorithm>
#include <limits>
#include <thread>

inline static bool same_point(const geometry_msgs::msg::Point& one,
                              const geometry_msgs::msg::Point& two)
{
  double dx = one.x - two.x;
  double dy = one.y - two.y;
  double dist = sqrt(dx * dx + dy * dy);
  return dist < 0.01;
}

inline static double point_distance(const geometry_msgs::msg::Point& one,
                                    const geometry_msgs::msg::Point& two)
{
  const double dx = one.x - two.x;
  const double dy = one.y - two.y;
  return sqrt(dx * dx + dy * dy);
}

namespace explore
{
Explore::Explore()
  : Node("explore_node")
  , logger_(this->get_logger())
  , tf_buffer_(this->get_clock())
  , tf_listener_(tf_buffer_)
  , costmap_client_(*this, &tf_buffer_)
  , last_markers_count_(0)
{
  double min_frontier_size;
  this->declare_parameter<float>("planner_frequency", 1.0);
  this->declare_parameter<bool>("visualize", false);
  this->declare_parameter<float>("potential_scale", 1e-3);
  this->declare_parameter<float>("orientation_scale", 0.0);
  this->declare_parameter<float>("gain_scale", 1.0);
  this->declare_parameter<float>("min_frontier_size", 0.5);
  this->declare_parameter<bool>("return_to_init", false);
  this->declare_parameter<bool>("start_paused", false);
  this->declare_parameter<std::string>("navigation_costmap_topic",
                                       "global_costmap/costmap");
  this->declare_parameter<int>("navigation_goal_max_cost", 80);
  this->declare_parameter<double>("navigation_goal_search_radius", 0.8);
  this->declare_parameter<double>("navigation_goal_clearance_m", 0.35);
  this->declare_parameter<double>("navigation_goal_min_robot_distance_m", 0.0);
  this->declare_parameter<double>("navigation_goal_max_projection_distance_m", 0.0);
  this->declare_parameter<double>("minimum_exploration_duration_sec", 120.0);
  this->declare_parameter<int>("completion_confirmation_cycles", 3);

  this->get_parameter("planner_frequency", planner_frequency_);
  this->get_parameter("visualize", visualize_);
  this->get_parameter("potential_scale", potential_scale_);
  this->get_parameter("orientation_scale", orientation_scale_);
  this->get_parameter("gain_scale", gain_scale_);
  this->get_parameter("min_frontier_size", min_frontier_size);
  this->get_parameter("return_to_init", return_to_init_);
  this->get_parameter("start_paused", start_paused_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  std::string navigation_costmap_topic;
  this->get_parameter("navigation_costmap_topic", navigation_costmap_topic);
  this->get_parameter("navigation_goal_max_cost", navigation_goal_max_cost_);
  this->get_parameter("navigation_goal_search_radius",
                      navigation_goal_search_radius_);
  this->get_parameter("navigation_goal_clearance_m",
                      navigation_goal_clearance_m_);
  this->get_parameter("navigation_goal_min_robot_distance_m",
                      navigation_goal_min_robot_distance_m_);
  this->get_parameter("navigation_goal_max_projection_distance_m",
                      navigation_goal_max_projection_distance_m_);
  this->get_parameter("minimum_exploration_duration_sec",
                      minimum_exploration_duration_sec_);
  this->get_parameter("completion_confirmation_cycles",
                      completion_confirmation_cycles_);
  completion_confirmation_cycles_ =
      std::max(1, completion_confirmation_cycles_);

  move_base_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
          this, ACTION_NAME);

  search_ = frontier_exploration::FrontierSearch(costmap_client_.getCostmap(),
                                                 potential_scale_, gain_scale_,
                                                 min_frontier_size, logger_);

  if (visualize_) {
    marker_array_publisher_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("explore/"
                                                                     "frontier"
                                                                     "s",
                                                                     10);
  }

  // Publisher for exploration status
  rclcpp::QoS status_qos(10);
  status_qos.transient_local();
  status_pub_ = this->create_publisher<explore_lite_msgs::msg::ExploreStatus>("explore/status", status_qos);

  auto nav_costmap_qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  navigation_costmap_subscription_ =
      this->create_subscription<nav_msgs::msg::OccupancyGrid>(
          navigation_costmap_topic, nav_costmap_qos,
          [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            navigation_costmap_ = msg;
          });
  const auto nav_costmap_wait_start = std::chrono::steady_clock::now();
  while (rclcpp::ok() && !navigation_costmap_ &&
         std::chrono::steady_clock::now() - nav_costmap_wait_start <
             std::chrono::seconds(2)) {
    rclcpp::spin_some(this->get_node_base_interface());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  if (!navigation_costmap_) {
    RCLCPP_WARN(logger_,
                "Navigation costmap validation is not ready yet; "
                "waiting for safe navigation cells before sending goals.");
  }

  RCLCPP_INFO(logger_, "Waiting to connect to move_base nav2 server");
  move_base_client_->wait_for_action_server();
  RCLCPP_INFO(logger_, "Connected to move_base nav2 server");

  if (return_to_init_) {
    RCLCPP_INFO(logger_, "Getting initial pose of the robot");
    geometry_msgs::msg::TransformStamped transformStamped;
    std::string map_frame = costmap_client_.getGlobalFrameID();
    try {
      transformStamped = tf_buffer_.lookupTransform(
          map_frame, robot_base_frame_, tf2::TimePointZero);
      initial_pose_.position.x = transformStamped.transform.translation.x;
      initial_pose_.position.y = transformStamped.transform.translation.y;
      initial_pose_.orientation = transformStamped.transform.rotation;
    } catch (tf2::TransformException& ex) {
      RCLCPP_ERROR(logger_, "Couldn't find transform from %s to %s: %s",
                   map_frame.c_str(), robot_base_frame_.c_str(), ex.what());
      return_to_init_ = false;
    }
  }

  exploring_timer_ = this->create_wall_timer(
      std::chrono::milliseconds((uint16_t)(1000.0 / planner_frequency_)),
      [this]() { makePlan(); });

  auto status_msg = explore_lite_msgs::msg::ExploreStatus();
  if (start_paused_) {
    exploring_timer_->cancel();
    status_msg.status =
        explore_lite_msgs::msg::ExploreStatus::EXPLORATION_PAUSED;
    status_pub_->publish(status_msg);
    RCLCPP_INFO(logger_, "Exploration node started paused.");
  } else {
    status_msg.status =
        explore_lite_msgs::msg::ExploreStatus::EXPLORATION_STARTED;
    status_pub_->publish(status_msg);
    exploration_active_ = true;
    exploration_started_at_ = this->now();
    makePlan();
  }

  // Create the resume/stop subscription only after the timer and initial state are
  // ready. The map-ready starter uses this subscription count as a readiness
  // signal, so creating it earlier lets a resume callback arrive during
  // construction and can dereference an uninitialized timer.
  resume_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "explore/resume", 10,
      std::bind(&Explore::resumeCallback, this, std::placeholders::_1));
  stop_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "explore/stop", 10,
      std::bind(&Explore::stopCallback, this, std::placeholders::_1));
}

Explore::~Explore()
{
  stop();
}

void Explore::resumeCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    resume();
  } else {
    stop();
  }
}

void Explore::stopCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg->data) {
    return;
  }
  if (!exploration_active_) {
    RCLCPP_DEBUG(logger_,
                 "Ignoring forced exploration stop; exploration is not active.");
    return;
  }
  RCLCPP_WARN(logger_,
              "Forced exploration stop requested; treating exploration as complete "
              "and returning to initial pose.");
  stop(true);
}

void Explore::visualizeFrontiers(
    const std::vector<frontier_exploration::Frontier>& frontiers)
{
  const auto blue = std_msgs::msg::ColorRGBA().set__b(1.0).set__a(0.5);
  const auto red = std_msgs::msg::ColorRGBA().set__r(1.0).set__a(0.5);
  const auto green = std_msgs::msg::ColorRGBA().set__g(1.0).set__a(0.5);

  RCLCPP_DEBUG(logger_, "visualising %lu frontiers", frontiers.size());
  visualization_msgs::msg::MarkerArray markers_msg;
  std::vector<visualization_msgs::msg::Marker>& markers = markers_msg.markers;
  visualization_msgs::msg::Marker m;

  m.header.frame_id = costmap_client_.getGlobalFrameID();
  m.header.stamp = this->now();
  m.ns = "frontiers";
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color.r = 0;
  m.color.g = 0;
  m.color.b = 255;
  m.color.a = 255;
  // m.lifetime defaults to 0, means lives forever
  m.frame_locked = true;

  // weighted frontiers are always sorted
  double min_cost = frontiers.empty() ? 0. : frontiers.front().cost;

  m.action = visualization_msgs::msg::Marker::ADD;
  size_t id = 0;
  for (auto& frontier : frontiers) {
    m.type = visualization_msgs::msg::Marker::POINTS;
    m.id = int(id);
    m.pose.position.x = 0.0;
    m.pose.position.y = 0.0;
    m.pose.position.z = 0.0;
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.points = frontier.points;
    if (goalOnBlacklist(frontier.middle)) {
      m.color = red;
    } else {
      m.color = blue;
    }
    markers.push_back(m);
    ++id;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.id = int(id);
    m.pose.position = frontier.centroid;
    // scale frontier according to its cost (costier frontiers will be smaller)
    double scale = std::min(std::abs(min_cost * 0.4 / frontier.cost), 0.5);
    m.scale.x = scale;
    m.scale.y = scale;
    m.scale.z = scale;
    m.points = {};
    m.color = green;
    markers.push_back(m);
    ++id;
  }
  size_t current_markers_count = markers.size();

  // delete previous markers, which are now unused
  m.action = visualization_msgs::msg::Marker::DELETE;
  for (; id < last_markers_count_; ++id) {
    m.id = int(id);
    markers.push_back(m);
  }

  last_markers_count_ = current_markers_count;
  marker_array_publisher_->publish(markers_msg);
}

void Explore::makePlan()
{
  if (!exploration_active_) {
    RCLCPP_DEBUG(logger_, "Skipping exploration planning because exploration is inactive.");
    return;
  }

  auto pose = costmap_client_.getRobotPose();

  if (!navigation_costmap_) {
    RCLCPP_WARN_THROTTLE(
        logger_, *this->get_clock(), 5000,
        "Waiting for navigation costmap before sending exploration goals.");
    return;
  }

  // Finish the accepted Nav2 goal before selecting another frontier. SLAM
  // updates can reorder frontiers every planning cycle; preempting on every
  // reorder makes the robot chase a moving ranking instead of exploring.
  if (navigation_goal_handle_) {
    RCLCPP_DEBUG_THROTTLE(
        logger_, *this->get_clock(), 5000,
        "Keeping the active exploration goal until it finishes.");
    return;
  }

  // find frontiers
  // get frontiers sorted according to cost
  auto frontiers = search_.searchFrom(pose.position);
  RCLCPP_DEBUG(logger_, "found %lu frontiers", frontiers.size());
  for (size_t i = 0; i < frontiers.size(); ++i) {
    RCLCPP_DEBUG(logger_, "frontier %zd cost: %f", i, frontiers[i].cost);
  }

  if (frontiers.empty()) {
    ++no_frontier_cycles_;
    const double elapsed = explorationElapsedSeconds();
    if (elapsed < minimum_exploration_duration_sec_ ||
        no_frontier_cycles_ < completion_confirmation_cycles_) {
      RCLCPP_WARN(
          logger_,
          "No frontiers found (%d/%d), but exploration has run for only "
          "%.1f/%.1fs; waiting for the map to update.",
          no_frontier_cycles_, completion_confirmation_cycles_, elapsed,
          minimum_exploration_duration_sec_);
      return;
    }
    RCLCPP_WARN(logger_,
                "No frontiers found for %d consecutive planning cycles; "
                "finishing exploration after %.1fs.",
                no_frontier_cycles_, elapsed);
    auto status_msg = explore_lite_msgs::msg::ExploreStatus();
    status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_COMPLETE;
    status_pub_->publish(status_msg);
    stop(true);
    return;
  }
  no_frontier_cycles_ = 0;

  // publish frontiers as visualization markers
  if (visualize_) {
    visualizeFrontiers(frontiers);
  }

  // find non blacklisted frontier
  auto frontier =
      std::find_if_not(frontiers.begin(), frontiers.end(),
                       [this](const frontier_exploration::Frontier& f) {
                         return goalOnBlacklist(f.middle);
                       });
  if (frontier == frontiers.end()) {
    ++exhausted_frontier_rounds_;
    const double elapsed = explorationElapsedSeconds();
    if (elapsed < minimum_exploration_duration_sec_ ||
        exhausted_frontier_rounds_ < completion_confirmation_cycles_) {
      RCLCPP_WARN(
          logger_,
          "All current frontiers have failed (recovery round %d/%d, "
          "elapsed %.1f/%.1fs); clearing the frontier blacklist and retrying "
          "with alternate navigation targets.",
          exhausted_frontier_rounds_, completion_confirmation_cycles_, elapsed,
          minimum_exploration_duration_sec_);
      frontier_blacklist_.clear();
      return;
    }
    RCLCPP_WARN(logger_,
                "All frontiers remained unreachable after %d recovery rounds; "
                "finishing exploration after %.1fs.",
                exhausted_frontier_rounds_, elapsed);
    auto status_msg = explore_lite_msgs::msg::ExploreStatus();
    status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_COMPLETE;
    status_pub_->publish(status_msg);
    stop(true);
    return;
  }
  geometry_msgs::msg::Point target_position = frontier->middle;
  // Use a representative boundary point instead of the raw centroid. Large or
  // concave frontiers can have centroids in unknown/occupied cells, so the
  // final Nav2 goal is projected to the nearest clear navigation cell below.
  RCLCPP_INFO(logger_, "Selected frontier target: (%.2f, %.2f), frontier size: %u",
              target_position.x, target_position.y, frontier->size);

  // Elapsed time never excludes a target. Keep an accepted goal active.
  bool same_goal = same_point(prev_goal_, target_position);

  prev_goal_ = target_position;

  // A resume command must re-evaluate the current frontier. The navigation
  // costmap may have become usable since the previous projection attempt.
  const bool force_replan = resuming_;
  resuming_ = false;

  // Keep an accepted Nav2 goal active, but retry projection when no goal was
  // accepted yet or when exploration has just resumed.
  if (same_goal && navigation_goal_handle_ && !force_replan) {
    return;
  }

  geometry_msgs::msg::Point navigation_target_position;
  if (!projectGoalToClearNavigationCell(target_position,
                                         pose.position,
                                         navigation_target_position,
                                         true)) {
    RCLCPP_WARN(
        logger_,
        "No directional backoff cell near frontier target (%.2f, %.2f); "
        "retrying with any known-safe cell in the search area.",
        target_position.x, target_position.y);
    if (!projectGoalToClearNavigationCell(target_position,
                                           pose.position,
                                           navigation_target_position,
                                           false)) {
      frontier_blacklist_.push_back(target_position);
      RCLCPP_WARN(
          logger_,
          "No clear navigation cell near frontier target (%.2f, %.2f); "
          "switching to another frontier immediately.",
          target_position.x, target_position.y);
      makePlan();
      return;
    }
  }

  RCLCPP_DEBUG(logger_, "Sending goal to move base nav2");

  // send goal to move_base if we have something new to pursue
  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.pose.position = navigation_target_position;
  goal.pose.pose.orientation.w = 1.;
  goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.pose.header.stamp = this->now();

  auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  const size_t goal_sequence = ++navigation_goal_sequence_;
  // send_goal_options.goal_response_callback =
  // std::bind(&Explore::goal_response_callback, this, _1);
  // send_goal_options.feedback_callback =
  //   std::bind(&Explore::feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
      [this, target_position, goal_sequence](
          const NavigationGoalHandle::WrappedResult& result) {
        if (goal_sequence != navigation_goal_sequence_) {
          RCLCPP_DEBUG(
              logger_,
              "Ignoring result from superseded navigation goal sequence %zu "
              "(current=%zu).",
              goal_sequence, navigation_goal_sequence_);
          return;
        }
        reachedGoal(result, target_position);
      };
  send_goal_options.goal_response_callback =
      [this, goal_sequence](NavigationGoalHandle::SharedPtr goal_handle) {
        if (goal_sequence != navigation_goal_sequence_) {
          return;
        }
        navigation_goal_handle_ = goal_handle;
      };
  move_base_client_->async_send_goal(goal, send_goal_options);
}

void Explore::returnToInitialPose()
{
  RCLCPP_INFO(logger_, "Returning to initial pose.");
  auto status_msg = explore_lite_msgs::msg::ExploreStatus();
  status_msg.status = explore_lite_msgs::msg::ExploreStatus::RETURNING_TO_ORIGIN;
  status_pub_->publish(status_msg);

  auto current_pose = costmap_client_.getRobotPose();
  const double initial_distance = point_distance(current_pose.position, initial_pose_.position);
  constexpr double kReturnAlreadyThereTolerance = 0.45;
  if (initial_distance <= kReturnAlreadyThereTolerance) {
    auto returned_msg = explore_lite_msgs::msg::ExploreStatus();
    returned_msg.status = explore_lite_msgs::msg::ExploreStatus::RETURNED_TO_ORIGIN;
    status_pub_->publish(returned_msg);
    RCLCPP_INFO(logger_,
                "Already within %.2fm of initial pose (distance=%.3fm); "
                "reporting returned to origin without sending a new Nav2 goal.",
                kReturnAlreadyThereTolerance, initial_distance);
    return;
  }

  geometry_msgs::msg::Point return_position = initial_pose_.position;
  if (projectGoalToClearNavigationCell(initial_pose_.position,
                                       current_pose.position,
                                       return_position)) {
    if (!same_point(return_position, initial_pose_.position)) {
      RCLCPP_WARN(logger_,
                  "Initial pose (%.2f, %.2f) is not clear in navigation costmap; "
                  "return goal projected to (%.2f, %.2f).",
                  initial_pose_.position.x, initial_pose_.position.y,
                  return_position.x, return_position.y);
    }
  } else {
    RCLCPP_WARN(logger_,
                "Could not find a clear navigation cell near initial pose "
                "(%.2f, %.2f); falling back to raw initial pose.",
                initial_pose_.position.x, initial_pose_.position.y);
  }

  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.pose.position = return_position;
  goal.pose.pose.orientation = initial_pose_.orientation;
  goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.pose.header.stamp = this->now();

  auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback =
      [this](const NavigationGoalHandle::WrappedResult& result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          auto status_msg = explore_lite_msgs::msg::ExploreStatus();
          status_msg.status = explore_lite_msgs::msg::ExploreStatus::RETURNED_TO_ORIGIN;
          status_pub_->publish(status_msg);
          RCLCPP_INFO(logger_, "Successfully returned to initial pose.");
        }
      };
  move_base_client_->async_send_goal(goal, send_goal_options);
}
bool Explore::goalOnBlacklist(const geometry_msgs::msg::Point& goal)
{
  constexpr static size_t tolerace = 5;
  nav2_costmap_2d::Costmap2D* costmap2d = costmap_client_.getCostmap();

  // check if a goal is on the blacklist for goals that we're pursuing
  for (auto& frontier_goal : frontier_blacklist_) {
    double x_diff = fabs(goal.x - frontier_goal.x);
    double y_diff = fabs(goal.y - frontier_goal.y);

    if (x_diff < tolerace * costmap2d->getResolution() &&
        y_diff < tolerace * costmap2d->getResolution())
      return true;
  }
  return false;
}

bool Explore::goalIsClearForNavigation(const geometry_msgs::msg::Point& goal)
{
  if (!navigation_costmap_) {
    return false;
  }

  const auto& map = *navigation_costmap_;
  const double resolution = map.info.resolution;
  if (resolution <= 0.0) {
    return false;
  }

  const double origin_x = map.info.origin.position.x;
  const double origin_y = map.info.origin.position.y;
  const int mx = static_cast<int>(std::floor((goal.x - origin_x) / resolution));
  const int my = static_cast<int>(std::floor((goal.y - origin_y) / resolution));
  return navigationCellIsClear(map, mx, my);
}

bool Explore::navigationCellIsClear(const nav_msgs::msg::OccupancyGrid& map,
                                    int mx, int my) const
{
  const int width = static_cast<int>(map.info.width);
  const int height = static_cast<int>(map.info.height);
  const double resolution = map.info.resolution;
  if (resolution <= 0.0 || width <= 0 || height <= 0 || mx < 0 || my < 0 ||
      mx >= width || my >= height) {
    return false;
  }

  const auto decode_cost = [](int8_t raw_cost) -> int {
    // Nav2 costmap values are uint8, but OccupancyGrid stores int8. Unknown
    // 255 appears as -1. Other negative values are valid uint8 costs
    // 128..254 after wrapping, so decode them before threshold checks.
    if (raw_cost == -1) {
      return -1;
    }
    if (raw_cost < -1) {
      return static_cast<int>(static_cast<unsigned char>(raw_cost));
    }
    return static_cast<int>(raw_cost);
  };

  const int center_cost = decode_cost(map.data[my * width + mx]);
  if (center_cost < 0 || center_cost > navigation_goal_max_cost_) {
    return false;
  }

  const int clearance_cells = std::max(
      0, static_cast<int>(std::ceil(navigation_goal_clearance_m_ / resolution)));
  const int clearance_sq = clearance_cells * clearance_cells;
  for (int dy = -clearance_cells; dy <= clearance_cells; ++dy) {
    for (int dx = -clearance_cells; dx <= clearance_cells; ++dx) {
      if (dx * dx + dy * dy > clearance_sq) {
        continue;
      }
      const int x = mx + dx;
      const int y = my + dy;
      if (x < 0 || y < 0 || x >= width || y >= height) {
        return false;
      }
      const int cost = decode_cost(map.data[y * width + x]);
      if (cost == -1) {
        // Frontiers sit next to unknown space by definition. Unknown cells in
        // the clearance ring are acceptable, but the goal center must be known.
        continue;
      }
      if (cost < 0 || cost > navigation_goal_max_cost_) {
        return false;
      }
    }
  }
  return true;
}

bool Explore::projectGoalToClearNavigationCell(
    const geometry_msgs::msg::Point& goal,
    const geometry_msgs::msg::Point& robot_position,
    geometry_msgs::msg::Point& projected,
    bool require_frontier_backoff)
{
  if (!require_frontier_backoff && goalIsClearForNavigation(goal)) {
    if (!navigationGoalDistanceIsAcceptable(goal, goal, robot_position)) {
      return false;
    }
    projected = goal;
    return true;
  }
  if (!navigation_costmap_) {
    return false;
  }

  const auto& map = *navigation_costmap_;
  const double resolution = map.info.resolution;
  if (resolution <= 0.0) {
    return false;
  }

  const double origin_x = map.info.origin.position.x;
  const double origin_y = map.info.origin.position.y;
  const int start_mx = static_cast<int>(std::floor((goal.x - origin_x) / resolution));
  const int start_my = static_cast<int>(std::floor((goal.y - origin_y) / resolution));
  if (start_mx < 0 || start_my < 0 ||
      start_mx >= static_cast<int>(map.info.width) ||
      start_my >= static_cast<int>(map.info.height)) {
    return false;
  }

  const int max_cells =
      std::max(1, static_cast<int>(std::ceil(navigation_goal_search_radius_ /
                                            resolution)));
  int best_x = -1;
  int best_y = -1;
  double best_score = std::numeric_limits<double>::infinity();

  int clear_candidate_count = 0;
  int rejected_robot_distance_count = 0;
  int rejected_projection_distance_count = 0;
  int rejected_not_backoff_count = 0;
  const double frontier_robot_distance = point_distance(robot_position, goal);
  constexpr double kFrontierBackoffMinM = 0.10;

  for (int dy = -max_cells; dy <= max_cells; ++dy) {
    for (int dx = -max_cells; dx <= max_cells; ++dx) {
      if (dx * dx + dy * dy > max_cells * max_cells) {
        continue;
      }
      const int x = start_mx + dx;
      const int y = start_my + dy;
      if (!navigationCellIsClear(map, x, y)) {
        continue;
      }
      ++clear_candidate_count;
      const int8_t raw_center_cost = map.data[y * map.info.width + x];
      const int center_cost =
          raw_center_cost < 0
              ? static_cast<int>(static_cast<unsigned char>(raw_center_cost))
              : static_cast<int>(raw_center_cost);
      geometry_msgs::msg::Point candidate;
      candidate.x = origin_x + (x + 0.5) * resolution;
      candidate.y = origin_y + (y + 0.5) * resolution;
      candidate.z = goal.z;
      const double robot_distance = point_distance(robot_position, candidate);
      if (require_frontier_backoff &&
          robot_distance + kFrontierBackoffMinM >= frontier_robot_distance) {
        ++rejected_not_backoff_count;
        continue;
      }
      if (navigation_goal_min_robot_distance_m_ > 0.0 &&
          robot_distance < navigation_goal_min_robot_distance_m_) {
        ++rejected_robot_distance_count;
        continue;
      }
      const double projection_distance = point_distance(goal, candidate);
      if (navigation_goal_max_projection_distance_m_ > 0.0 &&
          projection_distance > navigation_goal_max_projection_distance_m_) {
        ++rejected_projection_distance_count;
        continue;
      }
      const double dist_sq = dx * dx + dy * dy;
      const double score = dist_sq + 0.05 * center_cost;
      if (score < best_score) {
        best_score = score;
        best_x = x;
        best_y = y;
      }
    }
  }

  if (best_x < 0) {
    RCLCPP_WARN(logger_,
                "No navigation candidate accepted near frontier (%.2f, %.2f): "
                "clear_candidates=%d, rejected_too_close=%d, "
                "rejected_projection_far=%d, rejected_not_backoff=%d, "
                "min_robot_dist=%.2fm, max_projection_dist=%.2fm",
                goal.x, goal.y, clear_candidate_count,
                rejected_robot_distance_count,
                rejected_projection_distance_count,
                rejected_not_backoff_count,
                navigation_goal_min_robot_distance_m_,
                navigation_goal_max_projection_distance_m_);
    return false;
  }

  projected.x = origin_x + (best_x + 0.5) * resolution;
  projected.y = origin_y + (best_y + 0.5) * resolution;
  projected.z = goal.z;
  const double projection_distance = point_distance(goal, projected);
  const double robot_distance = point_distance(robot_position, projected);
  RCLCPP_INFO(logger_,
              "Projected frontier target from (%.2f, %.2f) to safe clear cell "
              "(%.2f, %.2f), clearance=%.2fm, max_cost=%d, "
              "projection_dist=%.2fm, robot_dist=%.2fm",
              goal.x, goal.y, projected.x, projected.y,
              navigation_goal_clearance_m_, navigation_goal_max_cost_,
              projection_distance, robot_distance);
  return true;
}

bool Explore::navigationGoalDistanceIsAcceptable(
    const geometry_msgs::msg::Point& frontier_goal,
    const geometry_msgs::msg::Point& navigation_goal,
    const geometry_msgs::msg::Point& robot_position) const
{
  const double robot_distance = point_distance(robot_position, navigation_goal);
  if (navigation_goal_min_robot_distance_m_ > 0.0 &&
      robot_distance < navigation_goal_min_robot_distance_m_) {
    return false;
  }

  const double projection_distance =
      point_distance(frontier_goal, navigation_goal);
  if (navigation_goal_max_projection_distance_m_ > 0.0 &&
      projection_distance > navigation_goal_max_projection_distance_m_) {
    return false;
  }

  return true;
}

double Explore::explorationElapsedSeconds() const
{
  if (exploration_started_at_.nanoseconds() == 0) {
    return 0.0;
  }
  return (this->now() - exploration_started_at_).seconds();
}

void Explore::reachedGoal(const NavigationGoalHandle::WrappedResult& result,
                          const geometry_msgs::msg::Point& frontier_goal)
{
  navigation_goal_handle_.reset();
  if (!exploration_active_) {
    RCLCPP_DEBUG(logger_,
                 "Ignoring a stale navigation result after exploration stopped.");
    return;
  }

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_DEBUG(logger_, "Goal was successful");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_WARN(
          logger_,
          "Navigation aborted near frontier (%.2f, %.2f); keeping the target "
          "eligible and retrying on the next planning cycle without blacklisting.",
          frontier_goal.x, frontier_goal.y);
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_DEBUG(logger_, "Goal was canceled");
      // If goal canceled might be because exploration stopped from topic. Don't make new plan.
      return;
    default:
      RCLCPP_WARN(logger_, "Unknown result code from move base nav2");
      break;
  }
  // find new goal immediately regardless of planning frequency.
  // execute via timer to prevent dead lock in move_base_client (this is
  // callback for sendGoal, which is called in makePlan). the timer must live
  // until callback is executed.
  // oneshot_ = relative_nh_.createTimer(
  //     ros::Duration(0, 0), [this](const ros::TimerEvent&) { makePlan(); },
  //     true);

  // Because of the 1-thread-executor nature of ros2 I think timer is not
  // needed.
  makePlan();
}

void Explore::start()
{
  RCLCPP_INFO(logger_, "Exploration started.");
  auto status_msg = explore_lite_msgs::msg::ExploreStatus();
  status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_STARTED;
  status_pub_->publish(status_msg);
}

void Explore::stop(bool finished_exploring)
{
  exploration_active_ = false;
  ++navigation_goal_sequence_;
  RCLCPP_INFO(logger_, "Exploration stopped.");

  // Only publish paused status if manually stopped (not finished exploring)
  if (!finished_exploring) {
    auto status_msg = explore_lite_msgs::msg::ExploreStatus();
    status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_PAUSED;
    status_pub_->publish(status_msg);
  }

  if (navigation_goal_handle_) {
    move_base_client_->async_cancel_goal(navigation_goal_handle_);
    navigation_goal_handle_.reset();
  }
  exploring_timer_->cancel();

  if (return_to_init_ && finished_exploring) {
    returnToInitialPose();
  }
}

void Explore::resume()
{
  if (!exploring_timer_) {
    RCLCPP_WARN(logger_, "Ignoring resume request before exploration timer is ready.");
    return;
  }
  if (exploration_active_) {
    RCLCPP_DEBUG(logger_, "Ignoring duplicate resume request; exploration is already active.");
    return;
  }
  exploration_active_ = true;
  resuming_ = true;
  RCLCPP_INFO(logger_, "Exploration resuming.");
  frontier_blacklist_.clear();
  no_frontier_cycles_ = 0;
  exhausted_frontier_rounds_ = 0;
  exploration_started_at_ = this->now();
  auto status_msg = explore_lite_msgs::msg::ExploreStatus();
  status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_IN_PROGRESS;
  status_pub_->publish(status_msg);
  // Reactivate the timer
  exploring_timer_->reset();
  // Resume immediately
  makePlan();
}

}  // namespace explore

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  // ROS1 code
  /*
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  } */
  rclcpp::spin(
      std::make_shared<explore::Explore>());  // std::move(std::make_unique)?
  rclcpp::shutdown();
  return 0;
}

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

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "map_server/map_client.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pb_omni_pid_pursuit_controller/pid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace pb_omni_pid_pursuit_controller
{
// OriginSim MapClient interface. The host serializes calls and publishes commands.
class OmniPidPursuitController
{
public:
  void initialize(
    const rclcpp::Node::WeakPtr & node, const std::string & name,
    const std::shared_ptr<map_server::MapClient> & costmap);
  void setPlan(const nav_msgs::msg::Path & path);
  void reset();
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & measured_speed);

protected:
  nav_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped & pose);
  std::unique_ptr<geometry_msgs::msg::PointStamped> createCarrotMsg(
    const geometry_msgs::msg::PoseStamped & carrot_pose);
  geometry_msgs::msg::PoseStamped getLookAheadPoint(
    const double & lookahead_dist, const nav_msgs::msg::Path & transformed_plan);
  geometry_msgs::msg::Point circleSegmentIntersection(
    const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2, double r);
  double getLookAheadDistance(const geometry_msgs::msg::Twist & speed);
  double approachVelocityScalingFactor(const nav_msgs::msg::Path & path) const;
  void applyApproachVelocityScaling(const nav_msgs::msg::Path & path, double & linear_vel) const;
  void applyCurvatureLimitation(
    const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & lookahead_pose,
    double & linear_vel);
  double calculateCurvature(
    const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & lookahead_pose,
    double forward_dist, double backward_dist) const;
  double calculateCurvatureRadius(
    const geometry_msgs::msg::Point & near_point, const geometry_msgs::msg::Point & current_point,
    const geometry_msgs::msg::Point & far_point) const;
  void visualizeCurvaturePoints(
    const geometry_msgs::msg::PoseStamped & backward_pose,
    const geometry_msgs::msg::PoseStamped & forward_pose) const;
  std::vector<double> calculateCumulativeDistances(const nav_msgs::msg::Path & path) const;
  geometry_msgs::msg::PoseStamped findPoseAtDistance(
    const nav_msgs::msg::Path & path, const std::vector<double> & cumulative_distances,
    double target_distance) const;

private:
  rclcpp::Node::WeakPtr node_;
  std::shared_ptr<map_server::MapClient> costmap_;
  std::unique_ptr<PID> move_pid_, heading_pid_;
  nav_msgs::msg::Path global_plan_;
  bool local_plan_reaches_goal_{false};
  double last_velocity_scaling_factor_{0.0};
  double control_duration_{0.05}, xy_goal_tolerance_{0.3};
  double collision_horizon_{0.5}, collision_margin_{0.01}, max_local_plan_distance_{5.0};
  double translation_kp_{3.0};
  double translation_ki_{0.1};
  double translation_kd_{0.3};
  bool enable_rotation_{false};
  double rotation_kp_{3.0};
  double rotation_ki_{0.1};
  double rotation_kd_{0.3};
  double min_max_sum_error_{1.0};
  double lookahead_dist_{0.3};
  bool use_velocity_scaled_lookahead_dist_{true};
  double min_lookahead_dist_{0.2};
  double max_lookahead_dist_{1.0};
  double lookahead_time_{1.0};
  bool use_interpolation_{true};
  bool use_rotate_to_heading_{false};
  double use_rotate_to_heading_treshold_{0.1};
  double min_approach_linear_velocity_{0.05};
  double approach_velocity_scaling_dist_{0.6};
  double v_linear_min_{-3.0};
  double v_linear_max_{3.0};
  double v_angular_min_{-3.0};
  double v_angular_max_{3.0};
  double curvature_min_{0.4};
  double curvature_max_{0.7};
  double reduction_ratio_at_high_curvature_{0.5};
  double curvature_forward_dist_{0.7};
  double curvature_backward_dist_{0.3};
  double max_velocity_scaling_factor_rate_{0.9};
  double max_robot_pose_search_dist_{5.0};
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr carrot_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr curvature_points_pub_;
};
}  // namespace pb_omni_pid_pursuit_controller

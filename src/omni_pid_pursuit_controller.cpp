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

#include "pb_omni_pid_pursuit_controller/omni_pid_pursuit_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "pb_omni_pid_pursuit_controller/collision_check.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using std::hypot;
namespace pb_omni_pid_pursuit_controller
{
namespace
{
bool finitePose(const geometry_msgs::msg::Pose & pose)
{
  const auto & p = pose.position;
  const auto & q = pose.orientation;
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) && std::isfinite(norm) &&
         std::abs(norm - 1.0) < 0.01;
}
double distance(
  const geometry_msgs::msg::PoseStamped & a, const geometry_msgs::msg::PoseStamped & b)
{
  return std::hypot(a.pose.position.x - b.pose.position.x, a.pose.position.y - b.pose.position.y);
}
}  // namespace

void OmniPidPursuitController::initialize(
  const rclcpp::Node::WeakPtr & parent, const std::string & name,
  const std::shared_ptr<map_server::MapClient> & costmap)
{
  const auto node = parent.lock();
  if (move_pid_) {
    throw std::logic_error("Controller already initialized");
  }
  if (!node || !costmap || name.empty()) {
    throw std::invalid_argument("Controller requires node, instance name and MapClient");
  }
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  const auto parameter = [&](const std::string & key, auto value) {
    using T = decltype(value);
    const auto full_name = name + "." + key;
    if (!node->has_parameter(full_name)) {
      return node->declare_parameter<T>(full_name, value, descriptor);
    }
    return node->get_parameter(full_name).get_value<T>();
  };
  translation_kp_ = parameter("translation_kp", 3.0);
  translation_ki_ = parameter("translation_ki", 0.1);
  translation_kd_ = parameter("translation_kd", 0.3);
  enable_rotation_ = parameter("enable_rotation", false);
  rotation_kp_ = parameter("rotation_kp", 3.0);
  rotation_ki_ = parameter("rotation_ki", 0.1);
  rotation_kd_ = parameter("rotation_kd", 0.3);
  min_max_sum_error_ = parameter("min_max_sum_error", 1.0);
  lookahead_dist_ = parameter("lookahead_dist", 0.3);
  use_velocity_scaled_lookahead_dist_ = parameter("use_velocity_scaled_lookahead_dist", true);
  min_lookahead_dist_ = parameter("min_lookahead_dist", 0.2);
  max_lookahead_dist_ = parameter("max_lookahead_dist", 1.0);
  lookahead_time_ = parameter("lookahead_time", 1.0);
  use_interpolation_ = parameter("use_interpolation", true);
  use_rotate_to_heading_ = parameter("use_rotate_to_heading", false);
  use_rotate_to_heading_treshold_ = parameter("use_rotate_to_heading_treshold", 0.1);
  min_approach_linear_velocity_ = parameter("min_approach_linear_velocity", 0.05);
  approach_velocity_scaling_dist_ = parameter("approach_velocity_scaling_dist", 0.6);
  v_linear_min_ = parameter("v_linear_min", -3.0);
  v_linear_max_ = parameter("v_linear_max", 3.0);
  v_angular_min_ = parameter("v_angular_min", -3.0);
  v_angular_max_ = parameter("v_angular_max", 3.0);
  curvature_min_ = parameter("curvature_min", 0.4);
  curvature_max_ = parameter("curvature_max", 0.7);
  reduction_ratio_at_high_curvature_ = parameter("reduction_ratio_at_high_curvature", 0.5);
  curvature_forward_dist_ = parameter("curvature_forward_dist", 0.7);
  curvature_backward_dist_ = parameter("curvature_backward_dist", 0.3);
  max_velocity_scaling_factor_rate_ = parameter("max_velocity_scaling_factor_rate", 0.9);
  max_robot_pose_search_dist_ = parameter("max_robot_pose_search_dist", 5.0);
  collision_horizon_ = parameter("collision_horizon", 0.5);
  collision_margin_ = parameter("collision_margin", 0.01);
  max_local_plan_distance_ = parameter("max_local_plan_distance", 5.0);
  const double frequency = node->has_parameter("controller_frequency")
                             ? node->get_parameter("controller_frequency").as_double()
                             : node->declare_parameter("controller_frequency", 20.0, descriptor);
  xy_goal_tolerance_ = node->has_parameter("xy_goal_tolerance")
                         ? node->get_parameter("xy_goal_tolerance").as_double()
                         : node->declare_parameter("xy_goal_tolerance", 0.3, descriptor);
  for (double value :
       {frequency, lookahead_dist_, min_lookahead_dist_, max_lookahead_dist_, lookahead_time_,
        approach_velocity_scaling_dist_, max_robot_pose_search_dist_, curvature_forward_dist_,
        curvature_backward_dist_, collision_horizon_, max_local_plan_distance_, v_linear_max_,
        v_angular_max_, max_velocity_scaling_factor_rate_}) {
    if (!std::isfinite(value) || value <= 0) {
      throw std::invalid_argument(
        "Controller distances, rates and maxima must be finite and positive");
    }
  }
  for (double value :
       {translation_kp_, translation_ki_, translation_kd_, rotation_kp_, rotation_ki_, rotation_kd_,
        min_max_sum_error_, min_approach_linear_velocity_, curvature_min_, curvature_max_,
        collision_margin_, xy_goal_tolerance_, use_rotate_to_heading_treshold_}) {
    if (!std::isfinite(value) || value < 0) {
      throw std::invalid_argument(
        "Controller gains, tolerances and margins must be finite and nonnegative");
    }
  }
  if (
    min_lookahead_dist_ > max_lookahead_dist_ || curvature_min_ >= curvature_max_ ||
    !std::isfinite(v_linear_min_) || v_linear_min_ > 0 || !std::isfinite(v_angular_min_) ||
    v_angular_min_ >= 0 || !std::isfinite(reduction_ratio_at_high_curvature_) ||
    reduction_ratio_at_high_curvature_ < 0 || reduction_ratio_at_high_curvature_ > 1 ||
    min_approach_linear_velocity_ > v_linear_max_ ||
    (!enable_rotation_ && use_rotate_to_heading_)) {
    throw std::invalid_argument("Inconsistent controller limits or rotation settings");
  }
  control_duration_ = 1.0 / frequency;
  node_ = parent;
  costmap_ = costmap;
  move_pid_ = std::make_unique<PID>(
    control_duration_, v_linear_max_, 0.0, translation_kp_, translation_kd_, translation_ki_,
    min_max_sum_error_);
  heading_pid_ = std::make_unique<PID>(
    control_duration_, v_angular_max_, v_angular_min_, rotation_kp_, rotation_kd_, rotation_ki_,
    min_max_sum_error_);
  local_path_pub_ = node->create_publisher<nav_msgs::msg::Path>(name + "/local_plan", 1);
  carrot_pub_ =
    node->create_publisher<geometry_msgs::msg::PointStamped>(name + "/lookahead_point", 1);
  curvature_points_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    name + "/curvature_points_marker_array", 1);
}

void OmniPidPursuitController::reset()
{
  global_plan_ = nav_msgs::msg::Path{};
  local_plan_reaches_goal_ = false;
  last_velocity_scaling_factor_ = 0.0;
  if (move_pid_) {
    move_pid_->reset();
  }
  if (heading_pid_) {
    heading_pid_->reset();
  }
}

void OmniPidPursuitController::setPlan(const nav_msgs::msg::Path & path)
{
  if (!move_pid_) {
    throw std::logic_error("Initialize before setting a plan");
  }
  reset();
  if (path.header.frame_id.empty() || path.poses.empty()) {
    throw std::invalid_argument("Path requires a frame and at least one pose");
  }
  for (const auto & pose : path.poses) {
    if (
      !finitePose(pose.pose) ||
      (!pose.header.frame_id.empty() && pose.header.frame_id != path.header.frame_id)) {
      throw std::invalid_argument("Invalid path pose or inconsistent frame");
    }
  }
  global_plan_ = path;
}

nav_msgs::msg::Path OmniPidPursuitController::transformGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose)
{
  auto transform = geometry_msgs::msg::TransformStamped{};
  transform.transform.rotation.w = 1.0;
  if (global_plan_.header.frame_id != pose.header.frame_id) {
    transform = costmap_->getTfBuffer()->lookupTransform(
      pose.header.frame_id, global_plan_.header.frame_id, tf2::TimePointZero);
  }
  if (
    std::abs(transform.transform.rotation.x) > 1e-3 ||
    std::abs(transform.transform.rotation.y) > 1e-3) {
    throw std::runtime_error("Controller requires planar path transforms");
  }
  const double yaw = tf2::getYaw(pose.pose.orientation);
  const double rotation = tf2::getYaw(transform.transform.rotation);
  const double c = std::cos(yaw), s = std::sin(yaw);
  std::vector<geometry_msgs::msg::PoseStamped> body;
  body.reserve(global_plan_.poses.size());
  for (const auto & point : global_plan_.poses) {
    geometry_msgs::msg::PoseStamped world, out;
    tf2::doTransform(point, world, transform);
    const double dx = world.pose.position.x - pose.pose.position.x;
    const double dy = world.pose.position.y - pose.pose.position.y;
    out.header = pose.header;
    out.header.frame_id = costmap_->getBaseFrameID();
    out.pose.position.x = c * dx + s * dy;
    out.pose.position.y = -s * dx + c * dy;
    tf2::Quaternion q;
    q.setRPY(0, 0, tf2::getYaw(point.pose.orientation) + rotation - yaw);
    out.pose.orientation = tf2::toMsg(q);
    body.push_back(out);
  }
  size_t nearest = 0;
  double arc = 0, best = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < body.size(); ++i) {
    if (i) {
      arc += distance(body[i - 1], body[i]);
    }
    if (arc > max_robot_pose_search_dist_) {
      break;
    }
    const double d = hypot(body[i].pose.position.x, body[i].pose.position.y);
    if (d < best) {
      best = d;
      nearest = i;
    }
  }
  nav_msgs::msg::Path local;
  local.header = body.front().header;
  size_t end = nearest;
  for (; end < body.size(); ++end) {
    if (hypot(body[end].pose.position.x, body[end].pose.position.y) > max_local_plan_distance_) {
      break;
    }
    local.poses.push_back(body[end]);
  }
  if (local.poses.empty()) {
    throw std::runtime_error("No path points in local tracking range");
  }
  local_plan_reaches_goal_ = end == body.size();
  global_plan_.poses.erase(global_plan_.poses.begin(), global_plan_.poses.begin() + nearest);
  local_path_pub_->publish(local);
  return local;
}

geometry_msgs::msg::TwistStamped OmniPidPursuitController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & speed)
{
  const auto node = node_.lock();
  if (!node || !move_pid_ || global_plan_.poses.empty()) {
    throw std::logic_error("Controller has no active plan");
  }
  if (
    pose.header.frame_id != costmap_->getGlobalFrameID() || !finitePose(pose.pose) ||
    !std::isfinite(speed.linear.x) || !std::isfinite(speed.linear.y) ||
    !std::isfinite(speed.angular.z)) {
    throw std::invalid_argument("Invalid robot pose, frame or measured speed");
  }
  const auto view = costmap_->view();
  if (!view || !costmap_->isCurrent(*view)) {
    throw std::runtime_error("Map is not current");
  }
  const auto local = transformGlobalPlan(pose);
  const auto carrot = getLookAheadPoint(getLookAheadDistance(speed), local);
  carrot_pub_->publish(createCarrotMsg(carrot));
  const double heading = std::atan2(carrot.pose.position.y, carrot.pose.position.x);
  const double goal_distance =
    hypot(local.poses.back().pose.position.x, local.poses.back().pose.position.y);
  const bool at_position = local_plan_reaches_goal_ && goal_distance <= xy_goal_tolerance_;
  const double yaw_error = tf2::getYaw(
    (use_rotate_to_heading_ || at_position) ? local.poses.back().pose.orientation
                                            : carrot.pose.orientation);
  double linear = move_pid_->calculate(hypot(carrot.pose.position.x, carrot.pose.position.y), 0);
  const bool rotate_only = enable_rotation_ && use_rotate_to_heading_ &&
                           std::abs(yaw_error) > use_rotate_to_heading_treshold_;
  if (rotate_only || at_position) {
    linear = 0;
    move_pid_->reset();
  }
  applyCurvatureLimitation(local, carrot, linear);
  if (local_plan_reaches_goal_) {
    applyApproachVelocityScaling(local, linear);
  }
  geometry_msgs::msg::TwistStamped command;
  command.header = local.header;
  command.header.stamp = node->now();
  command.twist.linear.x = linear * std::cos(heading);
  command.twist.linear.y = linear * std::sin(heading);
  command.twist.angular.z = enable_rotation_ ? heading_pid_->calculate(yaw_error, 0) : 0;
  double radius = costmap_->settings().robot_radius;
  for (const auto & point : costmap_->getRobotFootprint()) {
    radius = std::max(radius, hypot(point.x, point.y));
  }
  checkCommandCollision(
    *view, pose.pose, command.twist, radius + collision_margin_,
    std::max(collision_horizon_, control_duration_));
  if (!costmap_->isCurrent(*view)) {
    throw std::runtime_error("Map expired during control");
  }
  return command;
}

std::unique_ptr<geometry_msgs::msg::PointStamped> OmniPidPursuitController::createCarrotMsg(
  const geometry_msgs::msg::PoseStamped & carrot_pose)
{
  auto carrot_msg = std::make_unique<geometry_msgs::msg::PointStamped>();
  carrot_msg->header = carrot_pose.header;
  carrot_msg->point.x = carrot_pose.pose.position.x;
  carrot_msg->point.y = carrot_pose.pose.position.y;
  carrot_msg->point.z = 0.01;
  return carrot_msg;
}

geometry_msgs::msg::PoseStamped OmniPidPursuitController::getLookAheadPoint(
  const double & lookahead_dist, const nav_msgs::msg::Path & transformed_plan)
{
  auto goal_pose_it = std::find_if(
    transformed_plan.poses.begin(), transformed_plan.poses.end(), [&](const auto & ps) {
      return hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist;
    });

  if (goal_pose_it == transformed_plan.poses.end()) {
    goal_pose_it = std::prev(transformed_plan.poses.end());
  } else if (use_interpolation_ && goal_pose_it != transformed_plan.poses.begin()) {
    auto prev_pose_it = std::prev(goal_pose_it);
    auto point = circleSegmentIntersection(
      prev_pose_it->pose.position, goal_pose_it->pose.position, lookahead_dist);
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = prev_pose_it->header.frame_id;
    pose.header.stamp = goal_pose_it->header.stamp;
    pose.pose.position = point;
    const double length = distance(*prev_pose_it, *goal_pose_it);
    const double ratio =
      length > 1e-9
        ? std::clamp(
            std::hypot(
              point.x - prev_pose_it->pose.position.x, point.y - prev_pose_it->pose.position.y) /
              length,
            0.0, 1.0)
        : 1.0;
    tf2::Quaternion q1, q2;
    tf2::fromMsg(prev_pose_it->pose.orientation, q1);
    tf2::fromMsg(goal_pose_it->pose.orientation, q2);
    pose.pose.orientation = tf2::toMsg(q1.slerp(q2, ratio).normalized());
    return pose;
  }

  return *goal_pose_it;
}

geometry_msgs::msg::Point OmniPidPursuitController::circleSegmentIntersection(
  const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2, double r)
{
  const double dx = p2.x - p1.x, dy = p2.y - p1.y;
  const double a = dx * dx + dy * dy;
  if (a < 1e-18) {
    return p2;
  }
  const double b = 2 * (p1.x * dx + p1.y * dy);
  const double c = p1.x * p1.x + p1.y * p1.y - r * r;
  const double discriminant = b * b - 4 * a * c;
  if (discriminant < 0) {
    return p2;
  }
  const double t = std::clamp((-b + std::sqrt(discriminant)) / (2 * a), 0.0, 1.0);
  geometry_msgs::msg::Point point;
  point.x = p1.x + t * dx;
  point.y = p1.y + t * dy;
  return point;
}

double OmniPidPursuitController::getLookAheadDistance(const geometry_msgs::msg::Twist & speed)
{
  double lookahead_dist = lookahead_dist_;

  if (use_velocity_scaled_lookahead_dist_) {
    lookahead_dist = hypot(speed.linear.x, speed.linear.y) * lookahead_time_;
    lookahead_dist = std::clamp(lookahead_dist, min_lookahead_dist_, max_lookahead_dist_);
  }

  return lookahead_dist;
}

double OmniPidPursuitController::approachVelocityScalingFactor(
  const nav_msgs::msg::Path & transformed_path) const
{
  double remaining_distance = calculateCumulativeDistances(transformed_path).back();
  if (remaining_distance < approach_velocity_scaling_dist_) {
    auto & last = transformed_path.poses.back();
    double distance_to_last_pose = std::hypot(last.pose.position.x, last.pose.position.y);
    return distance_to_last_pose / approach_velocity_scaling_dist_;
  } else {
    return 1.0;
  }
}

void OmniPidPursuitController::applyApproachVelocityScaling(
  const nav_msgs::msg::Path & path, double & linear_vel) const
{
  double approach_vel = linear_vel;
  double velocity_scaling = approachVelocityScalingFactor(path);
  double unbounded_vel = approach_vel * velocity_scaling;
  if (unbounded_vel < min_approach_linear_velocity_) {
    approach_vel = min_approach_linear_velocity_;
  } else {
    approach_vel *= velocity_scaling;
  }

  linear_vel = std::min(linear_vel, approach_vel);
}

void OmniPidPursuitController::applyCurvatureLimitation(
  const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & lookahead_pose,
  double & linear_vel)
{
  double curvature =
    calculateCurvature(path, lookahead_pose, curvature_forward_dist_, curvature_backward_dist_);

  double scaled_linear_vel = linear_vel;
  if (curvature > curvature_min_) {
    double reduction_ratio = 1.0;
    if (curvature > curvature_max_) {
      reduction_ratio = reduction_ratio_at_high_curvature_;
    } else {
      reduction_ratio = 1.0 - (curvature - curvature_min_) / (curvature_max_ - curvature_min_) *
                                (1.0 - reduction_ratio_at_high_curvature_);
    }

    double target_scaled_vel = linear_vel * reduction_ratio;
    scaled_linear_vel =
      last_velocity_scaling_factor_ + std::clamp(
                                        target_scaled_vel - last_velocity_scaling_factor_,
                                        -max_velocity_scaling_factor_rate_ * control_duration_,
                                        max_velocity_scaling_factor_rate_ * control_duration_);
  }
  scaled_linear_vel = std::max(scaled_linear_vel, 2.0 * min_approach_linear_velocity_);

  linear_vel = std::min(linear_vel, scaled_linear_vel);
  last_velocity_scaling_factor_ = linear_vel;
}

double OmniPidPursuitController::calculateCurvature(
  const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & lookahead_pose,
  double forward_dist, double backward_dist) const
{
  geometry_msgs::msg::PoseStamped backward_pose, forward_pose;
  std::vector<double> cumulative_distances = calculateCumulativeDistances(path);

  double lookahead_pose_cumulative_distance = 0.0;
  double closest = std::numeric_limits<double>::infinity();
  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & a = path.poses[i - 1].pose.position;
    const auto & b = path.poses[i].pose.position;
    const double dx = b.x - a.x, dy = b.y - a.y, length2 = dx * dx + dy * dy;
    const auto & p = lookahead_pose.pose.position;
    const double t =
      length2 > 1e-18 ? std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / length2, 0.0, 1.0) : 0;
    const double d = std::hypot(p.x - a.x - t * dx, p.y - a.y - t * dy);
    if (d < closest) {
      closest = d;
      lookahead_pose_cumulative_distance = cumulative_distances[i - 1] + t * std::sqrt(length2);
    }
  }

  backward_pose = findPoseAtDistance(
    path, cumulative_distances, lookahead_pose_cumulative_distance - backward_dist);

  forward_pose = findPoseAtDistance(
    path, cumulative_distances, lookahead_pose_cumulative_distance + forward_dist);

  double curvature_radius = calculateCurvatureRadius(
    backward_pose.pose.position, lookahead_pose.pose.position, forward_pose.pose.position);
  double curvature = 1.0 / curvature_radius;
  visualizeCurvaturePoints(backward_pose, forward_pose);
  return curvature;
}

double OmniPidPursuitController::calculateCurvatureRadius(
  const geometry_msgs::msg::Point & near_point, const geometry_msgs::msg::Point & current_point,
  const geometry_msgs::msg::Point & far_point) const
{
  double x1 = near_point.x, y1 = near_point.y;
  double x2 = current_point.x, y2 = current_point.y;
  double x3 = far_point.x, y3 = far_point.y;

  double denom = 2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2));

  if (std::abs(denom) < 1e-9) {
    return 1e9;
  }

  double center_x = ((x1 * x1 + y1 * y1) * (y2 - y3) + (x2 * x2 + y2 * y2) * (y3 - y1) +
                     (x3 * x3 + y3 * y3) * (y1 - y2)) /
                    denom;

  double center_y = ((x1 * x1 + y1 * y1) * (x3 - x2) + (x2 * x2 + y2 * y2) * (x1 - x3) +
                     (x3 * x3 + y3 * y3) * (x2 - x1)) /
                    denom;

  double radius = std::hypot(x2 - center_x, y2 - center_y);
  if (std::isnan(radius) || std::isinf(radius) || radius < 1e-9) {
    return 1e9;
  }
  return radius;
}

void OmniPidPursuitController::visualizeCurvaturePoints(
  const geometry_msgs::msg::PoseStamped & backward_pose,
  const geometry_msgs::msg::PoseStamped & forward_pose) const
{
  visualization_msgs::msg::MarkerArray marker_array;

  visualization_msgs::msg::Marker near_marker;
  near_marker.header = backward_pose.header;
  near_marker.ns = "curvature_points";
  near_marker.id = 0;
  near_marker.type = visualization_msgs::msg::Marker::SPHERE;
  near_marker.action = visualization_msgs::msg::Marker::ADD;
  near_marker.pose = backward_pose.pose;
  near_marker.scale.x = 0.1;
  near_marker.scale.y = 0.1;
  near_marker.scale.z = 0.1;
  near_marker.color.g = 1.0;
  near_marker.color.a = 1.0;

  visualization_msgs::msg::Marker far_marker;
  far_marker.header = forward_pose.header;
  far_marker.ns = "curvature_points";
  far_marker.id = 1;
  far_marker.type = visualization_msgs::msg::Marker::SPHERE;
  far_marker.action = visualization_msgs::msg::Marker::ADD;
  far_marker.pose = forward_pose.pose;
  far_marker.scale.x = 0.1;
  far_marker.scale.y = 0.1;
  far_marker.scale.z = 0.1;
  far_marker.color.r = 1.0;
  far_marker.color.a = 1.0;

  marker_array.markers.push_back(near_marker);
  marker_array.markers.push_back(far_marker);

  curvature_points_pub_->publish(marker_array);
}

std::vector<double> OmniPidPursuitController::calculateCumulativeDistances(
  const nav_msgs::msg::Path & path) const
{
  std::vector<double> cumulative_distances;
  cumulative_distances.push_back(0.0);

  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & prev_pose = path.poses[i - 1].pose.position;
    const auto & curr_pose = path.poses[i].pose.position;
    double distance = hypot(curr_pose.x - prev_pose.x, curr_pose.y - prev_pose.y);
    cumulative_distances.push_back(cumulative_distances.back() + distance);
  }
  return cumulative_distances;
}

geometry_msgs::msg::PoseStamped OmniPidPursuitController::findPoseAtDistance(
  const nav_msgs::msg::Path & path, const std::vector<double> & cumulative_distances,
  double target_distance) const
{
  if (path.poses.empty() || cumulative_distances.empty()) {
    return geometry_msgs::msg::PoseStamped();
  }
  if (target_distance <= 0.0) {
    return path.poses.front();
  }
  if (target_distance >= cumulative_distances.back()) {
    return path.poses.back();
  }

  auto it =
    std::lower_bound(cumulative_distances.begin(), cumulative_distances.end(), target_distance);
  size_t index = std::distance(cumulative_distances.begin(), it);

  if (index == 0) {
    return path.poses.front();
  }

  double segment_dist = cumulative_distances[index] - cumulative_distances[index - 1];
  if (segment_dist < 1e-9) {
    return path.poses[index];
  }

  double ratio = (target_distance - cumulative_distances[index - 1]) / segment_dist;

  geometry_msgs::msg::PoseStamped pose1 = path.poses[index - 1];
  geometry_msgs::msg::PoseStamped pose2 = path.poses[index];

  geometry_msgs::msg::PoseStamped interpolated_pose;
  interpolated_pose.header = pose2.header;
  interpolated_pose.pose.position.x =
    pose1.pose.position.x + ratio * (pose2.pose.position.x - pose1.pose.position.x);
  interpolated_pose.pose.position.y =
    pose1.pose.position.y + ratio * (pose2.pose.position.y - pose1.pose.position.y);
  interpolated_pose.pose.position.z =
    pose1.pose.position.z + ratio * (pose2.pose.position.z - pose1.pose.position.z);
  interpolated_pose.pose.orientation = pose2.pose.orientation;

  return interpolated_pose;
}

}  // namespace pb_omni_pid_pursuit_controller

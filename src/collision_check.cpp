#include "pb_omni_pid_pursuit_controller/collision_check.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace pb_omni_pid_pursuit_controller
{
void checkCommandCollision(
  const map_server::MapView & view, const geometry_msgs::msg::Pose & pose,
  const geometry_msgs::msg::Twist & command, double radius, double horizon)
{
  if (
    !view.ready() || !std::isfinite(radius) || radius < 0 || !std::isfinite(horizon) ||
    horizon <= 0 || !std::isfinite(command.linear.x) || !std::isfinite(command.linear.y) ||
    !std::isfinite(command.angular.z)) {
    throw std::runtime_error("Invalid collision-check input or map");
  }
  const auto & grid = view.geometry();
  const double speed = std::hypot(command.linear.x, command.linear.y);
  const double resolution = grid.resolution;
  if (grid.width < 3 || grid.height < 3 || !std::isfinite(resolution) || resolution <= 0) {
    throw std::runtime_error("Unusable collision grid");
  }
  // A tiny safety expansion covers the space between samples as well.
  const double steps_needed = std::ceil(speed * horizon / (0.5 * resolution));
  if (!std::isfinite(steps_needed) || steps_needed > 10000) {
    throw std::runtime_error("Collision sweep exceeds sampling budget");
  }
  const int steps = std::max(1, static_cast<int>(steps_needed));
  const double swept_radius = radius + speed * horizon / (2.0 * steps);
  const double yaw = tf2::getYaw(pose.orientation);
  const double c = std::cos(yaw), s = std::sin(yaw), wz = command.angular.z;
  for (int i = 0; i <= steps; ++i) {
    const double t = horizon * i / steps;
    const double a = std::abs(wz) < 1e-9 ? t : std::sin(wz * t) / wz;
    const double b = std::abs(wz) < 1e-9 ? 0.0 : (1 - std::cos(wz * t)) / wz;
    const double dx = a * command.linear.x - b * command.linear.y;
    const double dy = b * command.linear.x + a * command.linear.y;
    const double x = pose.position.x + c * dx - s * dy;
    const double y = pose.position.y + s * dx + c * dy;
    unsigned x0, x1, y0, y1;
    if (
      !view.worldToCell(x - swept_radius, y - swept_radius, x0, y0) ||
      !view.worldToCell(x + swept_radius, y + swept_radius, x1, y1)) {
      throw std::runtime_error("Command footprint leaves map");
    }
    for (unsigned iy = y0; iy <= y1; ++iy) {
      for (unsigned ix = x0; ix <= x1; ++ix) {
        if (!view.latestOccupied(ix, iy)) {
          continue;
        }
        const auto center = view.center(ix, iy);
        const double ex = std::max(std::abs(center.x() - x) - 0.5 * resolution, 0.0);
        const double ey = std::max(std::abs(center.y() - y) - 0.5 * resolution, 0.0);
        if (ex * ex + ey * ey <= swept_radius * swept_radius) {
          throw std::runtime_error("Command footprint intersects an obstacle");
        }
      }
    }
  }
}
}  // namespace pb_omni_pid_pursuit_controller

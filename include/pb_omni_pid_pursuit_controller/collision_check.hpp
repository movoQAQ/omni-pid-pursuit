#pragma once

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "map_server/map_view.hpp"

namespace pb_omni_pid_pursuit_controller
{
// Sweep a circumscribed robot disc along the actual constant-body-twist command.
// MapView raw occupancy already includes latest dynamic observations, but no
// robot-radius inflation. Unknown/unavailable maps and map exits fail closed.
void checkCommandCollision(
  const map_server::MapView & view, const geometry_msgs::msg::Pose & pose,
  const geometry_msgs::msg::Twist & command, double radius, double horizon);
}  // namespace pb_omni_pid_pursuit_controller

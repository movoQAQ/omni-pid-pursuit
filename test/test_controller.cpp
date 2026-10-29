#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <limits>
#include <thread>

#include "map_server/costmap_generator.hpp"
#include "map_server/edt_2d.hpp"
#include "pb_omni_pid_pursuit_controller/collision_check.hpp"
#include "pb_omni_pid_pursuit_controller/omni_pid_pursuit_controller.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using pb_omni_pid_pursuit_controller::checkCommandCollision;
using pb_omni_pid_pursuit_controller::OmniPidPursuitController;
namespace
{
geometry_msgs::msg::PoseStamped pose(double x, double y, double yaw = 0)
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "map";
  p.pose.position.x = x;
  p.pose.position.y = y;
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  p.pose.orientation = tf2::toMsg(q);
  return p;
}
nav_msgs::msg::Path path(double x = 3, double y = 0)
{
  nav_msgs::msg::Path p;
  p.header.frame_id = "map";
  for (int i = 0; i <= 30; ++i) {
    p.poses.push_back(pose(i * x / 30, i * y / 30));
  }
  return p;
}
class Probe : public OmniPidPursuitController
{
public:
  using OmniPidPursuitController::circleSegmentIntersection;
  using OmniPidPursuitController::getLookAheadPoint;
};
TEST(Pid, ResetRemovesIntegralAndDerivativeHistory)
{
  PID pid(.05, 10, -10, 1, 1, 1, .2);
  const auto first = pid.calculate(1, 0);
  for (int i = 0; i < 100; ++i) {
    pid.calculate(1, 0);
  }
  EXPECT_NEAR(pid.calculate(1, 0), 1.2, 1e-12);
  pid.reset();
  EXPECT_DOUBLE_EQ(pid.calculate(1, 0), first);
  EXPECT_THROW(pid.calculate(NAN, 0), std::invalid_argument);
  EXPECT_THROW(PID(0, 1, -1, 1, 0, 0), std::invalid_argument);
}
TEST(Geometry, InterpolationPreservesUnitOrientationAndHandlesDuplicatePoints)
{
  Probe c;
  nav_msgs::msg::Path p;
  p.poses = {pose(0, 0, 0), pose(2, 0, 1)};
  const auto carrot = c.getLookAheadPoint(1, p);
  EXPECT_NEAR(carrot.pose.position.x, 1, 1e-9);
  EXPECT_NEAR(tf2::getYaw(carrot.pose.orientation), .5, 1e-9);
  auto point = c.circleSegmentIntersection(p.poses[0].pose.position, p.poses[0].pose.position, 1);
  EXPECT_TRUE(std::isfinite(point.x));
}
map_server::MapView makeView(bool obstacle)
{
  auto s = std::make_shared<map_server::StaticMap>();
  auto d = std::make_shared<map_server::DerivedMaps>();
  s->geometry = d->geometry = {100, 100, .1, -5, -5};
  s->occupancy = {s->geometry, std::vector<uint8_t>(10000, 0)};
  d->fused_obstacles = s->occupancy;
  if (obstacle) {
    s->occupancy.data[50 * 100 + 56] = 1;
    d->fused_obstacles = s->occupancy;
  }
  return map_server::MapView(s, d);
}
TEST(Collision, SweepsActualCommandAndRejectsBoundsAndUnreadyMaps)
{
  auto view = makeView(false);
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 1;
  EXPECT_NO_THROW(checkCommandCollision(view, pose(0, 0).pose, cmd, .2, 1));
  EXPECT_THROW(
    checkCommandCollision(makeView(true), pose(0, 0).pose, cmd, .2, 1), std::runtime_error);
  EXPECT_THROW(checkCommandCollision(view, pose(4.8, 0).pose, cmd, .2, 1), std::runtime_error);
  EXPECT_THROW(
    checkCommandCollision(map_server::MapView{}, pose(0, 0).pose, cmd, .2, 1), std::runtime_error);
  cmd.linear.x = NAN;
  EXPECT_THROW(checkCommandCollision(view, pose(0, 0).pose, cmd, .2, 1), std::runtime_error);
}
TEST(Collision, BodyVelocityIsRotatedIntoMap)
{
  auto view = makeView(true);
  geometry_msgs::msg::Twist cmd;
  cmd.linear.y = -1;
  EXPECT_THROW(
    checkCommandCollision(view, pose(0, 0, M_PI / 2).pose, cmd, .2, 1), std::runtime_error);
  EXPECT_NO_THROW(checkCommandCollision(view, pose(0, 0).pose, cmd, .2, 1));
}
class ControllerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void SetUp() override
  {
    node = std::make_shared<rclcpp::Node>(
      "pursuit_test",
      rclcpp::NodeOptions().parameter_overrides(
        {rclcpp::Parameter("PID.translation_kp", 1.0), rclcpp::Parameter("PID.translation_ki", 0.0),
         rclcpp::Parameter("PID.translation_kd", 0.0), rclcpp::Parameter("PID.v_linear_max", 1.0),
         rclcpp::Parameter("PID.use_velocity_scaled_lookahead_dist", false),
         rclcpp::Parameter("PID.lookahead_dist", 1.0)}));
    client = std::make_shared<map_server::MapClient>(
      "pursuit_map",
      rclcpp::NodeOptions().parameter_overrides(
        {rclcpp::Parameter("require_cloud", false), rclcpp::Parameter("map_timeout", 2.0)}));
    executor.add_node(client);
    executor.add_node(node);
    auto qos = rclcpp::QoS(1).reliable().transient_local();
    statics = node->create_publisher<map_server::msg::StaticMap>("/map_server/static", qos);
    maps = node->create_publisher<map_server::msg::DerivedMap>("/map_server/derived", qos);
    dynamics =
      node->create_publisher<map_server::msg::DynamicObstacles>("/map_server/dynamic", qos);
    statuses = node->create_publisher<map_server::msg::MapStatus>("/map_server/status", qos);
    ASSERT_TRUE(wait([&] {
      return statics->get_subscription_count() && maps->get_subscription_count() &&
             dynamics->get_subscription_count() && statuses->get_subscription_count();
    }));
    controller.initialize(node, "PID", client);
  }
  void TearDown() override
  {
    executor.remove_node(client);
    executor.remove_node(node);
  }
  template <class F>
  bool wait(F predicate)
  {
    for (int i = 0; i < 500; ++i) {
      executor.spin_some();
      if (predicate()) return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(4));
    }
    return false;
  }
  void publishMap(bool obstacle = false)
  {
    map_server::msg::StaticMap s;
    map_server::msg::DerivedMap d;
    map_server::msg::DynamicObstacles o;
    map_server::msg::MapStatus h;
    s.header.frame_id = d.header.frame_id = o.header.frame_id = h.header.frame_id = "map";
    s.header.stamp = d.header.stamp = o.header.stamp = h.header.stamp = node->now();
    s.info.width = s.info.height = 100;
    s.info.resolution = .1;
    s.info.origin.position.x = s.info.origin.position.y = -5;
    s.info.origin.orientation.w = 1;
    d.info = s.info;
    s.occupancy.assign(10000, 0);
    s.region_id.assign(10000, 0);
    if (obstacle) s.occupancy[50 * 100 + 56] = 1;
    map_server::DerivedMaps state;
    state.geometry = {100, 100, static_cast<double>(s.info.resolution), -5, -5};
    state.fused_obstacles = {state.geometry, s.occupancy};
    state.distance_field = map_server::preciseEdt(state.fused_obstacles);
    map_server::generateCosts(state, {});
    d.fused_obstacles = state.fused_obstacles.data;
    d.distance_field = state.distance_field.data;
    d.costmap = state.costmap.data;
    d.collision_mask = state.collision_mask.data;
    d.hard_radius_m = .3;
    s.producer_id = d.producer_id = o.producer_id = h.producer_id = 1;
    d.version = o.version = h.derived_version = h.dynamic_version = 1;
    d.generated_at_s = node->now().seconds();
    statics->publish(s);
    maps->publish(d);
    dynamics->publish(o);
    statuses->publish(h);
    ASSERT_TRUE(wait([&] { return client->isCurrent(); }));
  }
  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<map_server::MapClient> client;
  Probe controller;
  rclcpp::Publisher<map_server::msg::StaticMap>::SharedPtr statics;
  rclcpp::Publisher<map_server::msg::DerivedMap>::SharedPtr maps;
  rclcpp::Publisher<map_server::msg::DynamicObstacles>::SharedPtr dynamics;
  rclcpp::Publisher<map_server::msg::MapStatus>::SharedPtr statuses;
};
TEST_F(ControllerTest, ProducesBaseFrameForwardAndLateralCommands)
{
  publishMap();
  controller.setPlan(path());
  auto cmd = controller.computeVelocityCommands(pose(0, 0), geometry_msgs::msg::Twist{});
  EXPECT_EQ(cmd.header.frame_id, client->getBaseFrameID());
  EXPECT_NEAR(cmd.twist.linear.x, 1, 1e-9);
  EXPECT_NEAR(cmd.twist.linear.y, 0, 1e-9);
  controller.setPlan(path());
  cmd = controller.computeVelocityCommands(pose(0, 0, M_PI / 2), geometry_msgs::msg::Twist{});
  EXPECT_NEAR(cmd.twist.linear.x, 0, 1e-9);
  EXPECT_NEAR(cmd.twist.linear.y, -1, 1e-9);
  EXPECT_EQ(cmd.twist.angular.z, 0);
}
TEST_F(ControllerTest, RejectsInvalidInputAndClearsOldPlan)
{
  publishMap();
  controller.setPlan(path());
  auto bad = path();
  bad.poses[2].pose.position.x = NAN;
  EXPECT_THROW(controller.setPlan(bad), std::invalid_argument);
  EXPECT_THROW(
    controller.computeVelocityCommands(pose(0, 0), geometry_msgs::msg::Twist{}), std::logic_error);
  controller.setPlan(path());
  bad = path();
  bad.poses[2].header.frame_id = "odom";
  EXPECT_THROW(controller.setPlan(bad), std::invalid_argument);
  controller.setPlan(path());
  auto robot = pose(0, 0);
  robot.header.frame_id = "wrong";
  EXPECT_THROW(
    controller.computeVelocityCommands(robot, geometry_msgs::msg::Twist{}), std::invalid_argument);
  geometry_msgs::msg::Twist speed;
  speed.linear.y = NAN;
  EXPECT_THROW(controller.computeVelocityCommands(pose(0, 0), speed), std::invalid_argument);
  controller.reset();
  EXPECT_THROW(
    controller.computeVelocityCommands(pose(0, 0), geometry_msgs::msg::Twist{}), std::logic_error);
}
TEST_F(ControllerTest, StopsAtGoalAndFailsOnObstacle)
{
  publishMap(true);
  controller.setPlan(path());
  EXPECT_THROW(
    controller.computeVelocityCommands(pose(0, 0), geometry_msgs::msg::Twist{}),
    std::runtime_error);
  controller.setPlan(path());
  const auto cmd = controller.computeVelocityCommands(pose(3, 0), geometry_msgs::msg::Twist{});
  EXPECT_EQ(cmd.twist.linear.x, 0);
  EXPECT_EQ(cmd.twist.linear.y, 0);
}
TEST_F(ControllerTest, FailsWithoutFreshMapAndMissingTransform)
{
  controller.setPlan(path());
  EXPECT_THROW(
    controller.computeVelocityCommands(pose(0, 0), geometry_msgs::msg::Twist{}),
    std::runtime_error);
  publishMap();
  auto p = path();
  p.header.frame_id = "missing";
  for (auto & pt : p.poses) pt.header.frame_id = "missing";
  controller.setPlan(p);
  EXPECT_THROW(
    controller.computeVelocityCommands(pose(0, 0), geometry_msgs::msg::Twist{}),
    tf2::TransformException);
}
TEST_F(ControllerTest, TracksSCurveInClosedLoopWithoutNav2)
{
  publishMap();
  auto route = path();
  for (auto & p : route.poses) p.pose.position.y = .5 * std::sin(2 * M_PI * p.pose.position.x / 3);
  controller.setPlan(route);
  auto current = pose(0, 0);
  geometry_msgs::msg::Twist measured;
  for (int i = 0; i < 600; ++i) {
    const auto command = controller.computeVelocityCommands(current, measured);
    const double norm = std::hypot(command.twist.linear.x, command.twist.linear.y);
    ASSERT_TRUE(std::isfinite(norm));
    ASSERT_LE(norm, 1.0 + 1e-9);
    current.pose.position.x += command.twist.linear.x * .05;
    current.pose.position.y += command.twist.linear.y * .05;
    measured = command.twist;
    if (norm == 0) break;
  }
  EXPECT_LE(std::hypot(current.pose.position.x - 3, current.pose.position.y), .3);
}
TEST_F(ControllerTest, FailsOnStaleMap)
{
  publishMap();
  controller.setPlan(path());
  map_server::msg::MapStatus status;
  status.header.frame_id = "map";
  status.header.stamp = node->now() - rclcpp::Duration::from_seconds(10);
  status.producer_id = status.derived_version = status.dynamic_version = 1;
  statuses->publish(status);
  ASSERT_TRUE(wait([&] { return !client->isCurrent(); }));
  EXPECT_THROW(
    controller.computeVelocityCommands(pose(0, 0), geometry_msgs::msg::Twist{}),
    std::runtime_error);
}
TEST_F(ControllerTest, TransformsPathFrameBeforeBodyProjection)
{
  publishMap();
  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = "map";
  tf.child_frame_id = "route";
  tf.transform.translation.x = 1;
  tf.transform.translation.y = 1;
  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI / 2);
  tf.transform.rotation = tf2::toMsg(q);
  ASSERT_TRUE(client->getTfBuffer()->setTransform(tf, "test", true));
  auto route = path();
  route.header.frame_id = "route";
  for (auto & p : route.poses) p.header.frame_id = "route";
  controller.setPlan(route);
  const auto cmd = controller.computeVelocityCommands(pose(1, 1), geometry_msgs::msg::Twist{});
  EXPECT_NEAR(cmd.twist.linear.x, 0, 1e-9);
  EXPECT_NEAR(cmd.twist.linear.y, 1, 1e-9);
}
TEST_F(ControllerTest, RotationModeStopsTranslation)
{
  publishMap();
  node->declare_parameter("ROT.enable_rotation", true);
  node->declare_parameter("ROT.use_rotate_to_heading", true);
  Probe rotation;
  rotation.initialize(node, "ROT", client);
  auto route = path();
  for (auto & p : route.poses) p.pose.orientation = pose(0, 0, M_PI / 2).pose.orientation;
  rotation.setPlan(route);
  const auto cmd = rotation.computeVelocityCommands(pose(0, 0), geometry_msgs::msg::Twist{});
  EXPECT_EQ(cmd.twist.linear.x, 0);
  EXPECT_EQ(cmd.twist.linear.y, 0);
  EXPECT_GT(cmd.twist.angular.z, 0);
}
TEST_F(ControllerTest, RejectsBadParameters)
{
  auto invalid = std::make_shared<rclcpp::Node>(
    "invalid_pursuit", rclcpp::NodeOptions().parameter_overrides(
                         {rclcpp::Parameter("PID.enable_rotation", false),
                          rclcpp::Parameter("PID.use_rotate_to_heading", true)}));
  Probe bad;
  EXPECT_THROW(bad.initialize(invalid, "PID", client), std::invalid_argument);
}
}  // namespace

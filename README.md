# Omni PID Pursuit

用于全向底盘的 PID + Pure Pursuit 路径跟踪控制器，由原 `pb_omni_pid_pursuit_controller` 改造而来，适配 OriginSim 的普通 C++ 控制器接口。保留前视跟踪、横向运动、可选航向 PID、曲率降速与终点减速，移除所有 Nav2 编译和运行依赖。

仓库名为 `omni-pid-pursuit`；为保留已有代码引用，ROS 包名和 C++ 命名空间仍为 `pb_omni_pid_pursuit_controller`。

## 适配版本

参考 [movoQAQ/OriginSim](https://github.com/movoQAQ/OriginSim) 的 `nav` 分支提交 [`38702f56`](https://github.com/movoQAQ/OriginSim/commit/38702f56af3f4a8c1ecc21c077f5f7392a2e0cb1)。该版本使用 `map_server::MapClient` 和不可变 `MapView`，由 `PathExecutor` 直接调用控制器，已经没有 `ControllerPlugin` 插件接口。

本包不兼容较早的 `CostmapClient` / `nav_executor::ControllerPlugin` 接口，也不能作为 Nav2 的 `FollowPath` 插件加载。`map_server` 指 OriginSim 自带的包，不是 `nav2_map_server`。

## 输入与输出

```cpp
pb_omni_pid_pursuit_controller::OmniPidPursuitController controller;
controller.initialize(node, "PID", map_client);
controller.setPlan(path);
auto command = controller.computeVelocityCommands(pose, measured_speed);
controller.reset();  // 取消、结束或失败时清除路径和 PID 状态
```

| 接口 | 内容与坐标约定 |
|---|---|
| `initialize(node, name, map_client)` | 普通 `rclcpp::Node`、参数前缀和 OriginSim `MapClient` |
| `setPlan(nav_msgs::msg::Path)` | 非空路径，必须有路径坐标系；各点坐标系为空或与路径一致；四元数必须有效 |
| `computeVelocityCommands(pose, measured_speed)` | 位姿位于地图全局坐标系；速度是执行器已转换到控制基坐标系的 `Twist` |
| 返回 `TwistStamped` | `header.frame_id` 为 `MapClient::getBaseFrameID()`；`linear.x/y` 为该坐标系下的全向速度，`angular.z` 为角速度 |
| `reset()` | 清除旧路径、积分、微分和曲率降速状态 |

本库不订阅 `/odom`，也不发布 `/cmd_vel`。沿用 OriginSim 的执行器：它读取里程计与 TF，判断目标完成、超时和急停，调用控制器后发布速度。地图通过 `MapClient` 提供。控制器会在路径、TF、地图、参数或计算输入无效时抛出异常，由执行器捕获并停车。

调试话题为 `PID/local_plan`、`PID/lookahead_point`、`PID/curvature_points_marker_array`，前缀随初始化实例名变化。调用 `initialize`、`setPlan`、`reset` 和计算接口需要由宿主串行执行。

## 在 OriginSim 中接入

以下操作在上述参考版本的 OriginSim 工作空间根目录执行：

```bash
git clone https://github.com/movoQAQ/omni-pid-pursuit.git src/navigation/omni-pid-pursuit

git apply --check src/navigation/omni-pid-pursuit/integration/originsim-nav.patch
git apply src/navigation/omni-pid-pursuit/integration/originsim-nav.patch

source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to nav_executor --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source install/setup.bash
```

补丁修改 `nav_executor` 的控制器头文件、成员类型、初始化参数前缀、CMake/包依赖和两份 YAML 配置（包括启动入口实际加载的 `bringup/params/params.yaml`），将 MPC 切换为 PID Pursuit。它保留执行器原有的里程计、目标判断、停车和恢复机制，不会同时运行两套速度发布器。然后使用 OriginSim 原有启动入口即可。

如果 OriginSim 已经升级，先按接口表检查差异；补丁检查失败时不要强行套用。`config/originsim.yaml` 是控制器参数片段，不是完整的导航启动配置。

## 参数与行为

参数在初始化时读取，修改后需重启执行器。完整示例见 [`config/originsim.yaml`](config/originsim.yaml)。

- `controller_frequency`：执行器控制频率，PID 使用它计算步长；默认 20 Hz。
- `translation_kp/ki/kd`、`rotation_kp/ki/kd`：平移和旋转 PID 系数。
- `min_max_sum_error`：积分绝对值上限，在计算积分输出前限幅。
- `enable_rotation`：默认 `false`，适配 OriginSim 的 `base_link_fake` 链路；这条链路会丢弃角速度。需要旋转时应使用能执行角速度的真实控制基坐标系，并同步设置宿主目标航向容差。
- `use_rotate_to_heading`：默认 `false`；只有启用旋转时才允许设置为 `true`。阈值参数沿用旧拼写 `use_rotate_to_heading_treshold`。
- `lookahead_dist`、`use_velocity_scaled_lookahead_dist`、`min_lookahead_dist`、`max_lookahead_dist`、`lookahead_time`：固定或随速度变化的前视距离。
- `use_interpolation`：对前视点位置和姿态进行插值。
- `v_linear_max`：平移速度模长上限；库默认 3 m/s，接入配置设为 1 m/s。全向运动方向由前视点决定；兼容保留的 `v_linear_min` 必须不大于零，实际 PID 速度模长下限固定为零，防止减速时出现反向追踪。
- `v_angular_min/max`：有符号角速度上下限。
- `curvature_min/max`、`reduction_ratio_at_high_curvature`、`curvature_forward_dist/backward_dist`、`max_velocity_scaling_factor_rate`：曲率降速与变化速率；曲率采样按路径弧长定位。
- `approach_velocity_scaling_dist`、`min_approach_linear_velocity`：真正接近全局路径终点时减速，不把被裁剪的局部路径末端当成终点。
- `max_robot_pose_search_dist`：沿剩余路径查找最近点的最大弧长；`max_local_plan_distance` 为跟踪局部范围。二者默认 5 m。
- `collision_horizon`：检查实际输出速度未来的运动，默认 0.5 s，至少覆盖一个控制周期。
- `collision_margin`：物理轮廓外接圆之外的余量，默认 0.01 m。碰撞检查使用 `MapView::latestOccupied` 的原始静态和最新动态障碍，避免重复膨胀；越界或地图失效时拒绝输出。
- `xy_goal_tolerance`：读取执行器的位置容差；到达该范围后停止平移，最终完成判定仍由执行器负责。

这是路径跟踪器，不负责绕障规划；碰撞检查拒绝危险命令，不会生成绕行路径。它也不提供 MPC 的加速度约束、预测优化或通道专用策略。真实底盘仍需根据动力学调整速度与 PID 参数。

## 验证

在 ROS 2 Jazzy 下编译，使用参考提交的真实 `map_server`，没有用替代接口或 Nav2 库。接入补丁已在该提交上验证，切换后的完整 `nav_executor` 及所需依赖编译通过；控制器共享库没有 Nav2 链接依赖。13 项测试覆盖 PID 重置/积分限幅、插值四元数、全向坐标变换、速度限幅、路径失效、缺失 TF、地图失效、障碍/地图边界、终点停车及合成 S 弯闭环跟踪。

```bash
colcon build --packages-select pb_omni_pid_pursuit_controller --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select pb_omni_pid_pursuit_controller --event-handlers console_direct+
colcon test-result --verbose
```

测试在隔离 ROS domain 中运行，不向实际底盘发送速度。合成闭环使用理想运动学，不能代替 Gazebo 或实车调参。

## 来源与归档

原始源码来自 [PolarBear Omni PID Pursuit](https://github.com/SMBU-PolarBear-Robot-Team/pb_omni_pid_pursuit_controller)，保留原作者版权和 Apache-2.0 许可证。

`original-nav2` 标签保存迁移前的完整本地源码快照。当前版本的主要改动是普通类接口、OriginSim 地图输入、去 Nav2 依赖、状态重置与输入校验、稳健插值、实际速度的碰撞检查，以及接入配置和测试。Nav2 的生命周期函数、插件注册、目标检查器参数和未实现的速度限制接口已移除。

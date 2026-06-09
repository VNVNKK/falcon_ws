# FALCON + MARSIM Mid360 集成首轮审查

对照 `FALCON_MARSIM_Mid360_Plan.md` 检查当前实现，整体方向正确，以下是查漏补缺。

---

## 必须修正

### 1. `marsim_library.yaml` 的 z 范围与 init_z 大概率不匹配

配置里写的 PCD AABB 是 `z[-3.62, 2.06]`，但当前给的是：

- `init_z: 1.0`
- `box_min_z: 0.0`, `box_max_z: 2.2`
- `map_min_z: -4.0`, `map_max_z: 3.0`

如果 PCD 里 z=-3.62 是地面、z=2.06 是天花板，那 init_z=1.0 实际位于地面以上 4.62m——已经穿透天花板。更可能的情况是 PCD 坐标系里地面不在 z=0（典型如建模时原点在墙角空中）。

**动作**：用 `pcl_viewer` 看一眼 PCD 的"地板"实际处在什么 z，然后任选其一：

- 调整 `init_z` 与 `box_*_z` / `map_*_z` 起底
- 在 `marsim_mid360_single.launch` 的 `<param name="map_offset_z">` 平移整个 PCD 到 z=0 起底（注意：这是 MARSIM `map_pub` 的偏移机制，**不是 FALCON 的 `T_m_w`**——`T_m_w` 在 pointcloud 路径下根本不被用）

森林那张图的 `box_max_z=4.0` vs `map_max_z=16.0` 是合理的（不去飞树梢），但库里这个一定要核。

### 2. CATKIN_IGNORE 是必需依赖，不只是预防性，需要写进 README

`marsim_mid360_single.launch:54-65` 用了 `pkg="odom_visualization"` 并 remap `~odometry`。这个 remap 只对 **FALCON 版本**有效：

- FALCON `uav_simulator/utils/odom_visualization/src/odom_visualization.cpp:441` → `subscribe("odometry", ...)`
- MARSIM `Utils/odom_visualization/src/odom_visualization.cpp:545` → `subscribe("odom", ...)`

当前 git status 显示 MARSIM 那侧已经加了 `CATKIN_IGNORE`，先后顺序对了。**但 README 的"MARSIM Mid360 Simulation Mode"必须写一句**：

> 构建前需要在 `MARSIM/Utils/{odom_visualization, uav_utils, pose_utils, rviz_plugins, waypoint_generator, mars_quadrotor_msgs}` 添加空 `CATKIN_IGNORE`，避免与 FALCON 同名包冲突。

否则别人 clone 之后会立刻撞包。

### 3. 还有一份 quadrotor_msgs 副本没处理

计划第 6 条专门提到 `src/FALCON/uav_simulator/utils/multi_map_server/quadrotor_msgs/` 这份历史副本，`package.xml` 里包名同样是 `quadrotor_msgs`（和 `src/FALCON/uav_simulator/utils/quadrotor_msgs/` 撞名）。

当前没在它里头加 `CATKIN_IGNORE`。之前没报错是因为 catkin 的 "first found" 规则碰巧选对了；将来 clean build 顺序变化、或新机器构建时可能炸。

**动作**：在 `src/FALCON/uav_simulator/utils/multi_map_server/quadrotor_msgs/` 加一个空 `CATKIN_IGNORE`。

---

## 建议复核

### 4. `voxel_mapping.yaml` 还保留 `mode: "uav_simulator"`，对 360° lidar 有副作用

`map_server.cpp:107-114` 里 mode 只用来在 AIRSIM 模式下把 `tsdf.depth_axis_` 设成 Y，其它都是 Z。MARSIM 走 UAV_SIMULATOR 跑得通。

**但 depth_axis=Z + 360° lidar 会引入一个隐藏副作用**：

- `tsdf.cpp:36`：`depth = point_c.z()`（sensor-frame Z 分量）
- `tsdf.cpp:64`：`weight = 1.0 / (depth * depth)`

Mid360 水平方向的点 sensor-frame `|z|` 接近 0，权重会爆炸；正上/正下的点 z 大、权重很小。会让 TSDF 表面被水平点云过度主导。

这是 FALCON 假设 pinhole 相机的历史遗留问题，不阻塞跑通。**5.3 验证时如果发现 TSDF 出现奇怪的薄壳/噪声、或表面恒定向水平方向偏置，根因在这。**

要彻底干净需要在 `tsdf.cpp:36-37` 改成 `depth = point_c.norm()`，并在 `mode_string` 增加一档 `lidar` 开关。但这超出了"原 Gazebo 模式完整保留"的边界，先不动。

### 5. `traj_server` remap 的位置已正确，但有隐藏发布者待确认

`exploration_marsim.launch:32` 的 `<remap from="/planning/pos_cmd" to="/quad_0/planning/pos_cmd"/>` 没问题：

- `traj_server.cpp:415` 使用 `ros::NodeHandle node;`（默认根 namespace）
- `advertise("planning/pos_cmd", ...)` 解析为 `/planning/pos_cmd`
- remap 生效后输出到 `/quad_0/planning/pos_cmd`，被 cascadePID 接收 ✓

**但 `fast_planner` 内部还有其它 cmd publisher**，需要确认它们不会和 cascadePID 抢话题。这个问题原 launch 也存在，先标记不修。

### 6. `init_yaw` 没传给 `quadrotor_dynamics_node`

`marsim_mid360_single.launch:24-35` 的 `quad$(drone_id)_quadrotor_dynamics` 没有 `init_state_yaw` 参数。

对照 MARSIM 原始 `single_drone.xml:33-44` 也确实没传——`init_state_yaw` 只在 cascadePID 那侧设。当前 forest/library 的 `init_yaw` 都是 0，与 dynamics 节点默认值一致 → 不会漂移。

**但如果以后改 init_yaw≠0，dynamics 节点会从 yaw=0 起飞，cascadePID 立刻拉到目标 yaw 会出现起飞抖动。** 现在不改，留个备注。

### 7. `timestamp_tolerance=0.001s` 偏紧

MARSIM 的 `cloud` 和 `sensor_pose` 是同一节点同回调里依次 publish 的（`pointcloud_render_node.cpp` 同一帧两次 `ros::Time::now()`），1ms 内通常能 match。

但 ROS 网络栈 + 回调队列调度偶尔会拉开 ≥1ms，导致 `[Transformer] No match found for transform timestamp` 刷屏。

**动作**：先放到 `0.01`（10ms），跑一次没问题再考虑收紧。

---

## 已验证为正确的关键点

- `MapServer::pointcloudCallback` (`map_server.cpp:298`) 在 `/voxel_mapping/depth_image` 没有发布者时空转，无需修改 ✓
- `Transformer::poseCallback` (`transformer.cpp:124-138`) 不校验 `frame_id`，MARSIM 的 `sensor_pose.frame_id="/map"` 与 FALCON 的 `world` 不一致也无影响 ✓
- `pcl_render_node` 的 `cloud` (`pointcloud_render_node.cpp:1746`) frame_id 已是 `world`，可直接喂给 TSDF ✓
- 节点名 `quad0_pcl_render_node` (无下划线) → topic `/quad0_pcl_render_node/cloud` 与 `/quad0_pcl_render_node/sensor_pose` 解析正确 ✓
- `concurrent_pointcloud_input_max` 是死配置，pointcloud 路径实际由 `concurrent_depth_input_max` 控制（`map_server.cpp:304`）✓
- `T_b_c` / `T_m_w` 仅被 `mesh_render` 使用，pointcloud 路径下设单位矩阵安全 ✓
- `map_file: ""` 不会触发任何 assert，`MapServer` 不读这个参数；只有 `pointcloud_render_node` 和 `mesh_render` 读，但它们不在 MARSIM 模式启动 ✓
- FALCON odom_visualization 的 `mesh_resource: package://odom_visualization/meshes/hummingbird.mesh` 文件存在 ✓
- `/map_generator/global_cloud` (latched) 没被错误 remap 到 `/voxel_mapping/global_map`，探索语义保持 ✓

---

## 跑通前的最小动作清单

1. `pcl_viewer src/MARSIM/map_generator/resource/LibraryLG_01cutoff_sor.pcd`，按地板实际 z 校正 `marsim_library.yaml`
2. `touch src/FALCON/uav_simulator/utils/multi_map_server/quadrotor_msgs/CATKIN_IGNORE`
3. `voxel_mapping.yaml` 把 `timestamp_tolerance: 0.001` 临时调到 `0.01`
4. README 补一段构建前置条件（CATKIN_IGNORE 清单）

跑起来后若 frontier 一直空 / TSDF 不长 / 飞机起飞瞬间漂移，按计划 5.4 的回退矩阵排查即可。

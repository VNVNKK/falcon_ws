# FALCON 自主探索 + MARSIM Mid360 集成方案

## Context

FALCON（`src/FALCON/`）原本运行在自带的 `uav_simulator` 上：用 STL/mesh 渲染深度图 → 反投影点云 → TSDF/ESDF/OccupancyGrid 建图 → 前沿探索。现在希望把传感器后端换成 **MARSIM + Livox Mid360**（非重复扫描花瓣形点云）。

**可行性结论：完全可行，且改动很小。** 关键发现：
1. FALCON 的 `MapServer` 已原生支持点云直接输入（`map_server.cpp:298 pointcloudCallback`），无需修改建图核心。
2. MARSIM 的 `cascadePID` 直接 `#include <quadrotor_msgs/PositionCommand.h>`，与 FALCON 的 `traj_server` 输出完全同型——**不需要桥接节点**，仅 topic remap。
3. MARSIM 的 `pointcloud_render_node` 输出已变换到 `world` 系的 `PointCloud2`，与 FALCON 的 `transformer` 配 `pose_topic_type=pose` 即可对齐。
4. FALCON 的 `transformer.poseCallback` 不校验 `frame_id`（`transformer.cpp:124-138`），MARSIM 的 `frame_id="/map"` 历史不一致问题不影响。

**目标产出**：新增 MARSIM 流程，原 Gazebo+相机模式完整保留。

---

## 1. 新建文件

### 1.1 `src/FALCON/falcon_planner/exploration_manager/launch/exploration_marsim.launch`

要点：
- 复用 FALCON 原有的所有 yaml（robot/voxel_mapping/astar/fast_planner/frontier_finder/hgrid/perception_utils/exploration_manager），加载新的 map config（marsim_forest 或 marsim_library，`<arg name="map_name" default="marsim_forest"/>`）。
- `exploration_node` remap：
  - `/odom_world` → `/quad_0/lidar_slam/odom`
  - `/voxel_mapping/pointcloud` → `/quad0_pcl_render_node/cloud`（**新增**，注意节点名是 `quad0_pcl_render_node`，不是 `quad_0/...`）
  - `/transformer/sensor_pose_topic` → `/quad0_pcl_render_node/sensor_pose`
- `traj_server` remap：`/planning/pos_cmd` → `/quad_0/planning/pos_cmd`（让 cascadePID 接收）。
- **不启动**：`map_render_node`、`poscmd_2_odom`、`odom_visualization`（odom_visualization 由 MARSIM single_drone.xml 内部已启动）。
- 末尾 `<include file="$(find exploration_manager)/launch/marsim_mid360_single.launch"/>`（自有 fork，下条）。

### 1.2 `src/FALCON/falcon_planner/exploration_manager/launch/marsim_mid360_single.launch`

从 `src/MARSIM/test_interface/launch/single_drone_mid360.launch` 复制并参数化（不污染 MARSIM 源），由 1.1 调用。需要把 `init_x_/y_/z_` 与 FALCON map config 的 `init_x/y/z` 对齐；通过 `<arg name="map_pcd"/>` 接受 PCD 路径，由 1.1 传入。

### 1.3 两个 FALCON map config

**`src/FALCON/falcon_planner/exploration_manager/config/map/marsim_forest.yaml`**
- 对应 PCD：`src/MARSIM/map_generator/resource/small_forest01cutoff.pcd`
- 起飞：`init_x=0, init_y=0, init_z=1.5, init_yaw=0`
- map_size 初值（实现时用 `pcl_viewer` 测真实 AABB 后收紧）：`x∈[-32,32]`, `y∈[-32,32]`, `z∈[-0.5,5]`
- box（任务边界，比 map_size 内缩 ~3m）：`x∈[-28,28]`, `y∈[-28,28]`, `z∈[0.2,4]`
- vbox：略大于 box
- `map_file: ""`（globalMapCallback 不会触发，loadMap 不会被调用）
- `map_dimension: 3`
- `T_b_c` 单位矩阵或与原 yaml 同（pointcloud 路径不依赖此矩阵；保留以兼容 yaml schema）
- `T_m_w` 单位矩阵（点云已在 world 系）

**`src/FALCON/falcon_planner/exploration_manager/config/map/marsim_library.yaml`**
- 对应 PCD：`src/MARSIM/map_generator/resource/LibraryLG_01cutoff_sor.pcd`
- 起飞：`init_x=0, init_y=0, init_z=1.0`
- 其余结构同上，尺寸按图书馆室内场景缩小（`x∈[-20,20]`, `y∈[-20,20]`, `z∈[-0.2,3]`，box 内缩 2m）
- 实现时同样按真实 AABB 调整

参考结构：`src/FALCON/falcon_planner/exploration_manager/config/map/complex_office.yaml`

---

## 2. 现有文件修改

### 2.1 `src/FALCON/falcon_planner/voxel_mapping/config/voxel_mapping.yaml`

| 字段 | 旧值 | 新值 | 原因 |
|---|---|---|---|
| `transformer.pose_topic_type` | `transform` | `pose` | MARSIM 发 `geometry_msgs/PoseStamped` |
| `tsdf.raycast_max` | `5.0` | `15.0` | mid360 量程 15m |
| `tsdf.tsdf_truncated_distance_scale` | `2.0` | `4.0` | mid360 角分辨率较粗，截断带需更宽 |
| `voxel_mapping.concurrent_depth_input_max` | `1` | `2` | mid360 单帧点云较大；变量名带 depth 但同时管 pointcloud 路径（`map_server.cpp:304`） |

不修改 `resolution_fine/coarse`（保持 0.1/0.2，地图自适应逻辑会按 box 体积切换）；如果实测内存吃紧再放大到 0.15/0.25。

### 2.2 `src/FALCON/falcon_planner/voxel_mapping/src/map_server.cpp`

**不动**。`depthCallback` 在 `/voxel_mapping/depth_image` 没有发布者时自动空转。

### 2.3 `src/FALCON/falcon_planner/exploration_manager/config/robot/uav_model_simulator.yaml`

**不动**。相机内参在 pointcloud 路径下不被使用，可忽略。

### 2.4 MARSIM 端

**不修改**。仅在 FALCON 包内 fork `single_drone_mid360.launch`（见 1.2）。

### 2.5 `src/FALCON/README.md`

新增一节"MARSIM Mid360 仿真模式"：
- 依赖：MARSIM 子模块已在 workspace；GLEW/GLFW/OpenGL（CPU 模式可省）
- 运行命令：
  - `roslaunch exploration_manager exploration_marsim.launch map_name:=marsim_forest`
  - `roslaunch exploration_manager exploration_marsim.launch map_name:=marsim_library`
  - 另开终端：`roslaunch exploration_manager rviz.launch`
- 与原 Gazebo 模式（`exploration.launch`）的对照表

---

## 3. 话题映射

| 数据流 | MARSIM 端 | FALCON 端订阅 | 说明 |
|---|---|---|---|
| 里程计 | `/quad_0/lidar_slam/odom`（Odometry, world） | `/odom_world` | exploration_node remap |
| LiDAR 点云 | `/quad0_pcl_render_node/cloud`（PointCloud2, world，已变换） | `/voxel_mapping/pointcloud` | exploration_node remap |
| 传感器位姿 | `/quad0_pcl_render_node/sensor_pose`（PoseStamped, frame=/map） | `/transformer/sensor_pose_topic` | `pose_topic_type: pose` |
| 控制指令 | `/quad_0/planning/pos_cmd`（quadrotor_msgs/PositionCommand） | — | traj_server remap 输出端 |

**绝不要 remap** `/voxel_mapping/global_map` ← `/map_generator/global_cloud`，否则 FALCON 直接获得全局真值，探索失去意义。

---

## 4. 构建步骤

```bash
cd /home/yyf/falcon_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

潜在依赖问题：
- workspace 中存在两份 `quadrotor_msgs`：`src/FALCON/uav_simulator/utils/quadrotor_msgs/`（生效）与 `src/FALCON/uav_simulator/utils/multi_map_server/quadrotor_msgs/`（历史副本）。如果 catkin 报包名冲突，在副本目录下创建空 `CATKIN_IGNORE` 文件。当前未确认是否冲突，按编译报错决定。
- `mars_quadrotor_msgs` 留着无害（cascadePID 实际未用）。

---

## 5. 验证步骤

### 5.1 单跑 MARSIM，确认 mid360 点云
```bash
roslaunch test_interface single_drone_mid360.launch
rostopic hz /quad0_pcl_render_node/cloud      # 期望 ~10 Hz
rostopic echo -n1 /quad0_pcl_render_node/sensor_pose
rostopic list | grep -E "pcl_render|lidar_slam"   # 校对真实 topic 名
```
rviz：fixed_frame=`world`，加 PointCloud2，应看到 360° 半径 ~15m 的花瓣形点云随机体移动。

### 5.2 单独检查 PCD 真实尺寸
```bash
pcl_viewer /home/yyf/falcon_ws/src/MARSIM/map_generator/resource/small_forest01cutoff.pcd
pcl_viewer /home/yyf/falcon_ws/src/MARSIM/map_generator/resource/LibraryLG_01cutoff_sor.pcd
```
据此调整 `marsim_forest.yaml` / `marsim_library.yaml` 的 `map_size`、`box`、`vbox`、`init_*`。

### 5.3 合跑探索
```bash
roslaunch exploration_manager exploration_marsim.launch map_name:=marsim_forest
# 另开终端
roslaunch exploration_manager rviz.launch
```
关键日志（顺序）：
- `[MapServer] Voxel mapping server initialized`
- `[Transformer] Subscribe from transformer topic: /quad0_pcl_render_node/sensor_pose`
- 不应持续刷屏 `[Transformer] No match found for transform timestamp`
- `[MapServer] Received pointcloud with stamp:`
- `[ExplorationFSM] state: PLAN_TRAJ`
- `[traj_server] Receive new trajectory`

rviz 检查：
- `voxel_mapping/occupancy_grid` 在飞机周围逐渐扩张
- `voxel_mapping/tsdf` 表面与 MARSIM 点云吻合
- `frontier_finder/frontier_clusters` 出现 marker
- 飞机轨迹绕开树木/书架

### 5.4 失败回退矩阵

| 现象 | 检查点 |
|---|---|
| TSDF 一直为空 | `pose_topic_type` 是否改成 `pose`；`raycast_max` 是否生效；rosgraph 看 `/voxel_mapping/pointcloud` 的发布/订阅是否连通 |
| `pos_cmd` 发出但飞机不动 | cascadePID 的 `~position_cmd` 实际订阅名是否就是 `/quad_0/planning/pos_cmd`；traj_server 的 remap 是否生效 |
| 飞机起飞后漂移 | cascadePID 的 `init_state_*` 与 FALCON map config 的 `init_x/y/z` 是否一致 |
| 时间戳不匹配警告 | `transformer.timestamp_tolerance` 从 `0.001` 放宽到 `0.05` |
| rviz fixed_frame 报 `/map` 缺失 | `rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 world map`（注意去掉前导斜杠） |

---

## 6. 关键风险与待验证假设

1. **节点名空间**：MARSIM 的 `pcl_render_node` 节点名实际是 `quad0_pcl_render_node`（无下划线分隔），`~cloud` 解析为 `/quad0_pcl_render_node/cloud`。**必须 5.1 实测确认**后再写 launch。
2. **`sensor_pose` 频率**：必须 ≥ 点云速率，否则 transformer 队列空导致丢帧。如偏慢，扩大 `timestamp_tolerance`。
3. **空 `map_file` 的解析**：MapConfig 加载若 assert 非空则启动失败。若复现，回退方案是填一个真实 STL（如 `complex_office.stl`），但**不启动 map_render_node**——loadMap 只在 globalMapCallback 触发，无副作用。
4. **PCD 真实包围盒**：5.2 步必做，否则 box 范围不匹配会让 frontier_finder 立即终止或越界。
5. **`concurrent_depth_input_max` 调参**：mid360 单帧点云大，TSDF inputPointCloud 可能 >100ms。看 `[MapServer] inputPointCloud thread time too long` 警告决定是否再调高（map_server.cpp:281）。
6. **多份 quadrotor_msgs 包**：编译失败再处理（CATKIN_IGNORE）。

---

## Critical Files

- `src/FALCON/falcon_planner/exploration_manager/launch/exploration.launch` — 参照模板
- `src/FALCON/falcon_planner/exploration_manager/launch/exploration_marsim.launch` — 新建
- `src/FALCON/falcon_planner/exploration_manager/launch/marsim_mid360_single.launch` — 新建（fork MARSIM）
- `src/FALCON/falcon_planner/exploration_manager/config/map/marsim_forest.yaml` — 新建
- `src/FALCON/falcon_planner/exploration_manager/config/map/marsim_library.yaml` — 新建
- `src/FALCON/falcon_planner/exploration_manager/config/map/complex_office.yaml` — 参照模板
- `src/FALCON/falcon_planner/voxel_mapping/config/voxel_mapping.yaml` — 修改 4 个字段
- `src/FALCON/falcon_planner/voxel_mapping/src/map_server.cpp:298` — 阅读，无需修改
- `src/FALCON/falcon_planner/exploration_utils/src/transformer/transformer.cpp:124` — 阅读，无需修改
- `src/MARSIM/test_interface/launch/single_drone_mid360.launch` — fork 来源
- `src/MARSIM/test_interface/launch/single_drone.xml` — fork 来源
- `src/MARSIM/local_sensing/src/pointcloud_render_node.cpp:1746,1842,1956,1994` — 输出 frame/topic 参考
- `src/MARSIM/cascadePID/src/cascadePID_node.cpp:7,42` — 已确认接收 `quadrotor_msgs::PositionCommand`
- `src/FALCON/README.md` — 末尾追加 MARSIM 模式说明

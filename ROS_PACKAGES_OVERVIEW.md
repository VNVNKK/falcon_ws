# ROS 包作用说明

本文档按当前仓库中的 `package.xml` 统计 ROS 包，共 34 个。

其中 7 个包目录下存在 `CATKIN_IGNORE`，在当前工作区不会参与 `catkin_make`，主要原因是它们与 FALCON 自带工具包同名，直接同时编译会造成 catkin 包名冲突。

## 当前工作区结构

- `src/FALCON/falcon_planner`：FALCON 探索、建图、路径搜索和轨迹生成核心。
- `src/FALCON/uav_simulator`：FALCON 自带的仿真、控制、可视化和消息工具包。
- `src/MARSIM`：MARSIM 地图、LiDAR 感知、无人机动力学、级联 PID 控制和示例接口。
- `src/MARSIM/Utils`：MARSIM 附带的通用工具包；当前多数与 FALCON 工具包同名，已被 `CATKIN_IGNORE` 屏蔽。

## 构建状态说明

- **参与编译**：目录下没有 `CATKIN_IGNORE` 的包，catkin 会正常发现并尝试编译。
- **已屏蔽**：目录下存在 `CATKIN_IGNORE` 的包，catkin 不会把它当作当前工作区的包处理。
- **依赖可选硬件/库**：`mesh_render` 需要 Open3D，`pointcloud_render` 需要 CUDA；如果本机缺少这些依赖，相关编译错误可以按你的要求忽略，或在构建时临时黑名单屏蔽。

常用的避开 CUDA/Open3D 的构建命令：

```bash
catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_BLACKLIST_PACKAGES="mesh_render;pointcloud_render;map_render"
```

## FALCON 规划与建图包

| ROS 包名 | 路径 | 状态 | 作用 |
| --- | --- | --- | --- |
| `exploration_manager` | `src/FALCON/falcon_planner/exploration_manager` | 参与编译 | FALCON 探索任务的顶层调度包。主要节点是 `exploration_node`，负责探索状态机、调用前沿/网格预处理、触发路径规划和轨迹生成。当前新增的 MARSIM Mid360 集成 launch 和地图配置也放在这个包中。 |
| `exploration_preprocessing` | `src/FALCON/falcon_planner/exploration_preprocessing` | 参与编译 | 探索前处理库。包含 `frontier_finder`、`hierarchical_grid`、`connectivity_graph`，用于从体素地图里提取前沿、组织多层网格、维护区域连通关系，供 `exploration_manager` 决策使用。 |
| `exploration_utils` | `src/FALCON/falcon_planner/exploration_utils` | 参与编译 | 探索公共算法库。包含感知模型、raycast、坐标变换、计时/系统信息工具，以及 LKH TSP、SOP 求解器接口，用于视点评估和访问顺序优化。 |
| `fast_planner` | `src/FALCON/falcon_planner/fast_planner` | 参与编译 | 局部轨迹规划包。提供 `fast_planner` 库和 `traj_server` 节点；`traj_server` 订阅 `planning/bspline`，输出 `quadrotor_msgs/PositionCommand` 等轨迹跟踪命令。 |
| `pathfinding` | `src/FALCON/falcon_planner/pathfinding` | 参与编译 | 路径搜索库。主要实现 A* 搜索和路径代价评估，依赖 `voxel_mapping` 提供的地图查询能力，供探索和局部规划选择可行路径。 |
| `trajectory` | `src/FALCON/falcon_planner/trajectory` | 参与编译 | 轨迹表达与优化库。提供 B-spline、多项式轨迹、轨迹优化等功能，并定义 `trajectory/Bspline.msg`；同时带有 `trajectory_visualizer` 可视化节点。 |
| `voxel_mapping` | `src/FALCON/falcon_planner/voxel_mapping` | 参与编译 | 体素地图包。负责维护 TSDF/ESDF/占据栅格地图，接收深度图或点云以及传感器位姿，向规划模块提供碰撞检测、距离场、未知空间和地图边界查询。 |

## FALCON 仿真、控制与工具包

| ROS 包名 | 路径 | 状态 | 作用 |
| --- | --- | --- | --- |
| `mesh_render` | `src/FALCON/uav_simulator/camera_sensing/mesh_render` | 参与编译，但依赖 Open3D | 基于 mesh 的场景渲染库和 `mesh_render_node`。用于从三维网格场景生成相机/深度感知数据；本机没有 Open3D 时会编译失败。 |
| `pointcloud_render` | `src/FALCON/uav_simulator/camera_sensing/pointcloud_render` | 参与编译，但依赖 CUDA | 点云/深度渲染包。包含 CUDA 深度渲染库 `depth_render_cuda`、`pointcloud_render` 库和 `pcl_render_node`；本机没有 CUDA 时会编译失败。 |
| `map_render` | `src/FALCON/uav_simulator/map_render` | 参与编译，但依赖渲染包 | FALCON 仿真地图渲染封装。提供 `map_render_node`、`pointcloud_modifier`、`click_map`，通常依赖 `mesh_render` 和 `pointcloud_render`，所以缺少 Open3D/CUDA 时也建议一起黑名单屏蔽。 |
| `poscmd_2_odom` | `src/FALCON/uav_simulator/poscmd_2_odom` | 参与编译 | 简化仿真接口。节点名 `odom_generator`，订阅位置命令 `command`，发布 `odometry`，用于把规划输出转换成仿真里可用的里程计反馈。 |
| `so3_control` | `src/FALCON/uav_simulator/so3_control` | 参与编译 | SO(3) 飞控控制器。提供 `SO3Control` 库和 `so3_control_nodelet`，将期望位置/姿态等控制目标转换为四旋翼控制指令。 |
| `so3_disturbance_generator` | `src/FALCON/uav_simulator/so3_disturbance_generator` | 参与编译 | 扰动与噪声注入节点。订阅 `odom`，发布 `noisy_odom`、`correction`、`force_disturbance`、`moment_disturbance`，用于测试控制器和规划在噪声/外力扰动下的表现。 |
| `so3_quadrotor_simulator` | `src/FALCON/uav_simulator/so3_quadrotor_simulator` | 参与编译 | FALCON 自带四旋翼动力学仿真。提供 `quadrotor_dynamics` 库和 `quadrotor_simulator_so3` 节点，订阅控制输入和扰动，发布 `odom`、`imu`。 |
| `cmake_utils` | `src/FALCON/uav_simulator/utils/cmake_utils` | 参与编译 | CMake 辅助包。主要提供工程内复用的 CMake 配置/宏，不是运行时节点包。 |
| `multi_map_server` | `src/FALCON/uav_simulator/utils/multi_map_server` | 参与编译 | 多地图消息和可视化工具包。定义 `MultiOccupancyGrid`、`SparseMap3D`、`MultiSparseMap3D`、`VerticalOccupancyGridList` 等消息，并提供 `multi_map_visualization` 节点。 |
| `quadrotor_msgs` | `src/FALCON/uav_simulator/utils/quadrotor_msgs` | 参与编译 | 当前工作区实际使用的四旋翼消息包。定义 `PositionCommand`、`SO3Command`、`TRPYCommand`、`Odometry`、`LQRTrajectory`、`PolynomialTrajectory` 等消息，是 FALCON 规划、控制和 MARSIM 控制接口之间的重要消息依赖。 |
| `odom_visualization` | `src/FALCON/uav_simulator/utils/odom_visualization` | 参与编译 | 里程计和轨迹可视化节点。节点名 `odom_visualization`，订阅 `odometry` 和 `cmd`，发布 `pose`、`path`、速度/协方差/轨迹/机体模型等 RViz 可视化 marker。 |
| `pose_utils` | `src/FALCON/uav_simulator/utils/pose_utils` | 参与编译 | 位姿数学工具库。封装 Eigen、几何消息和姿态/变换相关工具函数，供控制、可视化、路径等包复用。 |
| `rviz_plugins` | `src/FALCON/uav_simulator/utils/rviz_plugins` | 参与编译 | FALCON 使用的自定义 RViz 插件包。提供 `Goal3DTool`、`ProbMapDisplay`、`AerialMapDisplay`、`MultiProbMapDisplay`，并额外包含 FALCON 版本的 `GameLikeInput` 交互插件。 |
| `uav_utils` | `src/FALCON/uav_simulator/utils/uav_utils` | 参与编译 | 无人机通用工具库。提供坐标、姿态、消息转换和常用数学工具，供仿真、控制和接口代码复用。 |
| `waypoint_generator` | `src/FALCON/uav_simulator/utils/waypoint_generator` | 参与编译 | 航点生成节点。节点名 `waypoint_generator`，订阅 `odom`、`goal`、`traj_start_trigger`，发布 `waypoints` 和 `waypoints_vis`，用于手动目标点或预设航点序列测试。 |
| `quadrotor_msgs` | `src/FALCON/uav_simulator/utils/multi_map_server/quadrotor_msgs` | 已屏蔽 | `multi_map_server` 目录内历史遗留的另一份 `quadrotor_msgs`。包名与主消息包冲突，当前用 `CATKIN_IGNORE` 屏蔽；不应与 `src/FALCON/uav_simulator/utils/quadrotor_msgs` 同时编译。 |

## MARSIM 核心包

| ROS 包名 | 路径 | 状态 | 作用 |
| --- | --- | --- | --- |
| `map_generator` | `src/MARSIM/map_generator` | 参与编译 | MARSIM 全局地图发布包。节点 `map_pub` 从配置/PCD 地图生成或加载全局点云，并发布 `/map_generator/global_cloud`，供 LiDAR 仿真和规划建图使用。 |
| `local_sensing_node` | `src/MARSIM/local_sensing` | 参与编译 | MARSIM 局部传感器仿真包。提供 CPU 点云渲染节点和 OpenGL 渲染节点，订阅全局地图与无人机里程计，发布局部 LiDAR 点云、传感器位姿、深度图、碰撞点云等。Mid360 仿真主要依赖这个包。 |
| `mars_drone_sim` | `src/MARSIM/mars_drone_sim` | 参与编译 | MARSIM 四旋翼动力学仿真包。节点 `quadrotor_dynamics_node` 订阅电机转速 `cmd_RPM`，发布 `odom` 和 `imu`，与 `cascadePID` 组成 MARSIM 的控制闭环。 |
| `cascadePID` | `src/MARSIM/cascadePID` | 参与编译 | MARSIM 级联 PID 控制器。节点 `cascadePID_node` 订阅 `odom`、`cmd_pose` 或 `quadrotor_msgs/PositionCommand`，输出 `cmd_RPM` 给 `mars_drone_sim`。 |
| `test_interface` | `src/MARSIM/test_interface` | 参与编译 | MARSIM 示例接口和 launch 包。包含 `test_interface_node`，可将测试目标转换为 `quadrotor_msgs/PositionCommand`；也保存了单机、双机、三机以及 Mid360/Avia/OS128 等原始示例 launch。 |

## MARSIM Utils 重复包

这些包在源码中仍然存在，但当前目录下有 `CATKIN_IGNORE`，不会参与编译。原因是它们与 FALCON 侧工具包同名；当前 FALCON+MARSIM 集成选择保留 FALCON 侧版本，避免包名、消息定义和 RViz 插件冲突。

| ROS 包名 | 路径 | 状态 | 作用 |
| --- | --- | --- | --- |
| `quadrotor_msgs` | `src/MARSIM/Utils/mars_quadrotor_msgs` | 已屏蔽 | MARSIM 附带的四旋翼消息包，目录名是 `mars_quadrotor_msgs`，但 `package.xml` 中包名仍是 `quadrotor_msgs`。它比 FALCON 版本包含更多消息，如 `MincoTrajectory`、`MpcPositionCommand`、`SwarmCommand`、`SwarmOdometry` 等，但 `PositionCommand` 等核心消息与 FALCON 版本存在差异，当前不参与编译。 |
| `odom_visualization` | `src/MARSIM/Utils/odom_visualization` | 已屏蔽 | MARSIM 版本的里程计可视化节点。功能与 FALCON 版本类似，但源码和资源更偏向 MARSIM 原始示例，例如 UAV mesh/PCD 可视化、同步点云等；当前为避免同名冲突而屏蔽。 |
| `pose_utils` | `src/MARSIM/Utils/pose_utils` | 已屏蔽 | MARSIM 附带的位姿工具库。与 FALCON 版本基本同类，当前不需要同时存在两个同名包参与构建。 |
| `rviz_plugins` | `src/MARSIM/Utils/rviz_plugins` | 已屏蔽 | MARSIM 版本 RViz 插件。提供 `Goal3DTool`、`ProbMapDisplay`、`AerialMapDisplay`、`MultiProbMapDisplay`，但没有 FALCON 版本中的 `GameLikeInput` 插件；当前屏蔽。 |
| `uav_utils` | `src/MARSIM/Utils/uav_utils` | 已屏蔽 | MARSIM 附带的 UAV 工具库。功能与 FALCON 侧 `uav_utils` 相近，部分 CMake/Eigen 包含路径处理不同；当前屏蔽以避免同名冲突。 |
| `waypoint_generator` | `src/MARSIM/Utils/waypoint_generator` | 已屏蔽 | MARSIM 版本航点生成节点。功能与 FALCON 版本基本一致，当前不参与编译。 |

## 同名包取舍建议

当前集成建议保留 FALCON 侧同名包，继续屏蔽 MARSIM `Utils` 侧同名包，原因如下：

- FALCON 探索主链路依赖 FALCON 版本的 `quadrotor_msgs/PositionCommand`、`rviz_plugins/GameLikeInput` 和相关工具包接口。
- MARSIM 核心包 `map_generator`、`local_sensing_node`、`mars_drone_sim`、`cascadePID`、`test_interface` 不需要依赖整套 MARSIM `Utils` 才能完成当前 Mid360 仿真集成。
- 同时保留两份同名 ROS 包会直接造成 catkin 包名冲突；即使强行改名，也要同步修改大量消息依赖和 launch/topic 配置，风险大于收益。

如果未来要完整恢复 MARSIM 原始示例工程，可以反过来启用 MARSIM `Utils`，但那时应屏蔽或移走 FALCON 侧同名包，并重新核对 `quadrotor_msgs` 的消息字段兼容性。

## 包之间的主要运行链路

当前 FALCON+MARSIM Mid360 集成的大致链路是：

```text
map_generator
  -> 发布全局地图点云
local_sensing_node
  -> 根据全局地图和无人机 odom 生成 Mid360 局部点云/传感器位姿
voxel_mapping
  -> 接收局部点云/位姿，维护体素地图
exploration_preprocessing + exploration_utils + pathfinding
  -> 提取前沿、评估视点、搜索路径
exploration_manager
  -> 调度探索状态机并调用规划模块
fast_planner + trajectory
  -> 生成 B-spline/位置命令
cascadePID + mars_drone_sim
  -> 将位置命令转换为电机转速并仿真无人机运动
```

RViz 可视化和交互主要由 `rviz_plugins`、`odom_visualization`、`multi_map_server`、`waypoint_generator` 等工具包辅助完成。

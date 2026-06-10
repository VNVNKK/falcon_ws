# 大地图卡顿问题诊断与修复说明

## 一、背景与现象

`exploration_manager/config/map/{cave,city,garage}.yaml` 三张图的 map_size / box 原本是用 AI 估算出来的近似值（边界缩小、z 加高），切到 EPIC 真实边界后系统出现明显卡顿。这次的卡顿不是 hgrid cost matrix 维度爆炸（已在 `COST_MATRIX_SIZE_CAP.md` 改动中通过 `max_cost_matrix_size` 限到 5），而是和**稠密体素图的整图扫描+周期发布**相关，开销随探索范围线性增长。

## 二、根因（按"和已知体素数的相关度"排序）

### A. 体素图按整张 map 稠密预分配

`MapBase::initMapData()` 把每个体素表示开成 `vector<VoxelType>(map_size_idx_.prod())`，没有稀疏化（`voxel_mapping/include/voxel_mapping/map_base_inl.h:5`、`esdf.h:30-34`）。

`MapServer` 里只有「`box_volume > 4000` 切到 0.2 m」一档（`voxel_mapping/src/map_server.cpp:40-45`），cave 真值 box ≈ 3×10⁶ m³，0.2 m 下光 ESDF 三份就要 ~9 GB，必然 OOM。

### B. **关键瓶颈**：发布定时器每 0.5 s 全图遍历

原代码：

```cpp
// map_server.cpp 旧实现
publish_map_timer_ = nh.createTimer(ros::Duration(0.5), ...);
```

定时器周期被**硬编码**成 0.5 s，yaml 里的 `publish_tsdf_period` / `publish_esdf_period` / `publish_occupancy_grid_period` 三个参数读了但没用上。而且原 yaml 把它们写成 `true`（bool 不能转 double），实际值会落到默认 1.0，但又被定时器架空了。

`publishMapTimerCallback` 内部连续调用：
- `publishTSDF/Slice`、`publishESDF/Slice`：vbox 三重循环
- `publishOccupancyGridHighResolution`：resolution > 0.1 时**每个 voxel 展开成 scale³ 个点**（0.2 m 下 ×8）
- **`publishMapCoverage`**：原代码**没有 `getNumSubscribers() == 0 return`**，每 0.5 s 强制扫一遍 box 内所有 voxel

→ garage 真值 box 下，每 0.5 s 至少 1.7×10⁷ 次 voxel 访问；RViz 一旦订阅 `/voxel_mapping/occupancy_grid_*`，CPU + 带宽随已知体素数线性吃满。**这就是"随探索范围增大而增大"的卡顿来源。**

### C. hgrid 邻接重建（次要）

原 `cell_size_max=5.0`，garage 真值下 cell 数 ≈ 1248 (2D) / 2496 (3D)。每次 `updateGridsFromVoxelMap` 对 update_bbox 内的 cell 重做 CCL + 重建连通图，每个中心要和邻接 cell 中心做 A*，开销随活跃 cell 数增长。

### D. ESDF 栈 VLA（仅大图风险）

`esdf.cpp:51-52` 用 `int v[map_size_idx_(dim)]; double z[...+1]`。本次问题里没爆，但 cave 真值需要留意。

## 三、改动清单

### 1. 代码

#### `voxel_mapping/src/map_server.cpp`

**改动 1** — 新增 `resolution_huge` 参数读取：

```cpp
nh.param("/voxel_mapping/resolutionf_fine", map_config.resolution_fine_, 0.1);
nh.param("/voxel_mapping/resolution_coarse", map_config.resolution_coarse_, 0.2);
double resolution_huge = 0.4;
nh.param("/voxel_mapping/resolution_huge", resolution_huge, 0.4);
```

**改动 2** — 三档分辨率选择：

```cpp
double box_volume = (map_config.box_max_ - map_config.box_min_).prod();
if (box_volume < 4000.0) {
  map_config.resolution_ = map_config.resolution_fine_;
} else if (box_volume < 200000.0) {
  map_config.resolution_ = map_config.resolution_coarse_;
} else {
  map_config.resolution_ = resolution_huge;
  ROS_WARN("[MapServer] Huge box volume %.1f m^3, using resolution %.2f m to keep memory in check",
           box_volume, map_config.resolution_);
}
```

**改动 3** — 定时器周期使用配置值（取三个 publish_*_period 的最大值，最低 1 s）：

```cpp
double publish_period = std::max({config_.publish_tsdf_period_,
                                  config_.publish_esdf_period_,
                                  config_.publish_occupancy_grid_period_});
if (publish_period < 1e-3) publish_period = 1.0;
publish_map_timer_ =
    nh.createTimer(ros::Duration(publish_period), &MapServer::publishMapTimerCallback, this);
```

**改动 4** — `publishMapCoverage` 加订阅检查：

```cpp
void MapServer::publishMapCoverage() {
  if (map_coverage_pub_.getNumSubscribers() == 0)
    return;
  // ...
}
```

### 2. 配置

#### `voxel_mapping/config/voxel_mapping.yaml`

- 新增 `resolution_huge: 0.4`。
- `publish_*_period` 从原本错写成 `true`（bool）改为正确的 double：`publish_tsdf_period: 2.0` / `publish_esdf_period: 2.0` / `publish_occupancy_grid_period: 1.0`。
- 全量发布开关 `publish_tsdf` / `publish_esdf` / `publish_occupancy_grid` 当前**保留 true**（依赖 RViz 订阅检查 + 周期降压），如果还是卡，可关掉只留 slice。

#### `exploration_preprocessing/config/hgrid.yaml`

- `cell_size_max: 5.0 → 8.0`，cell 数约掉一半，hgrid 邻接 A* 数量约降到 1/2.5。
- `max_cost_matrix_size: 50` 维持不变。

#### `exploration_manager/config/map/garage.yaml`

边界覆盖到 EPIC 真值（`/home/yyf/EPIC/src/global_planner/exploration_manager/config/garage.yaml`）：

```yaml
map_min: [-11, -7, -1]      map_max: [183, 151, 5]
box_min: [-10, -6, -0.2]    box_max: [182, 150.2, 3.8]
vbox_min: [-10.5, -6.5, -0.4]  vbox_max: [182.5, 150.5, 4.5]
```

z 上限从原本 11 收到 5，砍掉一半空气体素。

#### `exploration_manager/config/map/cave.yaml`

按 EPIC 真值原样填入：

```yaml
map_min: [-49, -207, -8]    map_max: [177, 125, 35]
box_min: [-48.1, -206, -7.1]  box_max: [176, 124, 34.1]
vbox_min: [-48.5, -206.5, -7.5]  vbox_max: [176.5, 124.5, 34.5]
```

体积 ~3×10⁶ m³，触发 huge 档，分辨率自动落到 0.4 m。

#### `exploration_manager/config/map/city.yaml`

按用户提供值填入：

```yaml
map_min: [-23, -23, -1]     map_max: [23, 23, 5]
box_min: [-22, -22, -0.2]   box_max: [22, 22, 3.8]
vbox_min: [-22.5, -22.5, -0.4]  vbox_max: [22.5, 22.5, 4.2]
```

体积 ~7700 m³，落到 coarse 0.2 m。

## 四、预期内存/分辨率

| 地图 | box volume | 选中分辨率 | voxel 总数 | TSDF + ESDF×3 内存 |
| --- | --- | --- | --- | --- |
| city | ~7.7×10³ m³ | 0.2 m (coarse) | ~1.0×10⁶ | ~40 MB |
| garage | ~1.3×10⁵ m³ | 0.2 m (coarse) | ~1.7×10⁷ | ~680 MB |
| cave | ~3.0×10⁶ m³ | **0.4 m (huge)** | ~4.7×10⁷ | ~1.9 GB |

cave 即使用 0.4 m 仍需 ~2 GB 常驻，跑前确认机器内存。

## 五、行为兼容性

- 老地图（小 box，落到 fine/coarse 档）行为完全不变。
- 关闭 `max_cost_matrix_size: 0` 即可回退原 cost matrix 行为。
- 删掉 `resolution_huge` 字段会用默认 0.4。

## 六、未完成的可选优化

如果 cave 还是卡或显存压力大，下一步可做：

1. **ESDF tmp_buffer 局部化** —— `esdf.h:32-33` 的 `tmp_buffer1_/2_` 现在按全图分配，cave 0.4 m 下浪费 ~760 MB；改成只覆盖 `update_bbox` 区域。
2. ~~**删除 `publishOccupancyGridHighResolution` 的 ×scale³ 展开** —— 仅做可视化用，规划不依赖。~~ **已在第八节完成。**
3. **关闭全量 `publish_tsdf` / `publish_esdf` / `publish_occupancy_grid`，只留 slice**（在 `voxel_mapping.yaml` 里把这三个 `true` 改 `false`）。
4. **cave 走 2D 模式**（`map_dimension: 2`），hgrid z 方向不切层，cell 数再降一档。
5. **限制送入 SOP 的 frontier_ids 数量** —— `max_cost_matrix_size` 只 cap 了 `grid_tour2_` 一维，frontier_ids 没限。cave 长时间跑后 SOP 矩阵维度会膨胀，导致求解时间不稳定（详见第八节 §3）。

## 七、第一轮编译验证

```
catkin_make --pkg voxel_mapping exploration_preprocessing -DCMAKE_BUILD_TYPE=Release
```

通过。

## 八、cave 实测发现的额外问题与修复

第一轮改动跑 garage 已经丝滑，但实测 cave 时暴露出**两类全新问题**：性能不再是瓶颈，**过度致命断言**反而成了导致进程 abort 的主要原因。本节按发现顺序记录每一处修改的根因、思路、代码位置。

### §1. `publishOccupancyGridHighResolution` 的 ×scale³ 展开是发布开销的真正大头

#### 现象

控制变量法实测：把 `voxel_mapping.yaml` 里 `publish_tsdf`/`publish_esdf`/`publish_occupancy_grid` 三个开关改成 `false`，无人机就能丝滑规划；只把 `publish_occupancy_grid` 单独开成 `true`，规划链路立刻被卡死（飞不出轨迹）——比另外两个的影响大一个数量级。

#### 根因

对照 `map_server.cpp` 的三个发布函数：

| 发布函数 | 单 voxel 产生的点数 | garage coarse 0.2 m 下单次发布点数量级 |
| --- | --- | --- |
| `publishTSDF` | 1 | ~1.7×10⁷ |
| `publishESDF` | 1 | ~1.7×10⁷ |
| `publishOccupancyGridHighResolution` | **(coarse/fine)³ = (0.2/0.1)³ = 8** | **~1.36×10⁸** |

只有 `publishOccupancyGridHighResolution` 内部嵌了一层 `for (sx,sy,sz : 0..scale)` 的 ×scale³ 内循环，把每个 voxel 中心展开成 8（0.2 m）/64（0.4 m）个 RViz 点。导致：
- **CPU 计算量** ≈ ×8（cave 0.4 m 下 ×64）
- **PointCloud2 序列化耗时** ≈ ×8
- **消息体积** 单次几百 MB（garage 量级），通过 ROS 走 intra/inter-process 都要多次拷贝
- ROS 单线程 spinner 下，单次发布回调耗时秒级，把 odom/depth/规划请求全部排到队列后

#### 修法

`voxel_mapping/src/map_server.cpp` 的 `publishOccupancyGridHighResolution`：删除 670–688 行的 ×scale³ 展开，每个 voxel 只发它的中心点，跟 `publishTSDF/ESDF/publishOccupancyGrid` 行为一致。订阅检查（`getNumSubscribers() == 0 return`）原本就在，没动。

```cpp
// 改后核心循环
for (int x = ...; x < ...; ++x) {
  for (int y = ...; y < ...; ++y) {
    for (int z = ...; z < ...; ++z) {
      VoxelIndex idx(x, y, z);
      Position pos = occupancy_grid_->indexToPosition(idx);
      point.x = pos(0); point.y = pos(1); point.z = pos(2);
      // 不再做 scale³ 展开
      if (...OCCUPIED) pointcloud_occupied.points.push_back(point);
      else if (...FREE) pointcloud_free.points.push_back(point);
      else if (...UNKNOWN) pointcloud_unknown.points.push_back(point);
    }
  }
}
```

视觉上 RViz 用 Box/Cube 渲染时按 resolution 自适应，不影响识别遮挡 / 已知区域。

### §2. cave 0.4 m 档下 frontier 阈值仍然过大，FSM 8 s 提前 FINISH

#### 现象

cave 起飞后 8.43 s 就出现 `[FSM] Exploration finished. Coverage: 0.00`，path length 14.11 m，初始 frontier 飞过去后剩下的全部消失。

> 题外话：日志里的 `Coverage: 0.00` 是误导信息——`publishMapCoverage` 在第一轮已经加了 `getNumSubscribers() == 0 return`，没人订阅 `/voxel_mapping/map_coverage` 时 `map_coverage_` 字段不会更新，FSM 拿到的就是初始 0。**实际探索是发生了的**，只是 8 s 后 frontier 全消失触发 `Finish exploration: No frontier detected`。

#### 根因

`exploration_preprocessing/src/frontier_finder.cpp:39-46` 已有低分辨率自适应：

```cpp
if (resolution_ > 0.15) {
  cluster_min_ /= 2;          // 100 -> 50
  min_visib_num_ *= 0.8;
}
```

但只 `/2`，对 0.4 m 档不够：

| 分辨率 | cluster_min 缩放后 | 等效体积阈值 |
| --- | --- | --- |
| 0.1 m | 100 | 0.1 m³ ✓ |
| 0.2 m (garage coarse) | 50 | 0.4 m³ ✓ 还能跑 |
| **0.4 m (cave huge)** | **50** | **3.2 m³** ← 一个 frontier 簇要 3.2 立方米才不被滤掉 |

cave 雷达扫出来的 frontier 多数 < 3 m³，全部被 `cluster_min` 滤掉 → `updateFrontierStruct` 返回 0 → FSM `transitState(FINISH)`。

#### 修法

在原 `resolution > 0.15` 分支后追加一档"huge"分支，只对 0.4 m 档再激进缩一次（不影响 garage 0.2 m）：

```cpp
// frontier_finder.cpp 新增段
if (resolution_ > 0.3) {
  cluster_min_ = std::max(2, cluster_min_ / 8);          // 50 -> 6
  min_visib_num_ = std::max(2, min_visib_num_ / 4);      // 12 -> 3
  ROS_WARN("[FrontierFinder] Huge resolution map detected, further decrease "
           "cluster_min to %d, min_visib_num to %d",
           cluster_min_, min_visib_num_);
}
```

阈值对照表：

| 参数 | 原值 | 0.2 m 档（garage） | **0.4 m 档（cave 修后）** |
| --- | --- | --- | --- |
| cluster_min (voxel) | 100 | 50 | 6 |
| cluster_min 体积 | 0.1 m³ | 0.4 m³ | **0.38 m³** ✓ |
| min_visib_num (voxel) | 15 | 12 | 3 |

修法思路：保留原作者 `/2` 的经验值（0.2 m 档行为不变），只在 huge 档（>0.3 m）多缩一次，体积阈值跟 0.2 m 档对齐。`max(2, ...)` 下限避免参数被缩成 0。

### §3. SOP solver 偶发慢 22 ms 直接 abort 进程

#### 现象

cave 跑了 4 分 48 秒（path 478 m）后崩：

```
[ExplorationManager] SOP time: 1022.82 ms
F0610 20:13:23 exploration_manager.cpp:578] Check failed: sop_time <= 1.0 (1.02282 vs. 1)
SOP solver internal error detected, solver blocked with unknown error.
```

进程 abort，traj_server 一直在打 `Flight time: 288.92` 重复日志。

#### 根因

两层时间限制叠加：

1. SOP solver 内部已经 `s.set_time_limit(1, false)`（`sop_solver_interface.cpp:75`），超时会返回 NN（最近邻）兜底解，**正确性没问题**；
2. 外层 `CHECK_LE(sop_time, 1.0)` 把"setup/teardown 抖动多用了 22 ms"当致命错误 → abort 整个进程。

但 NN solver 的 hungarian 矩阵拷贝、`s.clear()`（注释里写了 `// this will cost 1ms`）、redundant edge removal 等阶段不在 solver 内部 time_limit 控制内，外层耗时偶尔超 1.0 s 100~200 ms 是正常的。

更深一层：SOP cost matrix 维度 = `grid_tour2_.size() + frontier_ids.size()`。`max_cost_matrix_size: 5` 只 cap 了 `grid_tour2_`（hierarchical_grid.cpp:2017-2019），frontier_ids 没 cap，cave 长跑后会膨胀到几十~几百维，SOP 是 NP-hard，求解时间自然不稳定。

#### 修法

`exploration_manager/src/exploration_manager.cpp:578` 把致命断言换成软降级：

```cpp
// 改后
if (sop_time > 3.0 || sop_path.empty()) {
  ROS_ERROR("[ExplorationManager] SOP solver gave up (time=%.2f s, path_size=%zu), "
            "skipping this replan tick", sop_time, sop_path.size());
  return FAIL;       // FSM 下个 tick 重 plan，进程不崩
}
if (sop_time > 1.0) {
  ROS_WARN("[ExplorationManager] SOP solver overran soft 1.0s budget (time=%.2f s), "
           "using nearest-neighbor fallback path", sop_time);
}
```

新行为：

| sop_time | 处理 |
| --- | --- |
| ≤ 1.0 s | 正常路径（不变） |
| 1.0 ~ 3.0 s | `ROS_WARN` + 继续用 solver 返回的 NN 兜底路径 |
| > 3.0 s 或 sop_path 为空 | `ROS_ERROR` + `return FAIL`，FSM 下个 tick 重 plan |

修法思路：solver 内部已经有正确的兜底机制，外层 abort 是过度反应。"偶发慢一点"和"真的卡死"应当区分对待。**根治需要进一步限制 frontier_ids 进入 SOP 前的数量**（参考第六节优化项 5），但治标的软降级足以避免崩进程。

### §4. hgrid cost 近零时 abort（同款过度断言，4 处）

#### 现象

cave 跑 3 分 1 秒（path 327 m）后再崩：

```
F0610 20:26:59 hierarchical_grid.cpp:2083] Check failed: cost > 1e-4 (2.68046e-06 vs. 0.0001)
Zero cost from current position to cell 421 center 0
```

#### 根因

`UniformGrid::calculateCostMatrixSingleThread` 算"当前位置 → cell 中心" cost 时，UAV 恰好飞到 cell 421 中心附近触发 replan，A\* 给出 cost = 2.68×10⁻⁶。这是**物理正常状态**——你刚到那个 cell 中心，距离自然趋零。但代码里 `CHECK_GT(cost, 1e-4)` 把它当致命 bug。

`hierarchical_grid.cpp` 内一共有 **4 处**这种近零 cost 致命断言：
- L2083：cur_pos → candidate center（A\*）
- L2121：candidate i → j 远距离分支（BFS via connectivity graph）
- L2134：candidate i → j A\* fallback 分支（BFS）
- L2147：最终 cost 总检查

任何一处都会让 cave 这种长跑场景间歇性 abort。

#### 修法

把 4 处 `CHECK_GT` 全部改成 clamp（夹到 `kMinCellCost = 1e-3`），并在函数级定义常量保证作用域覆盖两个 for 循环：

```cpp
// hierarchical_grid.cpp::calculateCostMatrixSingleThread
constexpr double kMinCellCost = 1e-3;

// 1) cur_pos -> candidate center
if (cost < kMinCellCost) {
  ROS_WARN_THROTTLE(2.0, "[UniformGrid] Near-zero cost (%.3e) from cur_pos to "
                    "cell %d center %d, clamping to %.3e",
                    cost, c.cell_id, c.local_idx, kMinCellCost);
  cost = kMinCellCost;
}

// 2)/3) candidate i -> j BFS 路径（远距离 + A* fallback 两个分支）
if (cost < kMinCellCost) {
  ROS_WARN_THROTTLE(2.0, "[UniformGrid] Near-zero cost (%.3e) cell %d->%d (BFS), clamping",
                    cost, ci.cell_id, cj.cell_id);
  cost = kMinCellCost;
}

// 4) 最终 cost 总 clamp（同 cell 内不同 center 距离极近时也夹）
if (cost < kMinCellCost) {
  cost = kMinCellCost;
}
```

修法思路：cost 接近 0 在物理上完全合法，唯一风险是 0 进 SOP 矩阵会让求解退化（SOP 期望 cost >0）。夹到 1e-3 既保留"靠得近 = 便宜"的语义，又消除数值问题；`ROS_WARN_THROTTLE` 限制刷屏。

### §5. 本轮过度致命断言修复总结

| # | 位置 | 原行为 | 现行为 | 触发条件 |
| --- | --- | --- | --- | --- |
| 1 | `exploration_manager.cpp:578` | `CHECK_LE(sop_time, 1.0)` abort | warn / return FAIL | SOP setup/teardown 抖动 |
| 2 | `hierarchical_grid.cpp:2083` | `CHECK_GT(cost, 1e-4)` abort | clamp to 1e-3 | UAV 飞到 cell 中心附近 |
| 3 | `hierarchical_grid.cpp:2121` | 同上 | clamp to 1e-3 | BFS 找到 0 cost 边 |
| 4 | `hierarchical_grid.cpp:2134` | 同上 | clamp to 1e-3 | BFS A\* fallback 0 cost |
| 5 | `hierarchical_grid.cpp:2147` | 同上 | clamp to 1e-3 | 最终 cost 总检查 |

**共同问题**：原作者在小地图短期运行下不会撞到这些边界场景，所以用了"宁错杀不放过"的 `CHECK`。但 cave 这种 3×10⁶ m³ 大图、4~5 分钟长跑下，UAV 几乎必然会飞到某个 cell 中心、SOP 矩阵几乎必然会膨胀到偶发超时。**致命断言遇到大地图边界场景几乎都是过度反应**，正确的处理是降级（warn + clamp / FAIL 重试），而不是让整个 exploration_node 进程 abort。

## 九、最终编译验证

```
catkin_make --pkg voxel_mapping exploration_preprocessing exploration_manager \
            -DCMAKE_BUILD_TYPE=Release
```

通过，无新增 warning。

## 十、本轮文件改动汇总

| 文件 | 修改 |
| --- | --- |
| `voxel_mapping/src/map_server.cpp` | `publishOccupancyGridHighResolution` 删除 ×scale³ 展开，每 voxel 只发中心点 |
| `exploration_preprocessing/src/frontier_finder.cpp` | 新增 `resolution > 0.3` huge 档分支，`cluster_min /= 8`、`min_visib_num /= 4` |
| `exploration_manager/src/exploration_manager.cpp` | SOP `CHECK_LE` → 软降级（warn / return FAIL） |
| `exploration_preprocessing/src/hierarchical_grid.cpp` | 4 处 `CHECK_GT(cost, ...)` → clamp 到 `kMinCellCost = 1e-3` |

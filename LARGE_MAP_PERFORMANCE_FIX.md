# 大地图卡顿问题诊断与修复说明

## 一、背景与现象

`exploration_manager/config/map/{cave,city,garage}.yaml` 三张图的 map_size / box 原本是用 AI 估算出来的近似值（边界缩小、z 加高），切到 EPIC 真实边界后系统出现明显卡顿。这次的卡顿不是 hgrid cost matrix 维度爆炸（已在 `COST_MATRIX_SIZE_CAP.md` 改动中通过 `max_cost_matrix_size` 限到 50），而是和**稠密体素图的整图扫描+周期发布**相关，开销随探索范围线性增长。

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
2. **删除 `publishOccupancyGridHighResolution` 的 ×scale³ 展开** —— 仅做可视化用，规划不依赖。
3. **关闭全量 `publish_tsdf` / `publish_esdf` / `publish_occupancy_grid`，只留 slice**（在 `voxel_mapping.yaml` 里把这三个 `true` 改 `false`）。
4. **cave 走 2D 模式**（`map_dimension: 2`），hgrid z 方向不切层，cell 数再降一档。

## 七、编译验证

```
catkin_make --pkg voxel_mapping exploration_preprocessing -DCMAKE_BUILD_TYPE=Release
```

通过。

# 代价矩阵规模上限改动说明

## 一、背景与问题

在 `exploration_manager.cpp::planExploreMotionHGrid` 中，全局覆盖路径（CP）依赖：

1. `HierarchicalGrid::calculateCostMatrix2`：根据当前位姿与所有 active cell 的 free / unknown CCL 中心，构建两两可达代价矩阵 `cost_matrix2`；
2. `ExplorationManager::solveTSP`：对该矩阵调用 LKH 求解 ATSP，得到访问顺序，作为后续 SOP 与局部 refine 的输入。

**根因**：`calculateCostMatrix2` 实际转调 `UniformGrid::calculateCostMatrixSingleThread`，原实现把 **每个 cell 的 `centers_free_active_` + `centers_unknown_active_`** 全部累加为 tour points，矩阵维度

```
N = 1 + Σ_cell ( |centers_free_active_| + |centers_unknown_active_| )
```

没有任何上限。

代价矩阵填充开销是 O(N²)，每个元素还要执行 `getAStarCostYaw`（A\* + yaw 估计）或 `searchConnectivityGraphBFS`（连通图 BFS），LKH 还要在该矩阵上求解 ATSP。

在大地图（如 `city`）+ 较细的 CCL 切片 + 多 cell 多 center 情况下，N 可飙升到 10³~10⁴，导致：

- `[ExplorationManager] Hgrid cost matrix 2 time` 与 `TSP 2 time` 同时爆表；
- FSM 的 `[FSM] Total time too long!` 频繁触发，规划器无法保持实时。

---

## 二、改动思路

核心思路：**给代价矩阵维度设一个硬上限，超出时按到当前位姿的欧氏距离取最近 K 个候选中心**，把矩阵控制在可控规模内。

理由：

- 代价矩阵的目的是规划「下一步该往哪个 cell/center 去」，全局 CP 在 FSM 里以 ~10Hz 频率重规划，**远端 tour points 自然会在后续 replan 周期被覆盖**，无需在每个周期都纳入。
- 欧氏距离作为预过滤是 O(N)，远比 O(N²) 的 A\*/BFS 便宜；
- `std::nth_element` 选 K-nearest 是 O(N) 平均复杂度，预过滤本身几乎不增加开销。

为保持原有行为可回退，保留「无上限」这一选项（参数设为 0 即可）。

---

## 三、具体改动

涉及文件：

```
src/FALCON/falcon_planner/exploration_preprocessing/
├── include/exploration_preprocessing/hierarchical_grid.h   (Config 字段)
├── src/hierarchical_grid.cpp                               (核心实现 + 参数读取)
└── config/hgrid.yaml                                       (暴露参数)
```

### 1. `hierarchical_grid.h`

在 `UniformGrid::Config` 与 `HierarchicalGrid::Config` 中各新增一个字段：

```cpp
int max_cost_matrix_size_; // Max dimension for the cost matrix (0 = unlimited)
```

### 2. `hierarchical_grid.cpp::UniformGrid::calculateCostMatrixSingleThread`

完全重写其骨架：

1. **Flatten 候选中心**：遍历 `uniform_grid_`，把每个 cell 的 free / unknown active 中心连同元信息 (`cell_id`、`local_idx`、`is_free`、`active_idx`、到 `cur_pos` 的欧氏距离) 收集到 `std::vector<CandidateCenter> candidates`。其中：
   - `local_idx`：在该 cell 内 `(centers_free_active_ ‖ centers_unknown_active_)` 拼接后的下标，与原来 `cost_mat_id_to_cell_center_id` 的语义一致；
   - `active_idx`：原本用来构造连通图节点 id `cell_id*10 + active_idx (+free 数)`，这里完整保留以兼容 CG 查询。

2. **K-nearest 裁剪**：当 `candidates.size() > max_cost_matrix_size_ - 1` 时（保留下标 0 给当前位姿），使用

   ```cpp
   std::nth_element(begin, begin + keep, end,
                    [](a, b){ return a.dist_to_cur < b.dist_to_cur; });
   candidates.resize(keep);
   ```

   只保留最近的 K 个。触发时输出 `[UniformGrid] Cost matrix candidates capped to ... ` 警告。

3. **稳定排序**：再按 `(cell_id, local_idx)` 升序 `std::sort`，使 `cost_mat_id_to_cell_center_id` 在同一帧内部、跨帧之间都保持稳定的索引语义，避免下游 `getLayerCellCenters(cell_id, center_id)` 等接口在裁剪后取错位置。

4. **构建矩阵与映射**：
   - 用一个统一的 lambda `buildCgNodeId(c, cell)` 复刻原本「按 free/unknown + active_idx 决定 CG 节点 id 与 `no_cg_search` 标记」的逻辑；
   - 第 0 列：`cur_pos → c`，free 中心走 `hybrid_search_radius_` + A\*，unknown 中心保持原有 `(1000 + dist) × unknown_penalty_factor_` 重罚；
   - 中心间：先用欧氏距离判断是否走 hybrid search，长距离走 CG-BFS，短距离走 A\*；A\* 不可达 (cost > 499) 时按原逻辑回落到 CG-BFS，CG 也不可用时回落到 `1000 + dist`；至少一个端点是 unknown 时整体乘 `unknown_penalty_factor_`。

   所有 `CHECK_GT` 健全性检查、原有的零代价 dump 调试输出（去掉为减少噪声）等行为均与旧实现等价。

5. **维度**：`dim = 1 + candidates.size()`，矩阵 `cost_matrix = MatrixXd::Zero(dim, dim)`，仅当裁剪发生时小于原维度。

### 3. 参数读取

`HierarchicalGrid` 构造函数新增：

```cpp
nh.param("/hgrid/max_cost_matrix_size", config_.max_cost_matrix_size_, 300);
```

并在为每层 `UniformGrid` 构造 `ug_config` 时透传：

```cpp
ug_config.max_cost_matrix_size_ = config_.max_cost_matrix_size_;
```

### 4. `config/hgrid.yaml`

```yaml
hgrid:
  num_levels: 1
  cell_size_max: 5.0
  min_unknown_num_scale: 0.1
  ccl_step: 2
  max_cost_matrix_size: 300   # 新增：代价矩阵维度上限，0 表示不限制
  verbose: false
```

---

## 四、可配置参数

| 参数 | 路径 | 类型 | 默认值 | 说明 |
| --- | --- | --- | --- | --- |
| `max_cost_matrix_size` | `/hgrid/max_cost_matrix_size`（YAML 中位于 `hgrid:` 顶层） | `int` | `300` | 代价矩阵的最大维度（含「当前位置」这一行/列）。当候选 tour points 数量超过 `max_cost_matrix_size - 1` 时，按到当前位姿的欧氏距离保留最近的若干个。设为 `0` 表示**不做裁剪**，即恢复原始无上限行为。 |

### 调参建议

- **大地图 + 实时性敏感**（如 city、garage、power_plant）：保持默认 `300`，或下调到 `200`，可显著降低 cost matrix + TSP 总耗时。
- **小地图**（box 体积 < 1000 m³）：原代码已自动放宽 `hybrid_search_radius_`，矩阵规模本就不大，本参数基本不会触发；建议留默认值。
- **CPU 强、希望全局最优**：可上调到 `500 ~ 800`，但每次 replan 的耗时会随 N² 增长。
- **想退回旧行为做对比测试**：设 `max_cost_matrix_size: 0`。

### 触发可观察日志

裁剪发生时会打印：

```
[UniformGrid] Cost matrix candidates capped to <K> (max_cost_matrix_size=<N>).
Far-away tour points will be considered in later replans.
```

可配合原有的：

```
[ExplorationManager] Hgrid cost matrix 2 time: ... ms, TSP 2 time: ... ms
[ExplorationManager] Hgrid tour 2 cost matrix size: ...
```

来评估是否还需要继续下调上限。

---

## 五、行为兼容性

- 当 `candidates.size() ≤ max_cost_matrix_size - 1` 时，新实现输出的矩阵维度与原实现完全一致；只是内部多了一次稳定排序，维度小时无可感开销。
- `cost_mat_id_to_cell_center_id` 仍然是 `mat_idx → (cell_id, local_idx)`，下游 `ExplorationManager::planExploreMotionHGrid`、`initializeHierarchicalGrid` 通过 `getLayerCellCenters(0, cell_id, center_id)` 等接口的取值方式无需改动。
- `cost_matrix_sop` 的精度约束（`< 10000`）、`unknown_penalty_factor_`、`hybrid_search_radius_` 等所有外部参数语义保持一致。
- 设 `max_cost_matrix_size: 0` 即可一键关闭新逻辑，**完全等价于原来的无上限实现**（仅多了一次 flatten + sort）。

---

## 六、编译验证

```
catkin_make --pkg exploration_preprocessing -DCMAKE_BUILD_TYPE=Release   # OK
catkin_make --pkg exploration_manager      -DCMAKE_BUILD_TYPE=Release   # OK
```

均通过，仅有原有的与本改动无关的 CMake 弃用 / 命名警告。

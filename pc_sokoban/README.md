# 推箱子算法 PC 验证器

完整复现 `project/code/algo_sokoban_solver.c` 的 Python 版本，带图形界面。

## 运行

```bash
cd pc_sokoban
python sokoban_gui.py
```

命令行版：`python sokoban_validator.py`

单元测试：`python test_validator.py`

## 功能

| 功能 | 说明 |
|------|------|
| **Stage1** | 贪心：最近箱→最近目标 |
| **Stage2** | 指定箱→目标映射；**先侦查访问所有箱子**再推箱 |
| **Stage3** | 炸弹推入墙体、**3×3 炸墙**、再推剩余箱子；含侦查 |
| **随机地图** | 可调箱子数、内墙密度、随机种子 |
| **导入/导出** | ASCII 地图：`# - $ . * @`（与主控 `app_link.c` 一致） |
| **指令验证** | 粘贴 `SCOUT` / `PUSH` / `BOMB` 段，模拟并判定通关 |

## 地图格式（12 行 × 16 列）

```
################
#---@---$------#
#......$......#
...
```

- `#` 墙  `-` 空地  `.` 目标  `$` 箱子  `*` 炸弹  `@` 玩家起点

## 运行指令格式

```text
# 侦查（Stage2/3 必访所有箱子侧面观察点）
SCOUT: W7,5;W9,4
# 或方向串（仅小写行走）
SCOUT: rrrd

# 推炸弹（Stage3）
BOMB: rrrdL

# 推箱（小写走 大写推）
PUSH: rrrUdddLL
```

路点写法：`W列,行` 或 `W列 行`，多个用 `;` 或空格分隔。

## 与固件对应关系

| Python | C |
|--------|---|
| `nav_bfs` | `Algo_Nav_BFS` |
| `find_observe_point_for_box` | `Algo_Find_Nearest_Box_Observe_Point` |
| `sokoban_bfs_single` | `sokoban_bfs_single` |
| `solve_stage1/2/3` | `Sokoban_Solve_Stage*` |
| `apply_bomb_explosion` | `Sokoban_Apply_Bomb_Explosion` |

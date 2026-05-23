# 推箱子算法 PC 验证器

完整复现 `project/code/algo_sokoban_solver.c` 的 Python 版本，带图形界面。

## 运行

```bash
cd pc_sokoban
python sokoban_gui_pro.py   # 视觉升级版 (推荐)
python sokoban_gui.py       # 简化版 (旧)
```

命令行版：`python sokoban_validator.py`

单元测试：`python test_validator.py`

## PRO 版亮点 (`sokoban_gui_pro.py`)

| 升级项 | 说明 |
|---|---|
| **动态路径可视化** | 玩家完整路径分段着色（已走/未走/侦查/炸弹），推箱路径粗箭头 |
| **平滑插值动画** | 每步 5 帧 `smoothstep` 缓动，玩家与箱子同步滑行 |
| **精美渲染** | 砖墙纹理、目标脉动发光环、箱子到位金光 ★、玩家朝向箭头 |
| **爆炸特效** | 推炸弹撞墙触发 8 帧扩散闪光 |
| **子任务进度卡片** | 实时显示 SCOUT / PUSH / BOMB 各段状态（待执行 ● / 进行中 ◉ / 已完成 ✓）|
| **现代分组布局** | 顶部状态徽章条 + 中央大画布 + 右侧 Notebook + 底部彩色日志 |
| **悬浮提示** | 鼠标悬停显示格子坐标与内容 |

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

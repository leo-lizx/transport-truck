"""
sokoban_validator.py — 推箱子算法 PC 端验证器
=================================================
完整复现 algo_sokoban_solver.c 的 Python 版本，用于在 PC 上验证
地图求解、动画模拟、通关判断。

运行方式:
    python sokoban_validator.py

功能:
  - 自动生成随机地图（可指定种子）
  - 支持三阶段模式（Stage1 时间优化 / Stage2 指定映射 / Stage3 炸弹）
  - 调用推宏 A* 求解器，输出完整方向指令字符串
  - 逐步动画模拟，区分"行走"与"推箱"动作
  - 对每步路径进行合法性校验
  - 通关条件实时判断
"""

import os
import sys
import time
import random
import copy
import heapq
import math
from collections import deque
from typing import Optional, List, Tuple, Dict

# ============================================================
# 地图常量（与 C 代码一致）
# ============================================================
MAP_ROWS = 12
MAP_COLS = 16

INNER_R_MIN = 1
INNER_R_MAX = 10   # CHASSIS_GRID_MAX_Y - 1
INNER_C_MIN = 1
INNER_C_MAX = 14   # CHASSIS_GRID_MAX_X - 1

EMPTY  = 0
WALL   = 1
TARGET = 2
BOX    = 3
BOMB   = 4

MAX_BOXES = 8

# 单次推箱最大步数（镜像 C 端 algo_sokoban_solver.h::SOKOBAN_MAX_ACTIONS）
SOKOBAN_MAX_ACTIONS = 500

# 方向偏移：UP=0, DOWN=1, LEFT=2, RIGHT=3
DR = (-1,  1,  0,  0)   # row 偏移
DC = ( 0,  0, -1,  1)   # col 偏移

DIR_ARROW  = ('↑', '↓', '←', '→')
DIR_LETTER = ('U', 'D', 'L', 'R')   # 大写=推箱，小写=行走

# 地图字符（Unicode 模式）
CELL_UNICODE = {
    EMPTY:  '·',   # 空地
    WALL:   '█',   # 墙
    TARGET: '◎',   # 目标点
    BOX:    '□',   # 箱子
    BOMB:   '◆',   # 炸弹
}
BOX_ON_TARGET_CHAR  = '■'   # 箱子已在目标上
PLAYER_CHAR         = '@'
PLAYER_ON_TARGET    = '⊕'

# ASCII 回退
CELL_ASCII = {
    EMPTY:  '.',
    WALL:   '#',
    TARGET: 'O',
    BOX:    'B',
    BOMB:   'X',
}
BOX_ON_TARGET_ASCII  = '*'
PLAYER_ASCII         = '@'
PLAYER_ON_TARGET_ASCII = '+'

USE_UNICODE = True  # 运行时自动探测


def _detect_unicode() -> bool:
    """探测终端是否支持 Unicode 输出。"""
    try:
        sys.stdout.write('█')
        sys.stdout.flush()
        sys.stdout.write('\r \r')
        sys.stdout.flush()
        return True
    except Exception:
        return False


# ============================================================
# 内场边界检查
# ============================================================

def is_inner(r: int, c: int) -> bool:
    return INNER_R_MIN <= r <= INNER_R_MAX and INNER_C_MIN <= c <= INNER_C_MAX


def is_free(the_map: list, r: int, c: int) -> bool:
    """可通行：内场且为空地/目标点。"""
    if not is_inner(r, c):
        return False
    return the_map[r][c] in (EMPTY, TARGET)


# ============================================================
# 元素提取
# ============================================================

def extract_elements(the_map: list, elem_type: int) -> list:
    """提取地图中所有指定类型元素的坐标列表 [(row, col), ...]。"""
    result = []
    for r in range(INNER_R_MIN, INNER_R_MAX + 1):
        for c in range(INNER_C_MIN, INNER_C_MAX + 1):
            if the_map[r][c] == elem_type:
                result.append((r, c))
    return result


# ============================================================
# 点到点 BFS 导航
# ============================================================

def nav_bfs(the_map: list, start: tuple, end: tuple) -> Optional[list]:
    """
    点到点 BFS 寻路（不推箱）。
    返回路径坐标列表（含起终点），无解返回 None。
    对应 C 代码 Algo_Nav_BFS()。
    """
    if not (is_inner(*start) and is_inner(*end)):
        return None
    if not (is_free(the_map, *start) and is_free(the_map, *end)):
        return None
    if start == end:
        return [start]

    parent: Dict[tuple, Optional[tuple]] = {start: None}
    queue = deque([start])

    while queue:
        pos = queue.popleft()
        if pos == end:
            # 回溯路径
            path = []
            cur = pos
            while cur is not None:
                path.append(cur)
                cur = parent[cur]
            return list(reversed(path))
        r, c = pos
        for d in range(4):
            nr, nc = r + DR[d], c + DC[d]
            npos = (nr, nc)
            if is_free(the_map, nr, nc) and npos not in parent:
                parent[npos] = pos
                queue.append(npos)

    return None


NAV_TIME_MOVE_UNITS = 1
NAV_TIME_TURN_UNITS = 4


def _nav_time_search(the_map: list, start: tuple,
                     initial_direction: Optional[int] = None) -> tuple:
    """方向状态最短路，镜像固件 Algo_Nav_Time_*（代价单位 100ms）。"""
    if not is_inner(*start) or not is_free(the_map, *start):
        return {}, {}

    costs: Dict[tuple, int] = {}
    parent: Dict[tuple, Optional[tuple]] = {}
    queue = []
    for direction in range(4):
        pos = (start[0] + DR[direction], start[1] + DC[direction])
        if not is_free(the_map, *pos):
            continue
        state = (pos[0], pos[1], direction)
        cost = NAV_TIME_MOVE_UNITS
        if initial_direction is None or initial_direction != direction:
            cost += NAV_TIME_TURN_UNITS
        if cost < costs.get(state, 10 ** 18):
            costs[state] = cost
            parent[state] = None
            heapq.heappush(queue, (cost, state))

    while queue:
        cost, state = heapq.heappop(queue)
        if cost != costs.get(state):
            continue
        r, c, previous_direction = state
        for direction in range(4):
            pos = (r + DR[direction], c + DC[direction])
            if not is_free(the_map, *pos):
                continue
            next_state = (pos[0], pos[1], direction)
            next_cost = cost + NAV_TIME_MOVE_UNITS
            if direction != previous_direction:
                next_cost += NAV_TIME_TURN_UNITS
            if next_cost >= costs.get(next_state, 10 ** 18):
                continue
            costs[next_state] = next_cost
            parent[next_state] = state
            heapq.heappush(queue, (next_cost, next_state))
    return costs, parent


def nav_time_distance_flood(the_map: list, start: tuple,
                            initial_direction: Optional[int] = None) -> Dict[tuple, int]:
    costs, _ = _nav_time_search(the_map, start, initial_direction)
    result = {start: 0} if is_inner(*start) and is_free(the_map, *start) else {}
    for (r, c, _), cost in costs.items():
        pos = (r, c)
        if cost < result.get(pos, 10 ** 18):
            result[pos] = cost
    return result


def nav_time_path(the_map: list, start: tuple, end: tuple,
                  initial_direction: Optional[int] = None
                  ) -> Optional[Tuple[list, int, Optional[int]]]:
    if not (is_inner(*start) and is_inner(*end)):
        return None
    if not (is_free(the_map, *start) and is_free(the_map, *end)):
        return None
    if start == end:
        return [start], 0, initial_direction

    costs, parent = _nav_time_search(the_map, start, initial_direction)
    end_states = [(cost, state) for state, cost in costs.items()
                  if state[:2] == end]
    if not end_states:
        return None
    cost, state = min(end_states)
    end_direction = state[2]
    path = []
    while state is not None:
        path.append(state[:2])
        state = parent[state]
    path.append(start)
    path.reverse()
    return path, cost, end_direction


def nav_bfs_distance_flood(the_map: list, start: tuple) -> Dict[tuple, int]:
    """从 start 一次 BFS 扩散，返回各可达坐标的最短步数。"""
    if not is_inner(*start) or not is_free(the_map, *start):
        return {}

    distance = {start: 0}
    queue = deque([start])
    while queue:
        r, c = queue.popleft()
        for d in range(4):
            nr, nc = r + DR[d], c + DC[d]
            pos = (nr, nc)
            if is_free(the_map, nr, nc) and pos not in distance:
                distance[pos] = distance[(r, c)] + 1
                queue.append(pos)
    return distance


def nav_bfs_flood(the_map: list, start: tuple) -> set:
    """从 start 一次 BFS 扩散，返回全部可达坐标。镜像 Algo_Nav_BFS_Flood。"""
    return set(nav_bfs_distance_flood(the_map, start))


def is_reachable(reachable: set, target: tuple) -> bool:
    """基于 nav_bfs_flood 的结果执行 O(1) 可达性查询。"""
    return target in reachable


def _box_nav_distance(distance: Dict[tuple, int], box: tuple) -> int:
    """玩家到箱子四邻接可站立格的最短距离；完全不可接近时返回大值。"""
    return min(
        (distance[(box[0] + DR[d], box[1] + DC[d])]
         for d in range(4)
         if (box[0] + DR[d], box[1] + DC[d]) in distance),
        default=10 ** 9,
    )


# ============================================================
# 地图 ASCII 导入/导出（与 app_link.c / 主控一致）
#   # 墙  - 空地  . 目标  $ 箱子  * 炸弹  @ 玩家起点
# ============================================================

MAP_CHAR_TO_CELL = {
    '#': WALL, '-': EMPTY, '.': TARGET, '$': BOX, '*': BOMB,
    ' ': EMPTY,
}
MAP_CELL_TO_CHAR = {
    WALL: '#', EMPTY: '-', TARGET: '.', BOX: '$', BOMB: '*',
}


def parse_map_text(text: str) -> Tuple[Optional[list], Optional[tuple], str]:
    """
    解析 12×16 字符地图文本。
    返回 (map, player_pos, error_msg)；成功时 error_msg 为空串。
    """
    lines = [ln.rstrip('\r\n') for ln in text.strip().splitlines() if ln.strip()]
    if not lines:
        return None, None, "地图为空"

    if len(lines) != MAP_ROWS:
        return None, None, f"需要 {MAP_ROWS} 行，实际 {len(lines)} 行"

    the_map = [[EMPTY] * MAP_COLS for _ in range(MAP_ROWS)]
    player_pos = None

    for r, line in enumerate(lines):
        if len(line) != MAP_COLS:
            return None, None, f"第 {r} 行需要 {MAP_COLS} 列，实际 {len(line)} 列"
        for c, ch in enumerate(line):
            if ch == '@':
                if player_pos is not None:
                    return None, None, "地图中不能有多个 @"
                player_pos = (r, c)
                the_map[r][c] = EMPTY
                continue
            if ch not in MAP_CHAR_TO_CELL:
                return None, None, f"非法字符 '{ch}' 于 ({c},{r})"
            the_map[r][c] = MAP_CHAR_TO_CELL[ch]

    if player_pos is None:
        return None, None, "地图中缺少玩家标记 @"

    return the_map, player_pos, ""


def export_map_text(the_map: list, player_pos: tuple) -> str:
    """导出为可再次导入的 ASCII 地图（@ 覆盖玩家格）。"""
    lines = []
    for r in range(MAP_ROWS):
        row = []
        for c in range(MAP_COLS):
            if (r, c) == player_pos:
                row.append('@')
            else:
                row.append(MAP_CELL_TO_CHAR.get(the_map[r][c], '?'))
        lines.append(''.join(row))
    return '\n'.join(lines)


def default_box_mapping(boxes: list, targets: list) -> list:
    """按坐标排序后一一对应（Stage2 自动生成用）。"""
    order_b = sorted(range(len(boxes)), key=lambda i: (boxes[i][0], boxes[i][1]))
    order_t = sorted(range(len(targets)), key=lambda i: (targets[i][0], targets[i][1]))
    mapping = [0] * len(boxes)
    for rank, bi in enumerate(order_b):
        mapping[bi] = order_t[rank] if rank < len(order_t) else 0
    return mapping


# ============================================================
# 侦查阶段 — 复现 Algo_Find_Nearest_Box_Observe_Point + 全箱访问
# ============================================================

def find_nearest_box_observe_point(the_map: list, player_pos: tuple) -> Tuple[Optional[tuple], Optional[tuple]]:
    """
    BFS 找最近箱子的侧面观察点（玩家可站立、与箱子相邻）。
    返回 (observe_point, box_pos)，无解返回 (None, None)。
    """
    if not is_inner(*player_pos):
        return None, None

    parent: Dict[tuple, Optional[tuple]] = {player_pos: None}
    queue = deque([player_pos])

    while queue:
        r, c = pos = queue.popleft()
        for d in range(4):
            nr, nc = r + DR[d], c + DC[d]
            if is_inner(nr, nc) and the_map[nr][nc] == BOX:
                return pos, (nr, nc)
        for d in range(4):
            nr, nc = r + DR[d], c + DC[d]
            npos = (nr, nc)
            if is_free(the_map, nr, nc) and npos not in parent:
                parent[npos] = pos
                queue.append(npos)

    return None, None


def find_observe_point_for_box(the_map: list, player_pos: tuple,
                               box_pos: tuple) -> Optional[tuple]:
    """BFS 找到可观察指定箱子的站立格（与箱子四邻）。"""
    if not is_inner(*player_pos) or not is_inner(*box_pos):
        return None

    parent: Dict[tuple, Optional[tuple]] = {player_pos: None}
    queue = deque([player_pos])

    while queue:
        r, c = pos = queue.popleft()
        for d in range(4):
            nr, nc = r + DR[d], c + DC[d]
            if (nr, nc) == box_pos:
                return pos
        for d in range(4):
            nr, nc = r + DR[d], c + DC[d]
            npos = (nr, nc)
            if is_free(the_map, nr, nc) and npos not in parent:
                parent[npos] = pos
                queue.append(npos)

    return None


def path_to_actions(path: list) -> list:
    """将 nav_bfs 路径 [(row,col),...] 转为方向动作（跳过起点）。"""
    if not path or len(path) < 2:
        return []
    actions = []
    for i in range(1, len(path)):
        r0, c0 = path[i - 1]
        r1, c1 = path[i]
        dr, dc = r1 - r0, c1 - c0
        for d in range(4):
            if DR[d] == dr and DC[d] == dc:
                actions.append(d)
                break
    return actions


def plan_scout_phase(the_map: list, player_start: tuple,
                     boxes: Optional[list] = None) -> dict:
    """
    Stage2/3：贪心访问每个箱子的观察点（先侦查再推箱）。
    返回 scout_actions, visits, player_after_scout, scout_waypoints。

    注意: 这是旧版"全量遍历"实现, 仅访问 BOX 不访问 TARGET; 主要用作历史比对.
    新版逻辑请使用 plan_scout_phase_v2 (访问 box+target).
    """
    if boxes is None:
        boxes = extract_elements(the_map, BOX)

    remaining = list(range(len(boxes)))
    cur = player_start
    scout_actions: list = []
    visits: list = []
    waypoints_flat: list = []

    while remaining:
        best_i = None
        best_obs = None
        best_box = None
        best_cost = 10 ** 9

        for i in remaining:
            box = boxes[i]
            obs = find_observe_point_for_box(the_map, cur, box)
            if obs is None:
                continue
            path = nav_bfs(the_map, cur, obs)
            if path is None:
                continue
            cost = len(path)
            if cost < best_cost:
                best_cost = cost
                best_i = i
                best_obs = obs
                best_box = box

        if best_i is None or best_obs is None:
            break

        path = nav_bfs(the_map, cur, best_obs)
        if path is None:
            remaining.remove(best_i)
            continue

        acts = path_to_actions(path)
        scout_actions.extend(acts)
        waypoints_flat.extend(path[1:])
        visits.append({
            'box_idx': best_i,
            'box_pos': boxes[best_i],
            'observe': best_obs,
            'path_len': len(acts),
        })
        cur = best_obs
        remaining.remove(best_i)

    return {
        'scout_actions': scout_actions,
        'visits': visits,
        'player_after_scout': cur,
        'scout_waypoints': waypoints_flat,
        'all_visited': len(visits) == len(boxes),
    }


# ============================================================
# 侦查 V2 — 同时访问 BOX 和 TARGET (与 C 端 app_recognize 对齐)
# 赛题约定: 箱子 class_id ∈ {1..N}, 目标 class_id ∈ {1..N}, 集合相同;
#           箱-目一一对应 (相同 class_id 互推).
# ============================================================
EXACT_SCOUT_ITEM_LIMIT = 6
SCOUT_QUARTER_TURN_COST = 4


def static_push_distances(the_map: list, target: tuple,
                          walls_removable: bool = False) -> list:
    """反向推箱距离；放宽其他箱子/目标，只保留墙体和内场边界的必要约束。"""
    distance = [[None] * MAP_COLS for _ in range(MAP_ROWS)]

    def cell_open(r: int, c: int) -> bool:
        if not is_inner(r, c):
            return False
        if walls_removable:
            return True
        return the_map[r][c] not in (WALL, BOMB)

    if not cell_open(*target):
        return distance

    distance[target[0]][target[1]] = 0
    queue = deque([target])
    while queue:
        cur_r, cur_c = queue.popleft()
        next_distance = distance[cur_r][cur_c] + 1
        for d in range(4):
            prev_r, prev_c = cur_r - DR[d], cur_c - DC[d]
            stand_r, stand_c = prev_r - DR[d], prev_c - DC[d]
            if (not cell_open(prev_r, prev_c)
                    or not cell_open(stand_r, stand_c)
                    or distance[prev_r][prev_c] is not None):
                continue
            distance[prev_r][prev_c] = next_distance
            queue.append((prev_r, prev_c))
    return distance


def plan_scout_phase_v2(the_map: list, player_start: tuple,
                         box_classes: Optional[list] = None,
                         target_classes: Optional[list] = None,
                         initial_heading_quarters: int = 0) -> dict:
    """
    侦查规划 — 逐个实地访问每个箱子和目标点.

    参数:
        box_classes / target_classes:
            视觉端"真值": 第 i 个箱子/目标的 class_id (1..N).
            模拟时由地图生成器或外部分配. 若为 None, 默认按 extract 顺序赋 1..N.
        initial_heading_quarters:
            初始四向航向，0/1/2/3 分别对应 0/90/180/-90 度。精确 Tour
            使用平移格数与观察转角的组合代价，并保留实际选中的观察格。

    返回:
        scout_actions, scout_waypoints, visits, player_after_scout,
        all_visited, box_classes, target_classes, box_to_target_idx,
        visited_count
    """
    boxes   = extract_elements(the_map, BOX)
    targets = extract_elements(the_map, TARGET)
    n_box = len(boxes)
    n_tgt = len(targets)

    if box_classes is None:
        box_classes = [(i + 1) for i in range(n_box)]
    if target_classes is None:
        target_classes = [(i + 1) for i in range(n_tgt)]

    items: list = []
    for i, p in enumerate(boxes):
        items.append({
            'kind': 'box', 'item_idx': i, 'pos': p,
            'truth_class': box_classes[i] if i < len(box_classes) else 0,
            'class_id': 0, 'visited': False, 'ok': False,
        })
    for i, p in enumerate(targets):
        items.append({
            'kind': 'target', 'item_idx': i, 'pos': p,
            'truth_class': target_classes[i] if i < len(target_classes) else 0,
            'class_id': 0, 'visited': False, 'ok': False,
        })

    cur = player_start
    scout_actions: list = []
    waypoints_flat: list = []
    visits: list = []

    def face_dir_from_observe(observe: tuple, target: tuple) -> Optional[int]:
        dr = target[0] - observe[0]
        dc = target[1] - observe[1]
        for d in range(4):
            if DR[d] == dr and DC[d] == dc:
                return d
        return None

    def face_heading_quarters(observe: tuple, target: tuple) -> Optional[int]:
        face_dir = face_dir_from_observe(observe, target)
        if face_dir is None:
            return None
        # Direction U/D/L/R maps to fixed yaw 180/0/-90/90 degrees.
        return (2, 0, 3, 1)[face_dir]

    def quarter_turns(from_heading: int, to_heading: int) -> int:
        diff = abs((from_heading & 3) - (to_heading & 3))
        return min(diff, 4 - diff)

    def all_resolved() -> bool:
        return all(it['ok'] for it in items)

    def observe_options(target: tuple) -> list:
        opts = []
        for d in range(4):
            obs = (target[0] + DR[d], target[1] + DC[d])
            if is_free(the_map, *obs):
                opts.append(obs)
        return opts

    def shortest_observe_path(start: tuple, target: tuple,
                              start_heading: int) -> Tuple[Optional[int], Optional[tuple], Optional[list]]:
        best_cost = None
        best_obs = None
        best_path = None
        for obs in observe_options(target):
            route = nav_time_path(the_map, start, obs)
            if route is None:
                continue
            path, move_cost, _ = route
            heading = face_heading_quarters(obs, target)
            if heading is None:
                continue
            cost = (move_cost +
                    quarter_turns(start_heading, heading) *
                    SCOUT_QUARTER_TURN_COST)
            if best_cost is None or cost < best_cost:
                best_cost = cost
                best_obs = obs
                best_path = path
        return best_cost, best_obs, best_path

    tour_cache = []

    def choose_exact_tour_first(start: tuple, cand: list,
                                start_heading: int) -> Optional[tuple]:
        if not cand or len(cand) > EXACT_SCOUT_ITEM_LIMIT:
            return None

        obs_nodes = []
        for local_idx, it in enumerate(cand):
            opts = observe_options(it['pos'])
            if not opts:
                return None
            for obs in opts:
                heading = face_heading_quarters(obs, it['pos'])
                if heading is None:
                    return None
                obs_nodes.append((local_idx, obs, heading))
        if not obs_nodes:
            return None

        start_cost = []
        for _, obs, heading in obs_nodes:
            route = nav_time_path(the_map, start, obs)
            start_cost.append(None if route is None else
                              route[1] +
                              quarter_turns(start_heading, heading) *
                              SCOUT_QUARTER_TURN_COST)

        edge_cost = [[None] * len(obs_nodes) for _ in obs_nodes]
        for i, (_, from_obs, from_heading) in enumerate(obs_nodes):
            for j, (_, to_obs, to_heading) in enumerate(obs_nodes):
                route = nav_time_path(the_map, from_obs, to_obs)
                edge_cost[i][j] = None if route is None else (
                    route[1] +
                    quarter_turns(from_heading, to_heading) *
                    SCOUT_QUARTER_TURN_COST)

        full_mask = (1 << len(cand)) - 1
        dp = {}
        parent = {}
        for node_idx, (local_idx, _, _) in enumerate(obs_nodes):
            if start_cost[node_idx] is None:
                continue
            mask = 1 << local_idx
            key = (mask, node_idx)
            old = dp.get(key)
            value = (start_cost[node_idx], node_idx)
            if old is None or value[0] < old[0]:
                dp[key] = value
                parent[key] = None

        for mask in range(1, full_mask + 1):
            states = [(node_idx, val) for (state_mask, node_idx), val in dp.items()
                      if state_mask == mask]
            for node_idx, (base_cost, first_idx) in states:
                for next_node, (next_local, _, _) in enumerate(obs_nodes):
                    if mask & (1 << next_local):
                        continue
                    step = edge_cost[node_idx][next_node]
                    if step is None:
                        continue
                    next_mask = mask | (1 << next_local)
                    key = (next_mask, next_node)
                    value = (base_cost + step, first_idx)
                    old = dp.get(key)
                    if old is None or value[0] < old[0]:
                        dp[key] = value
                        parent[key] = (mask, node_idx)

        best = None
        best_key = None
        for (mask, node_idx), value in dp.items():
            if mask != full_mask:
                continue
            terminal_cost = min(
                (edge_cost[node_idx][to_idx]
                 for to_idx, (local_idx, _, _) in enumerate(obs_nodes)
                 if cand[local_idx]['kind'] == 'box' and
                 edge_cost[node_idx][to_idx] is not None),
                default=0)
            total_cost = value[0] + terminal_cost
            if best is None or total_cost < best[0]:
                best = (total_cost, value[1])
                best_key = (mask, node_idx)
        if best is None:
            return None

        order = []
        key = best_key
        while key is not None:
            _, node_idx = key
            local_idx, observe, _ = obs_nodes[node_idx]
            order.append((cand[local_idx], observe))
            key = parent[key]
        order.reverse()
        tour_cache[:] = order[1:]
        return order[0]

    def choose_next_item(start: tuple, cand: list,
                         start_heading: int) -> Tuple[Optional[dict], Optional[tuple]]:
        while tour_cache:
            cached_item, cached_observe = tour_cache.pop(0)
            if not cached_item['visited'] and cached_observe in observe_options(cached_item['pos']):
                return cached_item, cached_observe
        exact = choose_exact_tour_first(start, cand, start_heading)
        if exact is not None:
            return exact

        best_it = None
        best_obs = None
        best_cost = None
        for it in cand:
            cost, obs, _ = shortest_observe_path(start, it['pos'], start_heading)
            if cost is None:
                continue
            heading = face_heading_quarters(obs, it['pos'])
            next_cost = min(
                (value for other in cand if other is not it
                 for value, _, _ in
                 [shortest_observe_path(obs, other['pos'], heading)]
                 if value is not None),
                default=0)
            score = cost + next_cost
            if best_cost is None or score < best_cost:
                best_cost = score
                best_it = it
                best_obs = obs
        return best_it, best_obs

    current_heading = initial_heading_quarters & 3
    while not all_resolved():
        cand = [it for it in items if not it['visited'] and not it['ok']]
        if not cand:
            break

        best_it, best_obs = choose_next_item(cur, cand, current_heading)

        if best_it is None or best_obs is None:
            break

        route = nav_time_path(the_map, cur, best_obs)
        if route is None:
            best_it['visited'] = True
            continue
        path = route[0]

        acts = path_to_actions(path)
        scout_actions.extend(acts)
        waypoints_flat.extend(path[1:])

        cls = best_it['truth_class']
        best_it['class_id'] = cls
        best_it['ok'] = (cls > 0)
        best_it['visited'] = True

        face_dir = face_dir_from_observe(best_obs, best_it['pos'])
        visits.append({
            'kind':         best_it['kind'],
            'item_idx':     best_it['item_idx'],
            'pos':          best_it['pos'],
            'observe':      best_obs,
            'face_dir':     face_dir,
            'class_id':     cls,
            'path_actions': list(acts),
        })
        next_heading = face_heading_quarters(best_obs, best_it['pos'])
        if next_heading is not None:
            current_heading = next_heading
        cur = best_obs

    out_box_classes    = [0] * n_box
    out_target_classes = [0] * n_tgt
    for it in items:
        if it['kind'] == 'box':
            out_box_classes[it['item_idx']] = it['class_id']
        else:
            out_target_classes[it['item_idx']] = it['class_id']

    box_to_target_idx: list = [0] * n_box
    if all_resolved() and n_box == n_tgt:
        box_done = [False] * n_box
        ok = True

        for bi in range(n_box):
            if box_done[bi]:
                continue
            cls = out_box_classes[bi]
            if cls == 0:
                ok = False
                break

            group_boxes = [i for i, c in enumerate(out_box_classes) if c == cls]
            group_targets = [i for i, c in enumerate(out_target_classes) if c == cls]
            if len(group_boxes) != len(group_targets) or not group_boxes:
                ok = False
                break

            walls_removable = bool(extract_elements(the_map, BOMB))
            push_distance = {
                target_idx: static_push_distances(
                    the_map, targets[target_idx], walls_removable)
                for target_idx in group_targets
            }
            best_cost = None
            best_assign = None
            used = [False] * len(group_targets)
            cur_assign = [0] * len(group_boxes)

            def dfs(depth: int, cur_cost: int) -> None:
                nonlocal best_cost, best_assign
                if best_cost is not None and cur_cost >= best_cost:
                    return
                if depth >= len(group_boxes):
                    best_cost = cur_cost
                    best_assign = list(cur_assign)
                    return

                box_pos = boxes[group_boxes[depth]]
                for local_ti, target_idx in enumerate(group_targets):
                    if used[local_ti]:
                        continue
                    step_cost = push_distance[target_idx][box_pos[0]][box_pos[1]]
                    if step_cost is None:
                        continue
                    used[local_ti] = True
                    cur_assign[depth] = target_idx
                    dfs(depth + 1, cur_cost + step_cost)
                    used[local_ti] = False

            dfs(0, 0)
            if best_assign is None:
                ok = False
                break

            for local_bi, target_idx in enumerate(best_assign):
                box_idx = group_boxes[local_bi]
                box_to_target_idx[box_idx] = target_idx
                box_done[box_idx] = True

        if not ok:
            box_to_target_idx = []

    return {
        'scout_actions':       scout_actions,
        'scout_waypoints':     waypoints_flat,
        'visits':              visits,
        'player_after_scout':  cur,
        'all_visited':         all_resolved(),
        'box_classes':         out_box_classes,
        'target_classes':      out_target_classes,
        'box_to_target_idx':   box_to_target_idx,
        'visited_count':       len(visits),
    }


# ============================================================
# 单箱推宏 A* 求解器
# ============================================================

SOKO_COST_MOVE_MS = 100
SOKO_COST_WAYPOINT_MS = 400
# Firmware configChassis.h currently disables arrival Snap; keep the mirror's
# objective identical instead of charging a non-existent 200ms operation.
SOKO_COST_SNAP_MS = 0


def _action_from_points(src: tuple, dst: tuple) -> Optional[int]:
    for d in range(4):
        if (src[0] + DR[d], src[1] + DC[d]) == dst:
            return d
    return None


def _macro_edge(sub_map: list, player: tuple, box: tuple, target: tuple,
                 push_dir: int, previous_push_dir: Optional[int],
                 block_other_targets: bool,
                 walk_costs: Optional[Dict[tuple, int]] = None) -> Optional[tuple]:
    stand = (box[0] - DR[push_dir], box[1] - DC[push_dir])
    next_box = (box[0] + DR[push_dir], box[1] + DC[push_dir])
    if not (is_free(sub_map, *stand) and is_free(sub_map, *next_box)):
        return None
    if (block_other_targets and sub_map[next_box[0]][next_box[1]] == TARGET
            and next_box != target):
        return None

    if walk_costs is None:
        walk_map = [row[:] for row in sub_map]
        walk_map[box[0]][box[1]] = WALL
        route = nav_time_path(walk_map, player, stand, previous_push_dir)
        if route is None:
            return None
        walk_path, walk_cost_units, last_dir = route
    else:
        arrivals = [(cost, state[2]) for state, cost in walk_costs.items()
                    if state[:2] == stand]
        if not arrivals:
            if stand != player:
                return None
            walk_cost_units, last_dir = 0, previous_push_dir
        else:
            walk_cost_units, last_dir = min(arrivals)
        walk_path = None

    cost = walk_cost_units * SOKO_COST_MOVE_MS + SOKO_COST_MOVE_MS
    if last_dir is None or push_dir != last_dir:
        cost += SOKO_COST_WAYPOINT_MS
    if stand != player or previous_push_dir is None or push_dir != previous_push_dir:
        cost += SOKO_COST_SNAP_MS
    return next_box, walk_path, cost

def sokoban_bfs_single(sub_map: list, player: tuple, box: tuple,
                       target: tuple,
                       block_other_targets: bool = True) -> Optional[list]:
    """
    单箱推宏 A*。状态=(box_r, box_c, last_push_dir)，普通行走由导航 BFS 连接。
    返回动作序列 [0..3]，无解返回 None。
    对应 C 代码 sokoban_bfs_single()。
    """
    if not (is_inner(*player) and is_inner(*box) and is_inner(*target)):
        return None
    if box == target:
        return []

    work_map = [row[:] for row in sub_map]
    work_map[box[0]][box[1]] = EMPTY

    best_cost: Dict[tuple, int] = {}
    parent: Dict[tuple, Optional[tuple]] = {}
    queue = []

    initial_walk_map = [row[:] for row in work_map]
    initial_walk_map[box[0]][box[1]] = WALL
    initial_walk_costs, _ = _nav_time_search(initial_walk_map, player, None)
    for d in range(4):
        edge = _macro_edge(work_map, player, box, target, d, None,
                           block_other_targets, initial_walk_costs)
        if edge is None:
            continue
        next_box, _, edge_cost = edge
        state = (next_box[0], next_box[1], d)
        if edge_cost < best_cost.get(state, 10 ** 18):
            best_cost[state] = edge_cost
            parent[state] = None
            heuristic = (abs(next_box[0] - target[0])
                         + abs(next_box[1] - target[1])) * SOKO_COST_MOVE_MS
            heapq.heappush(queue, (edge_cost + heuristic, edge_cost, state))

    closed = set()
    goal_state = None
    while queue:
        _, cost, state = heapq.heappop(queue)
        if state in closed or cost != best_cost.get(state):
            continue
        closed.add(state)
        br, bc, previous_dir = state
        cur_box = (br, bc)
        cur_player = (br - DR[previous_dir], bc - DC[previous_dir])
        if cur_box == target:
            goal_state = state
            break

        state_walk_map = [row[:] for row in work_map]
        state_walk_map[cur_box[0]][cur_box[1]] = WALL
        state_walk_costs, _ = _nav_time_search(
            state_walk_map, cur_player, previous_dir)
        for d in range(4):
            edge = _macro_edge(work_map, cur_player, cur_box, target, d,
                               previous_dir, block_other_targets,
                               state_walk_costs)
            if edge is None:
                continue
            next_box, _, edge_cost = edge
            next_state = (next_box[0], next_box[1], d)
            next_cost = cost + edge_cost
            if next_state in closed or next_cost >= best_cost.get(next_state, 10 ** 18):
                continue
            best_cost[next_state] = next_cost
            parent[next_state] = state
            heuristic = (abs(next_box[0] - target[0])
                         + abs(next_box[1] - target[1])) * SOKO_COST_MOVE_MS
            heapq.heappush(queue, (next_cost + heuristic, next_cost, next_state))

    if goal_state is None:
        return None

    macro_states = []
    state = goal_state
    while state is not None:
        macro_states.append(state)
        state = parent[state]
    macro_states.reverse()

    actions = []
    cur_player = player
    cur_box = box
    for _, _, push_dir in macro_states:
        stand = (cur_box[0] - DR[push_dir], cur_box[1] - DC[push_dir])
        walk_map = [row[:] for row in work_map]
        walk_map[cur_box[0]][cur_box[1]] = WALL
        route = nav_time_path(walk_map, cur_player, stand,
                              actions[-1] if actions else None)
        if route is None:
            return None
        walk_path = route[0]
        for src, dst in zip(walk_path, walk_path[1:]):
            d = _action_from_points(src, dst)
            if d is None:
                return None
            actions.append(d)
        actions.append(push_dir)
        if len(actions) > SOKOBAN_MAX_ACTIONS:
            return None
        cur_player = cur_box
        cur_box = (cur_box[0] + DR[push_dir], cur_box[1] + DC[push_dir])

    return actions


# ============================================================
# 动作模拟 — 得到玩家和箱子的最终坐标
# ============================================================

def simulate_actions(actions: list, player: tuple, box: tuple) -> tuple:
    """
    模拟动作序列，返回 (player_final, box_final)。
    对应 C 代码 simulate_actions()。
    """
    pr, pc = player
    br, bc = box
    for d in actions:
        npr = pr + DR[d]
        npc = pc + DC[d]
        if npr == br and npc == bc:
            br += DR[d]
            bc += DC[d]
        pr, pc = npr, npc
    return (pr, pc), (br, bc)


# ============================================================
# 子地图构建
# ============================================================

def build_sub_map(base_map: list, boxes: list, targets: list,
                  solved: list, target_used: list,
                  cur_box_idx: int, cur_target_idx: int) -> list:
    """
    构建单箱子问题的子地图：
      - 当前箱子：从地图移除（BFS 状态跟踪）
      - 已完成箱子：空地
      - 其余未完成箱子：墙壁
      - 未使用目标点：保留，禁止箱子提前进入消除
      - 已使用目标点：改为空地
    对应 C 代码 build_sub_map()。
    """
    sub = [row[:] for row in base_map]
    for i, (br, bc) in enumerate(boxes):
        if i == cur_box_idx:
            sub[br][bc] = EMPTY
        elif solved[i]:
            sub[br][bc] = EMPTY
        else:
            sub[br][bc] = WALL

    for i, (tr, tc) in enumerate(targets):
        if i == cur_target_idx:
            continue
        if target_used[i] and sub[tr][tc] == TARGET:
            sub[tr][tc] = EMPTY

    return sub


# ============================================================
# Stage 1 求解 — 贪心（任意箱→任意目标）
# ============================================================

def _solve_stage1_greedy(the_map: list, player_pos: tuple) -> Optional[dict]:
    """
    贪心求解：每轮选最近未完成箱子，为其匹配最近未使用目标。
    返回解算结果字典，无解返回 None。
    对应 C 代码 Sokoban_Solve_Stage1()。
    """
    boxes   = extract_elements(the_map, BOX)
    targets = extract_elements(the_map, TARGET)

    if not boxes or len(boxes) != len(targets):
        return None

    n = len(boxes)
    solved = [False] * n
    t_used = [False] * n
    sub_solutions = []
    cur_player = player_pos

    for _ in range(n):
        # 一次扩散后按实际绕障距离排列箱子；最近候选无解时继续试后续候选。
        nav_map = [row[:] for row in the_map]
        for i, box in enumerate(boxes):
            if solved[i]:
                nav_map[box[0]][box[1]] = EMPTY
        distance = nav_bfs_distance_flood(nav_map, cur_player)
        box_candidates = sorted(
            (i for i in range(n) if not solved[i]),
            key=lambda i: (_box_nav_distance(distance, boxes[i]), i),
        )
        chosen = None
        for best_b in box_candidates:
            if _box_nav_distance(distance, boxes[best_b]) >= 10 ** 9:
                break
            target_candidates = sorted(
                (i for i in range(n) if not t_used[i]),
                key=lambda i: (
                    abs(targets[i][0] - boxes[best_b][0])
                    + abs(targets[i][1] - boxes[best_b][1]),
                    i,
                ),
            )
            for best_t in target_candidates:
                sub = build_sub_map(the_map, boxes, targets, solved, t_used,
                                    best_b, best_t)
                sol = sokoban_bfs_single(sub, cur_player, boxes[best_b],
                                         targets[best_t])
                if sol is not None:
                    chosen = (best_b, best_t, sol)
                    break
            if chosen is not None:
                break
        if chosen is None:
            return None
        best_b, best_t, sol = chosen

        cur_player, _ = simulate_actions(sol, cur_player, boxes[best_b])
        sub_solutions.append({
            'actions':    sol,
            'box_idx':    best_b,
            'target_idx': best_t,
            'player_end': cur_player,
        })
        solved[best_b] = True
        t_used[best_t] = True

    return {
        'sub_solutions': sub_solutions,
        'boxes':         boxes,
        'targets':       targets,
        'total_steps':   sum(len(s['actions']) for s in sub_solutions),
    }


# ============================================================
# Stage 2 求解 — 指定箱→目标映射
# ============================================================

def _solve_stage2_greedy(the_map: list, player_pos: tuple,
                         box_to_target_idx: list) -> Optional[dict]:
    """
    指定映射求解：box_to_target_idx[i] 表示第 i 个箱子→第几号目标。
    对应 C 代码 Sokoban_Solve_Stage2()。
    """
    boxes   = extract_elements(the_map, BOX)
    targets = extract_elements(the_map, TARGET)

    n = len(boxes)
    if n == 0 or len(box_to_target_idx) != n:
        return None

    solved = [False] * n
    t_used = [False] * n
    sub_solutions = []
    cur_player = player_pos

    for _ in range(n):
        # 与 Stage1 一致改用 BFS 绕障距离选箱 (曼哈顿距离在有阻挡时会选中
        # "直线近但绕路远/暂不可达"的箱子)。对应 C 代码 2026-07-08 修改。
        nav_map = [row[:] for row in the_map]
        for i, box in enumerate(boxes):
            if solved[i]:
                nav_map[box[0]][box[1]] = EMPTY
        distance = nav_bfs_distance_flood(nav_map, cur_player)
        box_candidates = sorted(
            (i for i in range(n) if not solved[i]),
            key=lambda i: (_box_nav_distance(distance, boxes[i]), i),
        )
        chosen = None
        for best_b in box_candidates:
            if _box_nav_distance(distance, boxes[best_b]) >= 10 ** 9:
                break
            ti = box_to_target_idx[best_b]
            if ti >= len(targets):
                return None
            sub = build_sub_map(the_map, boxes, targets, solved, t_used,
                                best_b, ti)
            sol = sokoban_bfs_single(sub, cur_player, boxes[best_b], targets[ti])
            if sol is not None:
                chosen = (best_b, ti, sol)
                break
        if chosen is None:
            return None
        best_b, ti, sol = chosen

        cur_player, _ = simulate_actions(sol, cur_player, boxes[best_b])
        sub_solutions.append({
            'actions':    sol,
            'box_idx':    best_b,
            'target_idx': ti,
            'player_end': cur_player,
        })
        solved[best_b] = True
        t_used[ti] = True

    return {
        'sub_solutions': sub_solutions,
        'boxes':         boxes,
        'targets':       targets,
        'total_steps':   sum(len(s['actions']) for s in sub_solutions),
    }


# ============================================================
# Stage 3 辅助 — 死局检测
# ============================================================

OPT_EXACT_BOX_LIMIT = 3
OPT_BRANCH_BOX_LIMIT = 5
STAGE1_PAIR_LIMIT = 256


def _mapping_valid(mapping: list, box_n: int, target_n: int) -> bool:
    if mapping is None or len(mapping) != box_n:
        return False
    used = set()
    for ti in mapping:
        if ti < 0 or ti >= target_n or ti in used:
            return False
        used.add(ti)
    return True


def _push_flags(actions: list, player: tuple, box: tuple) -> list:
    flags = []
    pr, pc = player
    br, bc = box
    for d in actions:
        npr, npc = pr + DR[d], pc + DC[d]
        pushed = (npr, npc) == (br, bc)
        flags.append(pushed)
        if pushed:
            br, bc = br + DR[d], bc + DC[d]
        pr, pc = npr, npc
    return flags


def sequence_time_cost(actions: list, push_flags: list) -> int:
    """镜像固件：行驶 + 每个方向航点停站 + 含推箱航段 Snap。"""
    cost = 0
    segment_start = 0
    for i, _ in enumerate(actions):
        cost += SOKO_COST_MOVE_MS
        segment_end = i + 1 == len(actions) or actions[i] != actions[i + 1]
        if segment_end:
            cost += SOKO_COST_WAYPOINT_MS
            if any(push_flags[segment_start:i + 1]):
                cost += SOKO_COST_SNAP_MS
            segment_start = i + 1
    return cost


def _path_to_actions(path: list) -> list:
    result = []
    for src, dst in zip(path, path[1:]):
        d = _action_from_points(src, dst)
        if d is None:
            return []
        result.append(d)
    return result


def build_return_path(the_map: list, player_pos: tuple,
                      home_pos: tuple = (5, 1)) -> Optional[dict]:
    """镜像固件：返库单命令由底盘依次执行 X/Y 轴，按曼哈顿行程计时。"""
    del the_map
    if not is_inner(*home_pos):
        return None
    distance_cells = (abs(home_pos[0] - player_pos[0]) +
                      abs(home_pos[1] - player_pos[1]))
    move_cost = distance_cells * SOKO_COST_MOVE_MS
    waypoint_cost = SOKO_COST_WAYPOINT_MS if distance_cells > 0 else 0
    return {
        'actions': [],
        'waypoints': [(home_pos, 'critical')],
        'time_cost_ms': move_cost + waypoint_cost + SOKO_COST_SNAP_MS,
        'direct': True,
    }


def _solve_stage1_time_opt(the_map: list, player_pos: tuple,
                           home_pos: tuple,
                           fixed_mapping: Optional[list] = None) -> Optional[dict]:
    boxes = extract_elements(the_map, BOX)
    targets = extract_elements(the_map, TARGET)
    n = len(boxes)
    if n == 0 or n != len(targets) or n > MAX_BOXES:
        return None
    if fixed_mapping is not None and not _mapping_valid(fixed_mapping, n, n):
        return None

    best = None
    best_cost = 10 ** 18
    cur_solutions = []
    if fixed_mapping is not None:
        node_limit = 0 if n <= OPT_BRANCH_BOX_LIMIT else 512
    else:
        node_limit = 0 if n <= OPT_EXACT_BOX_LIMIT else STAGE1_PAIR_LIMIT
    node_count = 0
    hit_limit = False

    def dfs(solved_mask: int, target_mask: int,
            cur_player: tuple, cost: int) -> None:
        nonlocal best, best_cost, node_count, hit_limit
        depth = len(cur_solutions)
        if cost >= best_cost or hit_limit:
            return
        if depth >= n:
            return_plan = build_return_path(the_map, cur_player, home_pos)
            if return_plan is None:
                return
            total_cost = cost + return_plan['time_cost_ms']
            if total_cost < best_cost:
                best_cost = total_cost
                best = {
                    'sub_solutions': [dict(s) for s in cur_solutions],
                    'boxes': boxes,
                    'targets': targets,
                    'total_steps': sum(len(s['actions']) for s in cur_solutions),
                    'time_cost_ms': total_cost,
                    'return_plan': return_plan,
                    'pair_evaluations': node_count,
                }
            return

        nav_map = [row[:] for row in the_map]
        for bi, box_pos in enumerate(boxes):
            if solved_mask & (1 << bi):
                nav_map[box_pos[0]][box_pos[1]] = EMPTY
        distance = nav_bfs_distance_flood(nav_map, cur_player)
        candidates = []
        for bi in range(n):
            if solved_mask & (1 << bi):
                continue
            box_distance = _box_nav_distance(distance, boxes[bi])
            if box_distance >= 10 ** 9:
                continue
            target_indices = ([fixed_mapping[bi]] if fixed_mapping is not None
                              else range(n))
            for ti in target_indices:
                if target_mask & (1 << ti):
                    continue
                target_distance = (abs(targets[ti][0] - boxes[bi][0])
                                   + abs(targets[ti][1] - boxes[bi][1]))
                candidates.append((box_distance * 32 + target_distance, bi, ti))
        candidates.sort(key=lambda item: item[0])

        solved = [bool(solved_mask & (1 << i)) for i in range(n)]
        target_used = [bool(target_mask & (1 << i)) for i in range(n)]
        for _, bi, ti in candidates:
            if node_limit and node_count >= node_limit:
                hit_limit = True
                return
            node_count += 1
            sub = build_sub_map(the_map, boxes, targets, solved, target_used, bi, ti)
            actions = sokoban_bfs_single(sub, cur_player, boxes[bi], targets[ti])
            if actions is None:
                continue
            push_flags = _push_flags(actions, cur_player, boxes[bi])
            next_cost = cost + sequence_time_cost(actions, push_flags)
            if next_cost >= best_cost:
                continue
            next_player, _ = simulate_actions(actions, cur_player, boxes[bi])
            cur_solutions.append({
                'actions': actions,
                'push_flags': push_flags,
                'box_idx': bi,
                'target_idx': ti,
                'player_end': next_player,
            })
            dfs(solved_mask | (1 << bi), target_mask | (1 << ti),
                next_player, next_cost)
            cur_solutions.pop()
            if hit_limit:
                return

    dfs(0, 0, player_pos, 0)
    if best is not None or not hit_limit:
        return best

    # 镜像正式固件：仅在多箱搜索预算耗尽且尚无完整叶子时，用已有贪心解保底。
    fallback = (_solve_stage2_greedy(the_map, player_pos, fixed_mapping)
                if fixed_mapping is not None
                else _solve_stage1_greedy(the_map, player_pos))
    if fallback is None:
        return None
    fallback_cost = 0
    cur_player = player_pos
    for sub in fallback['sub_solutions']:
        flags = _push_flags(sub['actions'], cur_player,
                            boxes[sub['box_idx']])
        sub['push_flags'] = flags
        fallback_cost += sequence_time_cost(sub['actions'], flags)
        cur_player = sub['player_end']
    return_plan = build_return_path(the_map, cur_player, home_pos)
    if return_plan is None:
        return None
    fallback['time_cost_ms'] = fallback_cost + return_plan['time_cost_ms']
    fallback['return_plan'] = return_plan
    fallback['pair_evaluations'] = node_count
    return fallback


def solve_stage1(the_map: list, player_pos: tuple,
                 home_pos: Optional[tuple] = None) -> Optional[dict]:
    """第一关全箱完成、时间成本优化；home_pos 默认保持兼容，取起点。"""
    return _solve_stage1_time_opt(the_map, player_pos,
                                  player_pos if home_pos is None else home_pos)


def solve_stage2(the_map: list, player_pos: tuple,
                 box_to_target_idx: list,
                 home_pos: Optional[tuple] = None) -> Optional[dict]:
    box_n = len(extract_elements(the_map, BOX))
    target_n = len(extract_elements(the_map, TARGET))
    if not _mapping_valid(box_to_target_idx, box_n, target_n):
        return None
    return _solve_stage1_time_opt(
        the_map, player_pos,
        player_pos if home_pos is None else home_pos,
        box_to_target_idx)


def is_blocker(the_map: list, r: int, c: int) -> bool:
    if not is_inner(r, c):
        return True
    return the_map[r][c] in (WALL, BOX, BOMB)


def _box_has_pushable_direction(the_map: list, br: int, bc: int) -> bool:
    """O3.2 镜像 C 端 box_has_pushable_direction():
    存在方向 d 使 去向格 box+Δd 与 玩家站位 box-Δd 同时可通行 → 还能推动。"""
    for d in range(4):
        dest_r, dest_c = br + DR[d], bc + DC[d]
        stand_r, stand_c = br - DR[d], bc - DC[d]
        if is_free(the_map, dest_r, dest_c) and is_free(the_map, stand_r, stand_c):
            return True
    return False


def _box_frozen_on_border_line(the_map: list, br: int, bc: int) -> bool:
    """O3.1 镜像 C 端 box_frozen_on_border_line():
    贴内场外边界的箱子永远脱不开所贴行/列, 该行/列无目标即死局。"""
    if br == INNER_R_MIN or br == INNER_R_MAX:
        for c in range(INNER_C_MIN, INNER_C_MAX + 1):
            if the_map[br][c] == TARGET:
                return False
        return True
    if bc == INNER_C_MIN or bc == INNER_C_MAX:
        for r in range(INNER_R_MIN, INNER_R_MAX + 1):
            if the_map[r][bc] == TARGET:
                return False
        return True
    return False


def check_deadlock(the_map: list) -> Tuple[bool, Optional[tuple]]:
    """
    死局检测。对应 C 代码 Sokoban_Is_Deadlock()。
    第一遍角落死局(行为与历史完全一致), 仅当无角落死局时第二遍补 O3.2/O3.1。
    返回 (is_dead, dead_box_pos)。
    """
    for r in range(INNER_R_MIN, INNER_R_MAX + 1):
        for c in range(INNER_C_MIN, INNER_C_MAX + 1):
            if the_map[r][c] != BOX:
                continue
            up    = is_blocker(the_map, r - 1, c)
            down  = is_blocker(the_map, r + 1, c)
            left  = is_blocker(the_map, r, c - 1)
            right = is_blocker(the_map, r, c + 1)
            if (up and left) or (up and right) or (down and left) or (down and right):
                return True, (r, c)

    for r in range(INNER_R_MIN, INNER_R_MAX + 1):
        for c in range(INNER_C_MIN, INNER_C_MAX + 1):
            if the_map[r][c] != BOX:
                continue
            if (not _box_has_pushable_direction(the_map, r, c)) or \
               _box_frozen_on_border_line(the_map, r, c):
                return True, (r, c)
    return False, None


# ============================================================
# Stage 3 辅助 — 炸弹爆炸 / 墙体搜索
# ============================================================

def apply_bomb_explosion(the_map: list, wall_pos: tuple) -> None:
    """3×3 范围清除内墙。对应 C 代码 Sokoban_Apply_Bomb_Explosion()。"""
    wr, wc = wall_pos
    for dr in range(-1, 2):
        for dc in range(-1, 2):
            r, c = wr + dr, wc + dc
            if 1 <= r < MAP_ROWS - 1 and 1 <= c < MAP_COLS - 1:
                if the_map[r][c] == WALL:
                    the_map[r][c] = EMPTY


def find_bomb_wall(the_map: list, player_pos: tuple,
                   blocked_target: Optional[tuple] = None) -> Optional[tuple]:
    """
    寻找最优炸弹目标墙体。对应 C 代码 Sokoban_Find_Bomb_Wall()。

    P0-1: 评分公式已与 C 端 Sokoban_Find_Bomb_Wall / Sokoban_Plan_Bomb 统一:
          reachable * 200 + cleared * 20 - blocked_len^2 / 50.
          同时将逐目标 BFS 改为一次 Flood + O(1) 查询，与 Plan_Bomb 对齐。
    """
    best_score = float('-inf')
    best_wall  = None

    for r in range(1, MAP_ROWS - 1):
        for c in range(1, MAP_COLS - 1):
            if the_map[r][c] != WALL:
                continue

            tmp = [row[:] for row in the_map]
            cleared = 0
            for dr in range(-1, 2):
                for dc in range(-1, 2):
                    rr, cc = r + dr, c + dc
                    if 1 <= rr < MAP_ROWS - 1 and 1 <= cc < MAP_COLS - 1:
                        if tmp[rr][cc] == WALL:
                            tmp[rr][cc] = EMPTY
                            cleared += 1

            # 一次 Flood 扩散后 O(1) 查询全部目标可达性
            reachable_cells = nav_bfs_flood(tmp, player_pos)
            if not reachable_cells:
                continue

            # 若指定了被卡目标，要求此墙能解锁该目标
            if blocked_target is not None:
                if not is_reachable(reachable_cells, blocked_target):
                    continue
                path = nav_bfs(tmp, player_pos, blocked_target)
                if path is None:
                    continue
                blocked_len = len(path)
            else:
                blocked_len = 0

            reachable = sum(
                1 for tr in range(INNER_R_MIN, INNER_R_MAX + 1)
                for tc in range(INNER_C_MIN, INNER_C_MAX + 1)
                if tmp[tr][tc] == TARGET and
                is_reachable(reachable_cells, (tr, tc))
            )

            # P0-1: 与 C 端 Sokoban_Find_Bomb_Wall / Sokoban_Plan_Bomb 统一评分公式
            score = reachable * 200 + cleared * 20 - (blocked_len * blocked_len) // 50
            if score > best_score:
                best_score = score
                best_wall  = (r, c)

    return best_wall


def plan_bomb(the_map: list, player_pos: tuple,
              blocked_target: Optional[tuple] = None
              ) -> Optional[Tuple[tuple, tuple, list]]:
    """
    多炸弹联合 (炸弹, 墙体) 规划。对应 C 代码 Sokoban_Plan_Bomb()。

    与 find_bomb_wall 的区别: 对全部 (炸弹, 墙体) 组合实际求解，并按
    “不可达目标罚时 + 推炸弹执行时间 + 爆破后关键目标导航时间”选择总耗时最小者；
    清墙数量只作为同成本时的次级判据。

    blocked_target:
        给定时作为破局门控 (该墙必须恢复其可达性);
        None 时退化为"最大化可达目标 + 清墙数"的通用破局。

    返回 (bomb_pos, wall_pos, bomb_actions); 无可行解返回 None。
    """
    bombs = extract_elements(the_map, BOMB)
    if not bombs:
        return None

    best_rank = 10 ** 18
    best_cleared = -1
    best_plan: Optional[Tuple[tuple, tuple, list]] = None
    target_count = len(extract_elements(the_map, TARGET))

    for r in range(1, MAP_ROWS - 1):
        for c in range(1, MAP_COLS - 1):
            if the_map[r][c] != WALL:
                continue
            wall = (r, c)

            tmp = [row[:] for row in the_map]
            cleared = 0
            for dr in range(-1, 2):
                for dc in range(-1, 2):
                    rr, cc = r + dr, c + dc
                    if 1 <= rr < MAP_ROWS - 1 and 1 <= cc < MAP_COLS - 1:
                        if tmp[rr][cc] == WALL:
                            tmp[rr][cc] = EMPTY
                            cleared += 1

            # P0-2: 打分阶段将所有炸弹从 tmp 中清除 (镜像 C 端)
            for (br, bc) in bombs:
                if tmp[br][bc] == BOMB:
                    tmp[br][bc] = EMPTY

            reachable_cells = nav_bfs_flood(tmp, player_pos)
            if not reachable_cells:
                continue

            if blocked_target is not None:
                if not is_reachable(reachable_cells, blocked_target):
                    continue
                route = nav_time_path(tmp, player_pos, blocked_target)
                if route is None:
                    continue
                blocked_cost_ms = route[1] * SOKO_COST_MOVE_MS
            else:
                blocked_cost_ms = 0

            reachable = sum(
                1 for tr in range(INNER_R_MIN, INNER_R_MAX + 1)
                for tc in range(INNER_C_MIN, INNER_C_MAX + 1)
                if tmp[tr][tc] == TARGET and
                is_reachable(reachable_cells, (tr, tc))
            )

            # 固件分时搜索仍按距离顺序验证，但会比较同一墙体的全部可行炸弹。
            order = sorted(range(len(bombs)),
                           key=lambda i: abs(bombs[i][0] - r) + abs(bombs[i][1] - c))
            for bi in order:
                sub = [row[:] for row in the_map]
                sub[bombs[bi][0]][bombs[bi][1]] = EMPTY
                sub[wall[0]][wall[1]] = TARGET
                acts = sokoban_bfs_single(sub, player_pos, bombs[bi], wall,
                                          block_other_targets=False)
                if acts is not None:
                    flags = _push_flags(acts, player_pos, bombs[bi])
                    rank = ((target_count - reachable) * 60000 +
                            sequence_time_cost(acts, flags) + blocked_cost_ms)
                    if (rank < best_rank or
                            (rank == best_rank and cleared > best_cleared)):
                        best_rank = rank
                        best_cleared = cleared
                        best_plan = (bombs[bi], wall, acts)

    return best_plan


# ============================================================
# Stage 3 完整求解
# ============================================================

def solve_stage3(the_map: list, player_pos: tuple,
                 box_to_target_idx: Optional[list] = None) -> Optional[dict]:
    """
    Stage3 完整流程：
      1. 找炸弹位置
      2. 找不可达目标（可选）
      3. 寻找最优爆破墙体
      4. BFS 求解推炸弹路径
      5. 模拟爆炸，更新地图
      6. 用 Stage1 (任意映射) 或 Stage2 (固定映射) 继续推剩余箱子

    box_to_target_idx: 若给定 → 爆炸后用 Stage2 (固定映射) 求解;
                        否则走 Stage1 时间成本优化.
    """
    bombs = extract_elements(the_map, BOMB)
    if not bombs:
        return None

    # 一次扩散后查找首个不可达目标
    blocked_target = None
    reachable_cells = nav_bfs_flood(the_map, player_pos)
    for t in extract_elements(the_map, TARGET):
        if not is_reachable(reachable_cells, t):
            blocked_target = t
            break

    # 多炸弹联合规划: 一次性选出可行的 (炸弹, 墙体, 推炸弹动作).
    # 给定 blocked_target 时优先破局; 否则退化为通用破局 (最大化可达目标 + 清墙数).
    plan = plan_bomb(the_map, player_pos, blocked_target)
    if plan is None and blocked_target is not None:
        plan = plan_bomb(the_map, player_pos, None)
    if plan is None:
        return None

    bomb_pos, wall_pos, bomb_actions = plan
    if bomb_actions is None:
        return None

    # 模拟推炸弹后的玩家位置
    cur_player, _ = simulate_actions(bomb_actions, player_pos, bomb_pos)

    # 应用爆炸
    new_map = [row[:] for row in the_map]
    new_map[bomb_pos[0]][bomb_pos[1]] = EMPTY
    apply_bomb_explosion(new_map, wall_pos)

    # 继续推剩余箱子
    if box_to_target_idx:
        # 注意: 爆炸后 boxes 顺序不变, 但 targets 顺序可能变(原墙体上的 TARGET 是旧的)
        # box_to_target_idx 是基于"爆炸前 extract" 的索引, 与爆炸后 extract 顺序保持
        # 因为我们没有移除 boxes/targets, 仅清了内墙 → 顺序一致, 可直接复用
        stage1_result = solve_stage2(new_map, cur_player, box_to_target_idx)
    else:
        stage1_result = solve_stage1(new_map, cur_player)

    return {
        'bomb_actions':    bomb_actions,
        'bomb_pos':        bomb_pos,
        'wall_pos':        wall_pos,
        'cur_player_after_bomb': cur_player,
        'map_after_bomb':  new_map,
        'stage1_result':   stage1_result,
    }


def solve_full(the_map: list, player_pos: tuple,
               box_to_target_idx: Optional[list] = None,
               max_rounds: int = MAX_BOXES + 2,
               home_pos: tuple = (5, 1)) -> Optional[dict]:
    """
    迭代多炸弹完整求解 —— 与固件 app_game_logic 的 stage_plan/stage_execute 循环一致:

      每轮:
        1. 无箱 -> 通关
        2. 尝试整体推箱 (有 mapping 走 Stage2, 否则 Stage1); 成功 -> 追加 push 段, 通关
        3. 否则需炸弹:
             - 角落死局 -> 取最近目标做 blocked_target -> plan_bomb
             - 仍无 -> 首个不可达目标 -> plan_bomb
             - 仍无 + 有炸弹 -> 通用破局 plan_bomb(None)
           找不到任何炸弹计划 -> 失败 (死局复位)
        4. 模拟"推炸弹到墙 + 3x3 爆破", 更新地图与玩家, 回到第 1 步

    单次 solve_stage3 只放 1 颗炸弹, 面对"需 ≥2 颗炸弹"的图 (如先解箱体死局再开通道)
    会失败; 本函数循环放弹直到可推箱或无解, 对应固件状态机的真实行为。

    返回 {phases, is_solved, map_after, player_after}; 无解返回 None。
    phases: 有序列表, 每项 {kind:'bomb'/'push', actions, movable, wall(仅bomb)}
    """
    work = [row[:] for row in the_map]
    player = player_pos
    phases: list = []

    def nearest_target(ref):
        best, bd = None, 1 << 30
        for (r, c) in extract_elements(work, TARGET):
            d = abs(r - ref[0]) + abs(c - ref[1])
            if d < bd:
                bd, best = d, (r, c)
        return best

    def first_unreachable():
        for t in extract_elements(work, TARGET):
            if nav_bfs(work, player, t) is None:
                return t
        return None

    for _ in range(max_rounds):
        boxes = extract_elements(work, BOX)
        if not boxes:
            return {'phases': phases, 'is_solved': True,
                    'map_after': work, 'player_after': player}

        # 整体推箱尝试
        if box_to_target_idx:
            push = solve_stage2(work, player, box_to_target_idx)
        else:
            push = solve_stage1(work, player, home_pos=home_pos)

        if push is not None:
            cur = player
            for sub in push['sub_solutions']:
                phases.append({
                    'kind': 'push',
                    'actions': sub['actions'],
                    'movable': push['boxes'][sub['box_idx']],
                    'player_start': cur,
                })
                cur = sub['player_end']
            return {'phases': phases, 'is_solved': True,
                    'map_after': work, 'player_after': cur,
                    'return_plan': push.get('return_plan')}

        # 需炸弹 (镜像 stage_plan_handler 决策链)
        dead, dbox = check_deadlock(work)
        plan = None
        if dead:
            bt = nearest_target(dbox)
            if bt is not None:
                plan = plan_bomb(work, player, bt)
        if plan is None:
            ut = first_unreachable()
            if ut is not None:
                plan = plan_bomb(work, player, ut)
        if plan is None and extract_elements(work, BOMB):
            plan = plan_bomb(work, player, None)

        if plan is None:
            return None     # 无可行炸弹 -> 死局复位

        bomb, wall, acts = plan
        phases.append({'kind': 'bomb', 'actions': acts,
                       'movable': bomb, 'wall': wall, 'player_start': player})
        pe, bend = simulate_actions(acts, player, bomb)
        assert bend == wall, (bend, wall)
        work[bomb[0]][bomb[1]] = EMPTY
        apply_bomb_explosion(work, wall)
        work[wall[0]][wall[1]] = EMPTY
        player = pe

    return None     # 轮次预算耗尽


def flatten_stage3(raw: Optional[dict], base_map: list) -> Optional[dict]:
    """将 solve_stage3 结果展平为 GUI/脚本 统一的 sub_solutions 列表。"""
    if raw is None:
        return None
    subs = [{
        'phase':      'bomb',
        'actions':    raw['bomb_actions'],
        'box_idx':    -1,
        'target_idx': -1,
        'player_end': raw['cur_player_after_bomb'],
        'box_start':  raw['bomb_pos'],
    }]
    s1 = raw.get('stage1_result')
    wall_pos = raw.get('wall_pos')
    if s1:
        for item in s1['sub_solutions']:
            d = dict(item)
            d['phase'] = 'push'
            d['box_start'] = s1['boxes'][item['box_idx']]
            subs.append(d)
        return {
            'sub_solutions': subs,
            'boxes': s1['boxes'],
            'targets': s1['targets'],
            'total_steps': sum(len(x['actions']) for x in subs),
            '_wall_pos': wall_pos,
            'stage': 3,
        }
    return {
        'sub_solutions': subs,
        'boxes': extract_elements(base_map, BOX),
        'targets': extract_elements(base_map, TARGET),
        'total_steps': len(raw['bomb_actions']),
        '_wall_pos': wall_pos,
        'stage': 3,
    }


def solve_level(stage: int, the_map: list, player_pos: tuple,
                box_to_target: Optional[list] = None,
                require_scout: bool = True,
                box_classes: Optional[list] = None,
                target_classes: Optional[list] = None) -> Optional[dict]:
    """
    统一关卡求解（含 Stage2/3 侦查阶段）。
    stage: 1/2/3
    require_scout: Stage2/3 是否先跑侦查再推箱/炸弹
    box_classes / target_classes: 视觉端真值 (1..N), 仅 Stage2/3 用
        - 默认 None → 按 extract 顺序 1..N (一一对应)
    """
    boxes = extract_elements(the_map, BOX)
    targets = extract_elements(the_map, TARGET)
    scout = None
    work_player = player_pos
    scout_mapping: Optional[list] = None

    if stage >= 2 and require_scout and boxes:
        # Stage3 默认不强求侦查全部 box+target (横墙隔断时 target 暂不可达,
        # 真实赛规是炸墙后再识别 target). 仅在用户显式传 box_classes/target_classes
        # 时, Stage3 也启用 v2 (假设地图本身允许全部可达).
        do_v2 = (stage == 2) or (
            stage == 3 and (box_classes is not None or target_classes is not None)
        )
        if do_v2:
            scout = plan_scout_phase_v2(the_map, player_pos,
                                         box_classes=box_classes,
                                         target_classes=target_classes)
            if not scout['all_visited']:
                return None
            work_player = scout['player_after_scout']
            scout_mapping = scout.get('box_to_target_idx') or None
        elif stage == 3:
            # 保持旧行为: Stage3 不做侦查, 直接进炸弹+stage1
            scout = None
            scout_mapping = None
        else:
            scout = plan_scout_phase_v2(the_map, player_pos,
                                         box_classes=box_classes,
                                         target_classes=target_classes)
            if not scout['all_visited']:
                return None
            work_player = scout['player_after_scout']
            scout_mapping = scout.get('box_to_target_idx') or None

    push_result = None
    if stage == 1:
        push_result = solve_stage1(the_map, work_player, home_pos=(5, 1))
    elif stage == 2:
        # 优先用侦查得到的映射; 外部显式传入则覆盖
        if box_to_target is None:
            box_to_target = scout_mapping or default_box_mapping(boxes, targets)
        push_result = solve_stage2(the_map, work_player, box_to_target)
    elif stage == 3:
        raw3 = solve_stage3(the_map, work_player,
                             box_to_target_idx=(box_to_target or scout_mapping))
        push_result = flatten_stage3(raw3, the_map)
    else:
        return None

    if push_result is None:
        return None

    subs = []
    if scout and scout['scout_actions']:
        subs.append({
            'phase':      'scout',
            'actions':    scout['scout_actions'],
            'box_idx':    -2,
            'target_idx': -1,
            'player_end': scout['player_after_scout'],
            'box_start':  None,
            'visits':     scout['visits'],
        })

    for sub in push_result.get('sub_solutions', []):
        s = dict(sub)
        if 'phase' not in s:
            s['phase'] = 'bomb' if s.get('box_idx') == -1 else 'push'
        subs.append(s)

    total = sum(len(s['actions']) for s in subs)
    out = {
        'sub_solutions': subs,
        'boxes':         push_result.get('boxes', boxes),
        'targets':       push_result.get('targets', targets),
        'total_steps':   total,
        'scout':         scout,
        'stage':         stage,
        '_player_start': player_pos,
        '_scout_start':  player_pos,
    }
    if push_result.get('_wall_pos'):
        out['_wall_pos'] = push_result['_wall_pos']
    return out


# ============================================================
# 运行指令解析与验证
# 格式说明:
#   SCOUT: <路点或方向>   — 侦查（仅行走，不可推箱）
#   PUSH:  <UDLR串>       — 推箱阶段（小写走 大写推）
#   BOMB:  <UDLR串>       — 推炸弹（Stage3）
#   路点: W列,行 或 W列 行，多个用 ; 或空格分隔
#   无标签时整段视为 PUSH
#   多段用空行或 --- 分隔
# ============================================================

def _parse_waypoints_token(tok: str) -> Optional[tuple]:
    tok = tok.strip().upper()
    if not tok.startswith('W'):
        return None
    body = tok[1:].strip()
    for sep in (',', ' '):
        if sep in body:
            parts = [p.strip() for p in body.replace(',', ' ').split() if p.strip()]
            if len(parts) >= 2:
                c, r = int(parts[0]), int(parts[1])
                return (r, c)
    return None


def parse_run_script(text: str) -> dict:
    """解析用户输入的运行脚本，返回 {scout, push, bomb} 各段。"""
    result = {'scout': None, 'push': None, 'bomb': None, 'raw_lines': []}
    current = None
    buf: list = []

    def flush():
        nonlocal current, buf
        if current and buf:
            result[current] = '\n'.join(buf).strip()
        buf = []

    for line in text.splitlines():
        s = line.strip()
        if not s or s.startswith('#'):
            continue
        if s == '---':
            flush()
            current = None
            continue
        upper = s.upper()
        if upper.startswith('SCOUT:'):
            flush()
            current = 'scout'
            rest = s.split(':', 1)[1].strip()
            if rest:
                buf.append(rest)
            continue
        if upper.startswith('PUSH:'):
            flush()
            current = 'push'
            rest = s.split(':', 1)[1].strip()
            if rest:
                buf.append(rest)
            continue
        if upper.startswith('BOMB:'):
            flush()
            current = 'bomb'
            rest = s.split(':', 1)[1].strip()
            if rest:
                buf.append(rest)
            continue
        if current is None:
            current = 'push'
        buf.append(s)

    flush()
    if result['scout'] is None and result['push'] is None and result['bomb'] is None:
        joined = '\n'.join(l for l in text.splitlines() if l.strip() and not l.strip().startswith('#'))
        if joined:
            result['push'] = joined.replace('|', '\n')
    return result


def _script_scout_to_actions(the_map: list, player: tuple, scout_text: str) -> Tuple[list, tuple, str]:
    """将 SCOUT 段转为动作序列。"""
    scout_text = scout_text.strip()
    if not scout_text:
        return [], player, ""

    if scout_text.upper().startswith('W') or ';' in scout_text or ',' in scout_text:
        tokens = scout_text.replace(';', ' ').split()
        actions_all = []
        cur = player
        for tok in tokens:
            wp = _parse_waypoints_token(tok)
            if wp is None:
                return [], player, f"无法解析路点: {tok}"
            path = nav_bfs(the_map, cur, wp)
            if path is None:
                return [], player, f"无法到达路点 {tok}"
            actions_all.extend(path_to_actions(path))
            cur = wp
        return actions_all, cur, ""

    actions, err = parse_direction_script(scout_text, allow_push=False)
    if err:
        return [], player, err
    ok, msg, end_p, _ = simulate_player_only(the_map, player, actions)
    if not ok:
        return [], player, f"侦查段: {msg}"
    return actions, end_p, ""


def parse_direction_script(script: str, allow_push: bool = True) -> Tuple[list, str]:
    """
    解析 UDLR 方向串。小写=行走，大写=推箱/推炸弹。
    allow_push=False 时拒绝大写字母。
    """
    actions = []
    for i, ch in enumerate(script):
        if ch in ' \t\r\n|':
            continue
        if ch in 'udlr':
            actions.append('UDLR'.index(ch.upper()))
        elif ch in 'UDLR':
            if not allow_push:
                return [], f"位置 {i+1}: 侦查段不允许大写推箱 '{ch}'"
            actions.append('UDLR'.index(ch))
        else:
            return [], f"位置 {i+1}: 非法字符 '{ch}'"
    return actions, ""


def simulate_player_only(the_map: list, player: tuple,
                         actions: list) -> Tuple[bool, str, tuple, list]:
    """仅移动玩家（不推箱），用于侦查段校验。"""
    pr, pc = player
    for step, d in enumerate(actions):
        npr, npc = pr + DR[d], pc + DC[d]
        if not is_inner(npr, npc):
            return False, f"步骤 {step+1}: 越界", (pr, pc), []
        cell = the_map[npr][npc]
        if cell not in (EMPTY, TARGET):
            return False, f"步骤 {step+1}: 不可进入 ({npc},{npr}) cell={cell}", (pr, pc), []
        pr, pc = npr, npc
    return True, "OK", (pr, pc), actions


class LevelSimulator:
    """完整关卡模拟：侦查 / 推炸弹+爆炸 / 推箱。"""

    def __init__(self, initial_map: list, player_pos: tuple):
        self.map = [row[:] for row in initial_map]
        self.player = player_pos
        self.boxes = extract_elements(initial_map, BOX)
        self.bomb_pos = None
        bombs = extract_elements(initial_map, BOMB)
        if bombs:
            self.bomb_pos = bombs[0]
        self.wall_pos = None
        self.step_log: list = []

    def _log(self, msg: str):
        self.step_log.append(msg)

    def run_scout(self, actions: list) -> Tuple[bool, str]:
        ok, msg, end, _ = simulate_player_only(self.map, self.player, actions)
        if ok:
            self.player = end
            self._log(f"侦查完成 → ({end[1]},{end[0]})")
        return ok, msg

    def run_push_phase(self, actions: list) -> Tuple[bool, str]:
        """推箱段：支持地图上所有箱子的逐步推动。"""
        pr, pc = self.player
        for step, d in enumerate(actions):
            npr, npc = pr + DR[d], pc + DC[d]
            if not is_inner(npr, npc):
                return False, f"步骤 {step+1}: 玩家越界"

            pushed = False
            for bi, (br, bc) in enumerate(self.boxes):
                if npr == br and npc == bc:
                    nbr, nbc = br + DR[d], bc + DC[d]
                    if not is_inner(nbr, nbc):
                        return False, f"步骤 {step+1}: 推箱越界"
                    dest = self.map[nbr][nbc]
                    if dest not in (EMPTY, TARGET):
                        return False, f"步骤 {step+1}: 推箱目标被挡 cell={dest}"
                    self.map[br][bc] = EMPTY
                    self.map[nbr][nbc] = TARGET if dest == TARGET else BOX
                    self.boxes[bi] = (nbr, nbc)
                    pushed = True
                    break

            if not pushed:
                cell = self.map[npr][npc]
                if cell not in (EMPTY, TARGET):
                    return False, f"步骤 {step+1}: 行走受阻 ({npc},{npr}) cell={cell}"

            pr, pc = npr, npc

        self.player = (pr, pc)
        return True, "OK"

    def run_bomb_phase(self, actions: list) -> Tuple[bool, str]:
        if self.bomb_pos is None:
            return False, "地图无炸弹"
        valid, msg = validate_path(self.map, actions, self.player, self.bomb_pos)
        if not valid:
            return False, msg

        pr, pc = self.player
        br, bc = self.bomb_pos
        wall_hit = None

        for step, d in enumerate(actions):
            npr, npc = pr + DR[d], pc + DC[d]
            if npr == br and npc == bc:
                nbr, nbc = br + DR[d], bc + DC[d]
                if self.map[nbr][nbc] == WALL:
                    wall_hit = (nbr, nbc)
                self.map[br][bc] = EMPTY
                self.map[nbr][nbc] = BOMB
                br, bc = nbr, nbc
                self.bomb_pos = (br, bc)
            pr, pc = npr, npc

        self.player = (pr, pc)
        if wall_hit is None:
            return False, "炸弹未推到墙体，无法爆炸"

        self.wall_pos = wall_hit
        apply_bomb_explosion(self.map, wall_hit)
        self.map[self.bomb_pos[0]][self.bomb_pos[1]] = EMPTY
        self.bomb_pos = None
        self._log(f"爆炸于 ({wall_hit[1]},{wall_hit[0]})，3×3 清墙")
        return True, "OK"

    def is_win(self) -> bool:
        return all(self.map[r][c] == TARGET for r, c in self.boxes)


def verify_run_script(the_map: list, player_pos: tuple, script_text: str,
                      stage: int = 1,
                      expect_scout: bool = False) -> dict:
    """
    验证用户输入的运行指令能否完成关卡。
    返回 {ok, message, simulator, parsed, phases_done}
    """
    parsed = parse_run_script(script_text)
    sim = LevelSimulator(the_map, player_pos)
    phases_done = []

    if parsed.get('scout'):
        acts, _, err = _script_scout_to_actions(the_map, sim.player, parsed['scout'])
        if err:
            return {'ok': False, 'message': err, 'simulator': sim, 'parsed': parsed,
                    'phases_done': phases_done}
        ok, msg = sim.run_scout(acts)
        phases_done.append('scout')
        if not ok:
            return {'ok': False, 'message': msg, 'simulator': sim, 'parsed': parsed,
                    'phases_done': phases_done}

    elif expect_scout and stage >= 2:
        boxes = extract_elements(the_map, BOX)
        if boxes:
            return {'ok': False,
                    'message': 'Stage2/3 需要 SCOUT 段（先访问所有箱子）',
                    'simulator': sim, 'parsed': parsed, 'phases_done': phases_done}

    if parsed.get('bomb'):
        acts, err = parse_direction_script(parsed['bomb'], allow_push=True)
        if err:
            return {'ok': False, 'message': err, 'simulator': sim, 'parsed': parsed,
                    'phases_done': phases_done}
        ok, msg = sim.run_bomb_phase(acts)
        phases_done.append('bomb')
        if not ok:
            return {'ok': False, 'message': msg, 'simulator': sim, 'parsed': parsed,
                    'phases_done': phases_done}

    if parsed.get('push'):
        push_text = parsed['push'].replace('\n', '').replace('|', '')
        acts, err = parse_direction_script(push_text, allow_push=True)
        if err:
            return {'ok': False, 'message': err, 'simulator': sim, 'parsed': parsed,
                    'phases_done': phases_done}
        if not sim.boxes:
            return {'ok': False, 'message': '推箱段：地图上无箱子',
                    'simulator': sim, 'parsed': parsed, 'phases_done': phases_done}
        ok, msg = sim.run_push_phase(acts)
        phases_done.append('push')
        if not ok:
            return {'ok': False, 'message': msg, 'simulator': sim, 'parsed': parsed,
                    'phases_done': phases_done}

    elif stage >= 1 and not parsed.get('bomb'):
        return {'ok': False, 'message': '缺少 PUSH 推箱指令',
                'simulator': sim, 'parsed': parsed, 'phases_done': phases_done}

    if sim.is_win():
        return {'ok': True, 'message': '★ 通关验证成功',
                'simulator': sim, 'parsed': parsed, 'phases_done': phases_done}

    on_t = sum(1 for r, c in sim.boxes if sim.map[r][c] == TARGET)
    return {'ok': False,
            'message': f'未完成：{on_t}/{len(sim.boxes)} 箱在目标上',
            'simulator': sim, 'parsed': parsed, 'phases_done': phases_done}


def build_script_from_solution(sol: dict, use_unicode: bool = False) -> str:
    """从求解结果生成可粘贴的运行脚本。"""
    lines = ['# 自动生成运行脚本 — 小写行走 大写推箱/炸弹']
    cur_p = sol.get('_player_start', (0, 0))
    push_parts = []

    for sub in sol.get('sub_solutions', []):
        phase = sub.get('phase', 'push')
        acts = sub['actions']
        if phase == 'scout':
            lines.append('SCOUT: ' + ''.join(
                ch.lower() for ch in actions_to_string(
                    acts, cur_p, cur_p, use_unicode=False)))
            cur_p = sub['player_end']
            continue
        if phase == 'bomb':
            lines.append('BOMB: ' + actions_to_string(
                acts, cur_p, sub['box_start'], use_unicode=False))
            cur_p = sub['player_end']
        else:
            bi = sub['box_idx']
            b0 = sub.get('box_start', sol['boxes'][bi] if bi >= 0 else cur_p)
            push_parts.append(actions_to_string(acts, cur_p, b0, use_unicode=False))
            cur_p = sub['player_end']

    if push_parts:
        lines.append('PUSH: ' + ''.join(push_parts))
    return '\n'.join(lines)


# ============================================================
# 动作序列 → 方向字符串
# ============================================================

def actions_to_string(actions: list, player_start: tuple,
                      box_start: tuple, use_unicode: bool = True) -> str:
    """
    生成方向字符串：推箱动作用大写，行走动作用小写。
    使用 Unicode 箭头或 ASCII 字母。
    """
    pr, pc = player_start
    br, bc = box_start
    chars = []
    arrows = DIR_ARROW if use_unicode else DIR_LETTER

    for d in actions:
        npr = pr + DR[d]
        npc = pc + DC[d]
        is_push = (npr == br and npc == bc)

        if use_unicode:
            # 推箱用【箭头】区别，行走直接用箭头
            chars.append(f'[{arrows[d]}]' if is_push else arrows[d])
        else:
            letter = DIR_LETTER[d]
            chars.append(letter.upper() if is_push else letter.lower())

        if is_push:
            br += DR[d]
            bc += DC[d]
        pr, pc = npr, npc

    return ''.join(chars)


def actions_to_waypoints(actions: list, start_pos: tuple) -> list:
    """转弯点压缩。对应 C 代码 Sokoban_Actions_To_Waypoints()。"""
    waypoints = []
    r, c = start_pos
    for i, d in enumerate(actions):
        r += DR[d]
        c += DC[d]
        if i == len(actions) - 1 or actions[i] != actions[i + 1]:
            waypoints.append((r, c))
    return waypoints


def actions_to_typed_waypoints(actions: list, push_flags: list,
                               start_pos: tuple) -> list:
    """转弯点压缩，并标记含推动作的关键航段。"""
    if len(actions) != len(push_flags):
        raise ValueError("actions 与 push_flags 长度必须一致")
    waypoints = []
    r, c = start_pos
    segment_has_push = False
    for i, d in enumerate(actions):
        r += DR[d]
        c += DC[d]
        segment_has_push = segment_has_push or bool(push_flags[i])
        if i + 1 == len(actions) or actions[i] != actions[i + 1]:
            waypoints.append(((r, c), 'critical' if segment_has_push else 'walk'))
            segment_has_push = False
    return waypoints


# ============================================================
# 路径合法性校验
# ============================================================

def validate_path(the_map: list, actions: list, player_start: tuple,
                  box_start: tuple) -> Tuple[bool, str]:
    """
    逐步校验路径合法性：
      - 玩家不走出边界或进入墙壁
      - 推箱时箱子前方可通行
    返回 (is_valid, error_message)。
    """
    pr, pc = player_start
    br, bc = box_start

    for step, d in enumerate(actions):
        npr = pr + DR[d]
        npc = pc + DC[d]

        if not is_inner(npr, npc):
            return False, f"步骤 {step+1}: 玩家越界 ({npc},{npr})"

        cell = the_map[npr][npc]

        if npr == br and npc == bc:
            # 推箱
            nbr = br + DR[d]
            nbc = bc + DC[d]
            if not is_inner(nbr, nbc):
                return False, f"步骤 {step+1}: 推箱越界 ({nbc},{nbr})"
            dest_cell = the_map[nbr][nbc]
            if dest_cell not in (EMPTY, TARGET):
                return False, f"步骤 {step+1}: 推箱目标格被阻挡 ({nbc},{nbr}) cell={dest_cell}"
            br, bc = nbr, nbc
        else:
            if cell not in (EMPTY, TARGET):
                return False, f"步骤 {step+1}: 行走进入非空格 ({npc},{npr}) cell={cell}"

        pr, pc = npr, npc

    return True, "路径合法"


def check_win(the_map: list, boxes_current: list) -> bool:
    """判断所有箱子是否到达目标点（通关条件）。"""
    if not boxes_current:
        return True
    return all(the_map[r][c] == TARGET for r, c in boxes_current)


# ============================================================
# 地图渲染
# ============================================================

def render_map(the_map: list, player_pos: tuple,
               boxes: list, use_unicode: bool = True) -> str:
    """渲染当前地图状态为字符串。"""
    pr, pc = player_pos
    box_set = set(boxes)
    cell_chars = CELL_UNICODE if use_unicode else CELL_ASCII
    boc = BOX_ON_TARGET_CHAR if use_unicode else BOX_ON_TARGET_ASCII
    pl_ch = PLAYER_CHAR
    pl_tgt = PLAYER_ON_TARGET if use_unicode else PLAYER_ON_TARGET_ASCII

    lines = []
    # 列号标尺
    header = '  ' + ''.join(f'{c%10}' for c in range(MAP_COLS))
    lines.append(header)

    for r in range(MAP_ROWS):
        row_str = f'{r:2d}'
        for c in range(MAP_COLS):
            cell = the_map[r][c]
            pos  = (r, c)

            if pos == (pr, pc):
                row_str += (pl_tgt if cell == TARGET else pl_ch)
            elif pos in box_set:
                row_str += (boc if cell == TARGET else cell_chars.get(BOX, 'B'))
            else:
                row_str += cell_chars.get(cell, '?')
        lines.append(row_str)

    return '\n'.join(lines)


def clear_screen() -> None:
    os.system('cls' if os.name == 'nt' else 'clear')


# ============================================================
# 地图自动生成
# ============================================================

def _inner_cells() -> list:
    return [(r, c) for r in range(INNER_R_MIN, INNER_R_MAX + 1)
            for c in range(INNER_C_MIN, INNER_C_MAX + 1)]


def _make_empty_map() -> list:
    m = [[EMPTY] * MAP_COLS for _ in range(MAP_ROWS)]
    for r in range(MAP_ROWS):
        for c in range(MAP_COLS):
            if r == 0 or r == MAP_ROWS - 1 or c == 0 or c == MAP_COLS - 1:
                m[r][c] = WALL
    return m


def _is_solvable(the_map: list, player_pos: tuple, box_count: int,
                 stage: int = 1) -> bool:
    """可解性检测。Stage3 使用 solve_stage3，其余用 Stage1 BFS。"""
    boxes   = extract_elements(the_map, BOX)
    targets = extract_elements(the_map, TARGET)
    if len(boxes) != box_count or len(targets) != box_count:
        return False
    if stage == 3:
        return solve_level(3, the_map, player_pos) is not None
    if stage == 2:
        boxes = extract_elements(the_map, BOX)
        targets = extract_elements(the_map, TARGET)
        mapping = default_box_mapping(boxes, targets)
        return solve_level(2, the_map, player_pos, mapping) is not None
    dl, _ = check_deadlock(the_map)
    if dl:
        return False
    return solve_stage1(the_map, player_pos) is not None


def _gen_stage3_map(box_count: int, wall_density: float,
                    rng) -> Tuple[Optional[list], Optional[tuple]]:
    """
    专用 Stage3 地图生成：
      - 用一条横墙将内场分为上下两半，使目标完全不可直达
      - 玩家、箱子、炸弹在上半部；目标在下半部
      - 炸弹紧贴墙上方，可被推入墙体触发 3×3 爆炸
      - 用 solve_stage3 验证可解
    """
    for _ in range(120):
        m = _make_empty_map()

        # 选墙行（内场中段 row 3~8）
        wall_r = rng.randint(INNER_R_MIN + 2, INNER_R_MAX - 2)

        # 填充整行横墙（内场全宽）
        for c in range(INNER_C_MIN, INNER_C_MAX + 1):
            m[wall_r][c] = WALL

        # 上半 / 下半可用空格
        top_cells = [(r, c) for r in range(INNER_R_MIN, wall_r)
                     for c in range(INNER_C_MIN, INNER_C_MAX + 1)
                     if m[r][c] == EMPTY]
        bot_cells = [(r, c) for r in range(wall_r + 1, INNER_R_MAX + 1)
                     for c in range(INNER_C_MIN, INNER_C_MAX + 1)
                     if m[r][c] == EMPTY]

        # 上半需要：1 玩家 + box_count 箱子 + 1 炸弹
        if len(top_cells) < 2 + box_count or len(bot_cells) < box_count:
            continue

        rng.shuffle(top_cells)
        rng.shuffle(bot_cells)

        idx = 0
        player_pos = top_cells[idx]; idx += 1

        boxes = []
        for _ in range(box_count):
            br, bc = top_cells[idx]; idx += 1
            m[br][bc] = BOX
            boxes.append((br, bc))

        # 炸弹优先放在紧贴横墙上方一行（便于向下推入墙体）
        bomb_row = wall_r - 1
        bomb_cands = [
            (bomb_row, c) for c in range(INNER_C_MIN + 1, INNER_C_MAX)
            if m[bomb_row][c] == EMPTY
            and (bomb_row, c) != player_pos
            and (bomb_row, c) not in boxes
        ]
        if bomb_cands:
            bomb_pos = rng.choice(bomb_cands)
        else:
            remaining = [p for p in top_cells[idx:]
                         if p not in boxes and p != player_pos]
            if not remaining:
                continue
            bomb_pos = remaining[0]

        m[bomb_pos[0]][bomb_pos[1]] = BOMB

        # 目标放在下半
        for tr, tc in bot_cells[:box_count]:
            m[tr][tc] = TARGET

        # 可选：上半随机少量内墙增加趣味性
        extra_n = max(0, int(len(top_cells) * wall_density * 0.4))
        extra_pool = [
            p for p in top_cells[idx + 1:]
            if p not in boxes and p != player_pos and p != bomb_pos
        ]
        rng.shuffle(extra_pool)
        for p in extra_pool[:extra_n]:
            m[p[0]][p[1]] = WALL

        try:
            sol = solve_stage3(m, player_pos)
            if sol is not None:
                # 要求爆炸后 Stage1 也可解（有箱子时必须验证）
                s1 = sol.get('stage1_result')
                boxes_left = extract_elements(m, BOX)
                if s1 is not None or len(boxes_left) == 0:
                    return m, player_pos
        except Exception:
            continue

    return None, None


def generate_map(stage: int = 1, box_count: int = 2,
                 wall_density: float = 0.10, seed: Optional[int] = None) -> Tuple[list, tuple]:
    """
    自动生成地图。返回 (map, player_pos)。
    stage=3 时使用专用横墙+炸弹生成器。
    """
    rng = random.Random(seed)
    if seed is not None:
        random.seed(seed)  # 同步全局随机以兼容旧调用

    # Stage3 专用生成器
    if stage == 3:
        m, p = _gen_stage3_map(box_count, wall_density, rng)
        if m is not None:
            return m, p
        return _make_simple_map(stage, box_count), (INNER_R_MIN + 1, INNER_C_MIN + 1)

    max_attempts = 50
    for _ in range(max_attempts):
        m = _make_empty_map()
        cells = _inner_cells()
        random.shuffle(cells)

        # 随机内墙（边缘两格内不放，避免完全堵死）
        wall_n = int(len(cells) * wall_density)
        wall_idx = 0
        placed_walls = 0
        taken = set()

        while placed_walls < wall_n and wall_idx < len(cells):
            r, c = cells[wall_idx]
            wall_idx += 1
            if r <= INNER_R_MIN + 1 or r >= INNER_R_MAX - 1:
                continue
            if c <= INNER_C_MIN + 1 or c >= INNER_C_MAX - 1:
                continue
            m[r][c] = WALL
            taken.add((r, c))
            placed_walls += 1

        avail = [p for p in cells if p not in taken]
        if len(avail) < 1 + box_count * 2:
            continue

        random.shuffle(avail)
        idx = 0

        player_pos = avail[idx]; idx += 1

        boxes_placed = []
        for _ in range(box_count):
            boxes_placed.append(avail[idx]); idx += 1
            m[avail[idx - 1][0]][avail[idx - 1][1]] = BOX

        targets_placed = []
        for _ in range(box_count):
            targets_placed.append(avail[idx]); idx += 1
            m[avail[idx - 1][0]][avail[idx - 1][1]] = TARGET

        if _is_solvable(m, player_pos, box_count, stage):
            return m, player_pos

    # 回退：简单安全地图
    return _make_simple_map(stage, box_count), (INNER_R_MIN + 1, INNER_C_MIN + 1)


def _make_simple_map(stage: int, box_count: int) -> list:
    """生成简单无障碍地图。"""
    m = _make_empty_map()
    cells = _inner_cells()
    random.shuffle(cells)
    idx = 0
    for _ in range(box_count):
        r, c = cells[idx]; idx += 1
        m[r][c] = BOX
    for _ in range(box_count):
        r, c = cells[idx]; idx += 1
        m[r][c] = TARGET
    if stage == 3 and idx < len(cells):
        r, c = cells[idx]; idx += 1
        m[r][c] = BOMB
    return m


# ============================================================
# 动画模拟引擎
# ============================================================

class Simulator:
    """逐步动画模拟器。"""

    def __init__(self, initial_map: list, player_pos: tuple, use_unicode: bool = True):
        self.initial_map = [row[:] for row in initial_map]
        # 工作地图：剥离 BOX 标记，箱子位置改用 self.boxes 列表独立跟踪
        # 这样 self.map 只含 WALL/EMPTY/TARGET/BOMB，渲染无歧义
        self.map = [row[:] for row in initial_map]
        for r in range(MAP_ROWS):
            for c in range(MAP_COLS):
                if self.map[r][c] == BOX:
                    self.map[r][c] = EMPTY
        self.player      = player_pos
        self.use_unicode = use_unicode
        self.boxes       = extract_elements(initial_map, BOX)  # [(row,col), ...]
        self.step_count  = 0
        self.push_count  = 0

    def _render(self, extra_info: str = '') -> None:
        clear_screen()
        print("=" * 50)
        print(" 推箱子算法验证器 — 动态模拟")
        print("=" * 50)
        print(render_map(self.map, self.player, self.boxes, self.use_unicode))
        print(f"\n 步数: {self.step_count}  推箱: {self.push_count}")
        if extra_info:
            print(f" {extra_info}")
        # 通关检测
        if check_win(self.map, self.boxes):
            print("\n ★ 通关！所有箱子已到达目标位置 ★")

    def run_phase(self, actions: list, box_start: tuple, box_idx: int,
                  target_pos: tuple, phase_label: str,
                  delay: float = 0.25, step_mode: bool = False) -> None:
        """
        执行一个子任务的动画。
        """
        dir_str = actions_to_string(actions, self.player, box_start, self.use_unicode)
        waypoints = actions_to_waypoints(actions, self.player)

        print(f"\n{'='*50}")
        print(f" {phase_label}")
        print(f" 方向指令串: {dir_str}")
        print(f" 总步数: {len(actions)}  转弯路点: {len(waypoints)}")
        print(f" 路点序列: {waypoints}")

        # 路径合法性校验
        valid, msg = validate_path(self.map, actions, self.player, box_start)
        status = "✓ 路径合法" if valid else f"✗ {msg}"
        print(f" 校验结果: {status}")

        print(f"\n 按 Enter 开始动画，输入 's' 跳过，'q' 退出: ", end='', flush=True)
        choice = input().strip().lower()
        if choice == 'q':
            sys.exit(0)
        if choice == 's':
            # 快速跳过：逐步模拟以正确统计推箱次数
            pr2, pc2 = self.player
            br2, bc2 = box_start
            push_n = 0
            for d in actions:
                npr2 = pr2 + DR[d]
                npc2 = pc2 + DC[d]
                if npr2 == br2 and npc2 == bc2:
                    br2 += DR[d]
                    bc2 += DC[d]
                    push_n += 1
                pr2, pc2 = npr2, npc2
            if box_idx < len(self.boxes):
                self.boxes[box_idx] = (br2, bc2)
            self.player = (pr2, pc2)
            self.step_count += len(actions)
            self.push_count += push_n
            return

        # 逐步动画
        pr, pc = self.player
        br, bc = box_start

        for i, d in enumerate(actions):
            npr = pr + DR[d]
            npc = pc + DC[d]
            is_push = (npr == br and npc == bc)
            info_parts = []

            if is_push:
                nbr = br + DR[d]
                nbc = bc + DC[d]
                # self.map 中没有 BOX 标记，直接更新 boxes 列表即可
                if box_idx < len(self.boxes):
                    self.boxes[box_idx] = (nbr, nbc)
                br, bc = nbr, nbc
                self.push_count += 1
                action_label = f"[推箱] {DIR_ARROW[d] if self.use_unicode else DIR_LETTER[d].upper()}"
                info_parts.append(action_label)
            else:
                action_label = f"[行走] {DIR_ARROW[d] if self.use_unicode else DIR_LETTER[d]}"
                info_parts.append(action_label)

            pr, pc = npr, npc
            self.player = (pr, pc)
            self.step_count += 1

            info_parts.append(f"步 {i+1}/{len(actions)}")
            self._render(' | '.join(info_parts))

            if step_mode:
                input(" 按 Enter 下一步...")
            else:
                time.sleep(delay)

        # 最终同步箱子位置（循环正常结束时 br,bc 即最终位置）
        if box_idx < len(self.boxes):
            self.boxes[box_idx] = (br, bc)

    def show_final(self) -> None:
        """显示最终结果。"""
        self._render()
        won = check_win(self.map, self.boxes)
        print("\n" + "=" * 50)
        if won:
            print(" ★★★ 通关验证成功！所有箱子均在目标位置 ★★★")
        else:
            on_target = sum(1 for r, c in self.boxes if self.map[r][c] == TARGET)
            print(f" 未完全通关：{on_target}/{len(self.boxes)} 个箱子到位")
        print(f" 总步数: {self.step_count}  推箱次数: {self.push_count}")
        print("=" * 50)


# ============================================================
# 主交互流程
# ============================================================

def print_header() -> None:
    print("=" * 60)
    print("   推箱子算法验证器 (对应 algo_sokoban_solver.c)")
    print("   地图: 12×16  内场: 行[1-10] 列[1-14]")
    print("=" * 60)


def prompt_int(msg: str, lo: int, hi: int, default: int) -> int:
    while True:
        s = input(f"{msg} [{lo}-{hi}, 默认{default}]: ").strip()
        if s == '':
            return default
        try:
            v = int(s)
            if lo <= v <= hi:
                return v
        except ValueError:
            pass
        print(f"  请输入 {lo}~{hi} 之间的整数。")


def stage1_flow(the_map: list, player_pos: tuple, use_unicode: bool) -> None:
    print("\n[Stage1] 连续计时时间优化：推箱 + 航点 + Snap + 返库")
    print("  正在求解...", end='', flush=True)
    t0 = time.time()
    result = solve_stage1(the_map, player_pos)
    elapsed = time.time() - t0
    if result is None:
        print("\n  ✗ 无解（地图可能存在死局或不连通）")
        return
    print(f" 完成 ({elapsed:.3f}s)")
    print(f"  箱子数: {len(result['boxes'])}  总步数: {result['total_steps']}")
    print(f"  子任务数: {len(result['sub_solutions'])}")

    # 打印全局方向字符串（按子任务拼接）
    full_dir = []
    cur_p = player_pos
    boxes = result['boxes']
    for sub in result['sub_solutions']:
        b_start = boxes[sub['box_idx']]
        ds = actions_to_string(sub['actions'], cur_p, b_start, use_unicode)
        full_dir.append(ds)
        cur_p = sub['player_end']
    print("\n  全局方向指令串（各子任务以 | 分隔）:")
    print("  " + " | ".join(full_dir))

    # 动画
    sim = Simulator(the_map, player_pos, use_unicode)
    for i, sub in enumerate(result['sub_solutions']):
        b_start = boxes[sub['box_idx']]
        t_pos   = result['targets'][sub['target_idx']]
        label   = (f"子任务 {i+1}/{len(result['sub_solutions'])}: "
                   f"箱子#{sub['box_idx']+1}({b_start[1]},{b_start[0]}) "
                   f"→ 目标#{sub['target_idx']+1}({t_pos[1]},{t_pos[0]})")
        sim.run_phase(sub['actions'], b_start, sub['box_idx'], t_pos, label)
    sim.show_final()


def stage2_flow(the_map: list, player_pos: tuple, use_unicode: bool) -> None:
    print("\n[Stage2] 指定映射模式：手动分配箱子→目标")
    boxes   = extract_elements(the_map, BOX)
    targets = extract_elements(the_map, TARGET)
    n = len(boxes)

    if n == 0:
        print("  地图中无箱子。")
        return

    print(f"  箱子列表 (列,行):")
    for i, (r, c) in enumerate(boxes):
        print(f"    箱子#{i+1}: ({c},{r})")
    print(f"  目标列表 (列,行):")
    for i, (r, c) in enumerate(targets):
        print(f"    目标#{i+1}: ({c},{r})")

    box_to_target = []
    for i in range(n):
        ti = prompt_int(f"  箱子#{i+1} 推到目标#", 1, len(targets), i + 1)
        box_to_target.append(ti - 1)

    print("  正在求解...", end='', flush=True)
    t0 = time.time()
    result = solve_stage2(the_map, player_pos, box_to_target)
    elapsed = time.time() - t0
    if result is None:
        print("\n  ✗ 无解（映射不合法或存在死局）")
        return
    print(f" 完成 ({elapsed:.3f}s)  总步数: {result['total_steps']}")

    full_dir = []
    cur_p = player_pos
    boxes_list = result['boxes']
    for sub in result['sub_solutions']:
        b_start = boxes_list[sub['box_idx']]
        ds = actions_to_string(sub['actions'], cur_p, b_start, use_unicode)
        full_dir.append(ds)
        cur_p = sub['player_end']
    print("\n  全局方向指令串:")
    print("  " + " | ".join(full_dir))

    sim = Simulator(the_map, player_pos, use_unicode)
    for i, sub in enumerate(result['sub_solutions']):
        b_start = boxes_list[sub['box_idx']]
        t_pos   = result['targets'][sub['target_idx']]
        label   = (f"子任务 {i+1}: 箱子#{sub['box_idx']+1} → 目标#{sub['target_idx']+1}")
        sim.run_phase(sub['actions'], b_start, sub['box_idx'], t_pos, label)
    sim.show_final()


def stage3_flow(the_map: list, player_pos: tuple, use_unicode: bool) -> None:
    print("\n[Stage3] 炸弹辅助模式：先推炸弹，再推箱子")

    bombs = extract_elements(the_map, BOMB)
    if not bombs:
        print("  地图中无炸弹，Stage3 不适用。")
        return

    print(f"  炸弹位置: {[(c,r) for r,c in bombs]}")

    print("  正在搜索最优爆破点...", end='', flush=True)
    t0 = time.time()
    result = solve_stage3(the_map, player_pos)
    elapsed = time.time() - t0

    if result is None:
        print("\n  ✗ 无解（无法为炸弹找到有效目标墙体）")
        return

    print(f" 完成 ({elapsed:.3f}s)")
    wr, wc = result['wall_pos']
    print(f"  最优爆破墙体: ({wc},{wr})")
    print(f"  推炸弹步数: {len(result['bomb_actions'])}")

    # 推炸弹方向串
    bomb_dir = actions_to_string(result['bomb_actions'], player_pos,
                                 result['bomb_pos'], use_unicode)
    print(f"  推炸弹指令串: {bomb_dir}")

    # Stage1 结果
    s1 = result['stage1_result']
    if s1:
        print(f"  爆炸后 Stage1 总步数: {s1['total_steps']}")
        full_dir = []
        cur_p = result['cur_player_after_bomb']
        boxes_list = s1['boxes']
        for sub in s1['sub_solutions']:
            b_start = boxes_list[sub['box_idx']]
            ds = actions_to_string(sub['actions'], cur_p, b_start, use_unicode)
            full_dir.append(ds)
            cur_p = sub['player_end']
        print("  Stage1 指令串: " + " | ".join(full_dir))
    else:
        print("  爆炸后无需推箱或无法求解后续。")

    # 动画：推炸弹阶段
    # 为动画目的，把炸弹当成"特殊箱子"添加到 simulator
    sim = Simulator(the_map, player_pos, use_unicode)
    # 将炸弹临时加入 boxes 列表中（用于渲染追踪）
    bomb_pos = result['bomb_pos']
    sim.boxes.append(bomb_pos)  # 添加炸弹追踪
    sim.map[bomb_pos[0]][bomb_pos[1]] = BOMB

    bom_idx = len(sim.boxes) - 1
    label_bomb = f"推炸弹 ({bomb_pos[1]},{bomb_pos[0]}) → 墙体 ({wc},{wr})"
    sim.run_phase(result['bomb_actions'], bomb_pos, bom_idx, result['wall_pos'], label_bomb)

    # 应用爆炸效果
    sim.boxes.pop()  # 移除炸弹追踪
    apply_bomb_explosion(sim.map, result['wall_pos'])
    sim.map[result['wall_pos'][0]][result['wall_pos'][1]] = EMPTY
    print("\n  ★ 爆炸！墙壁已清除。")
    print(render_map(sim.map, sim.player, sim.boxes, use_unicode))
    input("\n  按 Enter 继续 Stage1 求解...")

    # Stage1 动画
    if s1:
        boxes_list = s1['boxes']
        # 同步 sim.boxes 到 Stage1 的初始箱子列表
        sim.boxes = list(boxes_list)
        for i, sub in enumerate(s1['sub_solutions']):
            b_start = boxes_list[sub['box_idx']]
            t_pos   = s1['targets'][sub['target_idx']]
            label   = f"Stage1 子任务 {i+1}: 箱子#{sub['box_idx']+1} → 目标#{sub['target_idx']+1}"
            sim.run_phase(sub['actions'], b_start, sub['box_idx'], t_pos, label)
    sim.show_final()


def print_map_info(the_map: list, player_pos: tuple, use_unicode: bool) -> None:
    boxes   = extract_elements(the_map, BOX)
    targets = extract_elements(the_map, TARGET)
    bombs   = extract_elements(the_map, BOMB)
    walls   = sum(the_map[r][c] == WALL for r in range(MAP_ROWS) for c in range(MAP_COLS))
    print(render_map(the_map, player_pos, boxes, use_unicode))
    print(f"\n  玩家: ({player_pos[1]},{player_pos[0]})  "
          f"箱子: {len(boxes)}  目标: {len(targets)}  "
          f"炸弹: {len(bombs)}  墙体: {walls}")

    dead, dp = check_deadlock(the_map)
    if dead:
        print(f"  ⚠ 初始死局检测：箱子 ({dp[1]},{dp[0]}) 被卡住！")


def main() -> None:
    global USE_UNICODE
    USE_UNICODE = _detect_unicode()

    print_header()

    while True:
        print("\n========== 主菜单 ==========")
        print(" 1. 自动生成地图并求解")
        print(" 2. 使用预置地图")
        print(" 0. 退出")
        choice = input("请选择: ").strip()

        if choice == '0':
            print("再见！")
            break

        elif choice == '1':
            stage     = prompt_int("  游戏阶段 (1=贪心/2=指定映射/3=炸弹)", 1, 3, 1)
            box_count = prompt_int("  箱子数量", 1, MAX_BOXES, 2)
            seed_s    = input("  随机种子 (直接 Enter = 随机): ").strip()
            seed      = int(seed_s) if seed_s.isdigit() else None
            wall_d_s  = input("  内墙密度 [0.0-0.3, 默认0.10]: ").strip()
            try:
                wall_d = float(wall_d_s)
                wall_d = max(0.0, min(0.3, wall_d))
            except ValueError:
                wall_d = 0.10

            print("  正在生成地图...", end='', flush=True)
            the_map, player_pos = generate_map(stage, box_count, wall_d, seed)
            print(" 完成")
            print_map_info(the_map, player_pos, USE_UNICODE)

            if stage == 1:
                stage1_flow(the_map, player_pos, USE_UNICODE)
            elif stage == 2:
                stage2_flow(the_map, player_pos, USE_UNICODE)
            elif stage == 3:
                bombs = extract_elements(the_map, BOMB)
                if not bombs:
                    print("  当前地图无炸弹，自动降级到 Stage1。")
                    stage1_flow(the_map, player_pos, USE_UNICODE)
                else:
                    stage3_flow(the_map, player_pos, USE_UNICODE)

        elif choice == '2':
            # 预置测试地图（1 个箱子的简单关卡）
            the_map = _make_empty_map()
            # 布置一个 1 箱 1 目标的简单场景
            the_map[3][3] = BOX
            the_map[3][10] = TARGET
            the_map[5][5] = WALL
            the_map[5][6] = WALL
            the_map[4][4] = WALL
            player_pos = (5, 2)
            stage = prompt_int("  选择阶段 (1=贪心/2=指定映射)", 1, 2, 1)
            print_map_info(the_map, player_pos, USE_UNICODE)
            if stage == 1:
                stage1_flow(the_map, player_pos, USE_UNICODE)
            else:
                stage2_flow(the_map, player_pos, USE_UNICODE)

        else:
            print("  无效选项，请重试。")


if __name__ == '__main__':
    main()

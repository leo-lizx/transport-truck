"""
sokoban_car_sim.py — 推箱子"整车流程"验证器 (车载屏幕 UI 版)
================================================================
本验证器**完全复刻车上屏幕 (chassis_menu.c / IPS200) 的推箱子 UI**, 并
**严格按照固件 app_game_logic.c 的整车状态机流程**模拟运动:

    STAGE_WAIT_START  → 发车流程 (回 (5,1) → 上移至 (4,1))
    STAGE_RECOGNIZE_MAP → 识别地图 (第 2 关共线目标跳过观察, 其余逐个实地识别)
    STAGE_PLAN_PATH   → 规划 (第1关时间优化 / 第2关配对或单快照盲扫 / 第3关含炸弹)
    STAGE_EXECUTE_ACTION → 执行 (推箱; 第3关先推炸弹炸墙再推箱)
    STAGE_LEVEL_JUDGE → 关卡判定
    STAGE_DONE        → 完成

算法层完全复用 sokoban_validator.py (= 固件 algo_sokoban_solver.c 的 PC 镜像)。

运行:
    python sokoban_car_sim.py            # 图形界面
    python sokoban_car_sim.py --selftest # 无界面自检三关
"""

from __future__ import annotations

import sys
import copy
import json
import os
import random
from typing import Optional, List, Tuple, Dict

from sokoban_validator import (
    MAP_ROWS, MAP_COLS,
    INNER_R_MIN, INNER_R_MAX, INNER_C_MIN, INNER_C_MAX,
    EMPTY, WALL, TARGET, BOX, BOMB,
    DR, DC,
    extract_elements,
    is_inner,
    find_observe_point_for_box,
    plan_scout_phase_v2,
    detect_stage2_line_targets,
    solve_stage2_line_sweep,
    solve_full,
    solve_level,
    build_return_path,
    check_deadlock,
    apply_bomb_explosion,
    _make_empty_map,
    parse_map_text, export_map_text,
)

# 方向常量 (与 DR/DC 对齐): UP=0 DOWN=1 LEFT=2 RIGHT=3
UP, DOWN, LEFT, RIGHT = 0, 1, 2, 3

# ============================================================
# 正式固件发车契约 (app_game_logic.c)
#   唯一库位 (row=5,col=1) → 地图刷新触发格 (row=5,col=2)
# ============================================================
START_ROW = 5
ZONE_LEFT = 'left'
ZONE_RIGHT = 'right'  # 仅保留旧存档兼容；运行时统一按左库位处理


def launch_cells(zone: str) -> Tuple[tuple, tuple, list]:
    """返回正式比赛唯一的 (起步格, 触发格, 发车走廊)。"""
    del zone
    start = (5, 1)
    exit_c = (5, 2)
    corridor = [start, exit_c]
    return start, exit_c, corridor


CUSTOM_MAPS_FILE = os.path.join(os.path.dirname(__file__), 'custom_maps.json')

BUILTIN_MAPS = [
    {
        'name': '示例A-多炸弹通道',
        'level': 3,
        'text': """################
#-#---#--#----.#
#---#--#-####--#
#---#--#-------#
#-.#-#-#--#----#
#@-#-#--------##
#--#----------##
#-*----------*-#
#--$-#----##$*-#
#-$--#----####-#
#----#.--------#
################""",
    },
    {
        'name': '示例B-镜像多炸弹',
        'level': 3,
        'text': """################
#.----#--#---#-#
#--####-#--#---#
#-------#--#---#
#----#--#-#-#.-#
##--------#-#-@#
##----------#--#
#-*----------*-#
#-*$##----#-$--#
#-####----#--$-#
#--------.#----#
################""",
    },
    {
        'name': '示例C-单炸弹密墙',
        'level': 3,
        'text': """################
#---$-#..------#
#----#----#----#
#--#-#.#----#--#
#---------#----#
#-##-----------#
#@----#--$*----#
#--------------#
#-#---#-----#--#
#---$--#-------#
#--------------#
################""",
    },
    {
        'name': '示例D-折线路径',
        'level': 3,
        'text': """################
#--------------#
#---##.--#-----#
#----#$--#$--#-#
#----#---------#
#.---#-------#-#
#@---#-#--#----#
#----#-----#---#
#----#--$-*----#
#---#.--###-#--#
#--------------#
################""",
    },
    {
        'name': '示例E-封墙破局',
        'level': 3,
        'text': """################
#----.---------#
#--#---.-#-----#
#-#-#---#------#
#----$--##-----#
#-------#------#
#@------##--$--#
#-$----*#--.##-#
#--------------#
#--##-----###--#
#--------------#
################""",
    },
]


def _normalize_map_text(text: str) -> str:
    return '\n'.join(line.rstrip() for line in text.strip().splitlines())


def _normalize_classes(values: Optional[list], count: int) -> list:
    out = []
    for i in range(count):
        try:
            cls = int(values[i]) if values is not None and i < len(values) else (i + 1)
        except (TypeError, ValueError):
            cls = i + 1
        out.append(max(1, cls))
    return out


def _level_classes(level: int, values: Optional[list], count: int) -> list:
    if level < 2:
        return []
    return _normalize_classes(values, count)


def _info_from_map_text(text: str, level: int,
                        box_classes: Optional[list] = None,
                        target_classes: Optional[list] = None) -> dict:
    the_map, player_pos, err = parse_map_text(_normalize_map_text(text))
    if err:
        raise ValueError(err)

    zone = ZONE_LEFT
    start, exit_c, _ = launch_cells(zone)
    # 文本中的旧 @ 只作为格式占位；整车仿真始终使用当前固件固定库位。
    the_map[start[0]][start[1]] = EMPTY
    the_map[exit_c[0]][exit_c[1]] = EMPTY

    box_count = len(extract_elements(the_map, BOX))
    target_count = len(extract_elements(the_map, TARGET))
    return {
        'map': the_map,
        'start': start,
        'exit': exit_c,
        'zone': zone,
        'box_classes': _level_classes(level, box_classes, box_count),
        'target_classes': _level_classes(level, target_classes, target_count),
        'level': level,
    }


def _load_custom_maps() -> list:
    if not os.path.exists(CUSTOM_MAPS_FILE):
        return []
    try:
        with open(CUSTOM_MAPS_FILE, 'r', encoding='utf-8') as f:
            data = json.load(f)
    except (OSError, json.JSONDecodeError):
        return []
    if not isinstance(data, list):
        return []
    out = []
    for item in data:
        if not isinstance(item, dict):
            continue
        name = str(item.get('name', '')).strip()
        text = str(item.get('text', '')).strip()
        level = int(item.get('level', 3) or 3)
        if name and text:
            level = max(1, min(3, level))
            try:
                info = _info_from_map_text(text, level,
                                           item.get('box_classes'),
                                           item.get('target_classes'))
            except ValueError:
                continue
            out.append({'name': name, 'text': text, 'level': level,
                        'box_classes': info['box_classes'],
                        'target_classes': info['target_classes']})
    return out


def _save_custom_map(name: str, text: str, level: int,
                     box_classes: Optional[list] = None,
                     target_classes: Optional[list] = None):
    clean_name = name.strip()
    if not clean_name:
        raise ValueError('请输入地图名称')
    clean_text = _normalize_map_text(text)
    info = _info_from_map_text(clean_text, level, box_classes, target_classes)

    items = _load_custom_maps()
    rec = {'name': clean_name, 'text': clean_text, 'level': level,
           'box_classes': info['box_classes'],
           'target_classes': info['target_classes']}
    for i, item in enumerate(items):
        if item['name'] == clean_name:
            items[i] = rec
            break
    else:
        items.append(rec)

    with open(CUSTOM_MAPS_FILE, 'w', encoding='utf-8') as f:
        json.dump(items, f, ensure_ascii=False, indent=2)


# ============================================================
# 地图生成 (强制玩家从左库位起步, 预留向右发车走廊, 校验可解)
# ============================================================

def _inner_list() -> list:
    return [(r, c) for r in range(INNER_R_MIN, INNER_R_MAX + 1)
            for c in range(INNER_C_MIN, INNER_C_MAX + 1)]


def _all_boxes_observable(m: list, viewer: tuple, boxes: list) -> bool:
    return all(find_observe_point_for_box(m, viewer, b) is not None for b in boxes)


def _all_targets_observable(m: list, viewer: tuple, targets: list) -> bool:
    return all(find_observe_point_for_box(m, viewer, t) is not None for t in targets)


def gen_level_map(level: int, box_count: int = 2, wall_density: float = 0.08,
                  seed: Optional[int] = None,
                  zone: str = ZONE_LEFT) -> dict:
    """
    生成符合"整车流程"要求的关卡地图。
    返回 dict:
        map, start, exit, zone, box_classes, target_classes, level
    """
    rng = random.Random(seed)
    if level == 3:
        return _gen_bomb_map(box_count, wall_density, rng, zone)
    return _gen_open_map(level, box_count, wall_density, rng, zone)


def _scatter_walls(m: list, count: int, rng: random.Random, avoid: set) -> int:
    """在内场散布普通障碍墙 (跳过最外内圈以降低死局), 避开 avoid 集合。"""
    cand = [(r, c) for r in range(INNER_R_MIN + 1, INNER_R_MAX)
            for c in range(INNER_C_MIN + 1, INNER_C_MAX)
            if m[r][c] == EMPTY and (r, c) not in avoid]
    rng.shuffle(cand)
    placed = 0
    for (r, c) in cand:
        if placed >= count:
            break
        m[r][c] = WALL
        placed += 1
    return placed


def _make_perm(n: int, rng: random.Random, allow_identity: bool) -> list:
    """生成 1..n 的随机排列。allow_identity=False 时尽量返回非恒等排列。"""
    base = list(range(1, n + 1))
    p = base[:]
    rng.shuffle(p)
    if not allow_identity and n > 1:
        tries = 0
        while p == base and tries < 20:
            rng.shuffle(p)
            tries += 1
        if p == base:           # 仍恒等则手动错位
            p = base[1:] + base[:1]
    return p


def _gen_open_map(level: int, box_count: int, wall_density: float,
                  rng: random.Random, zone: str) -> dict:
    start, exit_c, corridor = launch_cells(zone)
    reserve = set(corridor)
    inner = _inner_list()

    # 第 2 关更复杂: 抬高内墙密度下限, 让障碍更多
    if level >= 2:
        wall_density = max(wall_density, 0.12)

    for _ in range(800):
        m = _make_empty_map()
        avail = [p for p in inner if p not in reserve]
        rng.shuffle(avail)

        # 随机内墙 (避开走廊与最贴边一圈, 降低死局概率)
        wall_n = int(len(inner) * wall_density)
        placed = 0
        idx = 0
        taken = set()
        while placed < wall_n and idx < len(avail):
            r, c = avail[idx]
            idx += 1
            if r <= INNER_R_MIN or r >= INNER_R_MAX or c <= INNER_C_MIN or c >= INNER_C_MAX:
                continue
            m[r][c] = WALL
            taken.add((r, c))
            placed += 1

        free = [p for p in avail if p not in taken]
        if len(free) < 2 * box_count + 1:
            continue
        rng.shuffle(free)

        boxes = free[:box_count]
        targets = free[box_count:2 * box_count]
        for (r, c) in boxes:
            m[r][c] = BOX
        for (r, c) in targets:
            m[r][c] = TARGET

        # class 真值 (按 extract 顺序): 第 1 关不显示/使用编号; 第 2 关才随机配对.
        # 第 2 关强制"非恒等"排列, 保证"按数字配对"真正起作用 (不是随便推哪个都行).
        if level == 1:
            bc = []
            tc = []
        else:
            bc = list(range(1, box_count + 1))
            tc = _make_perm(box_count, rng, allow_identity=False)

        if check_deadlock(m)[0]:
            continue

        if level == 1:
            ok = solve_level(1, m, exit_c) is not None
        else:
            ok = solve_level(
                2, m, exit_c,
                box_classes=bc,
                target_classes=tc,
            ) is not None
        if not ok:
            continue

        return {'map': m, 'start': start, 'exit': exit_c, 'zone': zone,
                'box_classes': bc, 'target_classes': tc, 'level': level}

    return _fallback_map(level, box_count, zone)


def _gen_bomb_map(box_count: int, wall_density: float,
                  rng: random.Random, zone: str) -> dict:
    """
    第 3 关 (含炸弹 + 按数字配对):
      - 所有箱子/目标必须在识别阶段均可走到面前实地观察;
      - 内墙 + 炸弹: 某段墙挡在箱子→目标的推进通道上, 必须先推炸弹炸开;
      - 识别 tour 必须 all_visited.
    """
    start, exit_c, corridor = launch_cells(zone)
    reserve = set(corridor)
    inner = _inner_list()
    wall_n = max(8, int(len(inner) * max(wall_density, 0.10)))

    for _ in range(2500):
        m = _make_empty_map()
        _scatter_walls(m, wall_n, rng, reserve)

        avail = [p for p in inner if m[p[0]][p[1]] == EMPTY and p not in reserve]
        if len(avail) < 2 * box_count + 2:
            continue
        rng.shuffle(avail)

        idx = 0
        boxes = avail[idx:idx + box_count]
        idx += box_count
        targets = avail[idx:idx + box_count]
        idx += box_count
        if idx >= len(avail):
            continue
        bomb_pos = avail[idx]

        for (r, c) in boxes:
            m[r][c] = BOX
        for (r, c) in targets:
            m[r][c] = TARGET
        m[bomb_pos[0]][bomb_pos[1]] = BOMB

        # 在箱子列与目标列之间加一道竖墙, 挡推箱通道 (玩家仍可绕路去观察)
        if boxes and targets:
            br, bc = boxes[0]
            tr, tc = targets[0]
            mid_c = (bc + tc) // 2
            for rr in range(min(br, tr), max(br, tr) + 1):
                if is_inner(rr, mid_c) and m[rr][mid_c] == EMPTY:
                    m[rr][mid_c] = WALL

        boxes_ex = extract_elements(m, BOX)
        tgts_ex = extract_elements(m, TARGET)
        if len(boxes_ex) != box_count or len(tgts_ex) != box_count:
            continue

        bc = list(range(1, box_count + 1))
        tc = _make_perm(box_count, rng, allow_identity=(box_count == 1))

        if not _all_boxes_observable(m, exit_c, boxes_ex):
            continue
        if not _all_targets_observable(m, exit_c, tgts_ex):
            continue

        scout = plan_scout_phase_v2(m, exit_c, box_classes=bc,
                                    target_classes=tc)
        if not scout['all_visited']:
            continue
        mapping = scout.get('box_to_target_idx')
        if not mapping:
            continue

        try:
            res = solve_full(m, exit_c, mapping)
        except Exception:
            continue
        if (res and res['is_solved']
                and any(p['kind'] == 'bomb' for p in res['phases'])):
            return {'map': m, 'start': start, 'exit': exit_c, 'zone': zone,
                    'box_classes': bc, 'target_classes': tc, 'level': 3}

    return _fallback_map(3, box_count, zone)


def _fallback_map(level: int, box_count: int, zone: str) -> dict:
    """保底关卡 (一定可解), 防随机生成超时。"""
    start, exit_c, _ = launch_cells(zone)
    m = _make_empty_map()
    if level == 3:
        # 全部目标可观察; 竖墙挡推箱通道, 炸弹炸开后才能按号配对推完
        m[4][4] = BOX
        m[4][11] = BOX
        m[8][4] = TARGET
        m[8][11] = TARGET
        for rr in range(4, 9):
            m[rr][7] = WALL
        m[5][8] = BOMB
        bc = [1, 2]
        tc = [2, 1]
    else:
        m[3][6] = BOX
        m[3][10] = BOX
        m[8][6] = TARGET
        m[8][10] = TARGET
        if level == 1:
            bc = []
            tc = []
        else:
            bc = [1, 2]
            tc = [2, 1]
    return {'map': m, 'start': start, 'exit': exit_c, 'zone': zone,
            'box_classes': bc, 'target_classes': tc, 'level': level}


# ============================================================
# CarSim — 整车状态机时间线生成器
# ============================================================

STAGE_NAMES = {
    'WAIT_START':   'WAIT_START',
    'RECOGNIZE':    'RECOGNIZE_MAP',
    'PLAN':         'PLAN_PATH',
    'EXECUTE':      'EXECUTE_ACTION',
    'JUDGE':        'LEVEL_JUDGE',
    'DONE':         'DONE',
    'DEADLOCK':     'DEADLOCK_RESET',
}


class CarSim:
    """逐格模拟整车流程, 把每一步记录成可回放的帧。"""

    def __init__(self, info: dict):
        self.level = info['level']
        self.zone = info['zone']
        self.start = info['start']
        self.exit = info['exit']
        self.box_classes = info.get('box_classes')
        self.target_classes = info.get('target_classes')

        src = info['map']
        # base: 仅 WALL/EMPTY/TARGET; 箱子/炸弹独立跟踪
        self.base = [[EMPTY] * MAP_COLS for _ in range(MAP_ROWS)]
        self.boxes: List[tuple] = []
        self.bombs: List[tuple] = []
        for r in range(MAP_ROWS):
            for c in range(MAP_COLS):
                v = src[r][c]
                if v == BOX:
                    self.boxes.append((r, c))
                    self.base[r][c] = EMPTY
                elif v == BOMB:
                    self.bombs.append((r, c))
                    self.base[r][c] = EMPTY
                else:
                    self.base[r][c] = v

        # 数字配对真值 (extract 顺序与 self.boxes 一致, 均为行优先扫描)
        n_box = len(self.boxes)
        bc = self.box_classes if self.box_classes else list(range(1, n_box + 1))
        self.box_class = [bc[i] if i < len(bc) else (i + 1) for i in range(n_box)]
        tgts_ex = extract_elements(src, TARGET)
        tcs = self.target_classes if self.target_classes else list(range(1, len(tgts_ex) + 1))
        self.target_class = [tcs[j] if j < len(tcs) else (j + 1) for j in range(len(tgts_ex))]
        self.target_class_by_pos: Dict[tuple, int] = {}
        for j, p in enumerate(tgts_ex):
            self.target_class_by_pos[p] = self.target_class[j]

        self.player = self.start
        self.last_dir = UP
        self.steps = 0
        self.frames: List[dict] = []
        self.log: List[str] = []
        self.mapping: Optional[list] = None
        self.line_sweep_plan: Optional[dict] = None
        self.line_sweep_mode = False
        self.recog_summary = ''

    # ---- 工具 ----
    def full_map(self) -> list:
        m = [row[:] for row in self.base]
        for (r, c) in self.boxes:
            m[r][c] = BOX
        for (r, c) in self.bombs:
            m[r][c] = BOMB
        return m

    def _win(self) -> bool:
        if not self.boxes:
            return self.line_sweep_mode and not extract_elements(
                self.base, TARGET)
        # 通关前提: 箱子两两不重叠, 且全部落在目标格
        if len(set(self.boxes)) != len(self.boxes):
            return False
        if not all(self.base[r][c] == TARGET for (r, c) in self.boxes):
            return False
        # 第 2/3 关追加: 每个箱子必须落在"与自己号码相同"的目标格 (不能随便一个)
        if self.level >= 2:
            for i, pos in enumerate(self.boxes):
                cls = self.box_class[i] if i < len(self.box_class) else None
                if self.target_class_by_pos.get(pos) != cls:
                    return False
        return True

    def _snap(self, stage: str, info: str,
              explosion: Optional[tuple] = None,
              event: Optional[str] = None):
        self.frames.append({
            'base':      [row[:] for row in self.base],
            'player':    self.player,
            'boxes':     list(self.boxes),
            'bombs':     list(self.bombs),
            'stage':     STAGE_NAMES.get(stage, stage),
            'stage_key': stage,
            'level':     self.level,
            'info':      info,
            'steps':     self.steps,
            'last_dir':  self.last_dir,
            'event':     event,
            'explosion': explosion,
            'win':       self._win(),
            'mapping':   list(self.mapping) if self.mapping else None,
            'line_sweep': self.line_sweep_mode,
            'recog':     self.recog_summary,
            'box_classes': list(self.box_class),
            'target_classes': list(self.target_class),
        })

    def _step(self, d: int, stage: str, info: str,
              mv_kind: Optional[str] = None, mv_idx: int = -1):
        """
        移动一格。仅推动本阶段指定的 movable (box / bomb);
        其它已就位的箱子按求解器 build_sub_map 语义"可穿过"(不推), 与固件一致。
        """
        pr, pc = self.player
        nr, nc = pr + DR[d], pc + DC[d]
        if mv_kind == 'box' and 0 <= mv_idx < len(self.boxes) and self.boxes[mv_idx] == (nr, nc):
            self.boxes[mv_idx] = (nr + DR[d], nc + DC[d])
        elif mv_kind == 'bomb' and 0 <= mv_idx < len(self.bombs) and self.bombs[mv_idx] == (nr, nc):
            self.bombs[mv_idx] = (nr + DR[d], nc + DC[d])
        self.player = (nr, nc)
        self.last_dir = d
        self.steps += 1
        self._snap(stage, info)

    def _walk(self, actions: list, stage: str, info: str,
              mv_kind: Optional[str] = None, mv_idx: int = -1):
        for d in actions:
            self._step(d, stage, info, mv_kind, mv_idx)

    @staticmethod
    def _dir_toward(frm: tuple, to: tuple) -> Optional[int]:
        """观察点 → 物体格 的 4-邻接方向 (与固件 recog_calc_face_yaw 语义一致)。"""
        fr, fc = frm
        tr, tc = to
        dr, dc = tr - fr, tc - fc
        if dr == -1 and dc == 0:
            return UP
        if dr == 1 and dc == 0:
            return DOWN
        if dr == 0 and dc == -1:
            return LEFT
        if dr == 0 and dc == 1:
            return RIGHT
        return None

    def _face(self, d: int, stage: str, info: str):
        """原地转向 (镜像 App_Recognize RECOG_SUB_FACE, 不移动、不计步)。"""
        self.last_dir = d
        self._snap(stage, info, event='FACE')

    # ---- 阶段 1: 发车 ----
    def launch(self):
        self.player = self.start
        self._snap('WAIT_START', 'phase0: 回到发车起点')
        self.log.append(f"[WAIT_START] 起点 {self._xy(self.start)}")
        while self.player != self.exit:
            pr, pc = self.player
            er, ec = self.exit
            if pr != er:
                step = DOWN if er > pr else UP
            else:
                step = RIGHT if ec > pc else LEFT
            self._step(step, 'WAIT_START', 'phase1: 驶向地图刷新触发格')
        self._snap('WAIT_START', '发车成功: 已到地图刷新触发格')
        self.log.append(f"[WAIT_START] 发车 → 触发格 {self._xy(self.exit)}")

    # ---- 阶段 2: 识别 ----
    def recognize(self):
        # 第一关没有数字配对要求，即使存在炸弹也跳过分类 Tour。
        if self.level < 2:
            self.recog_summary = '第1关: 无需数字分类 (DONE_NO_NEED)'
            self._snap('RECOGNIZE', '第1关跳过数字分类')
            self.log.append('[RECOGNIZE] 跳过 (NO_NEED: 第1关)')
            self.mapping = None
            return

        fm = self.full_map()
        if self.level == 2:
            line_info = detect_stage2_line_targets(fm)
            if line_info is not None:
                line_plan = solve_stage2_line_sweep(
                    fm, self.player, line_info,
                    box_classes=self.box_class,
                    target_classes=self.target_class,
                    home_pos=self.start,
                )
                if line_plan is not None:
                    self.line_sweep_plan = line_plan
                    self.line_sweep_mode = True
                    self.mapping = None
                    self.recog_summary = (
                        '共线目标盲扫: 跳过图案识别, '
                        '使用开局地图全量规划')
                    self._snap(
                        'RECOGNIZE',
                        '检测到连续共线目标, 跳过数字分类')
                    self.log.append(
                        '[RECOGNIZE] LINE_SWEEP_BYPASS: '
                        '开局地图读取 1 次, 不进行观察 Tour')
                    return
                self.log.append(
                    '[RECOGNIZE] 共线目标全量规划失败, '
                    '执行前回退常规数字识别')

        scout = plan_scout_phase_v2(fm, self.player,
                                    box_classes=self.box_classes,
                                    target_classes=self.target_classes)
        n_box = len(self.boxes)
        n_tgt = len(extract_elements(fm, TARGET))
        need = n_box + n_tgt
        self.log.append(f"[RECOGNIZE] 实地观察 {scout['visited_count']} 个"
                        f" / 共 {need} 个 (箱子{n_box}+目标{n_tgt})")
        for v in scout['visits']:
            kind = '箱子' if v['kind'] == 'box' else '目标'
            self._walk(v['path_actions'], 'RECOGNIZE',
                       f"走到 {kind}#{v['item_idx']+1} 前观察")
            face = v.get('face_dir')
            if face is None:
                face = self._dir_toward(v['observe'], v['pos'])
            if face is not None:
                self._face(face, 'RECOGNIZE',
                           f"转向 {kind}#{v['item_idx']+1}")
            self.recog_summary = f"识别 {kind}#{v['item_idx']+1} = {v['class_id']}号"
            self._snap('RECOGNIZE', self.recog_summary)
            self.log.append(f"  · 观察 {kind}#{v['item_idx']+1} = {v['class_id']}号")

        self.player = scout['player_after_scout']
        # 映射使用规则 (镜像 build_push_box_plan):
        #   - 第 1 关 (即便地图含炸弹): 规划用 Stage1 时间优化, 不做数字映射;
        #   - 第 2/3 关: 全部识别成功时按数字映射 (Stage2), 否则置 None 待炸墙后贪心.
        mp = scout.get('box_to_target_idx')
        if self.level >= 2:
            self.mapping = mp if (scout.get('all_visited') and mp) else None
        else:
            self.mapping = None
        if self.mapping:
            self.recog_summary = '识别完成, 已生成 箱→目标 映射'
            self.log.append(f"[RECOGNIZE] 映射 box→target = {self.mapping}")
        else:
            self.recog_summary = '识别完成 (目标暂不可达, 待炸墙后贪心)'
            self.log.append('[RECOGNIZE] 目标不可达, 炸墙后用贪心配对')
        self._snap('RECOGNIZE', self.recog_summary)

    # ---- 阶段 3/4: 规划 + 执行 ----
    def plan_and_execute(self):
        self._snap('PLAN', '规划推箱/炸弹路径 (推宏 A* + 时间成本搜索)')
        fm = self.full_map()
        if self.line_sweep_plan is not None:
            self._snap(
                'PLAN',
                '开局快照已生成全部盲扫路径, 准备一批连续执行')
            self.log.append(
                f"[PLAN] LINE_SWEEP: {len(self.line_sweep_plan['sub_solutions'])} "
                '只箱子, 1 次读图 / 1 个执行批次')
            for sub in self.line_sweep_plan['sub_solutions']:
                if not self._exec_line_sweep(sub):
                    self._snap('DEADLOCK', '盲扫动画状态与预计路径不一致')
                    self.log.append('[DEADLOCK_RESET] 盲扫执行校验失败')
                    return False
        else:
            if self.level >= 2 and self.mapping is None:
                self._snap('DEADLOCK', '分类映射不完整，禁止降级为任意配对')
                self.log.append('[DEADLOCK_RESET] 分类映射失败')
                return False
            res = solve_full(fm, self.player, self.mapping, home_pos=self.start)
            if res is None or not res['is_solved']:
                self._snap('DEADLOCK', '无解 → 回发车区静止 3s 复位')
                self.log.append('[DEADLOCK_RESET] 规划失败')
                return False

            for ph in res['phases']:
                if ph['kind'] == 'push':
                    self._exec_push(ph)
                else:
                    self._exec_bomb(ph)
                # 每炸完一颗炸弹, 固件会回 PLAN 重新规划 → 我们已用 solve_full 一次展开
                if ph['kind'] == 'bomb':
                    self._snap('PLAN', '爆破完成, 重新规划剩余推箱')

        return_plan = build_return_path(self.full_map(), self.player, self.start)
        if return_plan is None:
            self._snap('DEADLOCK', '通关但库位坐标无效')
            self.log.append('[WAIT_START] 直线返库目标无效')
            return False
        self._snap('WAIT_START', '通关后忽略虚拟障碍，直线返库')
        self.player = self.start
        self._snap('WAIT_START', '返库终点关键 Snap + 航向校准')
        self.log.append(f"[WAIT_START] 直线返库 → {self._xy(self.start)}")
        return True

    def _exec_line_sweep(self, sub: dict) -> bool:
        """按已经全量规划的子路径播放一只箱子。

        箱子到达同号目标时与目标同时消失；剩余的直线推送指令继续
        作为车辆前进动画，所以最终车位与固件预测一致。
        """
        box_idx = self._find_movable(self.boxes, sub['box_start'])
        matched_target = sub['matched_target']
        if box_idx < 0 or len(sub['actions']) != len(sub['push_flags']):
            return False
        if self.box_class[box_idx] != self.target_class_by_pos.get(
                matched_target):
            return False

        info = f"盲扫箱子 {self._xy(sub['box_start'])}"
        self._snap('EXECUTE', info)
        removed = False
        for action, is_push in zip(sub['actions'], sub['push_flags']):
            active_idx = box_idx if is_push and not removed else -1
            self._step(action, 'EXECUTE', info,
                       mv_kind='box', mv_idx=active_idx)
            if (not removed and is_push and
                    self.boxes[box_idx] == matched_target):
                target_positions = extract_elements(self.base, TARGET)
                if matched_target not in target_positions:
                    return False
                target_idx = target_positions.index(matched_target)
                self.boxes.pop(box_idx)
                self.box_class.pop(box_idx)
                self.base[matched_target[0]][matched_target[1]] = EMPTY
                self.target_class.pop(target_idx)
                self.target_class_by_pos.pop(matched_target, None)
                removed = True
                self._snap(
                    'EXECUTE',
                    f"命中同号目标 {self._xy(matched_target)}, 箱子/目标消失",
                    event='LINE_SWEEP_MATCH')

        if not removed or self.player != sub['player_end']:
            return False
        self.log.append(
            f"[EXECUTE] LINE_SWEEP {self._xy(sub['box_start'])} 完成, "
            f"遍历 {len(sub['sweep_targets'])} 个开局目标")
        return True

    def _exec_push(self, ph: dict):
        bi = self._find_movable(self.boxes, ph['movable'])
        info = f"推箱 {self._xy(ph['movable'])}"
        self._snap('EXECUTE', info)
        self._walk(ph['actions'], 'EXECUTE', info, mv_kind='box', mv_idx=bi)
        self.log.append(f"[EXECUTE] 推箱 {self._xy(ph['movable'])} 完成")

    def _exec_bomb(self, ph: dict):
        wall = ph['wall']
        bi = self._find_movable(self.bombs, ph['movable'])
        info = f"推炸弹 {self._xy(ph['movable'])} → 墙 {self._xy(wall)}"
        self._snap('EXECUTE', info)
        self._walk(ph['actions'], 'EXECUTE', info, mv_kind='bomb', mv_idx=bi)
        # 爆炸: 3x3 清墙
        apply_bomb_explosion(self.base, wall)
        self.base[wall[0]][wall[1]] = EMPTY
        self.bombs = [b for b in self.bombs if b != wall]
        for k in range(3):
            self._snap('EXECUTE', '★ 炸弹引爆! 3×3 清除内墙', explosion=wall)
        self.log.append(f"[EXECUTE] 炸弹引爆于 {self._xy(wall)}, 清墙")

    # ---- 阶段 5/6: 判定 / 完成 ----
    def finish(self):
        self._snap('JUDGE', '关卡判定: 地图是否还有箱子')
        if self._win():
            self._snap('DONE', '★★★ 通关! 所有箱子均已到位 ★★★')
            self.log.append('[DONE] 通关成功')
        else:
            on = sum(1 for (r, c) in self.boxes if self.base[r][c] == TARGET)
            self._snap('DONE', f'结束: {on}/{len(self.boxes)} 箱到位')
            self.log.append(f'[DONE] {on}/{len(self.boxes)} 箱到位')

    def run(self) -> List[dict]:
        self.launch()
        self.recognize()
        ok = self.plan_and_execute()
        if ok:
            self.finish()
        return self.frames

    # ---- 小工具 ----
    @staticmethod
    def _find_movable(items: list, pos: tuple) -> int:
        for i, p in enumerate(items):
            if p == pos:
                return i
        return -1

    @staticmethod
    def _xy(p: tuple) -> str:
        return f"({p[1]},{p[0]})"


def build_timeline(info: dict) -> Tuple[List[dict], List[str]]:
    sim = CarSim(info)
    frames = sim.run()
    return frames, sim.log


# ============================================================
# 自检 (无界面)
# ============================================================

def run_selftest() -> int:
    rc = 0
    expected_start = (5, 1)
    expected_exit = (5, 2)
    start, exit_c, corridor = launch_cells(ZONE_LEFT)
    if (start, exit_c, corridor) != (expected_start, expected_exit,
                                     [expected_start, expected_exit]):
        print(f"Launch contract mismatch: {start} -> {exit_c}, corridor={corridor}")
        rc = 1

    for level in (1, 2, 3):
        info = gen_level_map(level, box_count=2, seed=20260608 + level)
        if info['start'] != expected_start or info['exit'] != expected_exit:
            print(f"Level {level}: launch cells mismatch "
                  f"{info['start']} -> {info['exit']}")
            rc = 1
            continue
        if any(info['map'][r][c] != EMPTY for r, c in corridor):
            print(f"Level {level}: launch corridor is not empty")
            rc = 1
            continue
        if level == 1 and (info.get('box_classes') or info.get('target_classes')):
            print("Level 1: class labels should be empty")
            rc = 1
            continue
        if level >= 2 and (not info.get('box_classes') or not info.get('target_classes')):
            print(f"Level {level}: class labels missing")
            rc = 1
            continue
        frames, log = build_timeline(info)
        last = frames[-1]
        ok = last['win']
        if level >= 2:
            need_face = len(extract_elements(info['map'], BOX)) + len(extract_elements(info['map'], TARGET))
            face_count = sum(1 for fr in frames if fr.get('event') == 'FACE')
            expected_face = 0 if (level == 2 and
                                  any(fr.get('line_sweep') for fr in frames)) \
                else need_face
            if face_count != expected_face:
                print(f"Level {level}: FACE frames mismatch "
                      f"{face_count}/{expected_face}")
                ok = False
        print(f"Level {level}: stage={last['stage']} steps={last['steps']} "
              f"frames={len(frames)} win={ok}")
        for line in log:
            print('   ', line)
        if not ok:
            rc = 1

    line_map = _make_empty_map()
    for col in range(7, 11):
        line_map[5][col] = TARGET
    for row in (2, 4, 6, 8):
        line_map[row][2] = BOX
    line_info = {
        'map': line_map,
        'start': expected_start,
        'exit': expected_exit,
        'zone': ZONE_LEFT,
        'box_classes': [1, 2, 3, 4],
        'target_classes': [4, 2, 1, 3],
        'level': 2,
    }
    line_frames, line_log = build_timeline(line_info)
    line_last = line_frames[-1]
    line_matches = sum(
        1 for frame in line_frames
        if frame.get('event') == 'LINE_SWEEP_MATCH')
    line_faces = sum(
        1 for frame in line_frames if frame.get('event') == 'FACE')
    line_ok = (line_last['win'] and line_matches == 4 and line_faces == 0
               and not line_last['boxes']
               and not extract_elements(line_last['base'], TARGET))
    print(f"Level 2 line sweep: matches={line_matches} "
          f"faces={line_faces} win={line_ok}")
    for line in line_log:
        print('   ', line)
    if not line_ok:
        rc = 1
    print('SELFTEST', 'PASS' if rc == 0 else 'FAIL')
    return rc


# ============================================================
# 图形界面 (复刻车载 IPS200 屏幕风格)
# ============================================================

def launch_gui(initial_info: Optional[dict] = None,
               autoplay: bool = False):
    import tkinter as tk
    from tkinter import messagebox

    # --- 车屏配色 (来自 chassis_menu.c MENU_COLOR_*) ---
    COL_BG     = '#000000'
    COL_GRID   = '#808080'
    COL_TEXT   = '#FFFFFF'
    COL_TITLE  = '#00FFFF'
    COL_OK     = '#00FF00'
    COL_WAIT   = '#FFFF00'
    COL_WALL   = '#394152'   # RGB(57,65,82)
    COL_TARGET = '#E700FF'   # RGB(231,0,255)
    COL_BOX    = '#94B200'   # RGB(148,178,0)
    COL_BOMB   = '#FF184A'   # RGB(255,24,74)
    COL_CAR    = '#00FFFF'
    COL_ZONE   = '#5a5a18'   # 发车区描边 (暗黄)

    # 缩放: 车屏 320x240, 单元 12px, 原点(8,32)
    S = 2.5

    def px(v):
        return int(round(v * S))

    CELL = px(12)
    OX = px(8)
    OY = px(32)
    MAPW = MAP_COLS * CELL
    MAPH = MAP_ROWS * CELL
    SCR_W = px(320)
    SCR_H = px(240)

    F_TITLE = ('Consolas', max(10, px(7)), 'bold')
    F_TXT   = ('Consolas', max(8, px(6)))
    F_SMALL = ('Consolas', max(7, px(5)))

    root = tk.Tk()
    root.title('推箱子整车流程验证器 — 车载屏幕 UI')
    root.configure(bg='#15171e')

    main = tk.Frame(root, bg='#15171e')
    main.pack(fill='both', expand=True, padx=8, pady=8)

    # 左: 车屏画布
    canvas = tk.Canvas(main, width=SCR_W, height=SCR_H, bg=COL_BG,
                       highlightthickness=2, highlightbackground='#333845')
    canvas.grid(row=0, column=0, rowspan=2, sticky='n')

    # 右: 控制面板
    panel = tk.Frame(main, bg='#15171e')
    panel.grid(row=0, column=1, sticky='nw', padx=(12, 0))

    log = tk.Text(main, width=46, height=12, bg='#0c0e13', fg='#9fb0d0',
                  font=('Consolas', 9), relief='flat', highlightthickness=1,
                  highlightbackground='#2a2f3d')
    log.grid(row=1, column=1, sticky='nw', padx=(12, 0), pady=(8, 0))

    state = {
        'frames': [],
        'idx': 0,
        'playing': False,
        'info': None,
        'after': None,
    }

    # ---------- 控件 ----------
    def mk_label(parent, text):
        return tk.Label(parent, text=text, bg='#15171e', fg='#c0caf5',
                        font=('Segoe UI', 10))

    mk_label(panel, '关卡 (Level)').grid(row=0, column=0, sticky='w')
    level_var = tk.IntVar(value=1)
    lvfrm = tk.Frame(panel, bg='#15171e')
    lvfrm.grid(row=0, column=1, sticky='w')
    for i, txt in enumerate(['1 贪心', '2 配对', '3 炸弹']):
        tk.Radiobutton(lvfrm, text=txt, variable=level_var, value=i + 1,
                       bg='#15171e', fg='#c0caf5', selectcolor='#2c324a',
                       activebackground='#15171e', font=('Segoe UI', 9)).pack(side='left')

    mk_label(panel, '箱子数').grid(row=1, column=0, sticky='w', pady=2)
    box_var = tk.IntVar(value=2)
    tk.Spinbox(panel, from_=1, to=6, textvariable=box_var, width=6).grid(row=1, column=1, sticky='w')

    mk_label(panel, '内墙密度').grid(row=2, column=0, sticky='w', pady=2)
    dens_var = tk.DoubleVar(value=0.08)
    tk.Spinbox(panel, from_=0.0, to=0.25, increment=0.02, textvariable=dens_var,
               width=6, format='%.2f').grid(row=2, column=1, sticky='w')

    mk_label(panel, '随机种子').grid(row=3, column=0, sticky='w', pady=2)
    seed_var = tk.StringVar(value='')
    tk.Entry(panel, textvariable=seed_var, width=8).grid(row=3, column=1, sticky='w')

    mk_label(panel, '发车区').grid(row=4, column=0, sticky='w', pady=2)
    zone_var = tk.StringVar(value=ZONE_LEFT)
    zfrm = tk.Frame(panel, bg='#15171e')
    zfrm.grid(row=4, column=1, sticky='w')
    tk.Radiobutton(zfrm, text='左', variable=zone_var, value=ZONE_LEFT, bg='#15171e',
                   fg='#c0caf5', selectcolor='#2c324a', activebackground='#15171e').pack(side='left')

    mk_label(panel, '速度 (ms/帧)').grid(row=5, column=0, sticky='w', pady=2)
    speed_var = tk.IntVar(value=120)
    tk.Scale(panel, from_=20, to=500, orient='horizontal', variable=speed_var,
             bg='#15171e', fg='#c0caf5', highlightthickness=0, troughcolor='#2c324a',
             length=140).grid(row=5, column=1, sticky='w')

    mk_label(panel, '样例地图').grid(row=6, column=0, sticky='w', pady=2)
    map_choice_var = tk.StringVar(value='')
    map_choice_lookup: Dict[str, dict] = {}
    map_menu = tk.OptionMenu(panel, map_choice_var, '')
    map_menu.configure(bg='#2c324a', fg='#e8edff', activebackground='#414868',
                       activeforeground='#fff', relief='flat',
                       highlightthickness=0, width=14)
    map_menu['menu'].configure(bg='#2c324a', fg='#e8edff')
    map_menu.grid(row=6, column=1, sticky='w')

    def refresh_map_choices(selected_label: Optional[str] = None):
        records = []
        for rec in BUILTIN_MAPS:
            records.append({'source': '内置', **rec})
        for rec in _load_custom_maps():
            records.append({'source': '保存', **rec})

        menu = map_menu['menu']
        menu.delete(0, 'end')
        map_choice_lookup.clear()
        for rec in records:
            label = f"{rec['source']} | {rec['name']}"
            map_choice_lookup[label] = rec
            menu.add_command(label=label,
                             command=lambda value=label: map_choice_var.set(value))

        if not records:
            map_choice_var.set('')
        elif selected_label in map_choice_lookup:
            map_choice_var.set(selected_label)
        else:
            map_choice_var.set(next(iter(map_choice_lookup)))

    def log_write(lines):
        log.delete('1.0', 'end')
        log.insert('end', '\n'.join(lines))

    # ---------- 渲染 ----------
    def cell_rect(r, c):
        x = OX + c * CELL
        y = OY + r * CELL
        return x, y, x + CELL, y + CELL

    def render(fr):
        canvas.delete('all')
        show_classes = fr.get('level', 1) >= 2
        # 标题
        canvas.create_text(OX, px(4), anchor='nw', text='SOKOBAN CAR MONITOR',
                           fill=COL_TITLE, font=F_TITLE)
        # 状态行 (映射车屏 MAP:RX 行)
        stg = fr['stage']
        canvas.create_text(OX, px(18), anchor='nw',
                           text=f'STAGE: {stg}', fill=COL_OK if fr['win'] else COL_WAIT,
                           font=F_TXT)

        # 网格背景与格线
        canvas.create_rectangle(OX, OY, OX + MAPW, OY + MAPH, fill=COL_BG, outline='')
        # 发车区描边
        _draw_zone(fr)
        # 单元格
        base = fr['base']
        boxset = set(fr['boxes'])
        bombset = set(fr['bombs'])
        target_positions = [(r, c) for r in range(MAP_ROWS) for c in range(MAP_COLS)
                            if base[r][c] == TARGET]
        target_classes = fr.get('target_classes') or []
        target_class_by_pos = {
            p: target_classes[i] if i < len(target_classes) else (i + 1)
            for i, p in enumerate(target_positions)
        }
        for r in range(MAP_ROWS):
            for c in range(MAP_COLS):
                v = base[r][c]
                col = None
                if v == WALL:
                    col = COL_WALL
                elif v == TARGET:
                    col = COL_TARGET
                if col:
                    x0, y0, x1, y1 = cell_rect(r, c)
                    canvas.create_rectangle(x0 + 1, y0 + 1, x1 - 1, y1 - 1, fill=col, outline='')
        # 目标 (单独画小环, 避免被箱遮住时看不出已到位)
        for r in range(MAP_ROWS):
            for c in range(MAP_COLS):
                if base[r][c] == TARGET and (r, c) not in boxset:
                    x0, y0, x1, y1 = cell_rect(r, c)
                    canvas.create_oval(x0 + CELL * 0.3, y0 + CELL * 0.3,
                                       x1 - CELL * 0.3, y1 - CELL * 0.3,
                                       outline='#ffffff', width=1)
                    if show_classes:
                        canvas.create_text((x0 + x1) / 2, (y0 + y1) / 2,
                                           text=str(target_class_by_pos.get((r, c), '')),
                                           fill='#ffffff', font=F_SMALL)
        # 箱子
        box_classes = fr.get('box_classes') or []
        for i, (r, c) in enumerate(fr['boxes']):
            x0, y0, x1, y1 = cell_rect(r, c)
            on_t = base[r][c] == TARGET
            canvas.create_rectangle(x0 + 1, y0 + 1, x1 - 1, y1 - 1, fill=COL_BOX,
                                    outline=COL_TARGET if on_t else '', width=2 if on_t else 0)
            if show_classes:
                canvas.create_text((x0 + x1) / 2, (y0 + y1) / 2,
                                   text=str(box_classes[i] if i < len(box_classes) else (i + 1)),
                                   fill='#111827', font=F_SMALL)
        # 炸弹
        for (r, c) in fr['bombs']:
            x0, y0, x1, y1 = cell_rect(r, c)
            canvas.create_rectangle(x0 + 1, y0 + 1, x1 - 1, y1 - 1, fill=COL_BOMB, outline='')
        # 爆炸闪光
        if fr['explosion']:
            er, ec = fr['explosion']
            x0, y0, x1, y1 = cell_rect(er, ec)
            cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
            canvas.create_oval(cx - CELL * 1.6, cy - CELL * 1.6,
                               cx + CELL * 1.6, cy + CELL * 1.6,
                               outline='#ffac6c', width=3)
        # 格线
        for i in range(MAP_COLS + 1):
            x = OX + i * CELL
            canvas.create_line(x, OY, x, OY + MAPH, fill=COL_GRID)
        for i in range(MAP_ROWS + 1):
            y = OY + i * CELL
            canvas.create_line(OX, y, OX + MAPW, y, fill=COL_GRID)
        # 小车 (青色, 朝向三角)
        pr, pc = fr['player']
        x0, y0, x1, y1 = cell_rect(pr, pc)
        canvas.create_rectangle(x0 + 3, y0 + 3, x1 - 3, y1 - 3, fill=COL_CAR, outline='')
        _draw_heading(pr, pc, fr['last_dir'])

        # 图例
        _draw_legend()

        # 底部文字行 (映射车屏 CAR / POSE 行)
        ty = OY + MAPH + px(6)
        canvas.create_text(OX, ty, anchor='nw',
                           text=f'CAR: {pc:>2},{pr:<2}   LEVEL: {fr["level"]}   STEP: {fr["steps"]}',
                           fill=COL_TEXT, font=F_TXT)
        canvas.create_text(OX, ty + px(11), anchor='nw',
                           text=f'INFO: {fr["info"]}', fill=COL_TEXT, font=F_SMALL)
        if fr.get('recog'):
            canvas.create_text(OX, ty + px(20), anchor='nw',
                               text=f'RECOG: {fr["recog"]}', fill=COL_TARGET, font=F_SMALL)
        if fr['win']:
            canvas.create_text(OX, ty + px(29), anchor='nw',
                               text='*** PASS ***', fill=COL_OK, font=F_TXT)

    def _draw_zone(fr):
        # 正式比赛唯一左库位及其右侧地图刷新触发格。
        start, exit_c, _ = launch_cells(ZONE_LEFT)
        for r, c in (start, exit_c):
            x0, y0, x1, y1 = cell_rect(r, c)
            canvas.create_rectangle(x0 + 1, y0 + 1, x1 - 1, y1 - 1, outline=COL_ZONE, dash=(2, 2))

    def _draw_heading(r, c, d):
        x0, y0, x1, y1 = cell_rect(r, c)
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        h = CELL * 0.30
        if d == UP:
            pts = [cx, cy - h, cx - h, cy + h * 0.6, cx + h, cy + h * 0.6]
        elif d == DOWN:
            pts = [cx, cy + h, cx - h, cy - h * 0.6, cx + h, cy - h * 0.6]
        elif d == LEFT:
            pts = [cx - h, cy, cx + h * 0.6, cy - h, cx + h * 0.6, cy + h]
        else:
            pts = [cx + h, cy, cx - h * 0.6, cy - h, cx - h * 0.6, cy + h]
        canvas.create_polygon(pts, fill='#003a3a')

    def _draw_legend():
        lx = OX + MAPW + px(8)
        canvas.create_text(lx, OY, anchor='nw', text='LEGEND', fill=COL_TEXT, font=F_TXT)
        items = [('WALL', COL_WALL), ('BOX', COL_BOX), ('TARGET', COL_TARGET),
                 ('BOMB', COL_BOMB), ('CAR', COL_CAR)]
        for i, (name, col) in enumerate(items):
            y = OY + px(20) + i * px(20)
            canvas.create_rectangle(lx, y, lx + CELL, y + CELL, fill=col, outline='')
            canvas.create_text(lx + CELL + px(6), y, anchor='nw', text=name,
                               fill=COL_TEXT, font=F_SMALL)

    # ---------- 播放控制 ----------
    def show_frame():
        if not state['frames']:
            return
        idx = max(0, min(state['idx'], len(state['frames']) - 1))
        state['idx'] = idx
        render(state['frames'][idx])
        prog['text'] = f"帧 {idx + 1}/{len(state['frames'])}"

    def tick():
        if not state['playing']:
            return
        if state['idx'] >= len(state['frames']) - 1:
            state['playing'] = False
            btn_play['text'] = '▶ 播放'
            return
        state['idx'] += 1
        show_frame()
        state['after'] = root.after(speed_var.get(), tick)

    def do_generate():
        seed = None
        s = seed_var.get().strip()
        if s.lstrip('-').isdigit():
            seed = int(s)
        info = gen_level_map(level_var.get(), box_var.get(), dens_var.get(), seed, zone_var.get())
        _load_info(info)

    def _load_info(info):
        state['info'] = info
        frames, lines = build_timeline(info)
        state['frames'] = frames
        state['idx'] = 0
        state['playing'] = False
        btn_play['text'] = '▶ 播放'
        log_write([f'关卡 {info["level"]} | 发车区 {info["zone"]} | '
                   f'箱子 {len(extract_elements(info["map"], BOX))}'] + lines)
        show_frame()

    def _load_text_map(text: str, level: int,
                       box_classes: Optional[list] = None,
                       target_classes: Optional[list] = None):
        try:
            info = _info_from_map_text(text, level, box_classes, target_classes)
        except ValueError as err:
            messagebox.showerror('地图解析失败', str(err))
            return
        level_var.set(level)
        box_var.set(len(extract_elements(info['map'], BOX)))
        _load_info(info)

    def do_load_choice():
        rec = map_choice_lookup.get(map_choice_var.get())
        if not rec:
            messagebox.showwarning('未选择地图', '请先选择一张内置或保存的地图')
            return
        _load_text_map(rec['text'], int(rec.get('level', level_var.get())),
                       rec.get('box_classes'), rec.get('target_classes'))

    def do_play():
        if not state['frames']:
            return
        if state['playing']:
            state['playing'] = False
            btn_play['text'] = '▶ 播放'
            if state['after']:
                root.after_cancel(state['after'])
        else:
            if state['idx'] >= len(state['frames']) - 1:
                state['idx'] = 0
            state['playing'] = True
            btn_play['text'] = '⏸ 暂停'
            tick()

    def do_step(delta):
        state['playing'] = False
        btn_play['text'] = '▶ 播放'
        state['idx'] += delta
        show_frame()

    def do_reset():
        state['playing'] = False
        btn_play['text'] = '▶ 播放'
        state['idx'] = 0
        show_frame()

    def do_custom():
        initial = None
        initial_box_classes = None
        initial_target_classes = None
        if state.get('info'):
            initial = export_map_text(state['info']['map'], state['info']['start'])
            initial_box_classes = state['info'].get('box_classes')
            initial_target_classes = state['info'].get('target_classes')
        _open_custom_editor(root, _load_custom, level_var.get(),
                            on_save=_save_from_editor,
                            initial_text=initial,
                            initial_box_classes=initial_box_classes,
                            initial_target_classes=initial_target_classes)

    def _load_custom(text, level, box_classes=None, target_classes=None):
        _load_text_map(text, level, box_classes, target_classes)

    def _save_from_editor(name, text, level, box_classes=None, target_classes=None):
        try:
            _save_custom_map(name, text, level, box_classes, target_classes)
        except (ValueError, OSError) as err:
            messagebox.showerror('保存失败', str(err))
            return False
        refresh_map_choices(f"保存 | {name.strip()}")
        return True

    # 按钮区
    btns = tk.Frame(panel, bg='#15171e')
    btns.grid(row=7, column=0, columnspan=2, sticky='w', pady=(10, 0))

    def mkbtn(text, cmd):
        return tk.Button(btns, text=text, command=cmd, bg='#2c324a', fg='#e8edff',
                         activebackground='#414868', activeforeground='#fff',
                         relief='flat', font=('Segoe UI', 9), padx=8, pady=3)

    mkbtn('生成地图', do_generate).grid(row=0, column=0, padx=2, pady=2)
    mkbtn('自定义地图', do_custom).grid(row=0, column=1, padx=2, pady=2)
    btn_play = mkbtn('▶ 播放', do_play)
    btn_play.grid(row=0, column=2, padx=2, pady=2)
    mkbtn('载入地图', do_load_choice).grid(row=0, column=3, padx=2, pady=2)
    mkbtn('|◀', lambda: do_step(-1)).grid(row=1, column=0, padx=2, pady=2)
    mkbtn('▶|', lambda: do_step(1)).grid(row=1, column=1, padx=2, pady=2)
    mkbtn('⟲ 复位', do_reset).grid(row=1, column=2, padx=2, pady=2)

    prog = tk.Label(panel, text='帧 0/0', bg='#15171e', fg='#7a89b3', font=('Consolas', 9))
    prog.grid(row=8, column=0, columnspan=2, sticky='w', pady=(8, 0))

    # 初始演示
    refresh_map_choices()
    if initial_info is None:
        initial_info = gen_level_map(1, 2, 0.08, 20260608, ZONE_LEFT)
    _load_info(initial_info)
    if autoplay:
        root.after(500, do_play)

    root.mainloop()


def _open_custom_editor(root, on_load, default_level=1,
                        on_save=None, initial_text: Optional[str] = None,
                        initial_box_classes: Optional[list] = None,
                        initial_target_classes: Optional[list] = None):
    import tkinter as tk
    from tkinter import messagebox, simpledialog

    top = tk.Toplevel(root)
    top.title('自定义地图 (12 行 × 16 列)')
    top.configure(bg='#15171e')

    tip = ('字符: #=墙  -=空地  .=目标  $=箱子  *=炸弹  @=发车/起点\n'
           '需 12 行 × 16 列, 且仅一个 @。\n'
           '提示: 第 2/3 关或地图含炸弹时, 会逐个实地观察全部箱子与目标。')
    tk.Label(top, text=tip, bg='#15171e', fg='#9fb0d0', font=('Consolas', 9),
             justify='left').pack(anchor='w', padx=10, pady=(10, 4))

    lvfrm = tk.Frame(top, bg='#15171e')
    lvfrm.pack(anchor='w', padx=10)
    tk.Label(lvfrm, text='关卡:', bg='#15171e', fg='#c0caf5').pack(side='left')
    lv = tk.IntVar(value=default_level)
    for i in range(1, 4):
        tk.Radiobutton(lvfrm, text=str(i), variable=lv, value=i, bg='#15171e',
                       fg='#c0caf5', selectcolor='#2c324a').pack(side='left')

    namefrm = tk.Frame(top, bg='#15171e')
    namefrm.pack(anchor='w', padx=10, pady=(6, 0))
    tk.Label(namefrm, text='保存名:', bg='#15171e', fg='#c0caf5').pack(side='left')
    name_var = tk.StringVar(value='我的地图')
    tk.Entry(namefrm, textvariable=name_var, width=18, bg='#0c0e13',
             fg='#e8edff', insertbackground='#fff').pack(side='left', padx=(6, 0))

    body = tk.Frame(top, bg='#15171e')
    body.pack(padx=10, pady=8)

    txt = tk.Text(body, width=20, height=12, font=('Consolas', 14),
                  bg='#0c0e13', fg='#e8edff', insertbackground='#fff')
    txt.pack(side='left')

    preview = tk.Frame(body, bg='#15171e')
    preview.pack(side='left', padx=(12, 0), anchor='n')
    tk.Label(preview, text='点击箱子/目标设置编号', bg='#15171e', fg='#c0caf5',
             font=('Segoe UI', 9)).pack(anchor='w')
    cell = 24
    preview_canvas = tk.Canvas(preview, width=MAP_COLS * cell, height=MAP_ROWS * cell,
                               bg='#0c0e13', highlightthickness=0)
    preview_canvas.pack(pady=(4, 4))
    status_var = tk.StringVar(value='预览就绪')
    tk.Label(preview, textvariable=status_var, bg='#15171e', fg='#9fb0d0',
             font=('Segoe UI', 9)).pack(anchor='w')

    # 模板
    sample = (
        '################\n'
        '#--------------#\n'
        '#----$----.----#\n'
        '#--------------#\n'
        '#--------------#\n'
        '#--------------#\n'
        '#@-------------#\n'
        '#--------------#\n'
        '#----$----.----#\n'
        '#--------------#\n'
        '#--------------#\n'
        '################'
    )
    txt.insert('1.0', initial_text if initial_text else sample)

    initial_map, _, initial_err = parse_map_text(_normalize_map_text(txt.get('1.0', 'end')))
    box_class_by_pos: Dict[tuple, int] = {}
    target_class_by_pos: Dict[tuple, int] = {}
    if not initial_err:
        for i, pos in enumerate(extract_elements(initial_map, BOX)):
            cls = initial_box_classes[i] if initial_box_classes and i < len(initial_box_classes) else (i + 1)
            box_class_by_pos[pos] = int(cls)
        for i, pos in enumerate(extract_elements(initial_map, TARGET)):
            cls = initial_target_classes[i] if initial_target_classes and i < len(initial_target_classes) else (i + 1)
            target_class_by_pos[pos] = int(cls)

    def parse_current(show_error=False):
        clean_text = _normalize_map_text(txt.get('1.0', 'end'))
        the_map, player_pos, err = parse_map_text(clean_text)
        if err:
            status_var.set(f'地图错误: {err}')
            if show_error:
                messagebox.showerror('地图解析失败', err, parent=top)
            return None
        return the_map, player_pos, clean_text

    def sync_class_maps(show_error=False):
        parsed = parse_current(show_error)
        if parsed is None:
            return None
        the_map, _, _ = parsed
        boxes = extract_elements(the_map, BOX)
        targets = extract_elements(the_map, TARGET)
        old_boxes = dict(box_class_by_pos)
        old_targets = dict(target_class_by_pos)
        box_class_by_pos.clear()
        target_class_by_pos.clear()
        for i, pos in enumerate(boxes):
            box_class_by_pos[pos] = old_boxes.get(pos, i + 1)
        for i, pos in enumerate(targets):
            target_class_by_pos[pos] = old_targets.get(pos, i + 1)
        status_var.set(f'箱子 {len(boxes)} 个, 目标 {len(targets)} 个')
        return the_map, boxes, targets

    def draw_preview():
        preview_canvas.delete('all')
        synced = sync_class_maps(False)
        if synced is None:
            return
        the_map, _, _ = synced
        show_classes = lv.get() >= 2
        clean_text = _normalize_map_text(txt.get('1.0', 'end'))
        _, player_pos, _ = parse_map_text(clean_text)
        colors = {
            EMPTY: '#111827',
            WALL: '#30384d',
            TARGET: '#5f6f52',
            BOX: '#d8a348',
            BOMB: '#c55b5b',
        }
        for r in range(MAP_ROWS):
            for c in range(MAP_COLS):
                v = the_map[r][c]
                x0, y0 = c * cell, r * cell
                x1, y1 = x0 + cell, y0 + cell
                preview_canvas.create_rectangle(x0 + 1, y0 + 1, x1 - 1, y1 - 1,
                                                fill=colors.get(v, '#111827'),
                                                outline='#263044')
                if v == TARGET:
                    preview_canvas.create_oval(x0 + 5, y0 + 5, x1 - 5, y1 - 5,
                                               outline='#ffffff', width=1)
                    if show_classes:
                        preview_canvas.create_text((x0 + x1) / 2, (y0 + y1) / 2,
                                                   text=str(target_class_by_pos.get((r, c), '')),
                                                   fill='#ffffff', font=('Consolas', 9, 'bold'))
                elif v == BOX and show_classes:
                    preview_canvas.create_text((x0 + x1) / 2, (y0 + y1) / 2,
                                               text=str(box_class_by_pos.get((r, c), '')),
                                               fill='#111827', font=('Consolas', 9, 'bold'))
                elif v == BOMB:
                    preview_canvas.create_text((x0 + x1) / 2, (y0 + y1) / 2,
                                               text='*', fill='#ffffff',
                                               font=('Consolas', 10, 'bold'))
        if player_pos:
            r, c = player_pos
            x0, y0 = c * cell, r * cell
            preview_canvas.create_rectangle(x0 + 4, y0 + 4, x0 + cell - 4, y0 + cell - 4,
                                            fill='#7aa2f7', outline='')
            preview_canvas.create_text(x0 + cell / 2, y0 + cell / 2, text='@',
                                       fill='#111827', font=('Consolas', 9, 'bold'))

    def class_lists_for_current(show_error=False):
        synced = sync_class_maps(show_error)
        if synced is None:
            return None
        _, boxes, targets = synced
        box_classes = [box_class_by_pos[pos] for pos in boxes]
        target_classes = [target_class_by_pos[pos] for pos in targets]
        return box_classes, target_classes

    def set_cell_class(event):
        synced = sync_class_maps(True)
        if synced is None:
            return
        row = int(event.y // cell)
        col = int(event.x // cell)
        if not (0 <= row < MAP_ROWS and 0 <= col < MAP_COLS):
            return
        pos = (row, col)
        if pos in box_class_by_pos:
            current = box_class_by_pos[pos]
            label = '箱子'
            target_dict = box_class_by_pos
        elif pos in target_class_by_pos:
            current = target_class_by_pos[pos]
            label = '目标点'
            target_dict = target_class_by_pos
        else:
            status_var.set('请点击箱子($)或目标点(.)')
            return
        value = simpledialog.askinteger('设置编号',
                                        f'{label} ({row},{col}) 的编号:',
                                        initialvalue=current, minvalue=1, maxvalue=99,
                                        parent=top)
        if value is not None:
            target_dict[pos] = value
            draw_preview()

    def reset_classes():
        synced = sync_class_maps(True)
        if synced is None:
            return
        _, boxes, targets = synced
        for i, pos in enumerate(boxes):
            box_class_by_pos[pos] = i + 1
        for i, pos in enumerate(targets):
            target_class_by_pos[pos] = i + 1
        draw_preview()

    preview_canvas.bind('<Button-1>', set_cell_class)
    txt.bind('<KeyRelease>', lambda _event: draw_preview())
    lv.trace_add('write', lambda *_args: draw_preview())

    def submit():
        classes = class_lists_for_current(True)
        if classes is None:
            return
        box_classes, target_classes = classes
        if lv.get() < 2:
            box_classes, target_classes = [], []
        on_load(txt.get('1.0', 'end'), lv.get(), box_classes, target_classes)
        top.destroy()

    def save_current():
        if on_save is None:
            return
        classes = class_lists_for_current(True)
        if classes is None:
            return
        box_classes, target_classes = classes
        if lv.get() < 2:
            box_classes, target_classes = [], []
        ok = on_save(name_var.get(), txt.get('1.0', 'end'), lv.get(),
                     box_classes, target_classes)
        if ok:
            messagebox.showinfo('保存成功', f'已保存: {name_var.get().strip()}')

    preview_btns = tk.Frame(preview, bg='#15171e')
    preview_btns.pack(anchor='w', pady=(6, 0))
    tk.Button(preview_btns, text='刷新预览', command=draw_preview, bg='#2c324a', fg='#e8edff',
              relief='flat', padx=8, pady=3).pack(side='left', padx=(0, 4))
    tk.Button(preview_btns, text='重置编号', command=reset_classes, bg='#2c324a', fg='#e8edff',
              relief='flat', padx=8, pady=3).pack(side='left')

    btnfrm = tk.Frame(top, bg='#15171e')
    btnfrm.pack(pady=(0, 10))
    tk.Button(btnfrm, text='载入并模拟', command=submit, bg='#2c324a', fg='#e8edff',
              relief='flat', padx=12, pady=4).pack(side='left', padx=4)
    tk.Button(btnfrm, text='保存地图', command=save_current, bg='#2c324a', fg='#e8edff',
              relief='flat', padx=12, pady=4).pack(side='left', padx=4)
    draw_preview()


def main():
    if '--selftest' in sys.argv:
        sys.exit(run_selftest())
    if '--last-saved' in sys.argv:
        saved_maps = _load_custom_maps()
        if not saved_maps:
            raise SystemExit('没有可用的已保存地图')
        record = saved_maps[-1]
        info = _info_from_map_text(
            record['text'], record['level'],
            record.get('box_classes'), record.get('target_classes'))
        launch_gui(info, autoplay='--autoplay' in sys.argv)
        return
    launch_gui()


if __name__ == '__main__':
    main()

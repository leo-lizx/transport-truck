"""
sokoban_gui.py — 推箱子算法验证器（Tkinter GUI）
================================================
依赖：仅 Python 标准库（tkinter 内置）
运行：python sokoban_gui.py
"""

import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time
from typing import Optional

from sokoban_validator import (
    MAP_ROWS, MAP_COLS,
    INNER_R_MIN, INNER_R_MAX, INNER_C_MIN, INNER_C_MAX,
    EMPTY, WALL, TARGET, BOX, BOMB,
    DR, DC,
    extract_elements, is_inner,
    solve_stage1, solve_stage2, solve_stage3,
    actions_to_string, actions_to_waypoints,
    check_win, check_deadlock,
    generate_map,
)

# ── 配色 ────────────────────────────────────────────────────────────
THEME = {
    "bg":        "#1e1e2e",
    "panel":     "#2a2a3e",
    "border":    "#44475a",
    "accent":    "#bd93f9",
    "green":     "#50fa7b",
    "orange":    "#ffb86c",
    "red":       "#ff5555",
    "pink":      "#ff79c6",
    "text":      "#f8f8f2",
    "dim":       "#6272a4",
    "btn":       "#44475a",
    "btn_h":     "#6272a4",
}

CELL = {
    "wall":      "#44475a",
    "empty":     "#282a36",
    "target_bg": "#1a1a2e",
    "box":       "#ffb86c",
    "bomb":      "#ff5555",
    "box_ok":    "#50fa7b",
    "player":    "#bd93f9",
    "pl_tgt":    "#ff79c6",
    "grid":      "#2d2f3f",
    "frame":     "#6272a4",
}

CELL_PX  = 38
F_CELL   = ("Segoe UI", 11, "bold")
F_UI     = ("Segoe UI", 10)
F_MONO   = ("Consolas", 9)
F_H1     = ("Segoe UI", 14, "bold")
F_H2     = ("Segoe UI", 11, "bold")


# ════════════════════════════════════════════════════════════════════
class SokobanApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("推箱子算法验证器")
        self.configure(bg=THEME["bg"])
        self.resizable(True, True)

        self.cur_map:    Optional[list]  = None
        self.player:     tuple = (5, 7)
        self.boxes:      list  = []
        self._i_player:  tuple = (5, 7)
        self._i_boxes:   list  = []
        self.solution:   Optional[dict]  = None
        self._actions:   list  = []
        self._box_seq:   list  = []
        self._subtask_ends: list  = []
        self._anim_map:  Optional[list]  = None
        self._wall_pos:  Optional[tuple] = None
        self.step_idx:   int   = 0
        self.anim_run:   bool  = False
        self.push_count: int   = 0

        self.speed_var   = tk.DoubleVar(value=0.20)
        self.stage_var   = tk.IntVar(value=1)
        self.boxn_var    = tk.IntVar(value=2)
        self.seed_var    = tk.StringVar(value="")
        self.wall_var    = tk.DoubleVar(value=0.10)
        self.progress_v  = tk.IntVar(value=0)
        self.preview_var = tk.BooleanVar(value=False)

        self._build_ui()
        self.after(200, self._auto_gen)

    def _build_ui(self):
        hdr = tk.Frame(self, bg=THEME["bg"])
        hdr.pack(fill="x", padx=16, pady=(12, 4))
        tk.Label(hdr, text="推箱子算法验证器",
                 font=F_H1, bg=THEME["bg"], fg=THEME["accent"]).pack(side="left")
        tk.Label(hdr, text="algo_sokoban_solver.c  ·  PC验证",
                 font=F_UI, bg=THEME["bg"], fg=THEME["dim"]).pack(side="left", padx=10)
        _sep(self)

        body = tk.Frame(self, bg=THEME["bg"])
        body.pack(fill="both", expand=True, padx=16)
        self._build_left(body)
        self._build_right(body)

        _sep(self)
        self._build_log()

    def _build_left(self, parent):
        left = tk.Frame(parent, bg=THEME["bg"])
        left.pack(side="left", anchor="n")

        cw = MAP_COLS * CELL_PX
        ch = MAP_ROWS * CELL_PX
        self.canvas = tk.Canvas(left, width=cw, height=ch,
                                bg=CELL["empty"], highlightthickness=2,
                                highlightbackground=THEME["border"])
        self.canvas.pack()

        pf = tk.Frame(left, bg=THEME["bg"])
        pf.pack(fill="x", pady=(6, 0))
        tk.Label(pf, text="进度", font=F_UI, bg=THEME["bg"],
                 fg=THEME["dim"]).pack(side="left")
        ttk.Progressbar(pf, variable=self.progress_v, maximum=100,
                        length=200,
                        style="S.Horizontal.TProgressbar").pack(side="left", padx=6)
        self.lbl_prog = tk.Label(pf, text="0/0", font=F_UI,
                                  bg=THEME["bg"], fg=THEME["text"])
        self.lbl_prog.pack(side="left")

        sf = tk.Frame(left, bg=THEME["bg"])
        sf.pack(fill="x", pady=6)
        self.lbl_steps  = _stat(sf, "步数",  "0",  THEME["accent"])
        self.lbl_pushes = _stat(sf, "推箱",  "0",  THEME["orange"])
        self.lbl_status = _stat(sf, "状态",  "就绪", THEME["green"])

    def _build_right(self, parent):
        right = tk.Frame(parent, bg=THEME["bg"], width=310)
        right.pack(side="left", fill="both", expand=True, padx=(14, 0), anchor="n")
        right.pack_propagate(False)

        _sec(right, "地图配置")

        r1 = tk.Frame(right, bg=THEME["bg"])
        r1.pack(fill="x", pady=2)
        tk.Label(r1, text="阶段", font=F_UI, bg=THEME["bg"],
                 fg=THEME["text"]).pack(side="left")
        for s, lbl in [(1, "Stage1"), (2, "Stage2"), (3, "Stage3")]:
            tk.Radiobutton(r1, text=lbl, variable=self.stage_var, value=s,
                           font=F_UI, bg=THEME["bg"], fg=THEME["text"],
                           selectcolor=THEME["panel"],
                           activebackground=THEME["bg"],
                           command=self._on_stage_change).pack(side="left", padx=3)

        r2 = tk.Frame(right, bg=THEME["bg"])
        r2.pack(fill="x", pady=2)
        tk.Label(r2, text="箱子数", font=F_UI, bg=THEME["bg"],
                 fg=THEME["text"]).pack(side="left")
        tk.Spinbox(r2, from_=1, to=6, width=4, textvariable=self.boxn_var,
                   font=F_UI, bg=THEME["panel"], fg=THEME["text"],
                   buttonbackground=THEME["btn"],
                   insertbackground=THEME["text"]).pack(side="left", padx=6)
        tk.Label(r2, text="种子", font=F_UI, bg=THEME["bg"],
                 fg=THEME["text"]).pack(side="left", padx=(8, 0))
        tk.Entry(r2, textvariable=self.seed_var, width=7, font=F_UI,
                 bg=THEME["panel"], fg=THEME["text"],
                 insertbackground=THEME["text"]).pack(side="left", padx=4)

        r3 = tk.Frame(right, bg=THEME["bg"])
        r3.pack(fill="x", pady=2)
        tk.Label(r3, text="墙密度", font=F_UI, bg=THEME["bg"],
                 fg=THEME["text"]).pack(side="left")
        tk.Scale(r3, from_=0.0, to=0.30, resolution=0.01,
                 variable=self.wall_var, orient="horizontal", length=150,
                 bg=THEME["bg"], fg=THEME["text"], troughcolor=THEME["panel"],
                 highlightthickness=0,
                 activebackground=THEME["accent"]).pack(side="left", padx=4)

        _btn(right, "⟳  生成地图并求解", self._auto_gen, THEME["accent"], pady=6)

        # 分阶段预览开关
        prev_f = tk.Frame(right, bg=THEME["bg"])
        prev_f.pack(fill="x", pady=(0, 4))
        tk.Checkbutton(prev_f, text="分阶段预览 (各子任务完成后自动暂停)",
                       variable=self.preview_var,
                       bg=THEME["bg"], fg=THEME["text"],
                       selectcolor=THEME["panel"],
                       activebackground=THEME["bg"],
                       font=F_UI).pack(side="left")

        self.s2_frame = tk.LabelFrame(right, text="Stage2 箱→目标映射",
                                      font=F_UI, bg=THEME["bg"],
                                      fg=THEME["orange"], labelanchor="nw")
        self.s2_vars: list = []

        _sec(right, "动画控制")

        spd_f = tk.Frame(right, bg=THEME["bg"])
        spd_f.pack(fill="x", pady=2)
        tk.Label(spd_f, text="速度", font=F_UI, bg=THEME["bg"],
                 fg=THEME["text"]).pack(side="left")
        tk.Scale(spd_f, from_=0.03, to=1.0, resolution=0.03,
                 variable=self.speed_var, orient="horizontal", length=150,
                 bg=THEME["bg"], fg=THEME["text"], troughcolor=THEME["panel"],
                 highlightthickness=0,
                 activebackground=THEME["accent"]).pack(side="left", padx=4)
        tk.Label(spd_f, text="s/步", font=F_UI, bg=THEME["bg"],
                 fg=THEME["dim"]).pack(side="left")

        br_f = tk.Frame(right, bg=THEME["bg"])
        br_f.pack(fill="x", pady=4)
        _btn(br_f, "▶ 播放", self._play,  THEME["green"],  side="left", padx=(0, 4))
        _btn(br_f, "⏸ 暂停", self._pause, THEME["orange"], side="left", padx=(0, 4))
        _btn(br_f, "⏹ 重置", self._reset, THEME["red"],    side="left")

        step_f = tk.Frame(right, bg=THEME["bg"])
        step_f.pack(fill="x", pady=2)
        _btn(step_f, "← 上一步", self._step_back, THEME["btn"], side="left", padx=(0, 4))
        _btn(step_f, "下一步 →", self._step_fwd,  THEME["btn"], side="left")

        _sec(right, "方向指令串  (小写=行走 大写=推箱)")
        self.txt_cmd = _txt(right, height=4, fg=THEME["green"])

        _sec(right, "转弯路点序列")
        self.txt_wp = _txt(right, height=3, fg=THEME["orange"])

    def _build_log(self):
        lf = tk.Frame(self, bg=THEME["bg"])
        lf.pack(fill="x", padx=16, pady=(0, 10))
        tk.Label(lf, text="日志", font=F_H2,
                 bg=THEME["bg"], fg=THEME["dim"]).pack(anchor="w")
        self.txt_log = tk.Text(lf, height=5, wrap="word", font=F_MONO,
                                bg=THEME["panel"], fg=THEME["text"],
                                insertbackground=THEME["text"],
                                relief="flat", state="disabled")
        self.txt_log.pack(fill="x")
        self.txt_log.tag_config("ok",   foreground=THEME["green"])
        self.txt_log.tag_config("warn", foreground=THEME["orange"])
        self.txt_log.tag_config("err",  foreground=THEME["red"])
        self.txt_log.tag_config("info", foreground=THEME["accent"])

    def _on_stage_change(self):
        if self.stage_var.get() == 2:
            self.s2_frame.pack(fill="x", pady=4)
        else:
            self.s2_frame.pack_forget()

    def _auto_gen(self):
        if self.anim_run:
            self._pause()
        stage = self.stage_var.get()
        box_n = self.boxn_var.get()
        seed_s = self.seed_var.get().strip()
        seed   = int(seed_s) if seed_s.isdigit() else None
        wall_d = self.wall_var.get()
        self._log(f"生成地图  stage={stage} boxes={box_n} "
                  f"seed={seed if seed else '随机'} wall={wall_d:.2f}")
        self._set_status("生成中…", THEME["orange"])

        def worker():
            try:
                m, p = generate_map(stage, box_n, wall_d, seed)
                self.after(0, self._map_ready, m, p, stage)
            except Exception as e:
                self.after(0, self._log, f"生成失败: {e}", "err")
        threading.Thread(target=worker, daemon=True).start()

    def _map_ready(self, m, p, stage):
        self.cur_map       = m
        self.player        = p
        self._i_player     = p
        self.boxes         = extract_elements(m, BOX)
        self._i_boxes      = list(self.boxes)
        self.solution      = None
        self._actions      = []
        self._box_seq      = []
        self._subtask_ends = []
        self._anim_map     = None
        self._wall_pos     = None
        self.step_idx      = 0
        self.push_count    = 0

        dead, dp = check_deadlock(m)
        if dead:
            self._log(f"⚠ 死局：箱子({dp[1]},{dp[0]})被卡住", "warn")

        self._redraw()
        self._update_stats(0, 0)
        self._set_status("求解中…", THEME["orange"])
        self._log("地图就绪，开始BFS求解…")

        def solve():
            try:
                if stage == 1:
                    sol = solve_stage1(m, p)
                elif stage == 2:
                    sol = solve_stage1(m, p)
                else:
                    raw = solve_stage3(m, p)
                    sol = self._flatten3(raw)
                self.after(0, self._solve_done, sol, stage)
            except Exception as e:
                self.after(0, self._log, f"求解异常: {e}", "err")
        threading.Thread(target=solve, daemon=True).start()

    def _flatten3(self, raw) -> Optional[dict]:
        if raw is None:
            return None
        subs = []
        subs.append({
            'actions':    raw['bomb_actions'],
            'box_idx':    -1,
            'target_idx': -1,
            'player_end': raw['cur_player_after_bomb'],
            'box_start':  raw['bomb_pos'],
        })
        s1       = raw.get('stage1_result')
        wall_pos = raw.get('wall_pos')
        if s1:
            for item in s1['sub_solutions']:
                d = dict(item)
                d['box_start'] = s1['boxes'][item['box_idx']]
                subs.append(d)
            return {'sub_solutions': subs, 'boxes': s1['boxes'],
                    'targets': s1['targets'],
                    'total_steps': sum(len(x['actions']) for x in subs),
                    '_wall_pos': wall_pos}
        return {'sub_solutions': subs,
                'boxes':   extract_elements(self.cur_map, BOX),
                'targets': extract_elements(self.cur_map, TARGET),
                'total_steps': len(raw['bomb_actions']),
                '_wall_pos': wall_pos}

    def _solve_done(self, sol, stage):
        if sol is None:
            self._log("✗ 无解", "err")
            self._set_status("无解", THEME["red"])
            return
        self.solution  = sol
        self._wall_pos = sol.get('_wall_pos')
        self._log(f"✓ 求解完成  总步数: {sol['total_steps']}", "ok")
        self._expand(sol)
        self._fill_cmd(sol)
        self._update_progress(0)
        self._set_status("就绪", THEME["green"])
        if stage == 2:
            self._build_s2_ui(sol['boxes'], sol['targets'])

    def _expand(self, sol):
        self._actions      = []
        self._box_seq      = []
        self._subtask_ends = []
        cur_p = self._i_player
        boxes = list(sol.get('boxes', self.boxes))

        for sub in sol['sub_solutions']:
            bi = sub['box_idx']
            if bi >= 0:
                br, bc = sub.get('box_start', boxes[bi])
            else:
                br, bc = sub['box_start']
            for act in sub['actions']:
                self._actions.append(act)
                self._box_seq.append((bi, br, bc))
                npr = cur_p[0] + DR[act]
                npc = cur_p[1] + DC[act]
                if npr == br and npc == bc:
                    br += DR[act]
                    bc += DC[act]
                cur_p = (npr, npc)
            if self._actions:
                self._subtask_ends.append(len(self._actions) - 1)

    def _fill_cmd(self, sol):
        cmd_parts, wp_parts = [], []
        cur_p = self._i_player
        for sub in sol['sub_solutions']:
            bi = sub['box_idx']
            b_start = sub.get('box_start', sol['boxes'][bi]) if bi >= 0 else sub['box_start']
            cmd_parts.append(
                actions_to_string(sub['actions'], cur_p, b_start, use_unicode=False))
            wp_parts.append(str(
                [(c, r) for r, c in actions_to_waypoints(sub['actions'], cur_p)]))
            cur_p = sub['player_end']
        _txt_set(self.txt_cmd, ' | '.join(cmd_parts))
        _txt_set(self.txt_wp,  '\n'.join(wp_parts))

    def _build_s2_ui(self, boxes, targets):
        for w in self.s2_frame.winfo_children():
            w.destroy()
        self.s2_vars.clear()
        for i, (br, bc) in enumerate(boxes):
            row = tk.Frame(self.s2_frame, bg=THEME["bg"])
            row.pack(fill="x", pady=1)
            tk.Label(row, text=f"箱子{i+1}({bc},{br})→",
                     font=F_UI, bg=THEME["bg"], fg=THEME["text"]).pack(side="left")
            var = tk.IntVar(value=i + 1)
            self.s2_vars.append(var)
            ttk.Combobox(row, textvariable=var,
                         values=list(range(1, len(targets)+1)),
                         width=5, state="readonly",
                         font=F_UI).pack(side="left", padx=4)
        _btn(self.s2_frame, "应用映射重算",
             self._apply_s2, THEME["orange"], pady=4)
        self.s2_frame.pack(fill="x", pady=4)

    def _apply_s2(self):
        if not self.cur_map or not self.solution:
            return
        mapping = [v.get() - 1 for v in self.s2_vars]
        sol = solve_stage2(self.cur_map, self._i_player, mapping)
        self._solve_done(sol, 2)
        if sol:
            self._log(f"Stage2 重算  步数: {sol['total_steps']}", "ok")

    def _redraw(self):
        cv = self.canvas
        m  = self._anim_map if self._anim_map is not None else self.cur_map
        if m is None:
            return
        cv.delete("all")
        box_set = set(self.boxes)

        for row in range(MAP_ROWS):
            for col in range(MAP_COLS):
                x0, y0 = col * CELL_PX, row * CELL_PX
                x1, y1 = x0 + CELL_PX, y0 + CELL_PX
                cell = m[row][col]
                pos  = (row, col)

                if not is_inner(row, col) or cell == WALL:
                    bg = CELL["wall"]
                elif cell == TARGET:
                    bg = CELL["target_bg"]
                else:
                    bg = CELL["empty"]
                cv.create_rectangle(x0, y0, x1, y1, fill=bg,
                                    outline=CELL["grid"], width=1)

                cx, cy = x0 + CELL_PX//2, y0 + CELL_PX//2

                if pos == self.player:
                    fill = CELL["pl_tgt"] if cell == TARGET else CELL["player"]
                    cv.create_oval(x0+5, y0+5, x1-5, y1-5, fill=fill, outline="")
                    cv.create_text(cx, cy, text="@", font=F_CELL, fill=THEME["bg"])

                elif pos in box_set:
                    fill = CELL["box_ok"] if cell == TARGET else CELL["box"]
                    cv.create_rectangle(x0+4, y0+4, x1-4, y1-4,
                                        fill=fill, outline=THEME["bg"], width=2)
                    cv.create_text(cx, cy,
                                   text="★" if cell == TARGET else "□",
                                   font=F_CELL, fill=THEME["bg"])

                elif cell == TARGET:
                    cv.create_text(cx, cy, text="◎",
                                   font=F_CELL, fill=THEME["green"])

                elif cell == BOMB:
                    cv.create_oval(x0+6, y0+6, x1-6, y1-6,
                                   fill=CELL["bomb"], outline="")
                    cv.create_text(cx, cy, text="B", font=F_CELL, fill=THEME["text"])

                elif cell == WALL and is_inner(row, col):
                    cv.create_rectangle(x0+2, y0+2, x1-2, y1-2,
                                        fill=CELL["wall"],
                                        outline=THEME["border"], width=1)

        cv.create_rectangle(1, 1, MAP_COLS*CELL_PX-1, MAP_ROWS*CELL_PX-1,
                            outline=CELL["frame"], width=2, fill="")

    def _play(self):
        if self.solution is None:
            self._log("请先生成地图", "warn"); return
        if self.anim_run:
            return
        if self.step_idx >= len(self._actions):
            self._reset()

        self.anim_run = True
        self._set_status("播放中", THEME["green"])

        def loop():
            # 中间阶段暂停集合（最后一个子任务结束不暂停）
            pause_set = set(self._subtask_ends[:-1]) if len(self._subtask_ends) > 1 else set()
            sub_n = [0]
            while self.anim_run and self.step_idx < len(self._actions):
                self.after(0, self._exec, self.step_idx)
                cur = self.step_idx
                self.step_idx += 1
                time.sleep(self.speed_var.get())
                # 分阶段预览暂停
                if self.preview_var.get() and cur in pause_set:
                    sub_n[0] += 1
                    total = len(self._subtask_ends)
                    self.after(0, self._set_status,
                               f"第{sub_n[0]}/{total}段完成，1.5s后继续…",
                               THEME["pink"])
                    time.sleep(1.5)
                    if self.anim_run:
                        self.after(0, self._set_status, "播放中", THEME["green"])
            if self.step_idx >= len(self._actions):
                self.anim_run = False
                self.after(0, self._anim_done)
        threading.Thread(target=loop, daemon=True).start()

    def _pause(self):
        self.anim_run = False
        self._set_status("暂停", THEME["orange"])

    def _reset(self):
        self.anim_run   = False
        self.step_idx   = 0
        self.push_count = 0
        self.player    = self._i_player
        self.boxes     = list(self._i_boxes)
        self._anim_map = None
        self._redraw()
        self._update_stats(0, 0)
        self._update_progress(0)
        self._set_status("就绪", THEME["green"])

    def _step_fwd(self):
        if self.solution is None or self.step_idx >= len(self._actions):
            return
        self._exec(self.step_idx)
        self.step_idx += 1
        if self.step_idx >= len(self._actions):
            self._anim_done()

    def _step_back(self):
        if self.solution is None or self.step_idx <= 0:
            return
        self.step_idx  -= 1
        self.push_count = 0
        self.player    = self._i_player
        self.boxes     = list(self._i_boxes)
        self._anim_map = None   # 重建，由 _apply 逐步更新
        for i in range(self.step_idx):
            self._apply(i)
        self._redraw()
        self._update_stats(self.step_idx, self.push_count)
        self._update_progress(self.step_idx)

    def _apply(self, idx):
        act = self._actions[idx]
        bi, br, bc = self._box_seq[idx]
        pr, pc = self.player
        npr, npc = pr + DR[act], pc + DC[act]
        if bi == -1:
            # 炸弹步骤：更新 _anim_map
            if self._anim_map is None:
                self._anim_map = [row[:] for row in self.cur_map]
            if npr == br and npc == bc:
                nbr = br + DR[act]
                nbc = bc + DC[act]
                self._anim_map[br][bc] = EMPTY
                if self._wall_pos and (nbr, nbc) == self._wall_pos:
                    self._anim_map[nbr][nbc] = EMPTY
                    for dr2 in range(-1, 2):
                        for dc2 in range(-1, 2):
                            rr, cc = nbr + dr2, nbc + dc2
                            if 1 <= rr < MAP_ROWS - 1 and 1 <= cc < MAP_COLS - 1:
                                if self._anim_map[rr][cc] == WALL:
                                    self._anim_map[rr][cc] = EMPTY
                else:
                    self._anim_map[nbr][nbc] = BOMB
        elif npr == br and npc == bc:
            if 0 <= bi < len(self.boxes):
                self.boxes[bi] = (br + DR[act], bc + DC[act])
            self.push_count += 1
        self.player = (npr, npc)

    def _exec(self, idx):
        if idx >= len(self._actions):
            return
        act = self._actions[idx]
        bi, br, bc = self._box_seq[idx]
        pr, pc = self.player
        npr, npc = pr + DR[act], pc + DC[act]

        if bi == -1:
            # ── 炸弹步骤 ──
            if self._anim_map is None:
                self._anim_map = [row[:] for row in self.cur_map]
            if npr == br and npc == bc:          # 玩家推了炸弹
                nbr = br + DR[act]
                nbc = bc + DC[act]
                self._anim_map[br][bc] = EMPTY
                if self._wall_pos and (nbr, nbc) == self._wall_pos:
                    # 爆炸！清除 3×3 内墙
                    self._anim_map[nbr][nbc] = EMPTY
                    for dr2 in range(-1, 2):
                        for dc2 in range(-1, 2):
                            rr, cc = nbr + dr2, nbc + dc2
                            if 1 <= rr < MAP_ROWS - 1 and 1 <= cc < MAP_COLS - 1:
                                if self._anim_map[rr][cc] == WALL:
                                    self._anim_map[rr][cc] = EMPTY
                    self._log(f"💥 爆炸！墙({nbc},{nbr})周围3×3清除", "warn")
                else:
                    self._anim_map[nbr][nbc] = BOMB
        else:
            # ── 普通箱子步骤 ──
            if npr == br and npc == bc:
                if 0 <= bi < len(self.boxes):
                    self.boxes[bi] = (br + DR[act], bc + DC[act])
                self.push_count += 1

        self.player = (npr, npc)
        self._redraw()
        self._update_stats(idx + 1, self.push_count)
        self._update_progress(idx + 1)

        live_map = self._anim_map if self._anim_map is not None else self.cur_map
        if check_win(live_map, self.boxes):
            self._log("★★★ 通关！所有箱子到达目标 ★★★", "ok")
            self._set_status("通关 ★", THEME["green"])
            self.anim_run = False

    def _anim_done(self):
        self.anim_run = False
        live_map = self._anim_map if self._anim_map is not None else self.cur_map
        won = check_win(live_map, self.boxes)
        if won:
            self._set_status("通关 ★", THEME["green"])
            messagebox.showinfo("通关", "★ 所有箱子均已到达目标位置！★")
        else:
            on_t = sum(1 for r, c in self.boxes if live_map[r][c] == TARGET)
            self._set_status(f"{on_t}/{len(self.boxes)} 到位", THEME["orange"])

    def _update_stats(self, steps, pushes):
        self.lbl_steps.config(text=str(steps))
        self.lbl_pushes.config(text=str(pushes))

    def _set_status(self, text, color):
        self.lbl_status.config(text=text, fg=color)

    def _update_progress(self, cur):
        total = len(self._actions)
        self.progress_v.set(int(cur / total * 100) if total else 0)
        self.lbl_prog.config(text=f"{cur}/{total}")

    def _log(self, msg, tag="info"):
        self.txt_log.config(state="normal")
        self.txt_log.insert("end", msg + "\n", tag)
        self.txt_log.see("end")
        self.txt_log.config(state="disabled")


# ════════════════════════════════════════════════════════════════════
def _sep(parent):
    tk.Frame(parent, bg=THEME["border"], height=1).pack(
        fill="x", padx=16, pady=4)

def _sec(parent, text):
    f = tk.Frame(parent, bg=THEME["bg"])
    f.pack(fill="x", pady=(8, 2))
    tk.Label(f, text=text, font=F_H2, bg=THEME["bg"],
             fg=THEME["accent"]).pack(side="left")
    tk.Frame(f, bg=THEME["border"], height=1).pack(
        side="left", fill="x", expand=True, padx=6)

def _btn(parent, text, cmd, color, side="top", padx=0, pady=2):
    kw = {"padx": padx} if side != "top" else {"pady": pady}
    b = tk.Button(parent, text=text, command=cmd, font=F_UI,
                  bg=color, fg=THEME["bg"], relief="flat",
                  activebackground=THEME["btn_h"], activeforeground=THEME["text"],
                  cursor="hand2", bd=0, pady=5)
    b.pack(side=side, **kw)
    b.bind("<Enter>", lambda e: b.config(bg=THEME["btn_h"]))
    b.bind("<Leave>", lambda e: b.config(bg=color))
    return b

def _txt(parent, height=4, fg=None):
    t = tk.Text(parent, height=height, wrap="word", font=F_MONO,
                bg=THEME["panel"], fg=fg or THEME["text"],
                insertbackground=THEME["text"], relief="flat",
                state="disabled")
    t.pack(fill="x", pady=2)
    return t

def _txt_set(widget, text):
    widget.config(state="normal")
    widget.delete("1.0", "end")
    widget.insert("end", text)
    widget.config(state="disabled")

def _stat(parent, key, val, color):
    f = tk.Frame(parent, bg=THEME["panel"], padx=10, pady=4)
    f.pack(side="left", padx=(0, 6))
    tk.Label(f, text=key, font=F_UI, bg=THEME["panel"],
             fg=THEME["dim"]).pack()
    lbl = tk.Label(f, text=val, font=F_H2, bg=THEME["panel"], fg=color)
    lbl.pack()
    return lbl


def _apply_style():
    s = ttk.Style()
    s.theme_use("clam")
    s.configure("S.Horizontal.TProgressbar",
                troughcolor=THEME["panel"], background=THEME["accent"],
                darkcolor=THEME["accent"], lightcolor=THEME["accent"],
                bordercolor=THEME["panel"], thickness=8)
    s.configure("TCombobox",
                fieldbackground=THEME["panel"], background=THEME["btn"],
                foreground=THEME["text"], arrowcolor=THEME["text"],
                selectbackground=THEME["accent"], selectforeground=THEME["bg"])


if __name__ == "__main__":
    app = SokobanApp()
    _apply_style()
    app.mainloop()

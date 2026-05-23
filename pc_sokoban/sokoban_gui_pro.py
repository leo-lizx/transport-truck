"""
sokoban_gui_pro.py — 推箱子算法验证器 (PRO 视觉版)
==================================================
依赖：仅 Python 标准库 (tkinter 内置)
运行：python sokoban_gui_pro.py

相比 sokoban_gui.py 的提升：
  1. 动态路径可视化 — 玩家/推箱/侦查/炸弹路径分色叠加，已走段高亮
  2. 平滑插值动画 — 每步多个子帧，玩家平滑滑行；推箱同步插值
  3. 精美渲染 — 墙壁砖纹、目标脉动发光环、箱子到位金光、玩家朝向角
  4. 子任务进度卡片 — 实时显示每个箱→目标的状态
  5. 爆炸特效 — 炸弹引爆时 3x3 闪光帧
"""

from __future__ import annotations

import math
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
from typing import Optional, List, Tuple

# 复用算法层
from sokoban_validator import (
    MAP_ROWS, MAP_COLS,
    EMPTY, WALL, TARGET, BOX, BOMB,
    DR, DC,
    extract_elements, is_inner,
    solve_level,
    actions_to_string, actions_to_waypoints,
    check_win, check_deadlock,
    generate_map, default_box_mapping,
    parse_map_text, export_map_text,
    verify_run_script, build_script_from_solution,
)


# ════════════════════════════════════════════════════════════════════
# 主题与配色
# ════════════════════════════════════════════════════════════════════
THEME = {
    "bg":        "#0f1115",
    "bg_alt":    "#161922",
    "panel":     "#1c2030",
    "panel_alt": "#242940",
    "border":    "#2e3346",
    "accent":    "#7aa2f7",   # 主色
    "accent_2":  "#bb9af7",   # 紫
    "green":     "#9ece6a",
    "orange":    "#e0af68",
    "red":       "#f7768e",
    "pink":      "#ff9e64",
    "cyan":      "#7dcfff",
    "yellow":    "#e0af68",
    "text":      "#c0caf5",
    "text_dim":  "#7a89b3",
    "muted":     "#414868",
    "btn":       "#2c324a",
    "btn_h":     "#414868",
    "shadow":    "#05070b",
}

CELL = {
    "wall_top":      "#3b425e",
    "wall_bot":      "#1f2336",
    "wall_grout":    "#0d0f15",
    "empty_top":     "#1a1d2c",
    "empty_bot":     "#11131c",
    "target_glow":   "#9ece6a",
    "target_dim":    "#3a4a30",
    "box":           "#e0af68",
    "box_dark":      "#a37e44",
    "box_done":      "#9ece6a",
    "box_done_dark": "#5b7a40",
    "bomb":          "#f7768e",
    "bomb_dark":     "#a8455c",
    "player":        "#7aa2f7",
    "player_dark":   "#3a548a",
    "player_scout":  "#7dcfff",
    "scout_obs":     "#bb9af7",
    "grid":          "#1a1d2c",
    "grid_major":    "#252a3d",
    "frame":         "#414868",
    "shadow":        "#05070b",
    # 路径着色
    "path_walk_done":  "#414868",
    "path_walk_todo":  "#5a6584",
    "path_push":       "#e0af68",
    "path_push_done":  "#7a623a",
    "path_scout":      "#7dcfff",
    "path_bomb":       "#f7768e",
    "explosion":       "#ffac6c",
}

CELL_PX  = 44
F_CELL   = ("Segoe UI", 12, "bold")
F_UI     = ("Segoe UI", 10)
F_UI_S   = ("Segoe UI", 9)
F_MONO   = ("Consolas", 9)
F_MONO_S = ("Consolas", 8)
F_H1     = ("Segoe UI Semibold", 16)
F_H2     = ("Segoe UI Semibold", 11)
F_BADGE  = ("Segoe UI Semibold", 10)

# 动画子帧数（每步细分）
SUB_FRAMES        = 5
SUB_FRAME_MS      = 18
EXPLOSION_FRAMES  = 8
EXPLOSION_FRAME_MS = 35
TARGET_PULSE_MS   = 60   # 目标发光环脉动周期帧间隔



# ════════════════════════════════════════════════════════════════════
# 颜色工具
# ════════════════════════════════════════════════════════════════════
def _hex_to_rgb(h: str) -> Tuple[int, int, int]:
    h = h.lstrip("#")
    return int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16)


def _rgb_to_hex(rgb: Tuple[int, int, int]) -> str:
    r, g, b = (max(0, min(255, int(v))) for v in rgb)
    return f"#{r:02x}{g:02x}{b:02x}"


def _lerp_color(a: str, b: str, t: float) -> str:
    """在两个 hex 颜色之间插值。t∈[0,1]"""
    ar, ag, ab_ = _hex_to_rgb(a)
    br, bg, bb_ = _hex_to_rgb(b)
    return _rgb_to_hex((
        ar + (br - ar) * t,
        ag + (bg - ag) * t,
        ab_ + (bb_ - ab_) * t,
    ))


def _mix_alpha(fg: str, bg: str, alpha: float) -> str:
    """模拟透明度（Tk 不支持 alpha，按 alpha 与背景混合）"""
    return _lerp_color(bg, fg, alpha)


def _ease_inout(t: float) -> float:
    """缓动函数 — smoothstep"""
    return t * t * (3.0 - 2.0 * t)


# ════════════════════════════════════════════════════════════════════
# 主应用
# ════════════════════════════════════════════════════════════════════
class SokobanProApp(tk.Tk):
    """推箱子算法验证器（视觉升级版）。"""

    def __init__(self) -> None:
        super().__init__()
        self.title("推箱子算法验证器  ·  PRO")
        self.configure(bg=THEME["bg"])
        self.minsize(1180, 820)

        # ─ 数据 ────────────────────────────────────────────────────
        self.cur_map:    Optional[list]   = None
        self.player:     Tuple[int, int]  = (5, 7)
        self.boxes:      List[Tuple[int, int]] = []
        self._i_player:  Tuple[int, int]  = (5, 7)
        self._i_boxes:   List[Tuple[int, int]] = []
        self.solution:   Optional[dict]   = None

        # 展开后的逐步动作流
        self._actions:    List[int]   = []          # 0..3 方向, -1 = 停顿
        self._box_seq:    List[Tuple[int, int, int]] = []  # (box_idx, br, bc) before
        self._phase_seq:  List[str]   = []          # 'walk'|'push'|'scout'|'scout_observe'|'scout_infer'|'bomb'|'explode'
        self._sub_id_seq: List[int]   = []          # 子任务序号（含侦查为 -1）
        self._subtask_ends: List[int] = []
        self._step_events: List[Optional[dict]] = []  # 每步事件元数据
        # visit 信息: List[dict{kind, pos, observe, class_id, inferred}]
        self._scout_visits: List[dict] = []
        # 当前已揭示的 visit 列表 (按时序累积, 控制画布 class_id 徽章是否显示)
        self._revealed_visits: List[dict] = []
        self._wall_pos:   Optional[Tuple[int, int]] = None
        self._anim_map:   Optional[list] = None      # 含动态变化（炸墙）

        # 动画状态
        self.step_idx:   int   = 0
        self.anim_run:   bool  = False
        self.push_count: int   = 0
        self.player_facing: int = 0  # 0..3
        self._explosion_cells: List[Tuple[int, int, int]] = []  # (r,c,frame)
        self._anim_thread: Optional[threading.Thread] = None
        self._pulse_phase: float = 0.0
        self._tooltip_id: Optional[int] = None
        self._draw_lock = threading.Lock()
        self._sub_progress: List[dict] = []
        self._cur_sub_id:  int = -1

        # 子帧插值状态（player + 当前推动箱子）
        self._lerp_active: bool = False

        # ─ Tk 变量 ─────────────────────────────────────────────────
        self.speed_var   = tk.DoubleVar(value=0.18)
        self.stage_var   = tk.IntVar(value=1)
        self.boxn_var    = tk.IntVar(value=3)
        self.seed_var    = tk.StringVar(value="")
        self.wall_var    = tk.DoubleVar(value=0.10)
        self.progress_v  = tk.IntVar(value=0)
        self.preview_var = tk.BooleanVar(value=False)
        self.path_var    = tk.BooleanVar(value=True)
        self.smooth_var  = tk.BooleanVar(value=True)
        self.infer_var   = tk.BooleanVar(value=True)   # 排除法开关

        self._build_style()
        self._build_ui()
        self._start_pulse()
        self.after(220, self._auto_gen)

    # ============================================================
    # 样式
    # ============================================================
    def _build_style(self) -> None:
        s = ttk.Style(self)
        try:
            s.theme_use("clam")
        except tk.TclError:
            pass
        s.configure("Pro.Horizontal.TProgressbar",
                    troughcolor=THEME["panel"], background=THEME["accent"],
                    darkcolor=THEME["accent"], lightcolor=THEME["accent"],
                    bordercolor=THEME["panel"], thickness=10)
        s.configure("Pro.TCombobox",
                    fieldbackground=THEME["panel_alt"],
                    background=THEME["btn"],
                    foreground=THEME["text"],
                    arrowcolor=THEME["text"],
                    selectbackground=THEME["accent"],
                    selectforeground=THEME["bg"])
        s.map("Pro.TCombobox",
              fieldbackground=[("readonly", THEME["panel_alt"])])
        s.configure("Pro.TNotebook",
                    background=THEME["bg"], borderwidth=0,
                    tabmargins=[2, 5, 2, 0])
        s.configure("Pro.TNotebook.Tab",
                    background=THEME["panel"], foreground=THEME["text_dim"],
                    padding=[14, 6], font=F_UI, borderwidth=0)
        s.map("Pro.TNotebook.Tab",
              background=[("selected", THEME["panel_alt"])],
              foreground=[("selected", THEME["accent"])])


    # ============================================================
    # UI 布局
    # ============================================================
    def _build_ui(self) -> None:
        # 顶部标题栏
        hdr = tk.Frame(self, bg=THEME["bg"], height=56)
        hdr.pack(fill="x", padx=18, pady=(14, 6))
        hdr.pack_propagate(False)

        title_box = tk.Frame(hdr, bg=THEME["bg"])
        title_box.pack(side="left", anchor="w")
        tk.Label(title_box, text="推箱子算法验证器",
                 font=F_H1, bg=THEME["bg"], fg=THEME["accent"]).pack(side="top", anchor="w")
        tk.Label(title_box,
                 text="第 21 届智能车 视觉组  ·  12×16 网格  ·  侦查 / 推箱 / 炸弹",
                 font=F_UI_S, bg=THEME["bg"], fg=THEME["text_dim"]).pack(side="top", anchor="w")

        btn_box = tk.Frame(hdr, bg=THEME["bg"])
        btn_box.pack(side="right", anchor="e")
        _ghost_btn(btn_box, "📂  导入", self._import_map).pack(side="right", padx=4)
        _ghost_btn(btn_box, "💾  导出", self._export_map).pack(side="right", padx=4)
        _ghost_btn(btn_box, "🎲  随机种子", self._roll_seed).pack(side="right", padx=4)

        _sep(self, pady=(4, 6))

        # 主区域（左画布 + 右控制）
        body = tk.Frame(self, bg=THEME["bg"])
        body.pack(fill="both", expand=True, padx=18)

        self._build_canvas(body)
        self._build_panel(body)

        _sep(self, pady=(6, 4))
        self._build_log()

    # ----------------- 中央画布区 -----------------
    def _build_canvas(self, parent) -> None:
        wrap = tk.Frame(parent, bg=THEME["bg"])
        wrap.pack(side="left", anchor="n", padx=(0, 12))

        cw = MAP_COLS * CELL_PX
        ch = MAP_ROWS * CELL_PX

        # 顶部状态徽章条
        bar = tk.Frame(wrap, bg=THEME["bg"])
        bar.pack(fill="x", pady=(0, 6))
        self.badge_stage  = _badge(bar, "STAGE", "1",   THEME["accent"])
        self.badge_boxes  = _badge(bar, "箱子",   "0",   THEME["yellow"])
        self.badge_steps  = _badge(bar, "步数",   "0",   THEME["cyan"])
        self.badge_pushes = _badge(bar, "推箱",   "0",   THEME["pink"])
        self.badge_phase  = _badge(bar, "阶段",   "—",   THEME["text_dim"])
        self.badge_status = _badge(bar, "状态",   "就绪", THEME["green"])
        for b in (self.badge_stage, self.badge_boxes, self.badge_steps,
                  self.badge_pushes, self.badge_phase, self.badge_status):
            b.pack(side="left", padx=(0, 8))

        # 画布外阴影框
        sh = tk.Frame(wrap, bg=THEME["shadow"])
        sh.pack(padx=2, pady=(0, 0))
        ring = tk.Frame(sh, bg=CELL["frame"])
        ring.pack(padx=1, pady=1)

        self.canvas = tk.Canvas(ring, width=cw, height=ch,
                                 bg=CELL["empty_bot"],
                                 highlightthickness=0, bd=0)
        self.canvas.pack()
        self.canvas.bind("<Motion>",  self._on_canvas_hover)
        self.canvas.bind("<Leave>",   lambda _e: self._hide_tooltip())
        self.canvas.bind("<Button-1>", self._on_canvas_click)

        # 进度条
        prog = tk.Frame(wrap, bg=THEME["bg"])
        prog.pack(fill="x", pady=(8, 0))
        tk.Label(prog, text="进度", font=F_UI_S, bg=THEME["bg"],
                 fg=THEME["text_dim"]).pack(side="left")
        ttk.Progressbar(prog, variable=self.progress_v, maximum=100,
                        length=cw - 110, style="Pro.Horizontal.TProgressbar"
                        ).pack(side="left", padx=8)
        self.lbl_prog = tk.Label(prog, text="0 / 0", font=F_UI,
                                  bg=THEME["bg"], fg=THEME["text"])
        self.lbl_prog.pack(side="left")

        # 图例
        leg = tk.Frame(wrap, bg=THEME["panel"], padx=10, pady=6)
        leg.pack(fill="x", pady=(8, 0))
        legends = [
            ("@",  "玩家",     CELL["player"]),
            ("□",  "箱子",     CELL["box"]),
            ("◎",  "目标",     CELL["target_glow"]),
            ("◆",  "炸弹",     CELL["bomb"]),
            ("◇",  "侦查点",   CELL["scout_obs"]),
            ("━",  "推箱路径", CELL["path_push"]),
            ("┄",  "侦查路径", CELL["path_scout"]),
            ("┄",  "炸弹路径", CELL["path_bomb"]),
        ]
        for sym, txt, col in legends:
            tk.Label(leg, text=sym, font=F_CELL, bg=THEME["panel"],
                     fg=col).pack(side="left", padx=(0, 2))
            tk.Label(leg, text=txt, font=F_UI_S, bg=THEME["panel"],
                     fg=THEME["text_dim"]).pack(side="left", padx=(0, 12))

    # ----------------- 右侧控制面板 -----------------
    def _build_panel(self, parent) -> None:
        right = tk.Frame(parent, bg=THEME["bg"], width=420)
        right.pack(side="left", fill="both", expand=True, anchor="n")
        right.pack_propagate(False)

        nb = ttk.Notebook(right, style="Pro.TNotebook")
        nb.pack(fill="both", expand=True)

        # Tab 1: 求解
        tab_solve = tk.Frame(nb, bg=THEME["bg"])
        nb.add(tab_solve, text="  求解  ")
        self._build_solve_tab(tab_solve)

        # Tab 2: 动画
        tab_anim = tk.Frame(nb, bg=THEME["bg"])
        nb.add(tab_anim, text="  动画  ")
        self._build_anim_tab(tab_anim)

        # Tab 3: 指令验证
        tab_script = tk.Frame(nb, bg=THEME["bg"])
        nb.add(tab_script, text="  指令  ")
        self._build_script_tab(tab_script)

        # Tab 4: 子任务
        tab_sub = tk.Frame(nb, bg=THEME["bg"])
        nb.add(tab_sub, text="  子任务  ")
        self._build_sub_tab(tab_sub)


    def _build_solve_tab(self, parent) -> None:
        pad = tk.Frame(parent, bg=THEME["bg"])
        pad.pack(fill="both", expand=True, padx=14, pady=12)

        _section(pad, "地图配置")

        r1 = tk.Frame(pad, bg=THEME["bg"])
        r1.pack(fill="x", pady=4)
        tk.Label(r1, text="阶段", font=F_UI, bg=THEME["bg"],
                 fg=THEME["text"]).pack(side="left")
        for s, lbl, color in [(1, "Stage 1", THEME["green"]),
                              (2, "Stage 2", THEME["orange"]),
                              (3, "Stage 3", THEME["red"])]:
            rb = tk.Radiobutton(r1, text=lbl, variable=self.stage_var,
                                value=s, font=F_UI,
                                bg=THEME["bg"], fg=color,
                                selectcolor=THEME["panel_alt"],
                                activebackground=THEME["bg"],
                                activeforeground=color,
                                command=self._on_stage_change,
                                bd=0, indicatoron=True)
            rb.pack(side="left", padx=4)

        r2 = tk.Frame(pad, bg=THEME["bg"])
        r2.pack(fill="x", pady=4)
        _label(r2, "箱子数").pack(side="left")
        tk.Spinbox(r2, from_=1, to=8, width=4, textvariable=self.boxn_var,
                   font=F_UI, bg=THEME["panel_alt"], fg=THEME["text"],
                   buttonbackground=THEME["btn"],
                   insertbackground=THEME["text"],
                   relief="flat", borderwidth=0,
                   highlightthickness=1,
                   highlightbackground=THEME["border"]).pack(side="left", padx=8)

        _label(r2, "种子").pack(side="left", padx=(10, 0))
        e = tk.Entry(r2, textvariable=self.seed_var, width=8, font=F_UI,
                     bg=THEME["panel_alt"], fg=THEME["text"],
                     insertbackground=THEME["text"],
                     relief="flat",
                     highlightthickness=1,
                     highlightbackground=THEME["border"])
        e.pack(side="left", padx=4)

        r3 = tk.Frame(pad, bg=THEME["bg"])
        r3.pack(fill="x", pady=6)
        _label(r3, "墙密度").pack(side="left")
        sc = tk.Scale(r3, from_=0.0, to=0.30, resolution=0.01,
                      variable=self.wall_var, orient="horizontal", length=200,
                      bg=THEME["bg"], fg=THEME["text"],
                      troughcolor=THEME["panel_alt"],
                      highlightthickness=0, sliderrelief="flat",
                      activebackground=THEME["accent"],
                      showvalue=True)
        sc.pack(side="left", padx=4)

        # 主操作按钮
        bf = tk.Frame(pad, bg=THEME["bg"])
        bf.pack(fill="x", pady=(10, 4))
        _primary_btn(bf, "⟳  生成并求解", self._auto_gen).pack(side="left", padx=(0, 6),
                                                              fill="x", expand=True)
        _ghost_btn(bf, "求解", self._solve_only).pack(side="left", padx=2)

        # 排除法开关 (Stage2/3 用)
        infer_f = tk.Frame(pad, bg=THEME["bg"])
        infer_f.pack(fill="x", pady=(2, 4))
        _check(infer_f, "启用排除法 (识别 N-1 即可推断剩余 1 个)",
                self.infer_var).pack(anchor="w")

        # Stage2 映射
        self.s2_frame = tk.LabelFrame(pad,
                                       text="  Stage 2  箱→目标  映射  ",
                                       font=F_UI, bg=THEME["bg"],
                                       fg=THEME["orange"],
                                       labelanchor="nw",
                                       relief="flat",
                                       highlightthickness=1,
                                       highlightbackground=THEME["border"],
                                       bd=0)
        self.s2_vars: List[tk.IntVar] = []

        # 求解结果概览
        _section(pad, "求解结果")
        self.summary_frame = tk.Frame(pad, bg=THEME["panel"])
        self.summary_frame.pack(fill="x", pady=4)
        self.summary_label = tk.Label(self.summary_frame,
                                       text="（尚未求解）",
                                       justify="left",
                                       font=F_MONO, bg=THEME["panel"],
                                       fg=THEME["text_dim"],
                                       padx=10, pady=8, anchor="w")
        self.summary_label.pack(fill="x")

        _section(pad, "方向指令串")
        self.txt_cmd = _readonly_text(pad, height=4, fg=THEME["green"])
        _section(pad, "转弯路点")
        self.txt_wp  = _readonly_text(pad, height=2, fg=THEME["orange"])

    def _build_anim_tab(self, parent) -> None:
        pad = tk.Frame(parent, bg=THEME["bg"])
        pad.pack(fill="both", expand=True, padx=14, pady=12)

        _section(pad, "播放控制")

        spd = tk.Frame(pad, bg=THEME["bg"])
        spd.pack(fill="x", pady=4)
        _label(spd, "速度").pack(side="left")
        tk.Scale(spd, from_=0.03, to=0.8, resolution=0.01,
                 variable=self.speed_var, orient="horizontal", length=220,
                 bg=THEME["bg"], fg=THEME["text"],
                 troughcolor=THEME["panel_alt"],
                 highlightthickness=0, sliderrelief="flat",
                 activebackground=THEME["accent"]).pack(side="left", padx=4)
        tk.Label(spd, text="秒/步", font=F_UI_S, bg=THEME["bg"],
                 fg=THEME["text_dim"]).pack(side="left")

        # 播放主控
        ctrl = tk.Frame(pad, bg=THEME["bg"])
        ctrl.pack(fill="x", pady=8)
        self.btn_play = _primary_btn(ctrl, "▶  播放", self._play, color=THEME["green"])
        self.btn_play.pack(side="left", padx=(0, 4), fill="x", expand=True)
        _ghost_btn(ctrl, "⏸  暂停", self._pause).pack(side="left", padx=2)
        _ghost_btn(ctrl, "⏹  重置", self._reset, color=THEME["red"]
                   ).pack(side="left", padx=2)

        step = tk.Frame(pad, bg=THEME["bg"])
        step.pack(fill="x", pady=(0, 6))
        _ghost_btn(step, "◀  上一步",  self._step_back).pack(side="left", padx=(0, 4),
                                                              fill="x", expand=True)
        _ghost_btn(step, "下一步  ▶", self._step_fwd).pack(side="left", padx=2,
                                                            fill="x", expand=True)

        _section(pad, "可视化选项")
        opts = tk.Frame(pad, bg=THEME["bg"])
        opts.pack(fill="x", pady=2)
        _check(opts, "显示动态路径", self.path_var, self._redraw_safe).pack(anchor="w", pady=2)
        _check(opts, "平滑插值动画", self.smooth_var).pack(anchor="w", pady=2)
        _check(opts, "分阶段预览（每段完成后暂停 1.5s）",
               self.preview_var).pack(anchor="w", pady=2)

    def _build_script_tab(self, parent) -> None:
        pad = tk.Frame(parent, bg=THEME["bg"])
        pad.pack(fill="both", expand=True, padx=14, pady=12)
        _section(pad, "运行指令验证")
        tk.Label(pad,
                 text=("格式：\n"
                       "  SCOUT: <小写串> 或 W列,行;W列,行  (侦查段，仅行走)\n"
                       "  PUSH:  <UDLR串>  小写=行走  大写=推箱\n"
                       "  BOMB:  <UDLR串>  推炸弹（Stage 3）\n"
                       "示例：\n  SCOUT: W7,5;W9,4\n  PUSH: rrrUddd"),
                 font=F_UI_S, bg=THEME["bg"], fg=THEME["text_dim"],
                 justify="left").pack(anchor="w")
        self.txt_script = tk.Text(pad, height=10, wrap="word", font=F_MONO,
                                   bg=THEME["panel_alt"], fg=THEME["text"],
                                   insertbackground=THEME["text"],
                                   relief="flat",
                                   highlightthickness=1,
                                   highlightbackground=THEME["border"])
        self.txt_script.pack(fill="x", pady=4)
        bf = tk.Frame(pad, bg=THEME["bg"])
        bf.pack(fill="x", pady=4)
        _primary_btn(bf, "✓  验证指令", self._verify_script,
                     color=THEME["green"]).pack(side="left", padx=(0, 6),
                                                fill="x", expand=True)
        _ghost_btn(bf, "↻  从求解填充", self._fill_script).pack(side="left", padx=2)

    def _build_sub_tab(self, parent) -> None:
        pad = tk.Frame(parent, bg=THEME["bg"])
        pad.pack(fill="both", expand=True, padx=14, pady=12)
        _section(pad, "子任务进度")
        # 滚动容器
        cv_wrap = tk.Frame(pad, bg=THEME["bg"])
        cv_wrap.pack(fill="both", expand=True)
        self.sub_canvas = tk.Canvas(cv_wrap, bg=THEME["bg"],
                                     highlightthickness=0, bd=0, height=400)
        sb = tk.Scrollbar(cv_wrap, orient="vertical",
                          command=self.sub_canvas.yview, bg=THEME["panel"])
        self.sub_canvas.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self.sub_canvas.pack(side="left", fill="both", expand=True)
        self.sub_inner = tk.Frame(self.sub_canvas, bg=THEME["bg"])
        self.sub_canvas.create_window((0, 0), window=self.sub_inner,
                                       anchor="nw")
        self.sub_inner.bind("<Configure>",
                            lambda e: self.sub_canvas.configure(
                                scrollregion=self.sub_canvas.bbox("all")))

    def _build_log(self) -> None:
        lf = tk.Frame(self, bg=THEME["bg"])
        lf.pack(fill="x", padx=18, pady=(0, 12))
        hdr = tk.Frame(lf, bg=THEME["bg"])
        hdr.pack(fill="x")
        tk.Label(hdr, text="日志", font=F_H2, bg=THEME["bg"],
                 fg=THEME["accent"]).pack(side="left")
        tk.Frame(hdr, bg=THEME["border"], height=1).pack(side="left",
                                                          fill="x",
                                                          expand=True,
                                                          padx=8, pady=8)
        _ghost_btn(hdr, "清空", self._clear_log).pack(side="right")

        self.txt_log = tk.Text(lf, height=6, wrap="word", font=F_MONO,
                                bg=THEME["panel"], fg=THEME["text"],
                                insertbackground=THEME["text"],
                                relief="flat",
                                highlightthickness=1,
                                highlightbackground=THEME["border"],
                                state="disabled")
        self.txt_log.pack(fill="x", pady=(4, 0))
        self.txt_log.tag_config("ok",   foreground=THEME["green"])
        self.txt_log.tag_config("warn", foreground=THEME["orange"])
        self.txt_log.tag_config("err",  foreground=THEME["red"])
        self.txt_log.tag_config("info", foreground=THEME["accent"])
        self.txt_log.tag_config("dim",  foreground=THEME["text_dim"])


    # ============================================================
    # 事件回调
    # ============================================================
    def _on_stage_change(self) -> None:
        if self.stage_var.get() == 2:
            self.s2_frame.pack(fill="x", pady=6)
        else:
            self.s2_frame.pack_forget()

    def _roll_seed(self) -> None:
        import random
        self.seed_var.set(str(random.randint(0, 999999)))

    def _clear_log(self) -> None:
        self.txt_log.config(state="normal")
        self.txt_log.delete("1.0", "end")
        self.txt_log.config(state="disabled")

    def _import_map(self) -> None:
        path = filedialog.askopenfilename(
            title="导入地图",
            filetypes=[("文本", "*.txt"), ("所有", "*.*")])
        if not path:
            return
        try:
            with open(path, 'r', encoding='utf-8') as f:
                text = f.read()
            m, p, err = parse_map_text(text)
            if m is None:
                messagebox.showerror("导入失败", err or "格式错误")
                return
            self._log(f"已导入: {path}", "ok")
            self._map_ready(m, p, self.stage_var.get())
        except Exception as e:
            messagebox.showerror("导入失败", str(e))

    def _export_map(self) -> None:
        if not self.cur_map:
            messagebox.showwarning("导出", "请先生成或导入地图")
            return
        path = filedialog.asksaveasfilename(
            title="导出地图", defaultextension=".txt",
            filetypes=[("文本", "*.txt")])
        if not path:
            return
        text = export_map_text(self.cur_map, self._i_player)
        with open(path, 'w', encoding='utf-8') as f:
            f.write(text + '\n')
        self._log(f"已导出: {path}", "ok")

    # ============================================================
    # 求解流程
    # ============================================================
    def _auto_gen(self) -> None:
        if self.anim_run:
            self._pause()
        stage = self.stage_var.get()
        box_n = self.boxn_var.get()
        seed_s = self.seed_var.get().strip()
        seed = int(seed_s) if seed_s.isdigit() else None
        wall_d = self.wall_var.get()

        self._log(f"生成地图  stage={stage}  boxes={box_n}  "
                  f"seed={seed if seed is not None else '随机'}  "
                  f"wall={wall_d:.2f}", "info")
        self._set_status("生成中…", THEME["orange"])

        def worker() -> None:
            try:
                m, p = generate_map(stage, box_n, wall_d, seed)
                self.after(0, self._map_ready, m, p, stage)
            except Exception as e:
                self.after(0, self._log, f"生成失败: {e}", "err")
        threading.Thread(target=worker, daemon=True).start()

    def _solve_only(self) -> None:
        if not self.cur_map:
            self._log("请先生成或导入地图", "warn")
            return
        self._run_solve(self.cur_map, self._i_player, self.stage_var.get())

    def _map_ready(self, m, p, stage) -> None:
        self.cur_map       = m
        self.player        = p
        self._i_player     = p
        self.boxes         = extract_elements(m, BOX)
        self._i_boxes      = list(self.boxes)
        self.solution      = None
        self._actions      = []
        self._box_seq      = []
        self._phase_seq    = []
        self._sub_id_seq   = []
        self._subtask_ends = []
        self._step_events  = []
        self._scout_visits = []
        self._revealed_visits = []
        self._wall_pos     = None
        self._anim_map     = None
        self.step_idx      = 0
        self.push_count    = 0
        self.player_facing = 0
        self._cur_sub_id   = -1
        self._sub_progress = []
        self._explosion_cells = []

        # 更新顶部徽章
        self._set_badge(self.badge_stage, str(stage))
        self._set_badge(self.badge_boxes, str(len(self.boxes)))
        self._set_badge(self.badge_steps, "0")
        self._set_badge(self.badge_pushes, "0")
        self._set_badge(self.badge_phase, "—")

        dead, dp = check_deadlock(m)
        if dead:
            self._log(f"⚠ 初始死局：箱子({dp[1]},{dp[0]})被卡住", "warn")

        self._update_summary("（求解中…）")
        self._redraw()
        self._update_progress(0)
        self._run_solve(m, p, stage)

    def _run_solve(self, m, p, stage) -> None:
        self._set_status("求解中…", THEME["orange"])
        use_infer = bool(self.infer_var.get())

        def solve() -> None:
            try:
                mapping = None
                if stage == 2:
                    bx = extract_elements(m, BOX)
                    tg = extract_elements(m, TARGET)
                    mapping = default_box_mapping(bx, tg)
                t0 = time.time()
                sol = solve_level(stage, m, p, mapping,
                                  require_scout=(stage >= 2),
                                  use_inference=use_infer)
                elapsed = (time.time() - t0) * 1000
                self.after(0, self._solve_done, sol, stage, elapsed)
            except Exception as e:
                self.after(0, self._log, f"求解异常: {e}", "err")
        threading.Thread(target=solve, daemon=True).start()

    def _solve_done(self, sol, stage, elapsed_ms: float) -> None:
        if sol is None:
            self._log("✗ 无解（地图可能存在死局或不连通）", "err")
            self._set_status("无解", THEME["red"])
            self._update_summary("✗ 无解")
            return
        self.solution  = sol
        self._wall_pos = sol.get('_wall_pos')
        scout = sol.get('scout')
        if scout:
            self._scout_visits = list(scout.get('visits', []))

        self._log(f"✓ 求解完成  耗时 {elapsed_ms:.1f} ms  "
                  f"总步数 {sol['total_steps']}", "ok")

        self._expand_solution(sol)
        self._fill_cmd_text(sol)
        self._fill_script()
        self._build_sub_progress(sol)
        self._update_progress(0)
        self._set_status("就绪", THEME["green"])

        # 概览
        n_box = len(sol['boxes'])
        n_sub = sum(1 for s in sol['sub_solutions']
                    if s.get('phase') in ('push', 'bomb'))
        scout_steps = sum(len(s['actions']) for s in sol['sub_solutions']
                          if s.get('phase') == 'scout')
        push_steps  = sum(len(s['actions']) for s in sol['sub_solutions']
                          if s.get('phase') == 'push')
        bomb_steps  = sum(len(s['actions']) for s in sol['sub_solutions']
                          if s.get('phase') == 'bomb')
        wp_total = sum(len(actions_to_waypoints(s['actions'], (0, 0)))
                       for s in sol['sub_solutions'])

        info_lines = [
            f" Stage   : {sol.get('stage', stage)}",
            f" 箱子数   : {n_box}     子任务: {n_sub}",
            f" 总步数   : {sol['total_steps']}     转弯路点: {wp_total}",
            f" 侦查步数 : {scout_steps}",
            f" 推箱步数 : {push_steps}",
            f" 炸弹步数 : {bomb_steps}",
            f" BFS 耗时 : {elapsed_ms:.1f} ms",
        ]
        if scout:
            real_n = scout.get('visited_real', 0)
            inf_n  = scout.get('visited_inferred', 0)
            box_cls = scout.get('box_classes', [])
            tgt_cls = scout.get('target_classes', [])
            mapping = scout.get('box_to_target_idx', [])
            info_lines.append(
                f" 侦查物体 : 实测 {real_n}  推断 {inf_n}  "
                f"(节省 {inf_n} 个)")
            if box_cls:
                info_lines.append(f" 箱子类别 : {box_cls}")
            if tgt_cls:
                info_lines.append(f" 目标类别 : {tgt_cls}")
            if mapping:
                pairs = [f"box{i+1}→tgt{m+1}" for i, m in enumerate(mapping)]
                info_lines.append(f" 配对结果 : {'  '.join(pairs)}")
            self._log(f"侦查: 实测 {real_n} + 推断 {inf_n}", "info")

        self._update_summary("\n".join(info_lines))

        if stage == 2:
            self._build_s2_ui(sol['boxes'], sol['targets'])
        self._redraw()

    # ============================================================
    # 把解展平为逐步动作流（带 phase 标签 + 子任务序号）
    # 侦查段内部:
    #   - 实测访问段: 走 path_actions, 每步 phase='scout', act=0..3
    #   - 走到观察点后插入 1 个 'scout_observe' 停顿步 (act=-1) — 揭示该物体 class
    #   - 若本次实测触发了排除法连锁推断, 每个推断物体再插入 1 个
    #     'scout_infer' 停顿步 (act=-1) — 揭示推断物体 class
    # 这样动画上玩家走→停→挂牌→闪推断→走下一个, 节奏清晰
    # ============================================================
    def _expand_solution(self, sol: dict) -> None:
        self._actions      = []
        self._box_seq      = []
        self._phase_seq    = []
        self._sub_id_seq   = []
        self._subtask_ends = []
        self._step_events: List[Optional[dict]] = []   # 与 _actions 等长

        cur_p = self._i_player
        push_sub_id = -1

        for sub in sol['sub_solutions']:
            phase = sub.get('phase', 'push')
            bi = sub['box_idx']

            if phase == 'scout':
                # 按 visits 拆分: 每个 visit 推一段 walk + observe + (infer*N) 停顿
                self._expand_scout_segment(sub, cur_p)
                cur_p = sub['player_end']
                if self._actions:
                    self._subtask_ends.append(len(self._actions) - 1)
                continue

            if phase == 'bomb':
                cur_box = sub.get('box_start', (0, 0))
                box_kind = 'bomb'
                push_sub_id += 1
                sub_id = push_sub_id
            else:
                push_sub_id += 1
                cur_box = sub.get('box_start',
                                  sol['boxes'][bi] if bi >= 0 else (0, 0))
                box_kind = 'push'
                sub_id = push_sub_id

            br, bc = cur_box

            for act in sub['actions']:
                self._actions.append(act)
                self._box_seq.append((bi, br, bc))
                self._sub_id_seq.append(sub_id)
                self._step_events.append(None)

                npr = cur_p[0] + DR[act]
                npc = cur_p[1] + DC[act]

                if box_kind == 'bomb':
                    if npr == br and npc == bc:
                        self._phase_seq.append('bomb')
                        br += DR[act]; bc += DC[act]
                    else:
                        self._phase_seq.append('walk')
                else:
                    if npr == br and npc == bc:
                        self._phase_seq.append('push')
                        br += DR[act]; bc += DC[act]
                    else:
                        self._phase_seq.append('walk')
                cur_p = (npr, npc)

            if self._actions:
                self._subtask_ends.append(len(self._actions) - 1)

    def _expand_scout_segment(self, scout_sub: dict, start_pos: tuple) -> None:
        """把侦查子任务拆成逐步事件 (走+停顿+揭示)."""
        cur = start_pos
        visits = scout_sub.get('visits', [])

        for v_idx, v in enumerate(visits):
            # 推断项没有 path_actions, 只是一个揭示停顿
            if v.get('inferred'):
                # 推断项没有路径; 只在某次实测后跟随出现, 已在 'infer_chain' 里处理
                # 这里跳过它的独立路径(它已被 carrier 的 infer_chain 引出过)
                # 但若它没有 carrier (比如 truth 里允许首轮就推断), 也补一个停顿
                if not any(v in carrier.get('infer_chain', [])
                            for carrier in visits if not carrier.get('inferred')):
                    self._actions.append(-1)
                    self._box_seq.append((-1, v['pos'][0], v['pos'][1]))
                    self._phase_seq.append('scout_infer')
                    self._sub_id_seq.append(-1)
                    self._step_events.append({'reveal': v})
                continue

            # 1) 走到观察点
            for act in v.get('path_actions', []):
                self._actions.append(act)
                self._box_seq.append((-1, v['pos'][0], v['pos'][1]))
                self._phase_seq.append('scout')
                self._sub_id_seq.append(-1)
                self._step_events.append(None)
                cur = (cur[0] + DR[act], cur[1] + DC[act])

            # 2) 在观察点停顿采样 + 揭示该物体的 class id
            self._actions.append(-1)
            self._box_seq.append((-1, v['pos'][0], v['pos'][1]))
            self._phase_seq.append('scout_observe')
            self._sub_id_seq.append(-1)
            self._step_events.append({'reveal': v})

            # 3) 本次实测触发的连锁推断 — 每个推断物体一个揭示步
            for inferred_v in v.get('infer_chain', []):
                self._actions.append(-1)
                self._box_seq.append((-1, inferred_v['pos'][0],
                                       inferred_v['pos'][1]))
                self._phase_seq.append('scout_infer')
                self._sub_id_seq.append(-1)
                self._step_events.append({'reveal': inferred_v})


    def _fill_cmd_text(self, sol: dict) -> None:
        cmd_parts: List[str] = []
        wp_parts:  List[str] = []
        cur_p = self._i_player
        for sub in sol['sub_solutions']:
            phase = sub.get('phase', 'push')
            bi = sub['box_idx']
            if phase == 'scout':
                cmd_parts.append('[侦查] ' +
                                  ''.join('udlr'[a] for a in sub['actions']))
                wp_parts.append('  scout: ' + str([
                    (c, r) for r, c in actions_to_waypoints(
                        sub['actions'], cur_p)]))
            elif phase == 'bomb':
                b_start = sub['box_start']
                cmd_parts.append('[炸弹] ' + actions_to_string(
                    sub['actions'], cur_p, b_start, use_unicode=False))
                wp_parts.append('  bomb : ' + str([
                    (c, r) for r, c in actions_to_waypoints(
                        sub['actions'], cur_p)]))
            else:
                b_start = sub.get('box_start', sol['boxes'][bi])
                cmd_parts.append(actions_to_string(
                    sub['actions'], cur_p, b_start, use_unicode=False))
                wp_parts.append(f"  box{bi+1}: " + str([
                    (c, r) for r, c in actions_to_waypoints(
                        sub['actions'], cur_p)]))
            cur_p = sub['player_end']
        _set_text(self.txt_cmd, ' | '.join(cmd_parts))
        _set_text(self.txt_wp,  '\n'.join(wp_parts))

    def _fill_script(self) -> None:
        if not self.solution:
            return
        self.txt_script.delete("1.0", "end")
        self.txt_script.insert("end",
                               build_script_from_solution(self.solution))

    def _verify_script(self) -> None:
        if not self.cur_map:
            self._log("请先生成或导入地图", "warn")
            return
        text = self.txt_script.get("1.0", "end").strip()
        if not text:
            self._log("指令为空", "warn")
            return
        stage = self.stage_var.get()
        self._log("验证运行指令…", "info")

        def work() -> None:
            r = verify_run_script(self.cur_map, self._i_player, text,
                                   stage, expect_scout=(stage >= 2))
            self.after(0, self._verify_done, r)
        threading.Thread(target=work, daemon=True).start()

    def _verify_done(self, r: dict) -> None:
        if r['ok']:
            self._log(r['message'], "ok")
            self._set_status("指令验证通过", THEME["green"])
            messagebox.showinfo("验证通过",
                                f"{r['message']}\n阶段: {', '.join(r['phases_done'])}")
        else:
            self._log("✗ " + r['message'], "err")
            self._set_status("验证失败", THEME["red"])
            messagebox.showerror("验证失败", r['message'])

    def _build_s2_ui(self, boxes, targets) -> None:
        for w in self.s2_frame.winfo_children():
            w.destroy()
        self.s2_vars.clear()
        for i, (br, bc) in enumerate(boxes):
            row = tk.Frame(self.s2_frame, bg=THEME["bg"])
            row.pack(fill="x", pady=2, padx=8)
            tk.Label(row, text=f"箱子{i+1} ({bc},{br})  →  ",
                     font=F_UI, bg=THEME["bg"], fg=THEME["text"]
                     ).pack(side="left")
            var = tk.IntVar(value=i + 1)
            self.s2_vars.append(var)
            cb = ttk.Combobox(row, textvariable=var,
                              values=list(range(1, len(targets)+1)),
                              width=4, state="readonly",
                              style="Pro.TCombobox", font=F_UI)
            cb.pack(side="left", padx=2)
            tk.Label(row,
                     text=f"目标 ({targets[i % len(targets)][1]}," +
                          f"{targets[i % len(targets)][0]}) …",
                     font=F_UI_S, bg=THEME["bg"], fg=THEME["text_dim"]
                     ).pack(side="left", padx=8)
        bf = tk.Frame(self.s2_frame, bg=THEME["bg"])
        bf.pack(fill="x", pady=4, padx=8)
        _primary_btn(bf, "应用映射  ⟳ 重算", self._apply_s2,
                     color=THEME["orange"]).pack(fill="x")

    def _apply_s2(self) -> None:
        if not self.cur_map:
            return
        mapping = [v.get() - 1 for v in self.s2_vars]
        self._log(f"应用 Stage2 映射: {[m+1 for m in mapping]}", "info")
        use_infer = bool(self.infer_var.get())

        def work() -> None:
            t0 = time.time()
            sol = solve_level(2, self.cur_map, self._i_player,
                              mapping, require_scout=True,
                              use_inference=use_infer)
            elapsed = (time.time() - t0) * 1000
            self.after(0, self._solve_done, sol, 2, elapsed)
        threading.Thread(target=work, daemon=True).start()

    # ============================================================
    # 子任务进度卡片
    # ============================================================
    def _build_sub_progress(self, sol: dict) -> None:
        for w in self.sub_inner.winfo_children():
            w.destroy()
        self._sub_progress = []

        push_idx = 0
        for sub in sol['sub_solutions']:
            phase = sub.get('phase', 'push')
            bi = sub['box_idx']

            if phase == 'scout':
                title = "🔍  侦查 (访问观察点)"
                # 加入实测/推断概览
                visits = sub.get('visits', [])
                real = sum(1 for v in visits if not v.get('inferred'))
                inf  = sum(1 for v in visits if v.get('inferred'))
                detail = (f"步 {len(sub['actions'])}  · "
                          f"实测 {real} 推断 {inf}")
                color  = THEME["cyan"]
                kind   = 'scout'
            elif phase == 'bomb':
                bs = sub.get('box_start', (0, 0))
                wp = sol.get('_wall_pos', None)
                title = f"💣  炸弹  ({bs[1]},{bs[0]})  →  墙体"
                if wp:
                    title += f"({wp[1]},{wp[0]})"
                detail = f"步数 {len(sub['actions'])}"
                color  = THEME["red"]
                kind   = 'bomb'
                push_idx += 1
            else:
                bs = sub.get('box_start', sol['boxes'][bi])
                ti = sub.get('target_idx', 0)
                ts = sol['targets'][ti] if 0 <= ti < len(sol['targets']) else (0, 0)
                title = (f"📦  子任务 #{push_idx + 1}  "
                         f"箱{bi+1}({bs[1]},{bs[0]}) → 目{ti+1}({ts[1]},{ts[0]})")
                detail = f"步数 {len(sub['actions'])}"
                color  = THEME["orange"]
                kind   = 'push'
                push_idx += 1

            card = tk.Frame(self.sub_inner, bg=THEME["panel"],
                             padx=10, pady=8)
            card.pack(fill="x", pady=4, padx=2)

            # 状态指示
            badge = tk.Label(card, text="●", font=F_UI, bg=THEME["panel"],
                             fg=THEME["muted"])
            badge.pack(side="left", padx=(0, 8))
            tk.Label(card, text=title, font=F_UI, bg=THEME["panel"],
                     fg=color, anchor="w").pack(side="left")
            tk.Label(card, text=detail, font=F_MONO_S, bg=THEME["panel"],
                     fg=THEME["text_dim"]).pack(side="right")

            self._sub_progress.append({
                'kind': kind,
                'card': card,
                'badge': badge,
                'color': color,
                'sub_id': len(self._sub_progress),
            })

    def _update_sub_progress(self, current_sub_id: int,
                             phase: str = '') -> None:
        if not self._sub_progress:
            return
        is_scout_phase = phase in ('scout', 'scout_observe', 'scout_infer')
        for i, item in enumerate(self._sub_progress):
            badge = item['badge']
            kind  = item['kind']
            scout_match = (kind == 'scout' and is_scout_phase)
            push_or_bomb_match = (kind in ('push', 'bomb')
                                   and item['sub_id'] == current_sub_id
                                   and not is_scout_phase)
            if scout_match or (push_or_bomb_match and current_sub_id >= 0):
                badge.config(fg=THEME["accent"], text="◉")
                item['card'].config(bg=THEME["panel_alt"])
                for child in item['card'].winfo_children():
                    if isinstance(child, tk.Label):
                        child.config(bg=THEME["panel_alt"])
            elif (kind == 'scout'
                  and any(p in ('scout', 'scout_observe', 'scout_infer')
                            for p in self._phase_seq[:self.step_idx])
                  and not (kind == 'scout' and is_scout_phase
                            and self.step_idx > 0)):
                # 已完成的侦查
                badge.config(fg=THEME["green"], text="✓")
            else:
                badge.config(fg=THEME["muted"], text="●")
                item['card'].config(bg=THEME["panel"])
                for child in item['card'].winfo_children():
                    if isinstance(child, tk.Label):
                        child.config(bg=THEME["panel"])

        # 标记已完成的（在当前之前的 push/bomb sub_id）
        for item in self._sub_progress:
            if item['kind'] in ('push', 'bomb'):
                if item['sub_id'] < current_sub_id or (
                        current_sub_id == -1 and self.step_idx >= len(self._actions)):
                    item['badge'].config(fg=THEME["green"], text="✓")
            elif item['kind'] == 'scout':
                # 找到最后一个 scout step 的 idx
                last_scout = -1
                for k, ph in enumerate(self._phase_seq):
                    if ph in ('scout', 'scout_observe', 'scout_infer'):
                        last_scout = k
                if last_scout >= 0 and self.step_idx > last_scout:
                    item['badge'].config(fg=THEME["green"], text="✓")


    # ============================================================
    # 渲染 — 精美版
    # ============================================================
    def _start_pulse(self) -> None:
        """目标点发光环脉动 + 重绘节流。"""
        def tick() -> None:
            self._pulse_phase = (self._pulse_phase + 0.08) % (2 * math.pi)
            # 仅在静止时重绘（动画线程会自己刷新）
            if not self.anim_run and not self._lerp_active:
                self._redraw()
            self.after(TARGET_PULSE_MS, tick)
        self.after(TARGET_PULSE_MS, tick)

    def _redraw_safe(self) -> None:
        if not self.anim_run and not self._lerp_active:
            self._redraw()

    def _redraw(self,
                player_xy_override: Optional[Tuple[float, float]] = None,
                push_box_override: Optional[Tuple[int, float, float]] = None,
                explosion_overlay: bool = False) -> None:
        """主绘制函数。
        player_xy_override   : 用于子帧插值的玩家像素中心覆盖 (x, y)
        push_box_override    : (box_idx, x, y) 用于推箱时箱子像素覆盖
        explosion_overlay    : 是否绘制爆炸闪光层
        """
        with self._draw_lock:
            self._draw_internal(player_xy_override, push_box_override,
                                 explosion_overlay)

    def _draw_internal(self, player_xy_override, push_box_override,
                        explosion_overlay: bool) -> None:
        cv = self.canvas
        m  = self._anim_map if self._anim_map is not None else self.cur_map
        if m is None:
            return
        cv.delete("all")

        # ── 1. 底色棋盘格 + 阴影 ─────────────────────────────────
        for r in range(MAP_ROWS):
            for c in range(MAP_COLS):
                self._draw_floor_cell(cv, r, c, m)

        # ── 2. 已规划路径（淡色叠加） ──────────────────────────
        if self.path_var.get() and self._actions:
            self._draw_full_paths(cv)

        # ── 3. 静态实体：墙、目标、炸弹、箱子（不在动画位移中的） ─
        for r in range(MAP_ROWS):
            for c in range(MAP_COLS):
                cell = m[r][c]
                if cell == WALL:
                    self._draw_wall(cv, r, c)
                elif cell == TARGET:
                    self._draw_target(cv, r, c)
                elif cell == BOMB:
                    self._draw_bomb(cv, r, c)

        # 箱子（含到位高亮）
        for bi, (br, bc) in enumerate(self.boxes):
            # 当前正在被推动的箱子 → 像素插值
            if push_box_override is not None and push_box_override[0] == bi:
                _, px, py = push_box_override
                self._draw_box_at(cv, px, py, m[br][bc] == TARGET)
            else:
                cx, cy = self._cell_center(br, bc)
                self._draw_box_at(cv, cx, cy, m[br][bc] == TARGET)

        # ── 4. 侦查观察点 ────────────────────────────────────────
        # 仅显示已被"揭示"的 visit (动画里逐个亮起)
        for v in self._revealed_visits:
            self._draw_scout_marker(cv, v)
        # 对所有 visit 的观察点画浅色提示 (实测的灰色虚线菱形, 表示"待访问")
        for v in self._scout_visits:
            if v in self._revealed_visits:
                continue
            if v.get('inferred'):
                continue  # 推断项不预先提示, 突出"被推出"的惊喜
            obs = v.get('observe')
            if obs is not None:
                self._draw_diamond(cv, obs[0], obs[1],
                                    THEME["muted"], dashed=True, label="?")

        # ── 5. 玩家 ─────────────────────────────────────────────
        if player_xy_override is not None:
            px, py = player_xy_override
        else:
            px, py = self._cell_center(self.player[0], self.player[1])
        self._draw_player(cv, px, py)

        # ── 6. 爆炸层 ───────────────────────────────────────────
        if explosion_overlay or self._explosion_cells:
            self._draw_explosion_layer(cv)

        # ── 7. 边框 ─────────────────────────────────────────────
        cv.create_rectangle(1, 1, MAP_COLS * CELL_PX - 1,
                              MAP_ROWS * CELL_PX - 1,
                              outline=CELL["frame"], width=2)

    # --------- 辅助绘制函数 ---------
    def _cell_center(self, r: int, c: int) -> Tuple[float, float]:
        return (c * CELL_PX + CELL_PX / 2.0,
                r * CELL_PX + CELL_PX / 2.0)

    def _cell_box(self, r: int, c: int) -> Tuple[int, int, int, int]:
        return (c * CELL_PX, r * CELL_PX,
                c * CELL_PX + CELL_PX, r * CELL_PX + CELL_PX)

    def _draw_floor_cell(self, cv, r, c, m) -> None:
        x0, y0, x1, y1 = self._cell_box(r, c)
        if not is_inner(r, c) or m[r][c] == WALL:
            return  # 留给 wall draw
        # 垂直渐变：上深下浅模拟光照
        top, bot = CELL["empty_top"], CELL["empty_bot"]
        steps = 4
        for i in range(steps):
            t = i / max(1, steps - 1)
            color = _lerp_color(top, bot, t)
            yy0 = y0 + i * (y1 - y0) / steps
            yy1 = y0 + (i + 1) * (y1 - y0) / steps
            cv.create_rectangle(x0, yy0, x1, yy1, fill=color, outline="")
        # 网格线
        cv.create_line(x0, y0, x1, y0, fill=CELL["grid"], width=1)
        cv.create_line(x0, y0, x0, y1, fill=CELL["grid"], width=1)

    def _draw_wall(self, cv, r, c) -> None:
        x0, y0, x1, y1 = self._cell_box(r, c)
        # 顶面渐变
        top = CELL["wall_top"]
        bot = CELL["wall_bot"]
        steps = 5
        for i in range(steps):
            t = i / max(1, steps - 1)
            color = _lerp_color(top, bot, t)
            yy0 = y0 + i * (y1 - y0) / steps
            yy1 = y0 + (i + 1) * (y1 - y0) / steps
            cv.create_rectangle(x0, yy0, x1, yy1, fill=color, outline="")
        # 砖纹（错缝）
        brick_h = CELL_PX // 4
        for i in range(4):
            yy = y0 + i * brick_h
            offset = (CELL_PX // 2) if (i % 2) else 0
            for j in range(-1, 2):
                xx = x0 + j * CELL_PX + offset
                cv.create_line(xx, yy, xx + CELL_PX, yy,
                                fill=CELL["wall_grout"], width=1)
            cv.create_line(x0 + offset, yy, x0 + offset, yy + brick_h,
                            fill=CELL["wall_grout"], width=1)
        # 边框
        cv.create_rectangle(x0, y0, x1, y1,
                              outline=CELL["wall_grout"], width=1)

    def _draw_target(self, cv, r, c) -> None:
        x0, y0, x1, y1 = self._cell_box(r, c)
        cx, cy = self._cell_center(r, c)
        # 脉动外环
        radius_outer = CELL_PX * (0.42 + 0.05 * math.sin(self._pulse_phase))
        radius_inner = CELL_PX * 0.28
        glow = _lerp_color(CELL["target_dim"], CELL["target_glow"],
                            0.45 + 0.45 * math.sin(self._pulse_phase))
        cv.create_oval(cx - radius_outer, cy - radius_outer,
                        cx + radius_outer, cy + radius_outer,
                        outline=glow, width=2)
        cv.create_oval(cx - radius_inner, cy - radius_inner,
                        cx + radius_inner, cy + radius_inner,
                        outline=CELL["target_glow"], width=2)
        # 中心十字
        cv.create_line(cx - 4, cy, cx + 4, cy,
                        fill=CELL["target_glow"], width=2)
        cv.create_line(cx, cy - 4, cx, cy + 4,
                        fill=CELL["target_glow"], width=2)

    def _draw_bomb(self, cv, r, c) -> None:
        cx, cy = self._cell_center(r, c)
        rad = CELL_PX * 0.36
        # 外光晕
        for i, alpha in enumerate((0.18, 0.32, 0.55)):
            rr = rad + (3 - i) * 2.5
            cv.create_oval(cx - rr, cy - rr, cx + rr, cy + rr,
                            outline=_mix_alpha(CELL["bomb"],
                                                 CELL["empty_bot"], alpha),
                            width=1)
        cv.create_oval(cx - rad, cy - rad, cx + rad, cy + rad,
                        fill=CELL["bomb_dark"], outline="")
        cv.create_oval(cx - rad + 3, cy - rad + 3,
                        cx + rad - 3, cy + rad - 3,
                        fill=CELL["bomb"], outline="")
        # 高光
        cv.create_oval(cx - rad / 2, cy - rad / 2,
                        cx - rad / 8, cy - rad / 8,
                        fill="#ffd2dc", outline="")
        # 引线
        cv.create_line(cx, cy - rad, cx + 4, cy - rad - 6,
                        fill=THEME["yellow"], width=2)
        cv.create_oval(cx + 4 - 2, cy - rad - 6 - 2,
                        cx + 4 + 2, cy - rad - 6 + 2,
                        fill=THEME["yellow"], outline="")

    def _draw_box_at(self, cv, cx: float, cy: float,
                      on_target: bool) -> None:
        sz = CELL_PX * 0.78
        x0, y0 = cx - sz / 2, cy - sz / 2
        x1, y1 = cx + sz / 2, cy + sz / 2
        light, dark = ((CELL["box_done"], CELL["box_done_dark"])
                        if on_target
                        else (CELL["box"], CELL["box_dark"]))
        # 阴影
        cv.create_rectangle(x0 + 3, y0 + 3, x1 + 3, y1 + 3,
                              fill=CELL["shadow"], outline="")
        # 主体
        cv.create_rectangle(x0, y0, x1, y1,
                              fill=dark, outline="")
        cv.create_rectangle(x0 + 3, y0 + 3, x1 - 3, y1 - 3,
                              fill=light, outline="")
        # 木箱十字纹
        cv.create_line(x0 + 4, y0 + 4, x1 - 4, y1 - 4,
                        fill=dark, width=2)
        cv.create_line(x0 + 4, y1 - 4, x1 - 4, y0 + 4,
                        fill=dark, width=2)
        if on_target:
            cv.create_text(cx, cy, text="★", font=F_CELL,
                            fill=THEME["bg"])

    def _draw_scout_marker(self, cv, visit: dict) -> None:
        """绘制已揭示的侦查标记:
        - 实测: 在 observe 格画青色实心菱形 + 在 pos 上叠 class_id 数字
        - 推断: 在 pos 格画虚线菱形 + 浮 ⓘ 标识 + class_id (淡紫色)
        - 旧版兼容: 若 visit 是 (r,c) 元组直接画实心菱形
        """
        # 兼容旧调用 (元组)
        if isinstance(visit, tuple):
            self._draw_diamond(cv, visit[0], visit[1],
                                CELL["scout_obs"], dashed=False, label="")
            return

        cls = visit.get('class_id', 0)
        inferred = visit.get('inferred', False)
        pos = visit.get('pos')
        obs = visit.get('observe')
        kind = visit.get('kind', 'box')
        label = (f"{cls}" if cls > 0 else "?")

        color = CELL["scout_obs"] if not inferred else THEME["accent_2"]

        # 是否是"刚揭示"那个 — 加脉动外圈
        is_latest = (self._revealed_visits and
                     self._revealed_visits[-1] is visit)

        # 在物体所在格画 class id 大徽章 (盒/目都画)
        if pos is not None:
            self._draw_class_label(cv, pos[0], pos[1], label, color,
                                    inferred=inferred, kind=kind,
                                    pulse=is_latest)

        # 实测才画观察点 (青色菱形)
        if (not inferred) and obs is not None:
            self._draw_diamond(cv, obs[0], obs[1], color,
                                dashed=False, label="")

    def _draw_diamond(self, cv, r: int, c: int, color: str,
                       dashed: bool, label: str) -> None:
        x0, y0, x1, y1 = self._cell_box(r, c)
        m = 6
        kw = {"outline": color, "width": 2,
              "fill": _mix_alpha(color, CELL["empty_bot"], 0.18)}
        if dashed:
            kw["dash"] = (3, 3)
        cv.create_polygon(
            x0 + CELL_PX/2, y0 + m,
            x1 - m, y0 + CELL_PX/2,
            x0 + CELL_PX/2, y1 - m,
            x0 + m, y0 + CELL_PX/2,
            **kw)
        if label:
            cv.create_text(x0 + CELL_PX/2, y0 + CELL_PX/2,
                            text=label, font=F_CELL, fill=color)

    def _draw_class_label(self, cv, r: int, c: int,
                            label: str, color: str,
                            inferred: bool, kind: str,
                            pulse: bool = False) -> None:
        """在物体格右下角画 class_id 徽章; pulse=True 时画脉动光环"""
        x0, y0, x1, y1 = self._cell_box(r, c)
        bx = x1 - 9
        by = y1 - 9
        rad = 8

        if pulse:
            # 脉动外光圈 (3 层渐淡)
            for i, alpha in enumerate((0.7, 0.4, 0.18)):
                rr = rad + 3 + i * 3 + 2 * math.sin(self._pulse_phase * 2)
                cv.create_oval(bx - rr, by - rr, bx + rr, by + rr,
                                outline=_mix_alpha(color, CELL["empty_bot"],
                                                     alpha),
                                width=2)

        cv.create_oval(bx - rad, by - rad, bx + rad, by + rad,
                        fill=THEME["bg"],
                        outline=color, width=2 if not inferred else 1)
        if inferred:
            cv.create_oval(bx - rad - 2, by - rad - 2,
                            bx + rad + 2, by + rad + 2,
                            outline=color, width=1, dash=(2, 2))
        cv.create_text(bx, by, text=label,
                        font=("Segoe UI Semibold", 9),
                        fill=color)

    def _draw_player(self, cv, cx: float, cy: float) -> None:
        rad = CELL_PX * 0.34
        # 阴影
        cv.create_oval(cx - rad + 2, cy - rad + 4,
                        cx + rad + 2, cy + rad + 4,
                        fill=CELL["shadow"], outline="")
        # 是否处于侦查阶段
        in_scout = (self.step_idx > 0
                     and self.step_idx <= len(self._phase_seq)
                     and self._phase_seq[self.step_idx - 1] in
                         ('scout', 'scout_observe', 'scout_infer'))
        body  = CELL["player_scout"] if in_scout else CELL["player"]
        body_dark = _lerp_color(body, "#000000", 0.5)
        # 主体
        cv.create_oval(cx - rad, cy - rad, cx + rad, cy + rad,
                        fill=body_dark, outline="")
        cv.create_oval(cx - rad + 3, cy - rad + 3,
                        cx + rad - 3, cy + rad - 3,
                        fill=body, outline="")
        # 朝向箭头
        f = self.player_facing
        dx, dy = DC[f] * rad * 0.7, DR[f] * rad * 0.7
        cv.create_line(cx, cy, cx + dx, cy + dy,
                        fill=THEME["bg"], width=3,
                        arrow=tk.LAST, arrowshape=(7, 9, 4))
        # 高光
        cv.create_oval(cx - rad / 2, cy - rad / 2,
                        cx - rad / 8, cy - rad / 8,
                        fill=_lerp_color(body, "#ffffff", 0.45),
                        outline="")

    def _draw_explosion_layer(self, cv) -> None:
        """正在播放的爆炸闪光"""
        for (r, c, frame) in list(self._explosion_cells):
            x0, y0, x1, y1 = self._cell_box(r, c)
            t = frame / EXPLOSION_FRAMES
            # 颜色衰减：橙→黄→透
            color = _lerp_color(CELL["explosion"],
                                 THEME["yellow"], t)
            # 外圈白光
            inset = int(t * (CELL_PX / 2))
            cv.create_oval(x0 + inset, y0 + inset,
                            x1 - inset, y1 - inset,
                            outline=color, width=3,
                            fill=_mix_alpha("#ffffff",
                                             CELL["empty_bot"],
                                             max(0.0, 0.7 - t * 0.7)))


    # ============================================================
    # 路径可视化
    # ============================================================
    def _draw_full_paths(self, cv) -> None:
        """根据 _actions 绘制玩家完整路径，区分已走/未走，分相位着色。"""
        if not self._actions:
            return

        # 推算每一步的玩家坐标 + 类型
        cur_p = self._i_player
        path_pts: List[Tuple[float, float, str, bool]] = []
        # 起点
        cur_x, cur_y = self._cell_center(*cur_p)
        path_pts.append((cur_x, cur_y, 'start', True))

        for i, act in enumerate(self._actions):
            phase = self._phase_seq[i] if i < len(self._phase_seq) else 'walk'
            if act < 0:
                # 停顿步, 玩家不动 — 但仍记录节点 (与上一个相同), 用于计算 done 标记
                cx, cy = (path_pts[-1][0], path_pts[-1][1])
            else:
                cur_p = (cur_p[0] + DR[act], cur_p[1] + DC[act])
                cx, cy = self._cell_center(*cur_p)
            done = (i < self.step_idx)
            path_pts.append((cx, cy, phase, done))

        # 分段绘制：同 phase 的连续点合并为一条折线
        # 已走/未走分两层叠加
        # ── 未走层 ──（淡虚线）
        self._stroke_path_segments(cv, path_pts, only_done=False, dash=(4, 4),
                                    width=2, alpha=0.45)
        # ── 已走层 ──（实线 + 加粗）
        self._stroke_path_segments(cv, path_pts, only_done=True, dash=None,
                                    width=3, alpha=0.95)

        # 推箱路径单独高亮（粗实线带箭头）
        self._draw_push_paths(cv)

    def _stroke_path_segments(self, cv, path_pts, only_done: bool,
                                dash, width: int, alpha: float) -> None:
        """绘制玩家路径分段（按 phase 着色）"""
        if len(path_pts) < 2:
            return

        def color_for(phase: str) -> str:
            if phase in ('scout', 'scout_observe', 'scout_infer'):
                return CELL["path_scout"]
            if phase == 'bomb':
                return CELL["path_bomb"]
            if phase == 'push':
                return CELL["path_push"]
            return CELL["path_walk_done"] if only_done else CELL["path_walk_todo"]

        # 选择是否绘制本段：only_done 模式只绘制已走（done=True 的目标点段）
        seg_pts: List[Tuple[float, float]] = [path_pts[0][0:2]]
        seg_phase = path_pts[1][2] if len(path_pts) > 1 else 'walk'
        for i in range(1, len(path_pts)):
            x, y, phase, done = path_pts[i]
            seg_match = (only_done and done) or ((not only_done) and (not done))
            if seg_match and (phase == seg_phase or len(seg_pts) == 1):
                seg_phase = phase
                seg_pts.append((x, y))
            else:
                # flush 之前一段
                if len(seg_pts) >= 2:
                    self._draw_polyline(cv, seg_pts, color_for(seg_phase),
                                          width, dash, alpha)
                seg_pts = [(x, y)] if seg_match else []
                if seg_match:
                    seg_phase = phase
        if len(seg_pts) >= 2:
            self._draw_polyline(cv, seg_pts, color_for(seg_phase),
                                  width, dash, alpha)

    def _draw_polyline(self, cv, pts, color: str, width: int,
                        dash, alpha: float) -> None:
        if alpha < 0.99:
            color = _mix_alpha(color, CELL["empty_bot"], alpha)
        flat = [v for p in pts for v in p]
        kw = {"fill": color, "width": width, "smooth": False,
              "capstyle": "round", "joinstyle": "round"}
        if dash:
            kw["dash"] = dash
        cv.create_line(*flat, **kw)

    def _draw_push_paths(self, cv) -> None:
        """专门绘制每个箱子的推动路径（金色粗箭头）"""
        if not self.solution:
            return
        sol = self.solution
        # 用 expand 已经计算的箱子轨迹：从 box_seq 中重建
        # 思路：扫描每一段 box_idx 不变的连续 'push' 步骤（或 'bomb'）
        if not self._box_seq:
            return

        # 收集每个 sub_id 的推箱起止
        push_seg: dict = {}
        for i, (bi, br, bc) in enumerate(self._box_seq):
            ph = self._phase_seq[i]
            if ph not in ('push', 'bomb'):
                continue
            sub_id = self._sub_id_seq[i]
            after_r = br + DR[self._actions[i]]
            after_c = bc + DC[self._actions[i]]
            seg = push_seg.setdefault(sub_id,
                                        {'phase': ph, 'pts': [(br, bc)],
                                         'done': True})
            seg['pts'].append((after_r, after_c))
            if i >= self.step_idx:
                seg['done'] = False

        for sub_id, seg in push_seg.items():
            phase = seg['phase']
            pts   = seg['pts']
            if len(pts) < 2:
                continue
            color_done = (CELL["path_bomb"] if phase == 'bomb'
                          else CELL["path_push"])
            color_todo = _mix_alpha(color_done, CELL["empty_bot"], 0.5)
            color = color_done if seg['done'] else color_todo

            # 用 cell center 转像素
            xys = [self._cell_center(r, c) for r, c in pts]
            flat = [v for p in xys for v in p]
            cv.create_line(*flat, fill=color, width=4,
                            capstyle="round", joinstyle="round",
                            arrow=tk.LAST,
                            arrowshape=(10, 12, 5),
                            smooth=False)
            # 起点圆点
            sx, sy = xys[0]
            cv.create_oval(sx - 4, sy - 4, sx + 4, sy + 4,
                            fill=color, outline="")


    # ============================================================
    # 动画引擎
    # ============================================================
    def _play(self) -> None:
        if self.solution is None:
            self._log("请先生成地图", "warn")
            return
        if self.anim_run:
            return
        if self.step_idx >= len(self._actions):
            self._reset()
        self.anim_run = True
        self._set_status("播放中", THEME["green"])
        self._anim_thread = threading.Thread(target=self._anim_loop,
                                              daemon=True)
        self._anim_thread.start()

    def _pause(self) -> None:
        self.anim_run = False
        self._set_status("暂停", THEME["orange"])

    def _reset(self) -> None:
        self.anim_run      = False
        self._lerp_active  = False
        self.step_idx      = 0
        self.push_count    = 0
        self.player        = self._i_player
        self.boxes         = list(self._i_boxes)
        self._anim_map     = None
        self._explosion_cells = []
        self.player_facing = 0
        self._cur_sub_id   = -1
        self._revealed_visits = []
        self._update_progress(0)
        self._set_badge(self.badge_steps, "0")
        self._set_badge(self.badge_pushes, "0")
        self._set_badge(self.badge_phase, "—")
        self._set_status("就绪", THEME["green"])
        self._update_sub_progress(-1, '')
        self._redraw()

    def _step_fwd(self) -> None:
        if self.solution is None or self.step_idx >= len(self._actions):
            return
        self._exec_step(self.step_idx, animate_subframes=False)
        self.step_idx += 1
        if self.step_idx >= len(self._actions):
            self._anim_done()

    def _step_back(self) -> None:
        if self.solution is None or self.step_idx <= 0:
            return
        target_idx = self.step_idx - 1
        # 全量重放更鲁棒
        self.player        = self._i_player
        self.boxes         = list(self._i_boxes)
        self.push_count    = 0
        self._anim_map     = None
        self._explosion_cells = []
        self.player_facing = 0
        self._cur_sub_id   = -1
        self._revealed_visits = []
        for i in range(target_idx):
            self._exec_step(i, animate_subframes=False)
        self.step_idx = target_idx
        self._update_progress(self.step_idx)
        self._redraw()

    def _anim_loop(self) -> None:
        # 中间阶段暂停集合（最后一段不暂停）
        pause_set = (set(self._subtask_ends[:-1])
                      if len(self._subtask_ends) > 1 else set())
        sub_n = [0]

        while self.anim_run and self.step_idx < len(self._actions):
            idx = self.step_idx
            speed = self.speed_var.get()
            # 子帧插值
            self._exec_step(idx, animate_subframes=self.smooth_var.get(),
                              speed=speed)
            self.step_idx += 1

            # 分阶段预览暂停
            if self.preview_var.get() and idx in pause_set:
                sub_n[0] += 1
                self.after(0, self._set_status,
                            f"第 {sub_n[0]}/{len(self._subtask_ends)} 段完成，1.5s 后继续",
                            THEME["accent_2"])
                time.sleep(1.5)
                if self.anim_run:
                    self.after(0, self._set_status, "播放中", THEME["green"])

        if self.step_idx >= len(self._actions):
            self.anim_run = False
            self.after(0, self._anim_done)

    def _exec_step(self, idx: int,
                    animate_subframes: bool = True,
                    speed: float = 0.18) -> None:
        """执行第 idx 步动作（含子帧插值）。"""
        if idx >= len(self._actions):
            return

        act   = self._actions[idx]
        bi, br, bc = self._box_seq[idx]
        phase = self._phase_seq[idx]
        sub_id = self._sub_id_seq[idx]
        evt = (self._step_events[idx]
               if idx < len(self._step_events) else None)

        # 当前子任务变化通知 UI
        self.after(0, self._update_sub_progress, sub_id, phase)

        # ── 停顿步 (act=-1): 用于 scout_observe / scout_infer ──
        if act == -1:
            # 揭示该 visit (只解锁徽章; 玩家不动)
            if evt and evt.get('reveal'):
                v = evt['reveal']
                if v not in self._revealed_visits:
                    self._revealed_visits.append(v)
                kind_zh = '箱子' if v['kind'] == 'box' else '目标'
                if v.get('inferred'):
                    msg = f"💡 排除法推断: {kind_zh}#{v['item_idx']+1} 是 {v['class_id']} 号"
                    tag = 'info'
                else:
                    msg = f"👁  观察 {kind_zh}#{v['item_idx']+1} → {v['class_id']} 号"
                    tag = 'ok'
                self.after(0, self._log, msg, tag)
            # 用一个稍长的停顿凸显观察/推断时刻
            pause = max(0.25, speed * 3.0)
            # 在停顿期间持续刷新, 让 class id 闪一下
            n = max(3, int(pause / 0.04))
            for _ in range(n):
                self.after(0, self._redraw)
                time.sleep(pause / n)
            self.after(0, self._post_step_refresh, idx + 1)
            return

        pr, pc = self.player
        npr, npc = pr + DR[act], pc + DC[act]
        self.player_facing = act

        # 推动的箱子在子帧中需要同步插值
        push_box_idx: Optional[int] = None
        push_box_from: Optional[Tuple[int, int]] = None
        push_box_to:   Optional[Tuple[int, int]] = None

        # 提前计算地图变更（炸弹爆炸要在最后子帧再画）
        explosion_at: Optional[Tuple[int, int]] = None

        if phase == 'push':
            if 0 <= bi < len(self.boxes):
                push_box_idx  = bi
                push_box_from = (br, bc)
                push_box_to   = (br + DR[act], bc + DC[act])
        elif phase == 'bomb':
            if self._anim_map is None:
                self._anim_map = [row[:] for row in self.cur_map]
            if npr == br and npc == bc:
                # 玩家推炸弹：把炸弹原位清空
                self._anim_map[br][bc] = EMPTY
                push_box_idx  = -1  # 用特殊值表示炸弹（绘制函数中区分）
                push_box_from = (br, bc)
                push_box_to   = (br + DR[act], bc + DC[act])
                # 是否撞墙触发爆炸
                if self._wall_pos and push_box_to == self._wall_pos:
                    explosion_at = push_box_to

        # ── 子帧动画 ──
        if animate_subframes and phase not in ('scout',):
            from_x, from_y = self._cell_center(pr, pc)
            to_x, to_y     = self._cell_center(npr, npc)
            box_from_xy = (None if push_box_from is None
                           else self._cell_center(*push_box_from))
            box_to_xy   = (None if push_box_to is None
                           else self._cell_center(*push_box_to))

            self._lerp_active = True
            for f in range(1, SUB_FRAMES + 1):
                t = _ease_inout(f / SUB_FRAMES)
                px = from_x + (to_x - from_x) * t
                py = from_y + (to_y - from_y) * t
                if push_box_idx is not None and box_from_xy and box_to_xy:
                    bx = box_from_xy[0] + (box_to_xy[0] - box_from_xy[0]) * t
                    by = box_from_xy[1] + (box_to_xy[1] - box_from_xy[1]) * t
                    if push_box_idx == -1:  # 炸弹
                        self.after(0, self._render_with_bomb_lerp,
                                    px, py, bx, by)
                    else:
                        self.after(0, self._render_with_push_lerp,
                                    px, py, push_box_idx, bx, by)
                else:
                    self.after(0, self._render_with_player_lerp, px, py)
                time.sleep(SUB_FRAME_MS / 1000)
            self._lerp_active = False
        elif animate_subframes and phase == 'scout':
            # 侦查段也走插值, 但稍快
            from_x, from_y = self._cell_center(pr, pc)
            to_x, to_y     = self._cell_center(npr, npc)
            self._lerp_active = True
            for f in range(1, SUB_FRAMES + 1):
                t = _ease_inout(f / SUB_FRAMES)
                px = from_x + (to_x - from_x) * t
                py = from_y + (to_y - from_y) * t
                self.after(0, self._render_with_player_lerp, px, py)
                time.sleep(SUB_FRAME_MS / 1000)
            self._lerp_active = False
        else:
            # 跳过插值（步进或关闭）
            time.sleep(speed * 0.4)

        # ── 提交本步状态 ──
        if phase == 'push':
            if push_box_idx is not None and 0 <= push_box_idx < len(self.boxes):
                self.boxes[push_box_idx] = push_box_to
                self.push_count += 1
        elif phase == 'bomb':
            if push_box_idx == -1 and self._anim_map is not None:
                if explosion_at is not None:
                    er, ec = explosion_at
                    cells: List[Tuple[int, int]] = []
                    for dr2 in range(-1, 2):
                        for dc2 in range(-1, 2):
                            rr, cc = er + dr2, ec + dc2
                            if (1 <= rr < MAP_ROWS - 1
                                    and 1 <= cc < MAP_COLS - 1):
                                if self._anim_map[rr][cc] == WALL:
                                    self._anim_map[rr][cc] = EMPTY
                                cells.append((rr, cc))
                    self._anim_map[er][ec] = EMPTY
                    self.after(0, self._log,
                                f"💥 爆炸于 ({ec},{er})，3×3 范围清墙", "warn")
                    self._explosion_cells = [(r, c, 0) for r, c in cells]
                    self._play_explosion()
                else:
                    self._anim_map[push_box_to[0]][push_box_to[1]] = BOMB

        self.player = (npr, npc)

        # 主动刷新一次
        self.after(0, self._post_step_refresh, idx + 1)

        # 等待剩余 step 时间
        if animate_subframes and phase not in ('scout',):
            remaining = speed - SUB_FRAMES * SUB_FRAME_MS / 1000
            if remaining > 0:
                time.sleep(remaining)
        elif phase == 'scout':
            # scout 行走稍快
            remaining = speed * 0.5 - SUB_FRAMES * SUB_FRAME_MS / 1000
            if remaining > 0:
                time.sleep(remaining)

    def _render_with_player_lerp(self, px: float, py: float) -> None:
        self._redraw(player_xy_override=(px, py))

    def _render_with_push_lerp(self, px: float, py: float,
                                  bi: int, bx: float, by: float) -> None:
        self._redraw(player_xy_override=(px, py),
                     push_box_override=(bi, bx, by))

    def _render_with_bomb_lerp(self, px: float, py: float,
                                  bx: float, by: float) -> None:
        # 炸弹用 box_idx=-1，绘制函数把它当成普通箱子位置画一个圆
        # 简化处理：先画玩家插值 + 临时把 _anim_map 中炸弹位置改回 BOMB？
        # 我们先在 _anim_map 中把炸弹 from 已清，所以这里手工绘制一个炸弹
        self._redraw(player_xy_override=(px, py))
        # 在画布上叠一个浮动炸弹
        cv = self.canvas
        rad = CELL_PX * 0.32
        cv.create_oval(bx - rad, by - rad, bx + rad, by + rad,
                        fill=CELL["bomb_dark"], outline="")
        cv.create_oval(bx - rad + 3, by - rad + 3,
                        bx + rad - 3, by + rad - 3,
                        fill=CELL["bomb"], outline="")

    def _post_step_refresh(self, step_idx: int) -> None:
        self._set_badge(self.badge_steps, str(step_idx))
        self._set_badge(self.badge_pushes, str(self.push_count))
        # 更新 phase 徽章
        if step_idx > 0 and step_idx <= len(self._phase_seq):
            label = {
                'scout':         '🚶 侦查行走',
                'scout_observe': '👁  在观察点采样',
                'scout_infer':   '💡 排除法推断',
                'walk':          '🚶 行走',
                'push':          '📦 推箱',
                'bomb':          '💣 推炸弹',
            }.get(self._phase_seq[step_idx - 1], '—')
            self._set_badge(self.badge_phase, label)
        self._update_progress(step_idx)
        self._redraw()
        # 通关检查
        live_map = self._anim_map if self._anim_map is not None else self.cur_map
        if check_win(live_map, self.boxes) and step_idx >= len(self._actions):
            self.anim_run = False
            self._set_status("通关 ★", THEME["green"])
            self._log("★★★ 通关：所有箱子均到达目标 ★★★", "ok")

    def _play_explosion(self) -> None:
        """爆炸 8 帧闪光"""
        def step(frame: int) -> None:
            if frame >= EXPLOSION_FRAMES:
                self._explosion_cells = []
                self._redraw()
                return
            self._explosion_cells = [(r, c, frame)
                                       for (r, c, _) in self._explosion_cells]
            self._redraw(explosion_overlay=True)
            self.after(EXPLOSION_FRAME_MS, lambda: step(frame + 1))
        self.after(0, step, 0)

    def _anim_done(self) -> None:
        self.anim_run = False
        live_map = self._anim_map if self._anim_map is not None else self.cur_map
        won = check_win(live_map, self.boxes)
        if won:
            self._set_status("通关 ★", THEME["green"])
            self._log("★★★ 通关验证成功 ★★★", "ok")
        else:
            on_t = sum(1 for r, c in self.boxes if live_map[r][c] == TARGET)
            self._set_status(f"{on_t}/{len(self.boxes)} 到位", THEME["orange"])
        self._update_sub_progress(-2, '')

    # ============================================================
    # Tooltip / 点击交互
    # ============================================================
    def _on_canvas_hover(self, event) -> None:
        c = event.x // CELL_PX
        r = event.y // CELL_PX
        if not (0 <= r < MAP_ROWS and 0 <= c < MAP_COLS):
            self._hide_tooltip()
            return
        m = self._anim_map if self._anim_map is not None else self.cur_map
        if m is None:
            return
        cell = m[r][c]
        info = f"({c},{r})"
        if (r, c) == self.player:
            info += "  · 玩家"
        elif (r, c) in self.boxes:
            bi = self.boxes.index((r, c))
            info += f"  · 箱子 #{bi + 1}"
        elif cell == WALL:
            info += "  · 墙"
        elif cell == TARGET:
            info += "  · 目标"
        elif cell == BOMB:
            info += "  · 炸弹"
        self._show_tooltip(event.x + 14, event.y + 14, info)

    def _show_tooltip(self, x: int, y: int, text: str) -> None:
        cv = self.canvas
        if self._tooltip_id is not None:
            cv.delete(self._tooltip_id)
        # 阴影背景
        pad = 4
        tx = cv.create_text(x + pad + 2, y + pad + 1, anchor="nw",
                              text=text, font=F_UI_S,
                              fill=THEME["text"])
        bx = cv.bbox(tx)
        cv.delete(tx)
        if not bx:
            return
        bg = cv.create_rectangle(bx[0] - 4, bx[1] - 2, bx[2] + 4, bx[3] + 2,
                                  fill=THEME["panel"],
                                  outline=THEME["frame"])
        tx = cv.create_text(x + pad + 2, y + pad + 1, anchor="nw",
                              text=text, font=F_UI_S,
                              fill=THEME["text"])
        cv.tag_raise(tx)
        self._tooltip_id = bg
        self._tooltip_text_id = tx

    def _hide_tooltip(self) -> None:
        cv = self.canvas
        if self._tooltip_id is not None:
            cv.delete(self._tooltip_id)
            self._tooltip_id = None
        if hasattr(self, '_tooltip_text_id') and self._tooltip_text_id:
            cv.delete(self._tooltip_text_id)
            self._tooltip_text_id = None

    def _on_canvas_click(self, event) -> None:
        # 预留：未来可点击格子放置元素
        pass

    # ============================================================
    # 工具
    # ============================================================
    def _set_status(self, text: str, color: str) -> None:
        self._set_badge(self.badge_status, text, color)

    def _set_badge(self, badge: tk.Label, text: str,
                    color: Optional[str] = None) -> None:
        badge.config(text=text)
        if color is not None:
            badge.config(fg=color)

    def _update_progress(self, cur: int) -> None:
        total = len(self._actions)
        self.progress_v.set(int(cur / total * 100) if total else 0)
        self.lbl_prog.config(text=f"{cur} / {total}")

    def _update_summary(self, text: str) -> None:
        self.summary_label.config(text=text, fg=THEME["text"])

    def _log(self, msg: str, tag: str = "info") -> None:
        ts = time.strftime("%H:%M:%S")
        self.txt_log.config(state="normal")
        self.txt_log.insert("end", f"[{ts}] ", "dim")
        self.txt_log.insert("end", msg + "\n", tag)
        self.txt_log.see("end")
        self.txt_log.config(state="disabled")



# ════════════════════════════════════════════════════════════════════
# UI 辅助构造器
# ════════════════════════════════════════════════════════════════════
def _sep(parent, pady=(4, 4)) -> None:
    tk.Frame(parent, bg=THEME["border"], height=1).pack(
        fill="x", padx=18, pady=pady)


def _section(parent, text: str) -> tk.Frame:
    f = tk.Frame(parent, bg=THEME["bg"])
    f.pack(fill="x", pady=(10, 4))
    tk.Label(f, text=text, font=F_H2, bg=THEME["bg"],
             fg=THEME["accent"]).pack(side="left")
    tk.Frame(f, bg=THEME["border"], height=1).pack(side="left", fill="x",
                                                     expand=True, padx=8,
                                                     pady=8)
    return f


def _label(parent, text: str) -> tk.Label:
    return tk.Label(parent, text=text, font=F_UI,
                    bg=THEME["bg"], fg=THEME["text"])


def _badge(parent, key: str, val: str, color: str) -> tk.Label:
    """组合标签（key + 大写 value）"""
    f = tk.Frame(parent, bg=THEME["panel"], padx=10, pady=4)
    title = tk.Label(f, text=key, font=F_UI_S, bg=THEME["panel"],
                      fg=THEME["text_dim"])
    title.pack()
    val_lbl = tk.Label(f, text=val, font=F_BADGE, bg=THEME["panel"],
                        fg=color)
    val_lbl.pack()
    val_lbl._frame = f  # type: ignore
    return val_lbl


def _primary_btn(parent, text: str, cmd, color: Optional[str] = None) -> tk.Button:
    color = color or THEME["accent"]
    return _make_button(parent, text, cmd, color, primary=True)


def _ghost_btn(parent, text: str, cmd,
                color: Optional[str] = None) -> tk.Button:
    color = color or THEME["text"]
    return _make_button(parent, text, cmd, color, primary=False)


def _make_button(parent, text: str, cmd, color: str,
                  primary: bool) -> tk.Button:
    if primary:
        bg, fg = color, THEME["bg"]
        hover_bg = _lerp_color(color, "#ffffff", 0.18)
    else:
        bg, fg = THEME["btn"], color
        hover_bg = THEME["btn_h"]

    b = tk.Button(parent, text=text, command=cmd, font=F_UI,
                   bg=bg, fg=fg, relief="flat",
                   activebackground=hover_bg,
                   activeforeground=fg,
                   cursor="hand2", bd=0, padx=10, pady=7,
                   highlightthickness=0)
    b.bind("<Enter>", lambda _e: b.config(bg=hover_bg))
    b.bind("<Leave>", lambda _e: b.config(bg=bg))
    return b


def _readonly_text(parent, height: int = 4,
                    fg: Optional[str] = None) -> tk.Text:
    t = tk.Text(parent, height=height, wrap="word", font=F_MONO,
                 bg=THEME["panel_alt"], fg=fg or THEME["text"],
                 insertbackground=THEME["text"], relief="flat",
                 highlightthickness=1,
                 highlightbackground=THEME["border"],
                 state="disabled")
    t.pack(fill="x", pady=2)
    return t


def _set_text(widget: tk.Text, text: str) -> None:
    widget.config(state="normal")
    widget.delete("1.0", "end")
    widget.insert("end", text)
    widget.config(state="disabled")


def _check(parent, text: str, var: tk.BooleanVar,
            cmd=None) -> tk.Checkbutton:
    cb = tk.Checkbutton(parent, text=text, variable=var,
                         font=F_UI, bg=THEME["bg"], fg=THEME["text"],
                         selectcolor=THEME["panel_alt"],
                         activebackground=THEME["bg"],
                         activeforeground=THEME["text"],
                         command=cmd, anchor="w")
    return cb


# ════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    app = SokobanProApp()
    app.mainloop()

# ======================================================================
# OpenART RGB/LAB 自动标定脚本 (配套地图: map_calibrate.txt)
#
# 用途：
#   1. 在固定摄像头、固定屏幕亮度后，自动采样各地图元素 RGB。
#   2. 分两阶段采样：第一阶段采静态元素+暗色车辆，第二阶段采亮色车辆。
#   3. 将结果打印到串口终端，并保存到 SD 卡（文件名含当前时间戳）。
#   4. 输出格式可直接复制粘贴到 main copy.py 中。
#
# 配套标定地图 (map_calibrate.txt)：
#   ################
#   #$.*-----------#
#   #--------------#
#   #-------.------#
#   #------$-$-----#
#   #------*-------#
#   #--------------#
#   #------.-*-----#
#   #------$-------#
#   #--------------#
#   #-----------.$*#
#   ################
#
# 配套虚拟车脚本 (fake_car_for_calibration.py)：
#   在 PC 上运行，模拟 camera_opencv.exe 的 TCP 服务器，
#   自动把虚拟车放到暗色位置(1.5, 6)和亮色位置(8, 6)。
#   默认每个位置停留 15 秒，与本脚本时序匹配。
#
# 完整使用流程：
#   1. PC 上运行 close_ports.bat
#   2. PC 上运行 python fake_car_for_calibration.py
#   3. 启动游戏，加载 map_calibrate.txt
#   4. 用键盘方向键把车移出发车区（触发地图生成）
#   5. 把本脚本复制到 OpenART SD 卡根目录，改名 main.py
#   6. 启动 OpenART，等待自动完成两阶段采样
#   7. 从终端或 SD 卡读取结果，粘贴到 main copy.py
# ======================================================================

import sensor, image, time, math

# ----------------------------------------------------------------------
# 1. 摄像头初始化：保持和正式识别脚本一致
# ----------------------------------------------------------------------
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)
sensor.set_framerate(60)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
clock = time.clock()

# ----------------------------------------------------------------------
# 2. 网格参数：请从正式 main.py / main copy.py 复制最终标定值
# ----------------------------------------------------------------------
ROWS, COLS = 12, 16

GRID_CORNERS = {
    "tl": (37.0, 42.0),
    "tr": (282.0, 35.5),
    "bl": (47.5, 223.2),
    "br": (280.8, 221.5),
}

GRID_K1 = +0.000000

# ----------------------------------------------------------------------
# 3. 采样配置 —— 分两阶段
# ----------------------------------------------------------------------
# 第一阶段：静态元素 + 暗色车辆（车在左边缘 (1,6)）
# 第二阶段：亮色车辆（车在中心 (8,6)）
#
# 格式：(变量名, 格子x, 格子y, 像素偏移x, 像素偏移y)

# 第一阶段采样点（车在暗色位置 + 所有静态元素）
PHASE1_SPECS = (
    # 暗色车辆：车在左侧 (2,6)
    ("CAR_HEAD_DARK_RGB",    2,  6,  0,  -4),   # 车头绿色半块（上半）
    ("CAR_TAIL_DARK_RGB",    2,  6,  0,   4),   # 车尾青色半块（下半）

    # 墙体
    ("WALL_DARK_RGB",        0,  0,  0,  0),    # 左上角墙
    ("WALL_BRIGHT_RGB",      7,  0,  0,  0),    # 顶部中间墙

    # 空地
    ("FLOOR_DARK_RGB",       1,  2,  0,  0),    # 左侧边缘空地
    ("FLOOR_BRIGHT_RGB",     7,  6,  0,  0),    # 正中心空地

    # 目的地
    ("GOAL_DARK_RGB",        2,  1,  0,  0),    # 左上角附近目的地
    ("GOAL_BRIGHT_RGB",      8,  3,  0,  0),    # 中心区域目的地

    # 箱子
    ("BOX_DARK_RGB",         1,  1,  0,  0),    # 左上角附近箱子
    ("BOX_BRIGHT_RGB",       7,  4,  0,  0),    # 中心区域箱子

    # 炸弹
    ("BOMB_DARK_RGB",        3,  1,  0,  0),    # 左上角附近炸弹
    ("BOMB_BRIGHT_RGB",      7,  5,  0,  0),    # 中心区域炸弹
)

# 第二阶段采样点（车在亮色位置）
PHASE2_SPECS = (
    # 亮色车辆：车在中心 (8,6)
    ("CAR_HEAD_BRIGHT_RGB",  8,  6,  0,  -4),   # 车头绿色半块（上半）
    ("CAR_TAIL_BRIGHT_RGB",  8,  6,  0,   4),   # 车尾青色半块（下半）
)

# 时序参数
# 与 fake_car_for_calibration.py 的 HOLD_SECONDS=15 配合：
#   0s: 两个脚本同时启动，车在暗色位置
#   ~4s: Phase1 采样完成（3s等待 + ~1s采样）
#   15s: fake_car 自动切换到亮色位置
#   ~16s: Phase2 开始采样（等待12s后）
PHASE1_DELAY_MS = 3000       # 第一阶段启动前等待（让画面稳定）
PHASE1_FRAMES = 50           # 第一阶段采样帧数
PHASE2_WAIT_MS = 12000       # 第一阶段结束后等待车辆切换到亮色位置
PHASE2_FRAMES = 50           # 第二阶段采样帧数

SAMPLE_RADIUS = 2            # 采样半径 2 表示 5x5 像素
TRIM_COUNT = 3               # 按亮度排序后，去掉最暗/最亮各 N 个点
DRAW_PREVIEW = True

# 输出文件名使用当前时间戳
def make_output_filename():
    t = time.localtime()
    return "/sd/calib_%04d%02d%02d_%02d%02d%02d.txt" % (
        t[0], t[1], t[2], t[3], t[4], t[5])

# ----------------------------------------------------------------------
# 4. 网格投影与鲁棒取色
# ----------------------------------------------------------------------
def calc_grid_point(x_idx, y_idx, img_w, img_h):
    u = x_idx / (COLS - 1)
    v = y_idx / (ROWS - 1)

    tl_x, tl_y = GRID_CORNERS["tl"]
    tr_x, tr_y = GRID_CORNERS["tr"]
    bl_x, bl_y = GRID_CORNERS["bl"]
    br_x, br_y = GRID_CORNERS["br"]

    top_x = tl_x + (tr_x - tl_x) * u
    top_y = tl_y + (tr_y - tl_y) * u
    bot_x = bl_x + (br_x - bl_x) * u
    bot_y = bl_y + (br_y - bl_y) * u

    raw_x = top_x + (bot_x - top_x) * v
    raw_y = top_y + (bot_y - top_y) * v

    center_x, center_y = img_w / 2, img_h / 2
    dx, dy = raw_x - center_x, raw_y - center_y
    scale = 1 + GRID_K1 * (dx * dx + dy * dy)
    return int(center_x + dx * scale), int(center_y + dy * scale)

def robust_rgb_at(img, x, y, img_w, img_h):
    samples = []
    for oy in range(-SAMPLE_RADIUS, SAMPLE_RADIUS + 1):
        for ox in range(-SAMPLE_RADIUS, SAMPLE_RADIUS + 1):
            sx = x + ox
            sy = y + oy
            if 0 <= sx < img_w and 0 <= sy < img_h:
                rgb = img.get_pixel(sx, sy)
                lum = rgb[0] * 3 + rgb[1] * 6 + rgb[2]
                samples.append((lum, rgb[0], rgb[1], rgb[2]))

    if not samples:
        return (0, 0, 0)

    samples.sort()
    start = 0
    end = len(samples)
    if len(samples) > TRIM_COUNT * 2 + 4:
        start = TRIM_COUNT
        end = len(samples) - TRIM_COUNT

    r_sum = 0
    g_sum = 0
    b_sum = 0
    count = 0
    for sample in samples[start:end]:
        r_sum += sample[1]
        g_sum += sample[2]
        b_sum += sample[3]
        count += 1

    if count <= 0:
        return (0, 0, 0)
    return (r_sum // count, g_sum // count, b_sum // count)

def draw_sample_points(img, img_w, img_h, specs):
    for spec in specs:
        name, grid_x, grid_y, pix_dx, pix_dy = spec
        cx, cy = calc_grid_point(grid_x, grid_y, img_w, img_h)
        sx = cx + pix_dx
        sy = cy + pix_dy
        if 0 <= sx < img_w and 0 <= sy < img_h:
            img.draw_cross(sx, sy, color=(255, 255, 255), size=5, thickness=1)
            img.draw_circle(sx, sy, SAMPLE_RADIUS + 2, color=(255, 220, 0), thickness=1)
            img.draw_string(sx + 4, sy - 6, name[:4], color=(255, 220, 0), mono_space=False)

# ----------------------------------------------------------------------
# 5. RGB -> LAB 转换（与正式脚本一致）
# ----------------------------------------------------------------------
def _pivot_rgb_to_linear(c):
    if c <= 0.04045:
        return c / 12.92
    return ((c + 0.055) / 1.055) ** 2.4

def rgb_to_lab(rgb):
    r = _pivot_rgb_to_linear(rgb[0] / 255.0)
    g = _pivot_rgb_to_linear(rgb[1] / 255.0)
    b = _pivot_rgb_to_linear(rgb[2] / 255.0)

    x = r * 0.4124564 + g * 0.3575761 + b * 0.1804375
    y = r * 0.2126729 + g * 0.7151522 + b * 0.0721750
    z = r * 0.0193339 + g * 0.1191920 + b * 0.9503041

    xr = x / 0.95047
    yr = y / 1.00000
    zr = z / 1.08883

    eps = 0.008856
    kappa = 903.3

    def _f(t):
        if t > eps:
            return t ** (1.0 / 3.0)
        return (kappa * t + 16.0) / 116.0

    fx = _f(xr)
    fy = _f(yr)
    fz = _f(zr)

    l = 116.0 * fy - 16.0
    a = 500.0 * (fx - fy)
    bb = 200.0 * (fy - fz)
    return (l, a, bb)

# ----------------------------------------------------------------------
# 6. 采样单阶段
# ----------------------------------------------------------------------
def sample_phase(specs, delay_ms, num_frames, phase_name):
    """对一组采样点执行多帧平均采样，返回 {name: (r,g,b)} 字典"""
    sums = {}
    counts = {}
    for spec in specs:
        name = spec[0]
        sums[name] = [0, 0, 0]
        counts[name] = 0

    start_ms = time.ticks_ms()
    frame_index = 0
    sample_count = 0

    print("\n--- %s ---" % phase_name)
    print("等待 %d ms 后开始采样 %d 帧..." % (delay_ms, num_frames))

    while sample_count < num_frames:
        clock.tick()
        img = sensor.snapshot()
        img_w = img.width()
        img_h = img.height()

        if DRAW_PREVIEW:
            draw_sample_points(img, img_w, img_h, specs)

        elapsed = time.ticks_diff(time.ticks_ms(), start_ms)
        if elapsed < delay_ms:
            if frame_index % 15 == 0:
                remain = (delay_ms - elapsed + 999) // 1000
                print("[%s] 等待稳定... %d s" % (phase_name, remain))
            frame_index += 1
            continue

        for spec in specs:
            name, grid_x, grid_y, pix_dx, pix_dy = spec
            cx, cy = calc_grid_point(grid_x, grid_y, img_w, img_h)
            sx = cx + pix_dx
            sy = cy + pix_dy
            rgb = robust_rgb_at(img, sx, sy, img_w, img_h)
            sums[name][0] += rgb[0]
            sums[name][1] += rgb[1]
            sums[name][2] += rgb[2]
            counts[name] += 1

        sample_count += 1
        if sample_count % 10 == 0 or sample_count == 1:
            print("[%s] 采样 %d/%d | FPS %0.1f" % (phase_name, sample_count, num_frames, clock.fps()))

    # 计算平均值
    results = {}
    for spec in specs:
        name = spec[0]
        count = counts[name]
        if count > 0:
            results[name] = (sums[name][0] // count,
                             sums[name][1] // count,
                             sums[name][2] // count)
    return results

# ----------------------------------------------------------------------
# 7. 输出与保存
# ----------------------------------------------------------------------
def format_rgb_line(name, rgb):
    return "%s = (%d, %d, %d)" % (name, rgb[0], rgb[1], rgb[2])

def format_lab_line(name, lab):
    return "# LAB: %s = (%.1f, %.1f, %.1f)" % (name, lab[0], lab[1], lab[2])

# 正式脚本中的变量顺序
PASTE_ORDER = [
    "CAR_HEAD_DARK_RGB",
    "CAR_HEAD_BRIGHT_RGB",
    "CAR_TAIL_DARK_RGB",
    "CAR_TAIL_BRIGHT_RGB",
    "WALL_DARK_RGB",
    "WALL_BRIGHT_RGB",
    "FLOOR_DARK_RGB",
    "FLOOR_BRIGHT_RGB",
    "GOAL_DARK_RGB",
    "GOAL_BRIGHT_RGB",
    "BOX_DARK_RGB",
    "BOX_BRIGHT_RGB",
    "BOMB_DARK_RGB",
    "BOMB_BRIGHT_RGB",
]

def save_results(results):
    output_file = make_output_filename()
    try:
        f = open(output_file, "w")
        f.write("# ============================================\n")
        f.write("# RGB/LAB Calibration Result\n")
        f.write("# Generated by rgb_auto_calibrate.py\n")
        f.write("# Map: map_calibrate.txt\n")
        f.write("# ============================================\n")
        f.write("# Copy the block below directly into main copy.py\n\n")

        # 详细输出（含 LAB 参考）
        for name in PASTE_ORDER:
            if name in results:
                rgb = results[name]
                lab = rgb_to_lab(rgb)
                f.write(format_rgb_line(name, rgb) + "\n")
                f.write(format_lab_line(name, lab) + "\n")

        f.write("\n# ============================================\n")
        f.write("# Quick paste block (copy below to main copy.py):\n")
        f.write("# ============================================\n\n")

        for name in PASTE_ORDER:
            if name in results:
                f.write(format_rgb_line(name, results[name]) + "\n")

        f.close()
        print("\nSaved to %s" % output_file)
    except Exception as e:
        print("Save failed:", e)

# ----------------------------------------------------------------------
# 8. 主流程：两阶段自动采样
# ----------------------------------------------------------------------
def run_calibration():
    print("=" * 40)
    print(" RGB/LAB 两阶段自动标定")
    print(" 配套: map_calibrate.txt")
    print("       fake_car_for_calibration.py")
    print("=" * 40)

    # 第一阶段：采静态元素 + 暗色车辆
    phase1_results = sample_phase(
        PHASE1_SPECS,
        PHASE1_DELAY_MS,
        PHASE1_FRAMES,
        "Phase1: 静态+暗色车"
    )

    # 等待车辆移动到亮色位置
    print("\n[等待] 车辆正在移动到亮色位置...")
    print("[等待] 请确保 fake_car_for_calibration.py 已自动切换")
    wait_start = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), wait_start) < PHASE2_WAIT_MS:
        clock.tick()
        img = sensor.snapshot()
        if DRAW_PREVIEW:
            draw_sample_points(img, img.width(), img.height(), PHASE2_SPECS)
        elapsed = time.ticks_diff(time.ticks_ms(), wait_start)
        if elapsed % 2000 < 50:
            remain = (PHASE2_WAIT_MS - elapsed + 999) // 1000
            print("[等待] 剩余 %d s" % remain)

    # 第二阶段：采亮色车辆
    phase2_results = sample_phase(
        PHASE2_SPECS,
        1000,            # 短暂等待 1s 让画面稳定
        PHASE2_FRAMES,
        "Phase2: 亮色车"
    )

    # 合并结果
    all_results = {}
    all_results.update(phase1_results)
    all_results.update(phase2_results)

    # 打印结果
    print("\n" + "=" * 40)
    print(" RGB/LAB Calibration Result")
    print(" Map: map_calibrate.txt")
    print("=" * 40 + "\n")

    for name in PASTE_ORDER:
        if name in all_results:
            rgb = all_results[name]
            lab = rgb_to_lab(rgb)
            print(format_rgb_line(name, rgb))
            print(format_lab_line(name, lab))

    print("\n# ---- Quick paste block ----")
    for name in PASTE_ORDER:
        if name in all_results:
            print(format_rgb_line(name, all_results[name]))
    print("# ---------------------------\n")

    # 保存到文件
    save_results(all_results)

run_calibration()

# 保持画面预览，避免脚本结束后立即黑屏
print("\n[完成] 标定结束，保持预览中...")
all_specs = PHASE1_SPECS + PHASE2_SPECS
while True:
    clock.tick()
    img = sensor.snapshot()
    if DRAW_PREVIEW:
        draw_sample_points(img, img.width(), img.height(), all_specs)
    if int(clock.fps()) > 0:
        pass

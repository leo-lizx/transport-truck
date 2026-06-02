# ======================================================================
# OpenART RGB/LAB 自动标定脚本 (配套当前 OpenMV 预览界面)
#
# 用途：
#   1. 在固定摄像头、固定屏幕亮度后，自动采样各地图元素 RGB。
#   2. 按本文件写死的标定图坐标，一次性采样 14 个 RGB 模板。
#   3. 将结果打印到串口终端，并保存到 SD 卡（文件名含当前时间戳）。
#   4. 输出格式可直接复制粘贴到 main copy.py 中。
#
# 完整使用流程：
#   1. 在电脑上打开 map_calibrate 标定界面，并保持窗口/亮度/摄像头位置不变。
#   2. 把本脚本复制到 OpenART SD 卡根目录，改名 main.py。
#   3. 在 OpenMV IDE 中运行一次，先看彩色外框和预览十字，扶正后等待自动采样。
#   4. 从串口或 /sd/rgb_paste_to_main.py 读取结果，粘贴到 main copy.py/main.py。
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
"tl": (30.0, 37.0), #左上
"tr": (282.0, 31.5), #右上
"bl": (39.5, 213.2), #左下
"br": (284.8, 210.5), #右下
}

GRID_K1 = +0.000000

# ----------------------------------------------------------------------
# 3. 采样配置 —— 按当前 OpenMV 预览界面的 map_calibrate 布局采样
# ----------------------------------------------------------------------
# 格式：(变量名, 格子x, 格子y, 像素偏移x, 像素偏移y)
# 坐标按 16x12 地图格填写：x 从左到右 0~15，y 从上到下 0~11。
# 车辆色块在图中是上下半块：dy=-4 采车头，dy=+4 采车尾。
CALIBRATION_SPECS = (
    # 车辆：左侧样本作暗模板，中间样本作亮模板
    ("CAR_HEAD_DARK_RGB",    1,  6,  0, -4),    # 左侧车辆上半块
    ("CAR_TAIL_DARK_RGB",    1,  6,  0,  4),    # 左侧车辆下半块
    ("CAR_HEAD_BRIGHT_RGB",  8,  6,  0, -4),    # 中间车辆上半块
    ("CAR_TAIL_BRIGHT_RGB",  8,  6,  0,  4),    # 中间车辆下半块

    # 墙体
    ("WALL_DARK_RGB",        0,  0,  0,  0),    # 左上角墙
    ("WALL_BRIGHT_RGB",      7,  0,  0,  0),    # 顶部中间墙

    # 空地
    ("FLOOR_DARK_RGB",       1,  2,  0,  0),    # 左上侧蓝色空地
    ("FLOOR_BRIGHT_RGB",     7,  6,  0,  0),    # 中部蓝色空地

    # 目的地
    ("GOAL_DARK_RGB",        2,  1,  0,  0),    # 左上粉色目的地
    ("GOAL_BRIGHT_RGB",      8,  3,  0,  0),    # 中部粉色目的地

    # 箱子
    ("BOX_DARK_RGB",         1,  1,  0,  0),    # 左上黄色箱子
    ("BOX_BRIGHT_RGB",       7,  4,  0,  0),    # 中部黄色箱子

    # 炸弹
    ("BOMB_DARK_RGB",        3,  1,  0,  0),    # 左上红色炸弹
    ("BOMB_BRIGHT_RGB",      7,  5,  0,  0),    # 中部红色炸弹
)

# 时序参数
SAMPLE_DELAY_MS = 15000      # 启动前等待：先用彩色边框矫正画面，再自动采样
SAMPLE_FRAMES = 60           # 采样帧数

SAMPLE_RADIUS = 2            # 采样半径 2 表示 5x5 像素
TRIM_COUNT = 3               # 按亮度排序后，去掉最暗/最亮各 N 个点
DRAW_PREVIEW = True
FIXED_OUTPUT_FILE = "/sd/rgb_paste_to_main.py"

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

def draw_calibration_overlay(img, img_w, img_h):
    tl = calc_grid_point(0, 0, img_w, img_h)
    tr = calc_grid_point(COLS - 1, 0, img_w, img_h)
    bl = calc_grid_point(0, ROWS - 1, img_w, img_h)
    br = calc_grid_point(COLS - 1, ROWS - 1, img_w, img_h)

    img.draw_line(tl[0], tl[1], tr[0], tr[1], color=(255, 80, 80), thickness=1)
    img.draw_line(tr[0], tr[1], br[0], br[1], color=(80, 255, 80), thickness=1)
    img.draw_line(br[0], br[1], bl[0], bl[1], color=(80, 160, 255), thickness=1)
    img.draw_line(bl[0], bl[1], tl[0], tl[1], color=(255, 220, 80), thickness=1)

    img.draw_cross(tl[0], tl[1], color=(255, 80, 80), size=7, thickness=2)
    img.draw_cross(tr[0], tr[1], color=(80, 255, 80), size=7, thickness=2)
    img.draw_cross(bl[0], bl[1], color=(255, 220, 80), size=7, thickness=2)
    img.draw_cross(br[0], br[1], color=(80, 160, 255), size=7, thickness=2)

    return tl, tr, bl, br

def draw_calibration_preview(img, img_w, img_h, specs):
    draw_calibration_overlay(img, img_w, img_h)
    draw_sample_points(img, img_w, img_h, specs)

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
            draw_calibration_preview(img, img_w, img_h, specs)

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

def write_result_file(output_file, results):
    f = open(output_file, "w")
    f.write("# ============================================\n")
    f.write("# RGB/LAB Calibration Result\n")
    f.write("# Generated by rgb_auto_calibrate.py\n")
    f.write("# OpenMV map_calibrate preview\n")
    f.write("# ============================================\n")
    f.write("# Copy the block below directly into main copy.py/main.py\n\n")

    # 详细输出（含 LAB 参考）
    for name in PASTE_ORDER:
        if name in results:
            rgb = results[name]
            lab = rgb_to_lab(rgb)
            f.write(format_rgb_line(name, rgb) + "\n")
            f.write(format_lab_line(name, lab) + "\n")

    f.write("\n# ============================================\n")
    f.write("# Quick paste block:\n")
    f.write("# ============================================\n\n")

    for name in PASTE_ORDER:
        if name in results:
            f.write(format_rgb_line(name, results[name]) + "\n")

    f.close()

def write_paste_file(output_file, results):
    f = open(output_file, "w")
    for name in PASTE_ORDER:
        if name in results:
            f.write(format_rgb_line(name, results[name]) + "\n")
    f.close()

def save_results(results):
    timestamp_file = make_output_filename()
    try:
        write_result_file(timestamp_file, results)
        print("\nSaved to %s" % timestamp_file)
        write_paste_file(FIXED_OUTPUT_FILE, results)
        print("Saved paste block to %s" % FIXED_OUTPUT_FILE)
    except Exception as e:
        print("Save failed:", e)

# ----------------------------------------------------------------------
# 8. 主流程：静态标定图一次采样
# ----------------------------------------------------------------------
def run_calibration():
    print("=" * 40)
    print(" RGB/LAB static calibration")
    print(" Use the fixed sample points in CALIBRATION_SPECS")
    print("=" * 40)

    all_results = sample_phase(
        CALIBRATION_SPECS,
        SAMPLE_DELAY_MS,
        SAMPLE_FRAMES,
        "Static image"
    )

    # 打印结果
    print("\n" + "=" * 40)
    print(" RGB/LAB Calibration Result")
    print(" OpenMV map_calibrate preview")
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
while True:
    clock.tick()
    img = sensor.snapshot()
    if DRAW_PREVIEW:
        draw_calibration_preview(img, img.width(), img.height(), CALIBRATION_SPECS)
    if int(clock.fps()) > 0:
        pass

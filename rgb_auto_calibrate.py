# ======================================================================
# OpenART RGB 自动标定脚本
#
# 用途：
#   1. 在固定摄像头、固定屏幕亮度后，自动采样各地图元素 RGB。
#   2. 连续多帧求平均，降低单帧噪声。
#   3. 将结果打印到串口终端，并保存到 SD 卡 rgb_calibration.txt。
#
# 使用方法：
#   1. 先在正式识别脚本里把四角 GRID_CORNERS 调准，再把同样的值同步到本文件。
#   2. 在 SAMPLE_SPECS 里填写每个颜色样本所在的格子坐标。
#   3. 复制本文件到 OpenART SD 卡根目录，改名为 main.py 后运行。
#   4. 观察预览里的白色十字/黄色圆圈是否压在目标颜色内部。
#   5. 读取终端或 rgb_calibration.txt，把结果填回正式识别脚本。
#
# 调试顺序建议：
#   A. 先只保留 2~4 个样本做试跑，例如车绿、车青、暗墙、暗空地。
#   B. 确认采样点没有落到格线或相邻格后，再把其它元素逐个加回。
#   C. 如果某个采样点落到错误格，优先改格子坐标 x/y；
#      如果落在正确格但没有压中目标颜色中心，再改像素偏移 dx/dy。
#   D. 本脚本测的是 16x12 地图里的“格子坐标”，不是摄像头像素坐标。
# ======================================================================

import sensor, image, time

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
#
# 调试说明：
#   1. 如果四角还没调准，不要直接测 RGB。
#   2. RGB 测色依赖 calc_grid_point 把格子索引映射到画面像素点。
#   3. 四角一旦有误，后面的自动测色会混入格线或邻格颜色。
# ----------------------------------------------------------------------
ROWS, COLS = 12, 16

GRID_CORNERS = {
    "tl": (33.5, 37.5),
    "tr": (274.5, 35.0),
    "bl": (44.0, 209.8),
    "br": (273.8, 196.0),
}

GRID_K1 = +0.000000

# ----------------------------------------------------------------------
# 3. 自动采样配置
# ----------------------------------------------------------------------
# 坐标说明：
#   (变量名, 格子x, 格子y, 像素偏移x, 像素偏移y)
#
# 格子坐标：左上角为 (0,0)，右下角为 (15,11)。
# 像素偏移：以该格中心为原点，向右为 +x，向下为 +y。
#
# 对普通元素，偏移填 0,0 即可。
# 对双色车辆，如果绿/青在同一格内，请把偏移改到对应半块中心，
# 例如绿色在左半块可填 -4,0，青色在右半块可填 +4,0。
# 如果车辆半块方向不同，请按预览里的采样十字位置调整偏移。
#
# 调试说明：
#   1. 先数格子，再填坐标。程序坐标从 0 开始：
#      第 1 列是 x=0，第 1 行是 y=0。
#   2. 如果预览圆圈明显跑到旁边格子，说明 x/y 填错。
#   3. 如果圆圈在正确格里，但压到双色分界线、反光边缘或格线，
#      说明 dx/dy 需要继续调小步长，通常每次改 1~2 像素。
#   4. 车辆绿/青在同一格时，x/y 通常相同，只改最后两个偏移量。
#   5. 不确定某类元素位置时，先临时删掉这一行，避免错误样本污染结果。
SAMPLE_SPECS = (
    # 车辆：不区分车头车尾时，这两个就是车辆绿/青两个颜色原型。
    ("CAR_HEAD_RGB",       1,  6,  0,  -4),
    ("CAR_TAIL_RGB",       1,  6,  0,  4),

    # 地图元素：请把 x/y 改成当前画面里对应元素所在格。
    ("WALL_DARK_RGB",      0,  0,  0,  0),
    ("WALL_BRIGHT_RGB",    7,  0,  0,  0),
    ("FLOOR_DARK_RGB",     1,  1,  0,  0),
    ("FLOOR_BRIGHT_RGB",   7,  1,  0,  0),
    ("GOAL_DARK_RGB",      9,  4,  0,  0),
    ("GOAL_BRIGHT_RGB",    9,  4,  0,  0),
    ("BOX_DARK_RGB",       7,  4,  0,  0),
    ("BOX_BRIGHT_RGB",     7,  4,  0,  0),
    # ("BOMB_DARK_RGB",      0,  0,  0,  0),
    # ("BOMB_BRIGHT_RGB",    0,  0,  0,  0),
)

START_DELAY_MS = 3000        # 启动后等待画面稳定时间
CALIB_FRAMES = 50            # 连续采样帧数；越大越稳，耗时越长
SAMPLE_RADIUS = 2            # 采样半径 2 表示 5x5 像素
TRIM_COUNT = 3               # 按亮度排序后，去掉最暗/最亮各 N 个点
OUTPUT_FILE = "/sd/rgb_calibration.txt"
DRAW_PREVIEW = True

# 调试建议：
#   1. 初次调试时，建议把 CALIB_FRAMES 先降到 10，确认坐标没问题后再升回 50。
#   2. 如果画面抖动或有反光，可把 CALIB_FRAMES 提到 80~100。
#   3. 如果目标颜色块很小，可把 SAMPLE_RADIUS 先降到 1，避免采到边界。
#   4. 如果预览太乱，可暂时减少 SAMPLE_SPECS 条目数量，不建议先关 DRAW_PREVIEW。

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

def draw_sample_points(img, img_w, img_h):
    for spec in SAMPLE_SPECS:
        name, grid_x, grid_y, pix_dx, pix_dy = spec
        cx, cy = calc_grid_point(grid_x, grid_y, img_w, img_h)
        sx = cx + pix_dx
        sy = cy + pix_dy
        if 0 <= sx < img_w and 0 <= sy < img_h:
            img.draw_cross(sx, sy, color=(255, 255, 255), size=5, thickness=1)
            img.draw_circle(sx, sy, SAMPLE_RADIUS + 2, color=(255, 220, 0), thickness=1)
            # 名称只画前 4 个字符，避免屏幕过乱；调试时用它确认当前点对应哪一类样本。
            img.draw_string(sx + 4, sy - 6, name[:4], color=(255, 220, 0), mono_space=False)

# ----------------------------------------------------------------------
# 5. 采样主流程
# ----------------------------------------------------------------------
def format_rgb_line(name, rgb):
    return "%s = (%d, %d, %d)" % (name, rgb[0], rgb[1], rgb[2])

def save_results(results):
    try:
        f = open(OUTPUT_FILE, "w")
        f.write("# RGB calibration result\n")
        f.write("# Copy these lines back to main.py / main copy.py\n\n")
        for spec in SAMPLE_SPECS:
            name = spec[0]
            if name in results:
                f.write(format_rgb_line(name, results[name]) + "\n")
        f.close()
        print("Saved to %s" % OUTPUT_FILE)
    except Exception as e:
        print("Save failed:", e)

def run_calibration():
    sums = {}
    counts = {}
    for spec in SAMPLE_SPECS:
        name = spec[0]
        sums[name] = [0, 0, 0]
        counts[name] = 0

    start_ms = time.ticks_ms()
    frame_index = 0
    sample_count = 0
    done = False

    while not done:
        clock.tick()
        img = sensor.snapshot()
        img_w = img.width()
        img_h = img.height()

        if DRAW_PREVIEW:
            draw_sample_points(img, img_w, img_h)

        # 调试说明：
        #   1. 启动后的等待阶段用来让画面和自动曝光后的残余波动稳定下来。
        #   2. 这段时间只看预览，不采样，适合检查十字是否压在正确位置。
        #   3. 如果此时发现坐标不对，直接停机改 SAMPLE_SPECS，不要继续记录错误结果。

        elapsed = time.ticks_diff(time.ticks_ms(), start_ms)
        if elapsed < START_DELAY_MS:
            if frame_index % 10 == 0:
                remain = (START_DELAY_MS - elapsed + 999) // 1000
                print("Prepare sampling... %d s" % remain)
            frame_index += 1
            continue

        if sample_count < CALIB_FRAMES:
            for spec in SAMPLE_SPECS:
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
                print("Sampling %d/%d | FPS %0.1f" % (sample_count, CALIB_FRAMES, clock.fps()))
        else:
            results = {}
            print("\n===== RGB Calibration Result =====")
            print("# 如果某项数值明显异常，先回头检查该项的格子坐标和偏移，而不是直接照抄。\n")
            for spec in SAMPLE_SPECS:
                name = spec[0]
                count = counts[name]
                if count > 0:
                    rgb = (sums[name][0] // count,
                           sums[name][1] // count,
                           sums[name][2] // count)
                    results[name] = rgb
                    print(format_rgb_line(name, rgb))
            print("==================================\n")
            save_results(results)
            done = True

run_calibration()

# 保持画面预览，避免脚本结束后立即黑屏。
# 调试说明：
#   1. 结果打印并保存后，屏幕仍保留采样点，方便你对照检查最后一次配置。
#   2. 确认无误后再断电或替换回正式 main.py。
while True:
    clock.tick()
    img = sensor.snapshot()
    if DRAW_PREVIEW:
        draw_sample_points(img, img.width(), img.height())
    if int(clock.fps()) > 0:
        pass

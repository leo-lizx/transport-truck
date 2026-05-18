# ======================================================================
# 【项目名称】：智能车视觉导航 - 16x12 赛道解算与车辆实时定位系统
# 【实现目标】：
#   1. 建立 16x12 逻辑网格，通过“打点采样”识别全场地图元素。
#   2. 实时检索车辆标志位(@)，锁定车辆在地图中的行列坐标(X, Y)。
#   3. 通过自定义二进制协议帧，将 194 字节数据（地图+坐标）传给主控。
# 【核心原理】：
#   - 坐标计算：线性斜率补偿（修正由于摄像头安装倾斜导致的近大远小）。
#   - 元素匹配：多点采样后做 RGB 距离匹配，降低亮度波动与单像素噪声影响。
#   - 链路安全：CRC8 循环冗余校验（防止串口通信中产生的噪点导致误码）。
# ======================================================================
#
# 部署：复制本文件到 OpenART SD 卡根目录 main.py（与 README 中 F:\\main.py 对应）。
# ======================================================================

import sensor, image, time, math
from machine import UART

# ----------------------------------------------------------------------
# 1. 硬件资源与感光元件初始化
# ----------------------------------------------------------------------
sensor.reset()                         # 重置感光元件硬件
sensor.set_pixformat(sensor.RGB565)    # 设置图像为 RGB565 彩色格式
sensor.set_framesize(sensor.QVGA)      # 320x240 分辨率，平衡了清晰度与处理速度
sensor.skip_frames(time = 2000)        # 跳过初始不稳定帧，等待自动增益/白平衡稳定
sensor.set_framerate(60)               # 强行设置帧率为 60FPS，提升系统实时响应
clock = time.clock()                   # 计时器，用于监控 FPS
sensor.set_auto_gain(False)            # 【必须添加】关闭自动增益
sensor.set_auto_whitebal(False)        # 【必须添加】关闭自动白平衡
# 初始化专用通信串口 12 (模块上的白色 XH2.54 接口)
# 115200 波特率是目前平衡稳定与速度的首选，TX接单片机RX，RX接单片机TX
uart = UART(12, baudrate=115200)

frame_cnt = 0                          # 系统总运行帧数计数器

# ======================================================================
# 【P0-1】二进制通讯协议定义 (与电控组 STM32 协议严格同步)
# ======================================================================
PROTO_SOF1          = 0xAA             # 帧起始头1 (Sync Word 1)
PROTO_SOF2          = 0x55             # 帧起始头2 (Sync Word 2)
PROTO_TYPE_MAP      = 0x01             # 数据帧类型：代表“地图及定位”
PROTO_TYPE_HB       = 0x10             # 数据帧类型：代表“系统心跳”
HB_INTERVAL_MS      = 100              # 心跳包发送间隔，每100毫秒发一次

# CRC8 循环冗余校验表 (多项式 0x07, 初始值 0x00)
# 作用：单片机收到数据后会查此表重新计算，若计算结果与发送的一致，则认为数据未损坏
_CRC8_TABLE = bytes((
    0x00,0x07,0x0E,0x09,0x1C,0x1B,0x12,0x15,0x38,0x3F,0x36,0x31,0x24,0x23,0x2A,0x2D,
    0x70,0x77,0x7E,0x79,0x6C,0x6B,0x62,0x65,0x48,0x4F,0x46,0x41,0x54,0x53,0x5A,0x5D,
    0xE0,0xE7,0xEE,0xE9,0xFC,0xFB,0xF2,0xF5,0xD8,0xDF,0xD6,0xD1,0xC4,0xC3,0xCA,0xCD,
    0x90,0x97,0x9E,0x99,0x8C,0x8B,0x82,0x85,0xA8,0xAF,0xA6,0xA1,0xB4,0xB3,0xBA,0xBD,
    0xC7,0xC0,0xC9,0xCE,0xDB,0xDC,0xD5,0xD2,0xFF,0xF8,0xF1,0xF6,0xE3,0xE4,0xED,0xEA,
    0xB7,0xB0,0xB9,0xBE,0xAB,0xAC,0xA5,0xA2,0x8F,0x88,0x81,0x86,0x93,0x94,0x9D,0x9A,
    0x27,0x20,0x29,0x2E,0x3B,0x3C,0x35,0x32,0x1F,0x18,0x11,0x16,0x03,0x04,0x0D,0x0A,
    0x57,0x50,0x59,0x5E,0x4B,0x4C,0x45,0x42,0x6F,0x68,0x61,0x66,0x73,0x74,0x7D,0x7A,
    0x89,0x8E,0x87,0x80,0x95,0x92,0x9B,0x9C,0xB1,0xB6,0xBF,0xB8,0xAD,0xAA,0xA3,0xA4,
    0xF9,0xFE,0xF7,0xF0,0xE5,0xE2,0xEB,0xEC,0xC1,0xC6,0xCF,0xC8,0xDD,0xDA,0xD3,0xD4,
    0x69,0x6E,0x67,0x60,0x75,0x72,0x7B,0x7C,0x51,0x56,0x5F,0x58,0x4D,0x4A,0x43,0x44,
    0x19,0x1E,0x17,0x10,0x05,0x02,0x0B,0x0C,0x21,0x26,0x2F,0x28,0x3D,0x3A,0x33,0x34,
    0x4E,0x49,0x40,0x47,0x52,0x55,0x5C,0x5B,0x76,0x71,0x78,0x7F,0x6A,0x6D,0x64,0x63,
    0x3E,0x39,0x30,0x37,0x22,0x25,0x2C,0x2B,0x06,0x01,0x08,0x0F,0x1A,0x1D,0x14,0x13,
    0xAE,0xA9,0xA0,0xA7,0xB2,0xB5,0xBC,0xBB,0x96,0x91,0x98,0x9F,0x8A,0x8D,0x84,0x83,
    0xDE,0xD9,0xD0,0xD7,0xC2,0xC5,0xCC,0xCB,0xE6,0xE1,0xE8,0xEF,0xFA,0xFD,0xF4,0xF3,
))

def crc8(data):
    """计算 bytes 数据的 CRC8 校验值"""
    crc = 0
    tbl = _CRC8_TABLE
    for b in data: crc = tbl[crc ^ b]
    return crc

def pack_frame(type_byte, payload):
    """
    【打包函数】：将数据按照协议封装。
    结构：[0xAA, 0x55, 类型, 长度, 数据负载, 校验]
    """
    if not isinstance(payload, (bytes, bytearray)): payload = bytes(payload)
    length = len(payload)
    # 消息体 = [类型 + 长度 + 载荷]
    body = bytes((type_byte & 0xFF, length & 0xFF)) + payload
    # 完整帧 = [同步头 + 消息体 + CRC校验]
    return bytes((PROTO_SOF1, PROTO_SOF2)) + body + bytes((crc8(body),))

# 心跳逻辑变量
_hb_seq = 0                            # 心跳包序列号，单片机通过此值判断丢包率
_last_hb_ms = time.ticks_ms()          # 记录上次心跳的时间点

def send_heartbeat_if_due():
    """定时心跳发送函数：每 100ms 向主控发送一次心跳帧，证明视觉模块存活"""
    global _hb_seq, _last_hb_ms
    now = time.ticks_ms()
    if time.ticks_diff(now, _last_hb_ms) >= HB_INTERVAL_MS:
        _hb_seq = (_hb_seq + 1) & 0xFF  # 0~255循环自增
        try: uart.write(pack_frame(PROTO_TYPE_HB, bytes((_hb_seq,))))
        except: pass                   # 串口忙时跳过，不阻塞识别主逻辑
        _last_hb_ms = now

# ----------------------------------------------------------------------
# 2. 网格采样与算法参数配置 (四角标定 + 逻辑逆透视采样)
# ----------------------------------------------------------------------
ROWS, COLS = 12, 16                    # 赛道逻辑网格规模：12行x16列

# 四个点均为外圈 4 个格子中心的像素坐标，而不是屏幕物理边框角点。
# 调试时只需把白色采样点调到四个角落格子的中心，内部 16x12 点会自动双线性展开。
GRID_CORNERS = {
    "tl": (37.0, 44.5), #左上
    "tr": (282.5, 34.5), #右上
    "bl": (50.5, 226.8), #左下
    "br": (280.8, 221.5), #右下
}

GRID_K1 = +0.000000                    # 径向畸变系数；无畸变镜头可设为 0
CALIB_SHOW_CORNERS = True              # 四角标定模式：高亮四角与外框，便于精准调参
CAR_VOTE_FRAMES = 5                    # 车辆坐标最近 N 帧多数投票
CAR_VOTE_MIN = 2                       # 至少出现 N 次才认为是稳定坐标

car_vote_hist = []

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

def vote_car_position(found, x, y):
    global car_vote_hist
    if found:
        car_vote_hist.append((x, y))
        if len(car_vote_hist) > CAR_VOTE_FRAMES:
            car_vote_hist.pop(0)

    if not car_vote_hist:
        return x, y

    best_pos, best_count = car_vote_hist[-1], 0
    for pos in car_vote_hist:
        count = 0
        for other in car_vote_hist:
            if pos == other:
                count += 1
        if count > best_count:
            best_pos, best_count = pos, count

    if best_count >= CAR_VOTE_MIN:
        return best_pos
    return car_vote_hist[-1]

# ----------------------------------------------------------------------
# 3. 元素颜色特征库 (RGB 实测标定 -> RGB 空间匹配)
# ----------------------------------------------------------------------
# 只需在现场重新读取这一组 RGB 值。
CAR_HEAD_DARK_RGB = (25, 152, 0)      # 暗车头颜色（H-dark）
CAR_HEAD_BRIGHT_RGB = (74, 225, 239)    # 亮车头颜色（H-bright）
CAR_TAIL_DARK_RGB = (0, 152, 195)     # 暗车尾颜色（T-dark）
CAR_TAIL_BRIGHT_RGB = (107, 255, 33)   # 亮车尾颜色（T-bright）

WALL_DARK_RGB = (78, 96, 118)         # 暗墙颜色（#-dark）
WALL_BRIGHT_RGB = (132, 150, 172)     # 亮墙颜色（#-bright）

FLOOR_DARK_RGB = (36, 58, 214)        # 暗空地颜色（--dark）
FLOOR_BRIGHT_RGB = (58, 92, 255)      # 亮空地颜色（--bright）

GOAL_DARK_RGB = (194, 0, 214)         # 暗目的地颜色（.-dark）
GOAL_BRIGHT_RGB = (245, 14, 255)      # 亮目的地颜色（.-bright）

BOX_DARK_RGB = (114, 140, 0)          # 暗箱子颜色（$-dark）
BOX_BRIGHT_RGB = (170, 204, 10)       # 亮箱子颜色（$-bright）

BOMB_DARK_RGB = (215, 12, 56)         # 暗炸弹颜色（*-dark）
BOMB_BRIGHT_RGB = (255, 36, 92)       # 亮炸弹颜色（*-bright）

SYMBOL_MAP_RGB = {
    "#": (WALL_DARK_RGB, WALL_BRIGHT_RGB),  # 墙体双模板：暗墙/亮墙
    "-": (FLOOR_DARK_RGB, FLOOR_BRIGHT_RGB),# 空地双模板：暗空地/亮空地
    ".": (GOAL_DARK_RGB, GOAL_BRIGHT_RGB),  # 目的地双模板：暗目的地/亮目的地
    "$": (BOX_DARK_RGB, BOX_BRIGHT_RGB),    # 箱子双模板：暗箱子/亮箱子
    "*": (BOMB_DARK_RGB, BOMB_BRIGHT_RGB),  # 炸弹双模板：暗炸弹/亮炸弹
    "H": (CAR_HEAD_DARK_RGB, CAR_HEAD_BRIGHT_RGB),  # 车头双模板：暗车头/亮车头
    "T": (CAR_TAIL_DARK_RGB, CAR_TAIL_BRIGHT_RGB),  # 车尾双模板：暗车尾/亮车尾
}

# 推荐阈值：先做双模板距离判定，再走全局最近邻兜底。
# 阈值单位为 RGB 加权欧氏距离平方（dist^2），建议实地标定时在此基础上微调 ±15%。
SYMBOL_MAX_DIST_SQ = {
    "#": (5600, 10000),
    "-": (2500, 4300),
    ".": (2600, 3500),
    "$": (2900, 4200),
    "*": (2800, 4300),
    "H": (3200, 4200),
    "T": (3200, 4200),
}

SAMPLE_OFFSETS = ((-1, -1), (0, -1), (1, -1),
                  (-1,  0), (0,  0), (1,  0),
                  (-1,  1), (0,  1), (1,  1))
SAMPLE_TRIM = 1
RGB_R_WEIGHT = 1.00
RGB_G_WEIGHT = 1.00
RGB_B_WEIGHT = 1.00
WALL_GRAY_SPREAD_MAX = 42
WALL_RGB_MIN = 42
WALL_RGB_MAX = 195
WALL_CENTER_RGB_MIN_RELAX = 8
WALL_CENTER_RGB_MAX_BOOST = 90
WALL_CENTER_SPREAD_BOOST = 12
WALL_DARK_MAX_DIST_SQ = SYMBOL_MAX_DIST_SQ["#"][0]
WALL_BRIGHT_MAX_DIST_SQ = SYMBOL_MAX_DIST_SQ["#"][1]

def wall_thresholds_at(x, y, img_w, img_h):
    """按采样点位置给墙体阈值做轻量补偿，降低中心区域漏检。"""
    if img_w <= 1 or img_h <= 1:
        return WALL_RGB_MIN, WALL_RGB_MAX, WALL_GRAY_SPREAD_MAX

    cx = (img_w - 1) // 2
    cy = (img_h - 1) // 2
    nx = abs(x - cx) * 100 // (cx if cx > 0 else 1)
    ny = abs(y - cy) * 100 // (cy if cy > 0 else 1)
    center_ratio = 100 - ((nx + ny) // 2)
    if center_ratio < 0:
        center_ratio = 0

    rgb_min = WALL_RGB_MIN - (WALL_CENTER_RGB_MIN_RELAX * center_ratio) // 100
    rgb_max = WALL_RGB_MAX + (WALL_CENTER_RGB_MAX_BOOST * center_ratio) // 100
    gray_spread = WALL_GRAY_SPREAD_MAX + (WALL_CENTER_SPREAD_BOOST * center_ratio) // 100
    return rgb_min, rgb_max, gray_spread

def robust_rgb_at(img, x, y, img_w, img_h):
    samples = []
    for ox, oy in SAMPLE_OFFSETS:
        sx, sy = x + ox, y + oy
        if 0 <= sx < img_w and 0 <= sy < img_h:
            rgb = img.get_pixel(sx, sy)
            lum = rgb[0] * 3 + rgb[1] * 6 + rgb[2]
            samples.append((lum, rgb[0], rgb[1], rgb[2]))

    samples.sort()
    start, end = 0, len(samples)
    if len(samples) > SAMPLE_TRIM * 2 + 2:
        start = SAMPLE_TRIM
        end = len(samples) - SAMPLE_TRIM

    r_sum, g_sum, b_sum, count = 0, 0, 0, 0
    for sample in samples[start:end]:
        r_sum += sample[1]
        g_sum += sample[2]
        b_sum += sample[3]
        count += 1

    if count == 0:
        rgb = img.get_pixel(x, y)
        return (rgb[0], rgb[1], rgb[2])

    return (r_sum // count, g_sum // count, b_sum // count)

def avg_rgb_at(img, x, y, img_w, img_h):
    return robust_rgb_at(img, x, y, img_w, img_h)

def rgb_dist_sq(meas_rgb, std_rgb):
    dr = (meas_rgb[0] - std_rgb[0]) * RGB_R_WEIGHT
    dg = (meas_rgb[1] - std_rgb[1]) * RGB_G_WEIGHT
    db = (meas_rgb[2] - std_rgb[2]) * RGB_B_WEIGHT
    return dr * dr + dg * dg + db * db

def find_best_symbol(rgb, x=None, y=None, img_w=None, img_h=None):
    # 墙体采用双模板并行判定：暗墙/亮墙任一命中即判墙。
    c_max = max(rgb[0], rgb[1], rgb[2])
    c_min = min(rgb[0], rgb[1], rgb[2])
    c_avg = (rgb[0] + rgb[1] + rgb[2]) // 3
    if x is not None and y is not None and img_w is not None and img_h is not None:
        wall_rgb_min, wall_rgb_max, wall_gray_spread = wall_thresholds_at(x, y, img_w, img_h)
    else:
        wall_rgb_min, wall_rgb_max, wall_gray_spread = WALL_RGB_MIN, WALL_RGB_MAX, WALL_GRAY_SPREAD_MAX

    if (c_max - c_min) <= wall_gray_spread and wall_rgb_min <= c_avg <= wall_rgb_max:
        return "#"

    wall_dark_rgb, wall_bright_rgb = SYMBOL_MAP_RGB["#"]
    dark_wall_dist = rgb_dist_sq(rgb, wall_dark_rgb)
    bright_wall_dist = rgb_dist_sq(rgb, wall_bright_rgb)
    if dark_wall_dist <= WALL_DARK_MAX_DIST_SQ or bright_wall_dist <= WALL_BRIGHT_MAX_DIST_SQ:
        return "#"

    min_dist, matched = 999999, None
    nearest_sym, nearest_dist, nearest_limit = "-", 999999, SYMBOL_MAX_DIST_SQ["-"][1]
    for sym, std_rgb in SYMBOL_MAP_RGB.items():
        if sym == "#":
            continue
        if isinstance(std_rgb, tuple) and len(std_rgb) > 0 and isinstance(std_rgb[0], tuple):
            templates = std_rgb
        else:
            templates = (std_rgb,)

        max_dist_cfg = SYMBOL_MAX_DIST_SQ[sym]
        if isinstance(max_dist_cfg, tuple):
            limits = max_dist_cfg
        else:
            limits = (max_dist_cfg,)

        if len(limits) < len(templates):
            limits = limits + (limits[-1],) * (len(templates) - len(limits))

        best_dist_for_sym = 999999
        for i in range(len(templates)):
            dist_sq = rgb_dist_sq(rgb, templates[i])
            if dist_sq < best_dist_for_sym:
                best_dist_for_sym = dist_sq
            if dist_sq <= limits[i] and dist_sq < min_dist:
                min_dist = dist_sq
                matched = sym

        sym_limit = limits[0]
        if len(limits) > 1 and limits[1] > sym_limit:
            sym_limit = limits[1]
        if best_dist_for_sym < nearest_dist:
            nearest_dist = best_dist_for_sym
            nearest_sym = sym
            nearest_limit = sym_limit

    if matched is not None:
        return matched

    if nearest_dist <= nearest_limit:
        return nearest_sym

    return "-"

def find_car_pair(map_list):
    """在16x12网格中寻找相邻的H/T，返回车头坐标和车尾坐标。"""
    grid = [map_list[r * COLS:(r + 1) * COLS] for r in range(ROWS)]
    neighbors = ((1, 0), (-1, 0), (0, 1), (0, -1))

    for y in range(ROWS):
        for x in range(COLS):
            if grid[y][x] != "H":
                continue

            for dx, dy in neighbors:
                nx, ny = x + dx, y + dy
                if 0 <= nx < COLS and 0 <= ny < ROWS and grid[ny][nx] == "T":
                    return (x, y, nx, ny)

    return None

def find_car_single(map_list):
    """回退策略：当H/T未能成对时，使用单个H或T作为车辆坐标。"""
    for y in range(ROWS):
        for x in range(COLS):
            ch = map_list[y * COLS + x]
            if ch == "H" or ch == "T":
                return (x, y)
    return None

def build_map_with_single_car(map_list, car_found, car_x, car_y):
    """输出阶段统一只保留一个@，并与发送坐标严格一致。"""
    merged = []
    for ch in map_list:
        if ch == "H" or ch == "T":
            merged.append("-")
        else:
            merged.append(ch)

    if car_found and 0 <= car_x < COLS and 0 <= car_y < ROWS:
        merged[car_y * COLS + car_x] = "@"

    return merged

def classify_cell(img, x, y, img_w, img_h):
    rgb = avg_rgb_at(img, x, y, img_w, img_h)
    return find_best_symbol(rgb, x, y, img_w, img_h)

# ----------------------------------------------------------------------
# 3. 核心逻辑主循环 (图像识别 -> 位置锁定 -> 打包发送)
# ----------------------------------------------------------------------
while(True):
    clock.tick()                       # 开始计算帧处理时间
    img = sensor.snapshot()            # 捕获当前摄像头图像
    frame_cnt += 1                     # 总帧数加一

    img_w, img_h = img.width(), img.height()
    map_list = []                      # 临时容器：存放本帧识别出的 192 个地图字符
    car_x, car_y = 0, 0                # 默认车辆坐标 (0, 0)
    car_found = False                  # 本帧是否确实识别到车辆（H/T配对）
    tl_pt = tr_pt = bl_pt = br_pt = None

    # --- 阶段 A：双层嵌套循环：解析 16x12 赛道地图 ---
    for y_idx in range(ROWS):
        for x_idx in range(COLS):
            # 根据四角标定点做双线性反投影，得到真实采样点坐标
            tx, ty = calc_grid_point(x_idx, y_idx, img_w, img_h)

            # --- 安全检查与元素分类 ---
            if 0 <= tx < img_w and 0 <= ty < img_h:
                # 提取采样点周围颜色均值并在 LAB 空间分类
                char = classify_cell(img, tx, ty, img_w, img_h)
                map_list.append(char)

                # 在可视化缓冲区画出实心白点，用于调试对位情况
                img.draw_circle(tx, ty, 2, color=(255, 255, 255), fill=True)
            else:
                map_list.append("?")   # 若采样出界，记为问号补位

    car_pair = find_car_pair(map_list)
    if car_pair is not None:
        car_x, car_y = car_pair[0], car_pair[1]  # 发送车头坐标
        car_found = True

    # 若H/T未成对，回退到单符号坐标，避免地图有车而坐标仍为(0,0)。
    if not car_found:
        car_single = find_car_single(map_list)
        if car_single is not None:
            car_x, car_y = car_single
            car_found = True

    if CALIB_SHOW_CORNERS:
        tl_pt, tr_pt, bl_pt, br_pt = draw_calibration_overlay(img, img_w, img_h)

    # 对车辆坐标做短窗口多数投票，避免单帧色值波动导致坐标跳动
    car_x, car_y = vote_car_position(car_found, car_x, car_y)

    # 输出阶段统一为单一@，并与投票后的发送坐标严格一致。
    map_list_out = build_map_with_single_car(map_list, car_found, car_x, car_y)

    # --- 阶段 B：数据打包与发送 (194 字节完整协议帧) ---
    # 包内容：192个字节的赛道字符 + 1个字节的车辆坐标X + 1个字节的车辆坐标Y
    try:
        # 将 [#, -, @...] 列表转换为连续的 ASCII 字节流
        map_bytes = "".join(map_list_out).encode("ascii")
        # 拼接地图数据与坐标字节
        payload = map_bytes + bytes([car_x, car_y])

        # 通过 UART 12 发送 TYPE=0x01 的任务帧
        if len(payload) == 194:
            uart.write(pack_frame(PROTO_TYPE_MAP, payload))
    except Exception as e:
        print("串口发送异常:", e)

    # 定时维护通讯心跳包
    send_heartbeat_if_due()

    # --- 阶段 C：调试信息交互 (每 20 帧稳定更新一次打印) ---
    if frame_cnt % 20 == 0:
        print("\033[H", end="")        # 终端光标归零（清屏效果）
        print("系统帧率: %0.1f | 小车实时坐标: (%d, %d)" % (clock.fps(), car_x, car_y))
        if CALIB_SHOW_CORNERS and tl_pt is not None:
            print("TL=%s TR=%s BL=%s BR=%s" % (tl_pt, tr_pt, bl_pt, br_pt))
        # 打印 ASCII 预览图，检查视觉逻辑是否与实际场地一致
        for r in range(ROWS):
            print("".join(map_list_out[r*COLS : (r+1)*COLS]))

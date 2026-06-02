# ======================================================================
# 【项目名称】：智能车视觉导航 - 16x12 赛道解算与车辆实时定位系统
# 【实现目标】：
#   1. 建立 16x12 逻辑网格，通过“打点采样”识别全场地图元素。
#   2. 实时检索车辆标志位(@)，锁定车辆在地图中的行列坐标(X, Y)。
#   3. 通过自定义二进制协议帧，将 194 字节数据（地图+坐标）传给主控。
# 【核心原理】：
#   - 坐标计算：线性斜率补偿（修正由于摄像头安装倾斜导致的近大远小）。
#   - 元素匹配：提取 7x7 区域 LAB 三通道直方图，并做巴氏距离匹配。
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
    "tl": (31.0, 39.0), #左上
    "tr": (275.0, 28.5), #右上
    "bl": (41.5, 203.2), #左下
    "br": (275.8, 203.5), #右下
}

GRID_K1 = +0.000000                    # 径向畸变系数；无畸变镜头可设为 0
CALIB_SHOW_CORNERS = True              # 四角标定模式：高亮四角与外框，便于精准调参
CAR_VOTE_FRAMES = 4                    # 车辆坐标最近 N 帧多数投票
CAR_VOTE_MIN = 2                       # 至少出现 N 次才认为是稳定坐标
CAR_SEARCH_LOCAL_RADIUS = 1            # 锁定区域搜索：以 last_pos 为中心先搜 3x3
CAR_WEAK_BHATT_MARGIN = 0.15           # H/T 弱命中区间：阈值到阈值+0.10
CAR_WEAK_GATE_RADIUS_GRID = 2.2        # 弱命中轨迹门控半径（网格单位）

# 卡尔曼参数（二维常速度模型）
# 按“降低一点参数”要求，采用较温和参数，减少跳变与过度跟随。
KF_DT = 1.0
KF_Q_POS = 0.05
KF_Q_VEL = 0.03
KF_R_MEAS = 2.0

car_vote_hist = []
car_last_pos_grid = None               # 上一帧稳定网格坐标
kf_inited = False
kf_state = {
    "x": 0.0, "vx": 0.0,
    "y": 0.0, "vy": 0.0,
    "px00": 8.0, "px01": 0.0, "px10": 0.0, "px11": 2.0,
    "py00": 8.0, "py01": 0.0, "py10": 0.0, "py11": 2.0,
}

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

def _kalman_predict_1d(pos, vel, p00, p01, p10, p11):
    pos = pos + KF_DT * vel
    n00 = p00 + KF_DT * (p10 + p01) + (KF_DT * KF_DT) * p11 + KF_Q_POS
    n01 = p01 + KF_DT * p11
    n10 = p10 + KF_DT * p11
    n11 = p11 + KF_Q_VEL
    return pos, vel, n00, n01, n10, n11

def _kalman_update_1d(pos, vel, p00, p01, p10, p11, meas):
    y = meas - pos
    s = p00 + KF_R_MEAS
    if s <= 1e-6:
        return pos, vel, p00, p01, p10, p11
    k0 = p00 / s
    k1 = p10 / s
    pos = pos + k0 * y
    vel = vel + k1 * y
    u00 = (1.0 - k0) * p00
    u01 = (1.0 - k0) * p01
    u10 = p10 - k1 * p00
    u11 = p11 - k1 * p01
    return pos, vel, u00, u01, u10, u11

def kalman_predict_grid():
    global kf_state
    x = _kalman_predict_1d(kf_state["x"], kf_state["vx"], kf_state["px00"], kf_state["px01"], kf_state["px10"], kf_state["px11"])
    y = _kalman_predict_1d(kf_state["y"], kf_state["vy"], kf_state["py00"], kf_state["py01"], kf_state["py10"], kf_state["py11"])
    kf_state["x"], kf_state["vx"], kf_state["px00"], kf_state["px01"], kf_state["px10"], kf_state["px11"] = x
    kf_state["y"], kf_state["vy"], kf_state["py00"], kf_state["py01"], kf_state["py10"], kf_state["py11"] = y
    return kf_state["x"], kf_state["y"]

def kalman_update_grid(meas_x, meas_y):
    global kf_state
    x = _kalman_update_1d(kf_state["x"], kf_state["vx"], kf_state["px00"], kf_state["px01"], kf_state["px10"], kf_state["px11"], meas_x)
    y = _kalman_update_1d(kf_state["y"], kf_state["vy"], kf_state["py00"], kf_state["py01"], kf_state["py10"], kf_state["py11"], meas_y)
    kf_state["x"], kf_state["vx"], kf_state["px00"], kf_state["px01"], kf_state["px10"], kf_state["px11"] = x
    kf_state["y"], kf_state["vy"], kf_state["py00"], kf_state["py01"], kf_state["py10"], kf_state["py11"] = y
    return kf_state["x"], kf_state["y"]

# ----------------------------------------------------------------------
# 3. 元素颜色特征库 (RGB 实测标定 -> LAB 直方图指纹匹配)
# ----------------------------------------------------------------------
# 只需在现场重新读取这一组 RGB 值。
CAR_HEAD_DARK_RGB = (0, 142, 173)      # 暗车头颜色（H-dark）
CAR_HEAD_BRIGHT_RGB = (0, 235, 255)    # 亮车头颜色（H-bright）
CAR_TAIL_DARK_RGB = (8, 138, 8)     # 暗车尾颜色（T-dark）
CAR_TAIL_BRIGHT_RGB = (33, 210, 49)   # 亮车尾颜色（T-bright）

WALL_DARK_RGB = (78, 96, 118)         # 暗墙颜色（#-dark）
WALL_BRIGHT_RGB = (132, 150, 172)     # 亮墙颜色（#-bright）

FLOOR_DARK_RGB = (36, 58, 214)        # 暗空地颜色（--dark）
FLOOR_BRIGHT_RGB = (58, 92, 255)      # 亮空地颜色（--bright）

GOAL_DARK_RGB = (194, 0, 214)         # 暗目的地颜色（.-dark）
GOAL_BRIGHT_RGB = (245, 14, 255)      # 亮目的地颜色（.-bright）

BOX_DARK_RGB = (132, 198, 0)          # 暗箱子颜色（$-dark）
BOX_BRIGHT_RGB = (247, 255, 41)       # 亮箱子颜色（$-bright）

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

LAB_REGION_RADIUS = 3      # 7x7 区域：半径 3 表示中心点左右上下各取 3 像素
LAB_L_BINS = 8             # L 通道直方图桶数（0~100）
LAB_A_BINS = 8             # a 通道直方图桶数（-128~127）
LAB_B_BINS = 8             # b 通道直方图桶数（-128~127）
LAB_SOFT_SIGMA = 0.95      # 模板指纹软分布扩散系数，越小越尖锐

# 巴氏距离阈值（0 表示完全一致，越接近 1 差异越大）
# 每个元素保留暗/亮双模板阈值；先做阈值命中，再走最近邻兜底。
SYMBOL_MAX_BHATT = {
    "#": (0.60, 0.60),
    "-": (0.56, 0.56),
    ".": (0.56, 0.56),
    "$": (0.56, 0.56),
    "*": (0.56, 0.56),
    "H": (0.58, 0.58),
    "T": (0.58, 0.58),
}

# ----------------------------------------------------------------------
# 4. 位置先验约束 + 时域平滑参数
# ----------------------------------------------------------------------
# 语义类别索引：严格对应 prob_map[y][x][cls]
CLS_WALL = 0
CLS_FLOOR = 1
CLS_BOX = 2
CLS_GOAL = 3
CLS_BOMB = 4
CLS_CAR = 5

# 符号到语义类别的映射。车头/车尾都归并到“车”语义类别。
SYMBOL_TO_CLASS = {
    "#": CLS_WALL,
    "-": CLS_FLOOR,
    "$": CLS_BOX,
    ".": CLS_GOAL,
    "*": CLS_BOMB,
    "H": CLS_CAR,
    "T": CLS_CAR,
}

POSITION_PRIOR_WEIGHT = 0.22      # 位置先验惩罚权重，越大越依赖位置逻辑
BHATT_FALLBACK_MARGIN = 0.06      # 兜底余量：允许最优距离略高于阈值仍保留结果
CELL_STATE_CONFIRM_FRAMES = 5     # 单格状态切换需连续一致帧数

def _normalize_prob(vec):
    """将一组非负权重归一化为概率分布。"""
    s = 0.0
    for v in vec:
        if v > 0.0:
            s += v
    if s <= 0.0:
        # 极端情况下回退为“空地主导”的安全分布
        return [0.10, 0.70, 0.05, 0.05, 0.00, 0.10]

    out = [0.0] * len(vec)
    inv = 1.0 / s
    for i in range(len(vec)):
        v = vec[i]
        if v < 0.0:
            v = 0.0
        out[i] = v * inv
    return out

def build_position_prob_map():
    """
    构建 prob_map[12][16][6]：位置先验概率。
    先验设计原则：
    1) 最外圈更偏向墙体，炸弹先验压低（可设为 0）。
    2) 内圈以地板为主，箱子/目的地/车辆给适中概率。
    3) 先验只“轻拉回”识别结果，不直接替代颜色匹配。
    """
    table = []
    for y in range(ROWS):
        row = []
        for x in range(COLS):
            is_outer = (x == 0 or x == COLS - 1 or y == 0 or y == ROWS - 1)
            is_near_outer = (x <= 1 or x >= COLS - 2 or y <= 1 or y >= ROWS - 2)

            # 基础先验（墙, 地, 箱, 目标, 炸弹, 车）
            p = [0.22, 0.40, 0.12, 0.12, 0.06, 0.08]

            if is_outer:
                p = [0.62, 0.30, 0.03, 0.03, 0.00, 0.02]
            elif is_near_outer:
                p = [0.38, 0.42, 0.08, 0.08, 0.01, 0.03]
            else:
                # 中心区域更容易出现可移动元素
                p = [0.16, 0.44, 0.16, 0.14, 0.03, 0.07]

            row.append(_normalize_prob(p))
        table.append(row)
    return table

# 对外暴露为用户提出的命名：prob_map[12][16][6]
prob_map = build_position_prob_map()

# 时域平滑状态：
# last_grid_states 存放“已确认输出”的稳定语义；
# pending_grid_states/pending_grid_counts 存放“候选切换态”的连续计数。
last_grid_states = [["-" for _ in range(COLS)] for _ in range(ROWS)]
pending_grid_states = [[None for _ in range(COLS)] for _ in range(ROWS)]
pending_grid_counts = [[0 for _ in range(COLS)] for _ in range(ROWS)]

def _pivot_rgb_to_linear(c):
    """sRGB 分段逆伽马，输入 [0,1]，输出线性光强。"""
    if c <= 0.04045:
        return c / 12.92
    return ((c + 0.055) / 1.055) ** 2.4

def rgb_to_lab(rgb):
    """
    将 RGB888 转换为 CIE LAB。
    说明：
    1) 先做 sRGB -> 线性 RGB。
    2) 线性 RGB 乘以 D65 矩阵得到 XYZ。
    3) XYZ 再转 LAB。
    """
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

def _lab_bin_index(l, a, bb):
    """将 LAB 值映射到三个通道各自的桶索引。"""
    if l < 0.0:
        l = 0.0
    if l > 100.0:
        l = 100.0
    if a < -128.0:
        a = -128.0
    if a > 127.0:
        a = 127.0
    if bb < -128.0:
        bb = -128.0
    if bb > 127.0:
        bb = 127.0

    l_idx = int(l * LAB_L_BINS / 101.0)
    a_idx = int((a + 128.0) * LAB_A_BINS / 256.0)
    b_idx = int((bb + 128.0) * LAB_B_BINS / 256.0)

    if l_idx >= LAB_L_BINS:
        l_idx = LAB_L_BINS - 1
    if a_idx >= LAB_A_BINS:
        a_idx = LAB_A_BINS - 1
    if b_idx >= LAB_B_BINS:
        b_idx = LAB_B_BINS - 1
    return l_idx, a_idx, b_idx

def lab_histogram_at(img, x, y, img_w, img_h):
    """
    在 (x,y) 周围提取 7x7 区域 LAB 三通道直方图。
    返回 (hist_l, hist_a, hist_b)，三个列表均做了归一化，总和为 1。
    """
    hist_l = [0.0] * LAB_L_BINS
    hist_a = [0.0] * LAB_A_BINS
    hist_b = [0.0] * LAB_B_BINS

    count = 0
    for oy in range(-LAB_REGION_RADIUS, LAB_REGION_RADIUS + 1):
        sy = y + oy
        if sy < 0 or sy >= img_h:
            continue
        for ox in range(-LAB_REGION_RADIUS, LAB_REGION_RADIUS + 1):
            sx = x + ox
            if sx < 0 or sx >= img_w:
                continue

            rgb = img.get_pixel(sx, sy)
            l, a, bb = rgb_to_lab((rgb[0], rgb[1], rgb[2]))
            l_idx, a_idx, b_idx = _lab_bin_index(l, a, bb)
            hist_l[l_idx] += 1.0
            hist_a[a_idx] += 1.0
            hist_b[b_idx] += 1.0
            count += 1

    if count <= 0:
        # 理论上中心点在边界内时不会发生；兜底避免除零。
        hist_l[0] = 1.0
        hist_a[0] = 1.0
        hist_b[0] = 1.0
        return hist_l, hist_a, hist_b

    inv = 1.0 / count
    for i in range(LAB_L_BINS):
        hist_l[i] *= inv
    for i in range(LAB_A_BINS):
        hist_a[i] *= inv
    for i in range(LAB_B_BINS):
        hist_b[i] *= inv
    return hist_l, hist_a, hist_b

def _soft_hist(length, center_idx, sigma):
    """基于中心桶构造一维高斯软直方图，作为模板指纹的单通道原型。"""
    out = [0.0] * length
    s = 0.0
    two_sigma_sq = 2.0 * sigma * sigma
    for i in range(length):
        d = i - center_idx
        v = math.exp(-(d * d) / two_sigma_sq)
        out[i] = v
        s += v
    if s > 0:
        inv = 1.0 / s
        for i in range(length):
            out[i] *= inv
    return out

def build_template_hist_from_rgb(rgb):
    """
    将单个 RGB 模板颜色转为 LAB 后，构建三通道软直方图模板。
    作用：把“单点颜色模板”升级为“可容忍轻微偏色的分布模板”。
    """
    l, a, bb = rgb_to_lab(rgb)
    l_idx, a_idx, b_idx = _lab_bin_index(l, a, bb)
    return (
        _soft_hist(LAB_L_BINS, l_idx, LAB_SOFT_SIGMA),
        _soft_hist(LAB_A_BINS, a_idx, LAB_SOFT_SIGMA),
        _soft_hist(LAB_B_BINS, b_idx, LAB_SOFT_SIGMA),
    )

def build_symbol_hist_library():
    """将每个符号的暗/亮 RGB 模板预生成 LAB 直方图指纹，避免逐帧重复计算。"""
    library = {}
    for sym, rgb_templates in SYMBOL_MAP_RGB.items():
        if isinstance(rgb_templates, tuple) and len(rgb_templates) > 0 and isinstance(rgb_templates[0], tuple):
            tpls = rgb_templates
        else:
            tpls = (rgb_templates,)
        library[sym] = tuple(build_template_hist_from_rgb(rgb) for rgb in tpls)
    return library

SYMBOL_MAP_HIST = build_symbol_hist_library()

def bhattacharyya_distance(hist_p, hist_q):
    """计算两个一维概率直方图的巴氏距离，范围约为 [0,1]。"""
    bc = 0.0
    length = len(hist_p)
    for i in range(length):
        p = hist_p[i]
        q = hist_q[i]
        if p > 0.0 and q > 0.0:
            bc += math.sqrt(p * q)

    if bc > 1.0:
        bc = 1.0
    if bc < 0.0:
        bc = 0.0
    return math.sqrt(1.0 - bc)

def hist_distance_3ch(meas_hist, ref_hist):
    """三通道巴氏距离融合：分别比较 L/a/b 后取均值。"""
    d_l = bhattacharyya_distance(meas_hist[0], ref_hist[0])
    d_a = bhattacharyya_distance(meas_hist[1], ref_hist[1])
    d_b = bhattacharyya_distance(meas_hist[2], ref_hist[2])
    return (d_l + d_a + d_b) / 3.0

def find_best_symbol(hist_3ch, grid_x, grid_y):
    """
    使用 LAB 三通道直方图与模板库做匹配。
    判定策略：
    1) 逐模板计算巴氏距离，若低于该模板阈值则记为有效命中。
    2) 所有有效命中中取最小距离。
    3) 若无有效命中，则用“最近邻 + 最近邻阈值”兜底。
    """
    global kf_inited, kf_state, car_last_pos_grid
    min_score = 999999.0
    matched = None
    nearest_sym = "-"
    nearest_dist = 999999.0
    nearest_limit = SYMBOL_MAX_BHATT["-"][1]
    weak_car_sym = None
    weak_car_dist = 999999.0

    for sym, hist_templates in SYMBOL_MAP_HIST.items():
        max_dist_cfg = SYMBOL_MAX_BHATT[sym]
        if isinstance(max_dist_cfg, tuple):
            limits = max_dist_cfg
        else:
            limits = (max_dist_cfg,)

        if len(limits) < len(hist_templates):
            limits = limits + (limits[-1],) * (len(hist_templates) - len(limits))

        best_dist_for_sym = 999999.0
        for i in range(len(hist_templates)):
            dist = hist_distance_3ch(hist_3ch, hist_templates[i])
            if dist < best_dist_for_sym:
                best_dist_for_sym = dist

        # 颜色距离 + 位置先验惩罚融合评分。
        # 先验概率越低，(1-prior) 越大，得分越差。
        cls = SYMBOL_TO_CLASS[sym]
        prior = prob_map[grid_y][grid_x][cls]
        score = best_dist_for_sym + POSITION_PRIOR_WEIGHT * (1.0 - prior)

        # 仅对“颜色距离在阈值内”的类别参与主命中竞争，避免先验压倒颜色证据。
        sym_limit_for_match = limits[0]
        if len(limits) > 1 and limits[1] < sym_limit_for_match:
            sym_limit_for_match = limits[1]
        if best_dist_for_sym <= sym_limit_for_match and score < min_score:
            min_score = score
            matched = sym

        # 对 H/T 保留“弱命中”候选：阈值到阈值+0.10。
        if (sym == "H" or sym == "T") and best_dist_for_sym > sym_limit_for_match:
            if best_dist_for_sym <= (sym_limit_for_match + CAR_WEAK_BHATT_MARGIN):
                if best_dist_for_sym < weak_car_dist:
                    weak_car_dist = best_dist_for_sym
                    weak_car_sym = sym

        sym_limit = limits[0]
        if len(limits) > 1 and limits[1] > sym_limit:
            sym_limit = limits[1]
        if best_dist_for_sym < nearest_dist:
            nearest_dist = best_dist_for_sym
            nearest_sym = sym
            nearest_limit = sym_limit

    if matched is not None:
        return matched

    # 弱命中轨迹门控：若在卡尔曼预测轨迹半径内，则强制判为小车。
    if weak_car_sym is not None:
        gate_cx = None
        gate_cy = None
        if kf_inited:
            gate_cx = kf_state["x"]
            gate_cy = kf_state["y"]
        elif car_last_pos_grid is not None:
            gate_cx = car_last_pos_grid[0]
            gate_cy = car_last_pos_grid[1]

        if gate_cx is not None and gate_cy is not None:
            dx = grid_x - gate_cx
            dy = grid_y - gate_cy
            if (dx * dx + dy * dy) <= (CAR_WEAK_GATE_RADIUS_GRID * CAR_WEAK_GATE_RADIUS_GRID):
                return weak_car_sym

    # 若没有阈值内命中，使用最近邻兜底，但仍受阈值+余量约束。
    if nearest_dist <= (nearest_limit + BHATT_FALLBACK_MARGIN):
        return nearest_sym
    return "-"

def temporal_smooth_cell(grid_x, grid_y, curr_sym):
    """
    每格语义时域滤波：
    1) 若当前识别与稳定态相同，直接维持稳定态。
    2) 若发生跳变，不立刻切换；只有连续 CELL_STATE_CONFIRM_FRAMES 帧一致才更新。
    3) 这样可抑制单帧反光、噪声、瞬态曝光导致的误判跳变。
    """
    if curr_sym == "?":
        return last_grid_states[grid_y][grid_x]

    stable = last_grid_states[grid_y][grid_x]
    if curr_sym == stable:
        pending_grid_states[grid_y][grid_x] = None
        pending_grid_counts[grid_y][grid_x] = 0
        return stable

    pending = pending_grid_states[grid_y][grid_x]
    if pending == curr_sym:
        pending_grid_counts[grid_y][grid_x] += 1
    else:
        pending_grid_states[grid_y][grid_x] = curr_sym
        pending_grid_counts[grid_y][grid_x] = 1

    if pending_grid_counts[grid_y][grid_x] >= CELL_STATE_CONFIRM_FRAMES:
        last_grid_states[grid_y][grid_x] = curr_sym
        pending_grid_states[grid_y][grid_x] = None
        pending_grid_counts[grid_y][grid_x] = 0

    return last_grid_states[grid_y][grid_x]

def find_car_pair(map_list, center_pos=None):
    """在16x12网格中寻找相邻的H/T，返回车头坐标和车尾坐标。"""
    grid = [map_list[r * COLS:(r + 1) * COLS] for r in range(ROWS)]
    neighbors = ((1, 0), (-1, 0), (0, 1), (0, -1))

    def _scan_cells(cells):
        for x, y in cells:
            if grid[y][x] != "H":
                continue
            for dx, dy in neighbors:
                nx, ny = x + dx, y + dy
                if 0 <= nx < COLS and 0 <= ny < ROWS and grid[ny][nx] == "T":
                    return (x, y, nx, ny)
        return None

    # 先在上一帧附近 3x3 搜索。
    if center_pos is not None:
        cx, cy = center_pos
        local_cells = []
        for yy in range(cy - CAR_SEARCH_LOCAL_RADIUS, cy + CAR_SEARCH_LOCAL_RADIUS + 1):
            if yy < 0 or yy >= ROWS:
                continue
            for xx in range(cx - CAR_SEARCH_LOCAL_RADIUS, cx + CAR_SEARCH_LOCAL_RADIUS + 1):
                if xx < 0 or xx >= COLS:
                    continue
                local_cells.append((xx, yy))
        found = _scan_cells(local_cells)
        if found is not None:
            return found

    for y in range(ROWS):
        for x in range(COLS):
            if grid[y][x] != "H":
                continue

            for dx, dy in neighbors:
                nx, ny = x + dx, y + dy
                if 0 <= nx < COLS and 0 <= ny < ROWS and grid[ny][nx] == "T":
                    return (x, y, nx, ny)

    return None

def find_car_single(map_list, center_pos=None):
    """回退策略：当H/T未能成对时，使用单个H或T作为车辆坐标。"""
    if center_pos is not None:
        cx, cy = center_pos
        for yy in range(cy - CAR_SEARCH_LOCAL_RADIUS, cy + CAR_SEARCH_LOCAL_RADIUS + 1):
            if yy < 0 or yy >= ROWS:
                continue
            for xx in range(cx - CAR_SEARCH_LOCAL_RADIUS, cx + CAR_SEARCH_LOCAL_RADIUS + 1):
                if xx < 0 or xx >= COLS:
                    continue
                ch = map_list[yy * COLS + xx]
                if ch == "H" or ch == "T":
                    return (xx, yy)

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

def classify_cell(img, x, y, img_w, img_h, grid_x, grid_y):
    """
    单格分类入口：
    1) 从格子中心提取 7x7 区域 LAB 三通道直方图。
    2) 结合 prob_map 的位置先验做颜色-位置联合判定。
    3) 对该格执行连续 5 帧一致的时域平滑后输出稳定符号。
    """
    hist_3ch = lab_histogram_at(img, x, y, img_w, img_h)
    raw_sym = find_best_symbol(hist_3ch, grid_x, grid_y)
    return temporal_smooth_cell(grid_x, grid_y, raw_sym)

# ----------------------------------------------------------------------
# 3. 核心逻辑主循环 (图像识别 -> 位置锁定 -> 打包发送)
# ----------------------------------------------------------------------
# 【画面稳定控制】只有当画面真正变化（车移动/地图改变）时才刷新显示，
# 避免屏幕持续闪烁。稳定帧中只做轻量采样，不绘制调试覆盖层。
prev_map_str = ""                       # 上一帧的地图字符串，用于变化检测
stable_frame_count = 0                  # 连续稳定帧计数
needs_redraw = True                     # 当前帧是否需要绘制调试覆盖层
REFRESH_EVERY_N_STABLE = 30             # 即使稳定，每 N 帧强制刷新一次（约 0.5 秒）

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
                # 先做 LAB 直方图匹配，再叠加位置先验与时域平滑
                char = classify_cell(img, tx, ty, img_w, img_h, x_idx, y_idx)
                map_list.append(char)

                # 仅在画面有变化时才绘制采样白点，避免屏幕持续刷新闪烁
                if needs_redraw:
                    img.draw_circle(tx, ty, 2, color=(255, 255, 255), fill=True)
            else:
                map_list.append("?")   # 若采样出界，记为问号补位

    # 卡尔曼预测位置 + 3x3 局部优先搜索
    pred_center = car_last_pos_grid
    if kf_inited:
        pred_xf, pred_yf = kalman_predict_grid()
        pred_x = int(pred_xf + 0.5)
        pred_y = int(pred_yf + 0.5)
        if pred_x < 0:
            pred_x = 0
        if pred_x >= COLS:
            pred_x = COLS - 1
        if pred_y < 0:
            pred_y = 0
        if pred_y >= ROWS:
            pred_y = ROWS - 1
        pred_center = (pred_x, pred_y)

    car_pair = find_car_pair(map_list, pred_center)
    if car_pair is not None:
        meas_x, meas_y = car_pair[0], car_pair[1]  # 发送车头坐标
        if not kf_inited:
            kf_state["x"] = meas_x
            kf_state["y"] = meas_y
            kf_state["vx"] = 0.0
            kf_state["vy"] = 0.0
            kf_inited = True
        car_xf, car_yf = kalman_update_grid(meas_x, meas_y)
        car_x, car_y = int(car_xf + 0.5), int(car_yf + 0.5)
        car_found = True

    # 若H/T未成对，回退到单符号坐标，避免地图有车而坐标仍为(0,0)。
    if not car_found:
        car_single = find_car_single(map_list, pred_center)
        if car_single is not None:
            meas_x, meas_y = car_single
            if not kf_inited:
                kf_state["x"] = meas_x
                kf_state["y"] = meas_y
                kf_state["vx"] = 0.0
                kf_state["vy"] = 0.0
                kf_inited = True
            car_xf, car_yf = kalman_update_grid(meas_x, meas_y)
            car_x, car_y = int(car_xf + 0.5), int(car_yf + 0.5)
            car_found = True

    # 本帧没观测到小车时，使用卡尔曼预测值保持轨迹连续。
    if not car_found and kf_inited:
        car_xf = kf_state["x"]
        car_yf = kf_state["y"]
        car_x = int(car_xf + 0.5)
        car_y = int(car_yf + 0.5)
        if car_x < 0:
            car_x = 0
        if car_x >= COLS:
            car_x = COLS - 1
        if car_y < 0:
            car_y = 0
        if car_y >= ROWS:
            car_y = ROWS - 1
        car_found = True

    if CALIB_SHOW_CORNERS and needs_redraw:
        tl_pt, tr_pt, bl_pt, br_pt = draw_calibration_overlay(img, img_w, img_h)

    # 记录 last_pos，供下一帧 ROI 锁定搜索使用。
    if car_found:
        car_last_pos_grid = (car_x, car_y)

    # 输出阶段统一为单一@，并与投票后的发送坐标严格一致。
    map_list_out = build_map_with_single_car(map_list, car_found, car_x, car_y)

    # --- 画面变化检测：比对当前地图与上一帧，仅变化时才刷新显示 ---
    current_map_str = "".join(map_list_out)
    map_changed = (current_map_str != prev_map_str)

    if map_changed:
        # 仅当不是首次捕获（prev 非空）时，才安排下一帧重绘；
        # 首次已在 needs_redraw=True 时绘制过，无需重复。
        if prev_map_str != "":
            needs_redraw = True      # 真实变化 → 下一帧重绘覆盖层
        else:
            needs_redraw = False     # 首次地图已绘制，后续等真实变化再刷新
        prev_map_str = current_map_str
        stable_frame_count = 0
    else:
        stable_frame_count += 1
        if stable_frame_count >= REFRESH_EVERY_N_STABLE:
            needs_redraw = True      # 长时间稳定后强制刷新一次
            stable_frame_count = 0
        else:
            needs_redraw = False     # 跳过下一帧的覆盖层绘制

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

    # --- 阶段 C：调试信息交互 (仅在地图变化时刷新，避免终端闪烁) ---
    if map_changed or (stable_frame_count == 0 and needs_redraw):
        print("\033[H", end="")        # 终端光标归零（清屏效果）
        print("系统帧率: %0.1f | 小车实时坐标: (%d, %d) | 帧号: %d" % (clock.fps(), car_x, car_y, frame_cnt))
        if CALIB_SHOW_CORNERS and tl_pt is not None:
            print("TL=%s TR=%s BL=%s BR=%s" % (tl_pt, tr_pt, bl_pt, br_pt))
        # 打印 ASCII 预览图，检查视觉逻辑是否与实际场地一致
        for r in range(ROWS):
            print("".join(map_list_out[r*COLS : (r+1)*COLS]))

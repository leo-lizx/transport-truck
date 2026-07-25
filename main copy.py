# ======================================================================
# 【项目名称】：智能车视觉导航 - 16x12 赛道解算与车辆实时定位系统 (高帧率优化版)
# 【实现目标】：
#   1. 建立 16x12 逻辑网格，通过底层 C API 极速提取全场地图元素特征。
#   2. 实时检索车辆标志位(@)，锁定车辆在地图中的行列坐标(X, Y)。
#   3. 通过自定义二进制协议帧，将 194 字节数据（地图+坐标）传给主控。
# 【核心优化】：
#   - 物理抗反光：手动锁定曝光，关闭增益和白平衡。
#   - 极速提取：废弃 Python 循环，使用底层的 img.get_statistics() 获取 11x11 众数。
#   - 降维打击：废弃高耗时的巴氏距离，采用加权欧氏距离（L通道降权，过滤高光）。
# ======================================================================

import sensor, image, time, math
from machine import UART

# ----------------------------------------------------------------------
# 1. 硬件资源与感光元件初始化 (抗反光调参区)
# ----------------------------------------------------------------------
sensor.reset()                         # 重置感光元件硬件寄存器
sensor.set_pixformat(sensor.RGB565)    # 16位彩色，平衡内存和色彩精度
sensor.set_framesize(sensor.QVGA)      # 320x240 分辨率，处理速度最佳平衡点
sensor.skip_frames(time = 2000)        # 让感光元件稳定

# 【关键物理防御】：锁定感光参数，拒绝环境光干扰
sensor.set_auto_gain(False)            # 关闭自动增益，防止暗处噪点放大
sensor.set_auto_whitebal(False)        # 关闭白平衡，防止色温漂移导致“认错颜色”
sensor.set_auto_exposure(False, exposure_us=900) # 手动锁定曝光时间(8ms)
                                       # 注意：采用较暗的视角，以提升准确率
                                       # 这能大幅抑制 PVC 场地的反光亮斑

sensor.set_framerate(60)               # 目标帧率（解除软件限速）
clock = time.clock()                   # 帧率计时器

# 初始化 UART12 串口通讯 (TX接主控RX，RX接主控TX)
uart = UART(12, baudrate=115200)
frame_cnt = 0                          # 系统总帧数计数器

# ======================================================================
# 2. 通讯协议定义与底层函数
# ======================================================================
PROTO_SOF1          = 0xAA
PROTO_SOF2          = 0x55
PROTO_TYPE_MAP      = 0x01
PROTO_TYPE_HB       = 0x10
HB_INTERVAL_MS      = 100

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
    crc = 0
    tbl = _CRC8_TABLE
    for b in data: crc = tbl[crc ^ b]
    return crc

def pack_frame(type_byte, payload):
    if not isinstance(payload, (bytes, bytearray)): payload = bytes(payload)
    length = len(payload)
    body = bytes((type_byte & 0xFF, length & 0xFF)) + payload
    return bytes((PROTO_SOF1, PROTO_SOF2)) + body + bytes((crc8(body),))

_hb_seq = 0
_last_hb_ms = time.ticks_ms()

def send_heartbeat_if_due():
    global _hb_seq, _last_hb_ms
    now = time.ticks_ms()
    if time.ticks_diff(now, _last_hb_ms) >= HB_INTERVAL_MS:
        _hb_seq = (_hb_seq + 1) & 0xFF
        try: uart.write(pack_frame(PROTO_TYPE_HB, bytes((_hb_seq,))))
        except: pass
        _last_hb_ms = now


# 调试开关：控制是否绘制采样 ROI 的矩形边框用于视觉调试
# - 在比赛或正式运行时建议设为 False 以节省绘制开销
# - 在开发或现场标定时设为 True 便于观察每个网格的采样区域
DEBUG_DRAW_ROI = True
# ----------------------------------------------------------------------
# 3. 网格采样与逆透视映射
# ----------------------------------------------------------------------
ROWS, COLS = 12, 16

# 场地四角外侧格子的中心点坐标（需根据实际场地微调）
GRID_CORNERS = {
    "tl": (19, 47.0),  # 左上
    "tr": (260, 30.0), # 右上
    "bl": (27.0, 227.0), # 左下
    "br": (265.0, 227.0),# 右下
}

GRID_K1 = +0.000000
CALIB_SHOW_CORNERS = True
CAR_VOTE_FRAMES = 5
CAR_VOTE_MIN = 2
car_vote_hist = []

def calc_grid_point(x_idx, y_idx, img_w, img_h):
    """基于四角点进行双线性插值，计算网格真实物理坐标映射到像素的坐标"""
    u = x_idx / (COLS - 1)
    v = y_idx / (ROWS - 1)
    # ========================================================
    # 【新增】：极速非线性透视/畸变补偿 (抛物线推拉)
    # 作用：在不影响 0 和 1 这两端边框的前提下，专门对中间的网格进行推拉。
    # 针对你“中间偏右”的问题，我们给 U 加上一个负的补偿值把它向左拉。
    # ========================================================
    COMP_U = -0.06  # 左右补偿系数：负数向左拉，正数向右推 (建议从 -0.02 到 -0.08 之间微调)
    COMP_V = +0.02   # 上下补偿系数：如果中间行偏上或偏下，同样调这个值

    u = u + COMP_U * u * (1.0 - u)
    v = v + COMP_V * v * (1.0 - v)
    # ========================================================

    tl_x, tl_y = GRID_CORNERS["tl"]
    tr_x, tr_y = GRID_CORNERS["tr"]
    bl_x, bl_y = GRID_CORNERS["bl"]
    br_x, br_y = GRID_CORNERS["br"]

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
    """绘制标定框，帮助手动对齐摄像头视角"""
    tl = calc_grid_point(0, 0, img_w, img_h)
    tr = calc_grid_point(COLS - 1, 0, img_w, img_h)
    bl = calc_grid_point(0, ROWS - 1, img_w, img_h)
    br = calc_grid_point(COLS - 1, ROWS - 1, img_w, img_h)

    img.draw_line(tl[0], tl[1], tr[0], tr[1], color=(255, 80, 80), thickness=1)
    img.draw_line(tr[0], tr[1], br[0], br[1], color=(80, 255, 80), thickness=1)
    img.draw_line(br[0], br[1], bl[0], bl[1], color=(80, 160, 255), thickness=1)
    img.draw_line(bl[0], bl[1], tl[0], tl[1], color=(255, 220, 80), thickness=1)
    return tl, tr, bl, br

def vote_car_position(found, x, y):
    """多数投票机制：防抖，防止车辆坐标在两帧之间反复横跳"""
    global car_vote_hist
    if found:
        car_vote_hist.append((x, y))
        if len(car_vote_hist) > CAR_VOTE_FRAMES:
            car_vote_hist.pop(0)

    if not car_vote_hist: return x, y

    best_pos, best_count = car_vote_hist[-1], 0
    for pos in car_vote_hist:
        count = sum(1 for other in car_vote_hist if pos == other)
        if count > best_count:
            best_pos, best_count = pos, count

    return best_pos if best_count >= CAR_VOTE_MIN else car_vote_hist[-1]


# ----------------------------------------------------------------------
# 4. 颜色特征库与极速特征匹配算法 (全新优化核心 - 多维统计特征法)
# ----------------------------------------------------------------------
# ！！这部分 RGB 值依然需要你在比赛现场根据实际光照进行重新采样修改！！
CAR_HEAD_DARK_RGB = (25, 152, 0)      # 车头（H）
CAR_HEAD_BRIGHT_RGB = (30, 255, 255)
CAR_TAIL_DARK_RGB = (0, 152, 195)     # 车尾（T）
CAR_TAIL_BRIGHT_RGB = (58, 247, 16)
WALL_DARK_RGB = (41, 61, 66)          # 墙壁（#）
WALL_BRIGHT_RGB = (107, 170, 255)
FLOOR_DARK_RGB = (33, 12, 255)        # 空地（-）
FLOOR_BRIGHT_RGB = (49, 97, 255)
GOAL_DARK_RGB = (173, 0, 255)         # 终点（.）
GOAL_BRIGHT_RGB = (255, 32, 255)
BOX_DARK_RGB = (99, 138, 0)           # 箱子（$）
BOX_BRIGHT_RGB = (247, 255, 66)
BOMB_DARK_RGB = (181, 28, 58)         # 炸弹（*）
BOMB_BRIGHT_RGB = (255, 40, 82)

# 颜色匹配的置信度限制：保留现有实测色值，只阻止“仅仅相对更像炸弹”的格子被强制判为炸弹。
COLOR_UNKNOWN_MAX_DIST = 100.0
BOMB_MAX_MATCH_DIST = 45.0
BOMB_MIN_LEAD_DIST = 15.0
WALL_TEXTURE_L_STDEV = 15.0
WALL_TEXTURE_MAX_DIST = 70.0
WALL_TEXTURE_BONUS_DIST = 15.0

# 【架构保留】：依然保留所有靶点字典，作为纯色匹配和特征检测失败时的安全垫 (Fallback)
SYMBOL_MAP_RGB = {
    "#": (WALL_DARK_RGB, WALL_BRIGHT_RGB),
    "-": (FLOOR_DARK_RGB, FLOOR_BRIGHT_RGB),
    ".": (GOAL_DARK_RGB, GOAL_BRIGHT_RGB),
    "$": (BOX_DARK_RGB, BOX_BRIGHT_RGB),
    "*": (BOMB_DARK_RGB, BOMB_BRIGHT_RGB),
    # "H": (CAR_HEAD_DARK_RGB, CAR_HEAD_BRIGHT_RGB),
    # "T": (CAR_TAIL_DARK_RGB, CAR_TAIL_BRIGHT_RGB),
}

def _pivot_rgb_to_linear(c):
    if c <= 0.04045: return c / 12.92
    return ((c + 0.055) / 1.055) ** 2.4

def rgb_to_lab(rgb):
    """标准的 RGB 到 LAB 转换算法，用于将预设值转为目标靶点"""
    r, g, b = [_pivot_rgb_to_linear(v / 255.0) for v in rgb]
    x = r * 0.4124564 + g * 0.3575761 + b * 0.1804375
    y = r * 0.2126729 + g * 0.7151522 + b * 0.0721750
    z = r * 0.0193339 + g * 0.1191920 + b * 0.9503041

    xr, yr, zr = x / 0.95047, y / 1.00000, z / 1.08883
    eps, kappa = 0.008856, 903.3

    def _f(t): return t ** (1.0 / 3.0) if t > eps else (kappa * t + 16.0) / 116.0

    fx, fy, fz = _f(xr), _f(yr), _f(zr)
    l = 116.0 * fy - 16.0
    a = 500.0 * (fx - fy)
    bb = 200.0 * (fy - fz)
    return (l, a, bb)

# 【优化】系统初始化时，预先计算所有颜色模板的 LAB 靶向值
SYMBOL_MAP_LABTarget = {}
for sym, tpls in SYMBOL_MAP_RGB.items():
    SYMBOL_MAP_LABTarget[sym] = [rgb_to_lab(rgb) for rgb in tpls]


def classify_symbol_by_features(l_mode, a_mode, b_mode, l_stdev):
    """
    使用现有 LAB 色值模板匹配格子，并对墙和炸弹增加置信度约束。

    炸弹必须既足够接近炸弹模板，又明显优于其他类别；否则退回最接近的
    非炸弹类别。具有明暗纹理的区域在墙颜色距离合理时，给墙有限优先级。
    """
    best_sym = "-"
    min_dist = 999999.0
    best_non_bomb_sym = "-"
    best_non_bomb_dist = 999999.0
    best_non_wall_dist = 999999.0
    wall_dist = 999999.0
    bomb_dist = 999999.0

    for sym, lab_targets in SYMBOL_MAP_LABTarget.items():
        for target in lab_targets:
            tl, ta, tb = target
            # 计算加权欧氏距离（L降权处理，抵抗光斑）
            dist = math.sqrt(0.2 * (l_mode - tl)**2 + (a_mode - ta)**2 + (b_mode - tb)**2)

            if dist < min_dist:
                min_dist = dist
                best_sym = sym

            if sym != "*" and dist < best_non_bomb_dist:
                best_non_bomb_dist = dist
                best_non_bomb_sym = sym

            if sym != "#" and dist < best_non_wall_dist:
                best_non_wall_dist = dist

            if sym == "#" and dist < wall_dist:
                wall_dist = dist
            elif sym == "*" and dist < bomb_dist:
                bomb_dist = dist

    # 距离阈值保护：如果偏离所有已知色块过大，判定为背景空地
    if min_dist > COLOR_UNKNOWN_MAX_DIST:
        return "-"

    bomb_is_confident = (
        bomb_dist <= BOMB_MAX_MATCH_DIST and
        bomb_dist + BOMB_MIN_LEAD_DIST <= best_non_bomb_dist
    )
    if bomb_is_confident:
        return "*"

    # 亮度离散是墙的辅助证据，只允许它在颜色距离接近时纠正结果。
    wall_has_texture = l_stdev > WALL_TEXTURE_L_STDEV
    wall_color_is_plausible = (
        wall_dist <= WALL_TEXTURE_MAX_DIST and
        wall_dist <= best_non_wall_dist + WALL_TEXTURE_BONUS_DIST
    )
    if wall_has_texture and wall_color_is_plausible:
        return "#"

    # 炸弹没有通过严格门槛时，不再因“相对最近”而输出炸弹。
    if best_sym == "*":
        if best_non_bomb_dist <= COLOR_UNKNOWN_MAX_DIST:
            return best_non_bomb_sym
        return "-"

    return best_sym

def classify_cell(img, x, y, img_w, img_h):
    """
    【算力解放入口】：多维统计特征判别器 (O(1) 复杂度)
    彻底废弃 Python 层面的像素遍历！利用底层硬件加速，通过标准差(stdev)和众数(mode)
    实现对“纹理”、“双色”和“纯色”的降维打击分类。
    """
    radius = 4.5  # 采样半径 5 = 11x11 范围 (共 121 个像素点一起统计)

    # 严格的边界防护，防止图像框画到屏幕外面导致死机报错
    x_min = max(0, int(x - radius))
    y_min = max(0, int(y - radius))
    x_max = min(img_w - 1, int(x + radius))
    y_max = min(img_h - 1, int(y + radius))
    w = x_max - x_min + 1
    h = y_max - y_min + 1

    if w <= 0 or h <= 0:
        return "-"

    # 【一次提取，全部搞定】：极速底层 API 调用，获取该 ROI 内的所有统计学特征
    stats = img.get_statistics(roi=(x_min, y_min, w, h))

    # 调试可视化：在 ROI 周围绘制方框边界
    if DEBUG_DRAW_ROI:
        try:
            img.draw_rectangle(x_min, y_min, w, h, color=(255, 255, 255))
        except Exception:
            try:
                img.draw_line(x_min, y_min, x_min + w - 1, y_min, color=(255, 255, 255))
                img.draw_line(x_min + w - 1, y_min, x_min + w - 1, y_min + h - 1, color=(255, 255, 255))
                img.draw_line(x_min + w - 1, y_min + h - 1, x_min, y_min + h - 1, color=(255, 255, 255))
                img.draw_line(x_min, y_min + h - 1, x_min, y_min, color=(255, 255, 255))
            except Exception:
                pass

    # ======================================================================
    # 【高阶特征判别树】：优先级与双重校验优化版
    # ======================================================================

    # 提前获取众数（无论走哪个分支都会用到，放前面不浪费算力）
    l_mode = stats.l_mode()
    a_mode = stats.a_mode()
    b_mode = stats.b_mode()

    # 【防误判策略 1：优先级反转 + 严苛指纹】-> 唯一合法的小车判定点
    # 解释：
    # 1. a_mean < -5   : 整体必须偏青/绿
    # 2. b_stdev > 15  : 必须同时跨越蓝色(青)和黄色(绿)，产生极大离散
    # 3. a_stdev > 5   : 青色和绿色在 A 通道上也有一定差异，防止单色伪装
    # 4. l_mean > 25   : 亮度不能太暗！墙角的纯黑噪点极容易产生极端的离散值，直接过滤！

    if stats.a_mean() < -5 and stats.b_stdev() > 15 and stats.a_stdev() > 5 and stats.l_mean() > 25:
        return "H"

    # 墙和炸弹在一次模板遍历中完成颜色匹配与置信度判定。
    return classify_symbol_by_features(
        l_mode, a_mode, b_mode, stats.l_stdev())


# ----------------------------------------------------------------------
# 5. 车辆坐标解析逻辑（单点识别，整张地图仅允许一辆车）
# ----------------------------------------------------------------------
def find_car_single(map_list):
    """
    在16x12网格中寻找车辆：不再要求车头(H)和车尾(T)相邻，
    只要识别到任意一个 H 或 T 即判定为车辆位置。

    整张地图只允许一辆车；如果检测到多个 H/T，按行优先
    （从左上到右下）选择第一个作为车辆位置。
    """
    for y in range(ROWS):
        for x in range(COLS):
            ch = map_list[y * COLS + x]
            if ch == "H" or ch == "T":
                return (x, y)
    return None

def build_map_with_single_car(map_list, car_found, car_x, car_y):
    """输出洗牌：在发给 STM32 的包中，抹去H/T，统一换成唯一标志符 @ """
    merged = []
    for ch in map_list:
        if ch == "H" or ch == "T": merged.append("-")
        else: merged.append(ch)

    if car_found and 0 <= car_x < COLS and 0 <= car_y < ROWS:
        merged[car_y * COLS + car_x] = "@"
    return merged


# ======================================================================
# 6. 系统主循环 (正常运行)
# ======================================================================
while(True):
    clock.tick()
    img = sensor.snapshot()
    frame_cnt += 1

    img_w, img_h = img.width(), img.height()
    map_list = []
    car_x, car_y = 225, 225    #没识别到小车时，发送无效坐标，以防止主控误判车的位置
    car_found = False
    tl_pt = tr_pt = bl_pt = br_pt = None

    # --- 阶段 A：扫描解析赛道 ---
    for y_idx in range(ROWS):
        for x_idx in range(COLS):
            # 获取物理逆透视坐标点
            tx, ty = calc_grid_point(x_idx, y_idx, img_w, img_h)

            if 0 <= tx < img_w and 0 <= ty < img_h:
                # 获取该点 11x11 范围的元素分类
                char = classify_cell(img, tx, ty, img_w, img_h)
                map_list.append(char)
            else:
                map_list.append("-")

    # --- 阶段 B：坐标解算 ---
    # 新逻辑：不再寻找相邻的 H/T 配对，只要检测到 H 或 T 即判定为车辆（整张地图只允许一辆车）
    car_single = find_car_single(map_list)
    if car_single is not None:
        car_x, car_y = car_single
        car_found = True

    # 绘制外圈标定线框
    if CALIB_SHOW_CORNERS:
        tl_pt, tr_pt, bl_pt, br_pt = draw_calibration_overlay(img, img_w, img_h)

    # 车辆坐标防抖滤波
    car_x, car_y = vote_car_position(car_found, car_x, car_y)

    # 组装最终给主控的单@字符地图
    map_list_out = build_map_with_single_car(map_list, car_found, car_x, car_y)

    # --- 阶段 C：串口打包发送 (194 字节全场通讯) ---
    try:
        map_bytes = "".join(map_list_out).encode("ascii")
        payload = map_bytes + bytes([car_x, car_y])
        if len(payload) == 194:
            uart.write(pack_frame(PROTO_TYPE_MAP, payload))
    except Exception as e:
        print("UART TX Error:", e)

    # 发送系统心跳
    send_heartbeat_if_due()

    # --- 阶段 D：终端监控防阻塞 (极度重要) ---
    # 【改动】：每 10 帧更新一次 IDE 打印信息。
    # 绝对禁止每帧打印！I/O 阻塞会直接卡死摄像头进程导致帧率断崖下跌。
    if frame_cnt % 10 == 0:
        print("\033[H", end="") # 清屏
        print("FPS: %0.1f | 小车坐标: (%d, %d)" % (clock.fps(), car_x, car_y))
        if CALIB_SHOW_CORNERS and tl_pt is not None:
            print("基准: TL=%s TR=%s BL=%s BR=%s" % (tl_pt, tr_pt, bl_pt, br_pt))

        # 打印字符地图阵列
        for r in range(ROWS):
            print("".join(map_list_out[r*COLS : (r+1)*COLS]))

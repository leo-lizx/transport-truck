# OpenART Plus / OpenMV 最小验证：物块中心标定 + 实时追踪
#
# 用法：
#   1. 物块放到画面中心，运行本脚本；
#   2. 屏幕提示 LOCKED 后，标定完成（记录质心与颜色阈值）；
#   3. 移动物块，屏幕实时显示相对标定中心的 dx / dy / 距离（像素）。
#
# 说明：
#   - 只做识别与相对距离，不做毫米标定、不接 UART；
#   - 标定 = 记录一次质心 + 从该物块学习 LAB 颜色阈值；
#   - 重新标定：停止脚本再运行一次即可；
#   - 屏幕文字与串口输出保持 ASCII，中文只出现在注释里。

import sensor, image, time, math

# ------------------------- 可调参数 -------------------------
FRAME_W = 320
FRAME_H = 240
CENTER_X = FRAME_W // 2
CENTER_Y = FRAME_H // 2

CENTER_TOL_PX = 30        # 标定时物块质心距画面中心的最大偏差
CALIB_STABLE_FRAMES = 12  # 连续稳定多少帧后锁定
STABLE_DELTA_PX = 1.5     # 相邻帧质心变化小于该值才算稳定
LOST_FRAMES = 12          # 连续丢失多少帧判定 LOST
AREA_MIN = 200            # 候选最小面积（像素）
AREA_MAX = 9000           # 候选最大面积（像素）
DENSITY_MIN = 0.45        # 像素数 / 外接矩形面积
ASPECT_MIN = 0.5          # 外接矩形宽高比下限
ASPECT_MAX = 2.0          # 外接矩形宽高比上限
PX_PER_MM = 0.0           # 已知像素当量时填入，0 表示只输出像素

# 任意高饱和颜色的粗阈值，仅用于第一次找到物块（L, A, B 六元组）
BROAD_THRESHOLDS = [
    (10, 95, 15, 127, -128, 127),    # 偏红/品红
    (10, 95, -128, -15, -128, 127),  # 偏绿
    (10, 95, -128, 127, 15, 127),    # 偏黄
    (10, 95, -128, 127, -128, -15),  # 偏蓝
]

# ------------------------- 工具函数 -------------------------
def clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def blob_center(b):
    """优先用矩计算亚像素质心，失败时退回整数质心。"""
    try:
        m = b.moments()
        if m[0] > 0:
            return (m[1] / m[0], m[2] / m[0])
    except Exception:
        pass
    return (float(b.cx()), float(b.cy()))


def shape_ok(b):
    if b.w() <= 0 or b.h() <= 0:
        return False
    aspect = b.w() / b.h()
    density = b.pixels() / (b.w() * b.h())
    return (ASPECT_MIN <= aspect <= ASPECT_MAX) and (density >= DENSITY_MIN)


def find_candidates(img, thresholds, roi=None):
    out = []
    for th in thresholds:
        if roi is None:
            blobs = img.find_blobs([th], x_stride=2, y_stride=2,
                                   area_threshold=AREA_MIN,
                                   pixels_threshold=AREA_MIN,
                                   merge=True, margin=6)
        else:
            blobs = img.find_blobs([th], roi=roi, x_stride=2, y_stride=2,
                                   area_threshold=AREA_MIN,
                                   pixels_threshold=AREA_MIN,
                                   merge=True, margin=6)
        for b in blobs:
            if AREA_MIN <= b.pixels() <= AREA_MAX and shape_ok(b):
                out.append(b)
    return out


def pick_nearest(blobs, px, py):
    best = None
    best_d2 = None
    for b in blobs:
        cx, cy = blob_center(b)
        d2 = (cx - px) * (cx - px) + (cy - py) * (cy - py)
        if best_d2 is None or d2 < best_d2:
            best = b
            best_d2 = d2
    return best


def sample_lab(img, rect):
    """取矩形内一半区域，返回 LAB 的均值与标准差。优先 get_statistics。"""
    x, y, w, h = rect
    x0 = x + w // 4
    y0 = y + h // 4
    sw = max(4, w // 2)
    sh = max(4, h // 2)
    x0 = clamp(x0, 0, FRAME_W - 1)
    y0 = clamp(y0, 0, FRAME_H - 1)
    sw = clamp(sw, 1, FRAME_W - x0)
    sh = clamp(sh, 1, FRAME_H - y0)

    try:
        st = img.get_statistics(roi=(x0, y0, sw, sh))
        return (st.l_mean(), st.a_mean(), st.b_mean(),
                max(0.0, st.l_stdev()), max(0.0, st.a_stdev()),
                max(0.0, st.b_stdev()))
    except Exception:
        pass

    ls = []
    afs = []
    bfs = []
    step = max(1, min(sw, sh) // 8)
    for yy in range(y0, y0 + sh, step):
        for xx in range(x0, x0 + sw, step):
            try:
                rgb = img.get_pixel(xx, yy)
                l, a, b = image.rgb_to_lab(rgb)
                ls.append(l)
                afs.append(a)
                bfs.append(b)
            except Exception:
                pass
    n = len(ls)
    if n == 0:
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def mean(v):
        return sum(v) / len(v)

    def std(v, m):
        s = 0.0
        for x in v:
            s += (x - m) * (x - m)
        return math.sqrt(s / len(v))

    lm = mean(ls)
    am = mean(afs)
    bm = mean(bfs)
    return (lm, am, bm, std(ls, lm), std(afs, am), std(bfs, bm))


def build_threshold(stats):
    lm, am, bm, ls, as_, bs = stats
    Lmin = clamp(lm - 3.0 * ls, 0, 100)
    Lmax = clamp(lm + 3.0 * ls, 0, 100)
    Amin = clamp(am - 2.5 * as_, -128, 127)
    Amax = clamp(am + 2.5 * as_, -128, 127)
    Bmin = clamp(bm - 2.5 * bs, -128, 127)
    Bmax = clamp(bm + 2.5 * bs, -128, 127)

    # 纯色物块标准差很小，保底给一点宽度，避免阈值退化
    Lmin = min(Lmin, lm - 8)
    Lmax = max(Lmax, lm + 8)
    Amin = min(Amin, am - 8)
    Amax = max(Amax, am + 8)
    Bmin = min(Bmin, bm - 8)
    Bmax = max(Bmax, bm + 8)

    return (int(clamp(Lmin, 0, 100)), int(clamp(Lmax, 0, 100)),
            int(clamp(Amin, -128, 127)), int(clamp(Amax, -128, 127)),
            int(clamp(Bmin, -128, 127)), int(clamp(Bmax, -128, 127)))


def draw_overlay(img, target, cur, status, found):
    # 标定中心十字与目标框
    tx = int(target[0])
    ty = int(target[1])
    img.draw_cross(tx, ty, size=8, color=(255, 255, 255), thickness=1)
    img.draw_rectangle(tx - 15, ty - 15, 30, 30, color=(255, 255, 255))

    if found and cur is not None:
        cx = int(cur[0])
        cy = int(cur[1])
        img.draw_cross(cx, cy, size=6, color=(0, 255, 0), thickness=2)
        img.draw_line(tx, ty, cx, cy, color=(0, 255, 0), thickness=1)
        dx = cur[0] - target[0]
        dy = cur[1] - target[1]
        dist = math.sqrt(dx * dx + dy * dy)
        if PX_PER_MM > 0:
            text = "dx=%.1f dy=%.1f d=%.1fpx %.1fmm" % (dx, dy, dist, dist / PX_PER_MM)
        else:
            text = "dx=%.1f dy=%.1f d=%.1fpx" % (dx, dy, dist)
        img.draw_string(4, 4, text, color=(255, 255, 0), scale=1)
    else:
        img.draw_string(4, 4, "BLOCK LOST", color=(255, 0, 0), scale=1)

    img.draw_string(4, FRAME_H - 14, status, color=(0, 255, 255), scale=1)


# ------------------------- 主流程 -------------------------
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=1500)
clock = time.clock()

print("PLACE BLOCK AT CENTER")

target = None
threshold = None
history = []
stable = 0
last = None
lost = 0
cur = None

while True:
    clock.tick()
    img = sensor.snapshot()

    if target is None:
        # ---------- 标定阶段 ----------
        cands = find_candidates(img, BROAD_THRESHOLDS)
        best = pick_nearest(cands, CENTER_X, CENTER_Y)
        if best is not None:
            cx, cy = blob_center(best)
            if abs(cx - CENTER_X) <= CENTER_TOL_PX and abs(cy - CENTER_Y) <= CENTER_TOL_PX:
                if last is not None and abs(cx - last[0]) <= STABLE_DELTA_PX \
                        and abs(cy - last[1]) <= STABLE_DELTA_PX:
                    stable += 1
                else:
                    stable = 0
                    history = []
                last = (cx, cy)
                history.append((cx, cy))
                if len(history) > CALIB_STABLE_FRAMES:
                    history.pop(0)
                if stable >= CALIB_STABLE_FRAMES - 1:
                    n = len(history)
                    tx = sum(p[0] for p in history) / n
                    ty = sum(p[1] for p in history) / n
                    target = (tx, ty)
                    threshold = build_threshold(sample_lab(img, best.rect()))
                    cur = target
                    print("LOCKED target=(%.1f,%.1f) th=%s" % (tx, ty, str(threshold)))
                    if abs(threshold[2] + threshold[3]) < 16 and \
                            abs(threshold[4] + threshold[5]) < 16:
                        print("WARN low saturation, color threshold may be unstable")
                img.draw_cross(int(cx), int(cy), size=6, color=(0, 255, 0), thickness=2)
            else:
                stable = 0
                history = []
                last = None
        else:
            stable = 0
            history = []
            last = None

        img.draw_cross(CENTER_X, CENTER_Y, size=10, color=(255, 0, 0))
        img.draw_string(4, 4, "PLACE BLOCK AT CENTER %d/%d" % (stable, CALIB_STABLE_FRAMES),
                        color=(255, 255, 0), scale=1)
        if target is not None:
            draw_overlay(img, target, cur, "TRACK", True)
        continue

    # ---------- 追踪阶段 ----------
    cands = find_candidates(img, [threshold])
    best = None
    if cands:
        # 有上一帧位置时优先跟最近的候选；重新捕获时以目标点为准，不做跳变门限
        ref = last if last is not None else target
        best = pick_nearest(cands, ref[0], ref[1])
        if best is not None and last is not None:
            bx, by = blob_center(best)
            d2 = (bx - last[0]) * (bx - last[0]) + (by - last[1]) * (by - last[1])
            if d2 > 120 * 120:
                best = None

    if best is not None:
        cx, cy = blob_center(best)
        cur = (cx, cy)
        last = cur
        lost = 0
        dx = cx - target[0]
        dy = cy - target[1]
        dist = math.sqrt(dx * dx + dy * dy)
        print("%d,1,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%.1f" %
              (time.ticks_ms(), cx, cy, dx, dy, dist, best.pixels(), clock.fps()))
    else:
        lost += 1
        if lost > LOST_FRAMES:
            cur = None
            last = None
            print("%d,0,-1,-1,-1,-1,-1,0,%.1f" %
                  (time.ticks_ms(), clock.fps()))

    status = "TRACK" if cur is not None else "LOST"
    draw_overlay(img, target, cur, status, cur is not None)

print("done")

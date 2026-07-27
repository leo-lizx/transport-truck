from machine import UART
import sensor, image, time, math
import os, tf, gc

# ==========================================
# 0. 主控通信协议
# ==========================================
def crc8_update(crc, byte):
    crc ^= byte
    for _ in range(8):
        if crc & 0x80:
            crc = (crc << 1) ^ 0x07
        else:
            crc <<= 1
        crc &= 0xFF
    return crc

def crc8(data):
    crc = 0x00
    for byte in data:
        crc = crc8_update(crc, byte)
    return crc

def pack_frame(type_byte, payload):
    if not isinstance(payload, (bytes, bytearray)):
        payload = bytes(payload)
    length = len(payload)
    body = bytes((type_byte & 0xFF, length & 0xFF)) + payload
    return bytes((0xAA, 0x55)) + body + bytes((crc8(body),))

# ==========================================
# 1. 硬件初始化
# ==========================================
uart = UART(12, 115200, timeout_char=1000)

TYPE_BOX_CLASS = 0x02
TYPE_HEARTBEAT = 0x10
TYPE_RECOG_REQUEST = 0x11

OBJ_KIND_BOX = 0
OBJ_KIND_TARGET = 1
REQUEST_PAYLOAD_LEN = 2

# 主控请求帧: AA 55 11 02 obj_kind request_id CRC8。
# OpenART2 只运行主控指定的 BOX 或 TARGET 模型。
request_rx_state = 0
request_rx_type = 0
request_rx_len = 0
request_rx_index = 0
request_rx_crc = 0
request_rx_payload = bytearray(REQUEST_PAYLOAD_LEN)
active_request_kind = OBJ_KIND_BOX
active_request_id = 0

def feed_request_byte(byte):
    global request_rx_state, request_rx_type, request_rx_len
    global request_rx_index, request_rx_crc
    global active_request_kind, active_request_id

    if request_rx_state == 0:
        if byte == 0xAA:
            request_rx_state = 1
    elif request_rx_state == 1:
        if byte == 0x55:
            request_rx_state = 2
        elif byte != 0xAA:
            request_rx_state = 0
    elif request_rx_state == 2:
        request_rx_type = byte
        request_rx_crc = crc8_update(0x00, byte)
        request_rx_state = 3
    elif request_rx_state == 3:
        request_rx_len = byte
        request_rx_crc = crc8_update(request_rx_crc, byte)
        request_rx_index = 0
        request_rx_state = 4 if request_rx_len == REQUEST_PAYLOAD_LEN else 0
    elif request_rx_state == 4:
        request_rx_payload[request_rx_index] = byte
        request_rx_crc = crc8_update(request_rx_crc, byte)
        request_rx_index += 1
        if request_rx_index >= request_rx_len:
            request_rx_state = 5
    else:
        if (byte == request_rx_crc and
            request_rx_type == TYPE_RECOG_REQUEST and
            request_rx_payload[0] in (OBJ_KIND_BOX, OBJ_KIND_TARGET) and
            request_rx_payload[1] != 0):
            active_request_kind = request_rx_payload[0]
            active_request_id = request_rx_payload[1]
        request_rx_state = 0

def poll_recognition_request():
    available = uart.any()
    while available > 0:
        chunk = uart.read(available)
        if not chunk:
            return
        for byte in chunk:
            feed_request_byte(byte)
        available = uart.any()

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA) # 320x240

# 识别框坐标 (ROI)
ROI_X1 = 42
ROI_Y1 = 0
ROI_X2 = 320
ROI_Y2 = 199

if not (0 <= ROI_X1 < ROI_X2 <= 320 and 0 <= ROI_Y1 < ROI_Y2 <= 240):
    raise ValueError("ROI 坐标非法！")

RECOGNITION_ROI = (ROI_X1, ROI_Y1, ROI_X2 - ROI_X1, ROI_Y2 - ROI_Y1)

sensor.set_brightness(2000)
sensor.skip_frames(time = 20)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(True, (0, 0x80, 0))
clock = time.clock()
sensor.set_vflip(True)
sensor.set_auto_exposure(False, exposure_us=950)

vision_seq = 0
heartbeat_seq = 0
last_heartbeat_time = time.ticks_ms()

# ==========================================
# 2. 双检测模型与标签加载
# ==========================================
BOX_MODEL_PATH = "/sd/yolo3_renwu_iou_final_with_post_processing2.tflite"
TARGET_MODEL_PATH = "/sd/yolo3_iou_smartcar_final_with_post_processing_num.tflite"

print("=== 主控请求模式启动：正在加载 BOX/TARGET 检测模型 ===")
try:
    box_net = tf.load(BOX_MODEL_PATH, load_to_fb=True)
    target_net = tf.load(TARGET_MODEL_PATH, load_to_fb=True)
    print("BOX/TARGET 模型加载成功！")
except Exception as e:
    print("模型加载失败，请检查两个 tflite 文件是否位于 SD 卡根目录！")
    raise e

try:
    labels = [line.rstrip() for line in open("/sd/labels.txt")]
except:
    print("未找到 labels.txt，使用内置双模型标签列表")
    labels = [
        "00mickey_mouse", "01pikachu", "02spongebob_squarepants", "03pleasant_sheep",
        "04donald_duck", "05nezha", "06big_head_son", "07gg_bond",
        "08calabash_brothers", "09grey_wolf",
        "num_0", "num_1", "num_2", "num_3", "num_4",
        "num_5", "num_6", "num_7", "num_8", "num_9"
    ]

# labels.txt 仍按“10 个动漫人物 + 10 个数字”排列，仅用于屏幕显示；
# 两个检测模型的类别索引都独立从 0 开始，发送时统一映射为 1..10。
# 兼容 SD 卡上尚未删除 background 的旧 21 行标签文件。
if len(labels) == 21 and labels[10].lower() == "background":
    del labels[10]
if len(labels) != 20:
    raise ValueError("labels.txt 必须是 10 个动漫人物加 10 个数字")

CLASSES_PER_KIND = 10
model_nets = (box_net, target_net)
VOTE_CLASS_COUNT = CLASSES_PER_KIND
FAST_CONFIRM_CONFIDENCE = 0.60
FAST_CONFIRM_FRAMES = 2
VOTE_WINDOW_MS = 1000
VOTE_CONFIDENCE_THRESHOLD = 0.60
VOTE_WIN_MIN_COUNT = 2
VOTE_WIN_MIN_SHARE_PERCENT = 60

vote_request_kind = -1
vote_request_id = 0
vote_start_ms = 0
vote_counts = [0] * VOTE_CLASS_COUNT
vote_confidence_sums = [0.0] * VOTE_CLASS_COUNT
locked_vote_index = -1
locked_confidence = 0.0
vote_window_failed = False
consecutive_vote_index = -1
consecutive_count = 0
consecutive_confidence_sum = 0.0

def reset_vote_window(obj_kind, request_id, now_ms):
    global vote_request_kind, vote_request_id, vote_start_ms
    global vote_counts, vote_confidence_sums
    global locked_vote_index, locked_confidence
    global vote_window_failed
    global consecutive_vote_index, consecutive_count
    global consecutive_confidence_sum

    vote_request_kind = obj_kind
    vote_request_id = request_id
    vote_start_ms = now_ms
    vote_counts = [0] * VOTE_CLASS_COUNT
    vote_confidence_sums = [0.0] * VOTE_CLASS_COUNT
    locked_vote_index = -1
    locked_confidence = 0.0
    vote_window_failed = False
    consecutive_vote_index = -1
    consecutive_count = 0
    consecutive_confidence_sum = 0.0

def detect_model_best(net, roi_img):
    best_class_index = -1
    best_confidence = 0.0
    best_detection = None
    for detection in tf.detect(net, roi_img):
        x1, y1, x2, y2, class_index, confidence = detection
        class_index = int(class_index)
        if (0 <= class_index < CLASSES_PER_KIND and
            confidence > best_confidence):
            best_class_index = class_index
            best_confidence = confidence
            best_detection = (x1, y1, x2, y2)
    return best_class_index, best_confidence, best_detection

def select_vote_winner():
    winner = -1
    winner_votes = 0
    total_votes = 0
    for vote_index in range(VOTE_CLASS_COUNT):
        votes = vote_counts[vote_index]
        total_votes += votes
        if (votes > winner_votes or
            (votes == winner_votes and votes > 0 and winner >= 0 and
             vote_confidence_sums[vote_index] > vote_confidence_sums[winner])):
            winner = vote_index
            winner_votes = votes
    return winner, winner_votes, total_votes

# ==========================================
# 3. 主循环：按主控请求停车识别
# ==========================================
while(True):
    poll_recognition_request()
    clock.tick()
    img = sensor.snapshot()

    current_time = time.ticks_ms()
    if time.ticks_diff(current_time, last_heartbeat_time) >= 100:
        uart.write(pack_frame(TYPE_HEARTBEAT, bytes([heartbeat_seq])))
        heartbeat_seq = (heartbeat_seq + 1) % 256
        last_heartbeat_time = current_time

    # 尚未收到主控请求时不广播无归属的分类帧。
    if active_request_id == 0:
        img.draw_rectangle(RECOGNITION_ROI, color=(255, 255, 0), thickness=2)
        img.draw_string(10, 10, "WAIT REQUEST", color=(255, 255, 0), scale=2)
        gc.collect()
        continue

    # 主控每200ms可能重发同一请求；类型或编号变化时才重置窗口。
    if (vote_request_kind != active_request_kind or
        vote_request_id != active_request_id):
        reset_vote_window(active_request_kind, active_request_id,
                          time.ticks_ms())

    best_detection = None
    best_vote_index = -1
    best_confidence = 0.0
    should_send_result = False

    if locked_vote_index < 0 and not vote_window_failed:
        roi_img = img.copy(roi=RECOGNITION_ROI)
        # 主控已经根据地图指定物体类型，本帧只运行对应的一个模型。
        model_result = detect_model_best(model_nets[active_request_kind], roi_img)
        if (model_result[0] >= 0 and
            model_result[1] >= VOTE_CONFIDENCE_THRESHOLD):
            best_vote_index = model_result[0]
            best_confidence = model_result[1]
            best_detection = model_result[2]

        if best_vote_index >= 0:
            vote_counts[best_vote_index] += 1
            vote_confidence_sums[best_vote_index] += best_confidence

            if best_vote_index == consecutive_vote_index:
                consecutive_count += 1
                consecutive_confidence_sum += best_confidence
            else:
                consecutive_vote_index = best_vote_index
                consecutive_count = 1
                consecutive_confidence_sum = best_confidence

            if (consecutive_count >= FAST_CONFIRM_FRAMES and
                best_confidence >= FAST_CONFIRM_CONFIDENCE):
                locked_vote_index = consecutive_vote_index
                locked_confidence = (consecutive_confidence_sum /
                                     consecutive_count)
        else:
            consecutive_vote_index = -1
            consecutive_count = 0
            consecutive_confidence_sum = 0.0
        del roi_img

        if (locked_vote_index < 0 and
            time.ticks_diff(time.ticks_ms(), vote_start_ms) >= VOTE_WINDOW_MS):
            winner, winner_votes, total_votes = select_vote_winner()
            # 连续两帧未成立时，最多收集1秒内的同模型结果。
            if (winner >= 0 and
                winner_votes >= VOTE_WIN_MIN_COUNT and
                winner_votes * 100 >= total_votes * VOTE_WIN_MIN_SHARE_PERCENT):
                locked_vote_index = winner
                locked_confidence = (vote_confidence_sums[winner] /
                                     winner_votes)
                should_send_result = True
            else:
                vote_window_failed = True

    if locked_vote_index >= 0:
        result_obj_kind = active_request_kind
        class_id = locked_vote_index + 1
        label_index = active_request_kind * CLASSES_PER_KIND + locked_vote_index
        best_label = labels[label_index]
        best_confidence = locked_confidence
        box_color = (0, 255, 0)
        kind_name = "BOX" if result_obj_kind == OBJ_KIND_BOX else "TARGET"
        display_str = "%s C%d %.1f%%" % (kind_name, class_id,
                                          locked_confidence * 100)
        should_send_result = True
        print("FPS:%4.1f | REQ:%d | LOCK %-6s %-22s | %5.1f%% | CLASS:%d" %
              (clock.fps(), active_request_id, kind_name, best_label,
               locked_confidence * 100, class_id))
    else:
        winner, winner_votes, total_votes = select_vote_winner()
        box_color = (255, 255, 0)
        if vote_window_failed:
            best_label = "UNSTABLE"
            result_obj_kind = active_request_kind
            class_id = 0
            display_str = "FAIL %d/%d" % (winner_votes, total_votes)
            should_send_result = True
        elif winner >= 0:
            winner_class_id = winner + 1
            label_index = active_request_kind * CLASSES_PER_KIND + winner
            best_label = labels[label_index]
            kind_short = "B" if active_request_kind == OBJ_KIND_BOX else "T"
            display_str = "VOTE %s%d %d/%d" % (kind_short, winner_class_id,
                                                winner_votes, total_votes)
        else:
            best_label = "NONE"
            display_str = "VOTE NONE"
        print("FPS:%4.1f | REQ:%d | VOTE %-22s | %d/%d" %
              (clock.fps(), active_request_id, best_label,
               winner_votes, total_votes))

    img.draw_rectangle(RECOGNITION_ROI, color=box_color, thickness=2)
    if best_detection is not None:
        x1, y1, x2, y2 = best_detection
        detection_rect = (
            ROI_X1 + int(x1 * RECOGNITION_ROI[2]),
            ROI_Y1 + int(y1 * RECOGNITION_ROI[3]),
            int((x2 - x1) * RECOGNITION_ROI[2]),
            int((y2 - y1) * RECOGNITION_ROI[3])
        )
        img.draw_rectangle(detection_rect, color=box_color, thickness=2)
    img.draw_string(10, 10, display_str, color=box_color, scale=2)

    # 单帧低置信度不会发送 0；只有1秒汇总仍不稳定才回传 0。
    # 有效结论已经在视觉端确认，主控收到第一帧即可采信。
    if should_send_result:
        result_payload = bytes([result_obj_kind, class_id,
                                vision_seq, active_request_id])
        uart.write(pack_frame(TYPE_BOX_CLASS, result_payload))
        vision_seq = (vision_seq + 1) % 256

    gc.collect()

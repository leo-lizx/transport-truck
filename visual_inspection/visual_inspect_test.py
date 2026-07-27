# OpenMV 独立分类测试开关：只需修改这一行。
# 0 = 识别箱子图案，1 = 识别数字目标。
TEST_OBJ_KIND = 0

from machine import UART
import sensor, time, tf, gc

OBJ_KIND_BOX = 0
OBJ_KIND_TARGET = 1
MODEL_CLASS_COUNT = 20
BOX_CLASS_START = 0
BOX_CLASS_END = 10
TARGET_CLASS_START = 10
TARGET_CLASS_END = 20
CONFIDENCE_THRESHOLD = 0.70
TEST_REQUEST_ID = 1


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
    body = bytes((type_byte & 0xFF, len(payload) & 0xFF)) + payload
    return bytes((0xAA, 0x55)) + body + bytes((crc8(body),))


if TEST_OBJ_KIND == OBJ_KIND_BOX:
    candidate_start = BOX_CLASS_START
    candidate_end = BOX_CLASS_END
    test_kind_name = "BOX"
elif TEST_OBJ_KIND == OBJ_KIND_TARGET:
    candidate_start = TARGET_CLASS_START
    candidate_end = TARGET_CLASS_END
    test_kind_name = "TARGET"
else:
    raise ValueError("TEST_OBJ_KIND 必须为 0（箱子）或 1（数字目标）")


# UART(12) 为 OpenMV/OpenART 端口号，115200 与主控 UART1 保持一致。
uart = UART(12, 115200, timeout_char=1000)

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)

ROI_X1 = 42
ROI_Y1 = 0
ROI_X2 = 320
ROI_Y2 = 199

if not (0 <= ROI_X1 < ROI_X2 <= 320 and 0 <= ROI_Y1 < ROI_Y2 <= 240):
    raise ValueError("ROI 坐标必须在 320x240 画面内")

RECOGNITION_ROI = (ROI_X1, ROI_Y1, ROI_X2 - ROI_X1, ROI_Y2 - ROI_Y1)

sensor.set_brightness(2000)
sensor.skip_frames(time=20)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(True, (0, 0x80, 0))
sensor.set_vflip(True)
sensor.set_auto_exposure(False, exposure_us=950)
clock = time.clock()

print("正在加载 20 类模型...")
net = tf.load("smartcar_mobilenetv2_int8.tflite", load_to_fb=True)

try:
    labels = [line.rstrip() for line in open("/sd/labels.txt")]
except:
    print("未找到 labels.txt，使用内置 20 类标签")
    labels = [
        "00mickey_mouse",
        "01pikachu",
        "02spongebob_squarepants",
        "03pleasant_sheep",
        "04donald_duck",
        "05nezha",
        "06big_head_son",
        "07gg_bond",
        "08calabash_brothers",
        "09grey_wolf",
        "num_0",
        "num_1",
        "num_2",
        "num_3",
        "num_4",
        "num_5",
        "num_6",
        "num_7",
        "num_8",
        "num_9"
    ]

if len(labels) != MODEL_CLASS_COUNT:
    raise ValueError("labels.txt 必须恰好包含 20 个类别")

print("独立测试已启动：%s，无需等待主控请求" % test_kind_name)

vision_seq = 0
heartbeat_seq = 0
last_heartbeat_time = time.ticks_ms()

while True:
    clock.tick()
    img = sensor.snapshot()

    current_time = time.ticks_ms()
    if time.ticks_diff(current_time, last_heartbeat_time) >= 100:
        uart.write(pack_frame(0x10, bytes((heartbeat_seq,))))
        heartbeat_seq = (heartbeat_seq + 1) % 256
        last_heartbeat_time = current_time

    for result in tf.classify(net, img, roi=RECOGNITION_ROI):
        predictions = result.output()
        if len(predictions) != MODEL_CLASS_COUNT:
            raise ValueError("模型输出必须恰好包含 20 个类别")

        max_index = candidate_start
        for index in range(candidate_start + 1, candidate_end):
            if predictions[index] > predictions[max_index]:
                max_index = index

        max_confidence = predictions[max_index]
        best_label = labels[max_index]
        mapped_class_id = 0
        if max_confidence > CONFIDENCE_THRESHOLD:
            mapped_class_id = (max_index - candidate_start) + 1
        confidence_pct = int(max_confidence * 100)
        if confidence_pct < 0:
            confidence_pct = 0
        elif confidence_pct > 100:
            confidence_pct = 100

        print("%s | FPS: %5.1f | 类别: %s | 置信率: %.2f | 发送ID: %d" %
              (test_kind_name, clock.fps(), best_label,
               max_confidence, mapped_class_id))

        box_color = (0, 255, 0) if mapped_class_id != 0 else (255, 0, 0)
        img.draw_rectangle(RECOGNITION_ROI, color=box_color, thickness=2)
        img.draw_string(10, 10, "%s %s" % (test_kind_name, best_label),
                        color=(255, 0, 0), scale=2)
        img.draw_string(10, 34, "CONF: %.2f ID: %d" %
                        (max_confidence, mapped_class_id),
                        color=(255, 0, 0), scale=2)

        payload = bytes((TEST_OBJ_KIND, mapped_class_id, vision_seq,
                         TEST_REQUEST_ID, confidence_pct))
        uart.write(pack_frame(0x02, payload))
        vision_seq = (vision_seq + 1) % 256

    gc.collect()

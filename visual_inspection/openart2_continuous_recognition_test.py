import sensor, time, tf, gc

# OpenART2 独立连续识别测试：不等待主控请求，不发送比赛协议帧。
# 直接修改 TEST_KIND 选择测试人物或数字模型。

BOX_MODEL_PATH = "/sd/yolo3_renwu_iou_final_with_post_processing2.tflite"
TARGET_MODEL_PATH = "/sd/yolo3_iou_smartcar_final_with_post_processing_num.tflite"
LABELS_PATH = "/sd/labels.txt"

CLASSES_PER_KIND = 10
OBJ_KIND_BOX = 0
OBJ_KIND_TARGET = 1

# 测试箱子人物图案用 OBJ_KIND_BOX；测试目标点数字用 OBJ_KIND_TARGET。
TEST_KIND = OBJ_KIND_BOX

ROI_X1 = 42
ROI_Y1 = 0
ROI_X2 = 320
ROI_Y2 = 199
RECOGNITION_ROI = (ROI_X1, ROI_Y1, ROI_X2 - ROI_X1, ROI_Y2 - ROI_Y1)

DEFAULT_LABELS = [
    "00mickey_mouse", "01pikachu", "02spongebob_squarepants", "03pleasant_sheep",
    "04donald_duck", "05nezha", "06big_head_son", "07gg_bond",
    "08calabash_brothers", "09grey_wolf",
    "num_0", "num_1", "num_2", "num_3", "num_4",
    "num_5", "num_6", "num_7", "num_8", "num_9"
]


def load_labels():
    try:
        loaded_labels = [line.rstrip() for line in open(LABELS_PATH)]
    except:
        print("未找到 labels.txt，使用内置标签")
        loaded_labels = DEFAULT_LABELS

    # 兼容 SD 卡上仍带 background 的旧21行标签文件。
    if len(loaded_labels) == 21 and loaded_labels[10].lower() == "background":
        del loaded_labels[10]
    if len(loaded_labels) != 20:
        raise ValueError("labels.txt 必须是10个人物加10个数字")
    return loaded_labels


def detect_best(net, roi_img):
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


def draw_detection(img, detection, color):
    if detection is None:
        return

    x1, y1, x2, y2 = detection
    detection_rect = (
        ROI_X1 + int(x1 * RECOGNITION_ROI[2]),
        ROI_Y1 + int(y1 * RECOGNITION_ROI[3]),
        int((x2 - x1) * RECOGNITION_ROI[2]),
        int((y2 - y1) * RECOGNITION_ROI[3])
    )
    img.draw_rectangle(detection_rect, color=color, thickness=2)


sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_brightness(2000)
sensor.skip_frames(time=20)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(True, (0, 0x80, 0))
sensor.set_vflip(True)
sensor.set_auto_exposure(False, exposure_us=950)

clock = time.clock()
labels = load_labels()

if TEST_KIND == OBJ_KIND_BOX:
    active_model_path = BOX_MODEL_PATH
    active_label_offset = 0
    active_kind_name = "BOX"
elif TEST_KIND == OBJ_KIND_TARGET:
    active_model_path = TARGET_MODEL_PATH
    active_label_offset = CLASSES_PER_KIND
    active_kind_name = "TARGET"
else:
    raise ValueError("TEST_KIND 只能是 OBJ_KIND_BOX 或 OBJ_KIND_TARGET")

print("正在加载 %s 模型..." % active_kind_name)
try:
    active_net = tf.load(active_model_path, load_to_fb=True)
except Exception as error:
    print("模型加载失败，请检查 SD 卡中的 tflite 文件")
    raise error
print("模型加载成功，开始连续识别 %s" % active_kind_name)

while True:
    clock.tick()
    img = sensor.snapshot()
    roi_img = img.copy(roi=RECOGNITION_ROI)

    best_result = detect_best(active_net, roi_img)
    del roi_img

    img.draw_rectangle(RECOGNITION_ROI, color=(255, 255, 0), thickness=1)

    if best_result[0] >= 0:
        class_id = best_result[0] + 1
        label_name = labels[active_label_offset + best_result[0]]
        confidence_percent = best_result[1] * 100.0
        draw_detection(img, best_result[2], (0, 255, 0))
        img.draw_string(4, 4, "BEST %s C%02d %5.1f%%" %
                        (active_kind_name, class_id, confidence_percent),
                        color=(0, 255, 0), scale=1)
        img.draw_string(4, 22, label_name, color=(0, 255, 0), scale=1)
    else:
        class_id = 0
        label_name = "NONE"
        confidence_percent = 0.0
        img.draw_string(4, 4, "BEST NONE 0.0%",
                        color=(255, 0, 0), scale=1)

    img.draw_string(4, 204, "%s C%02d %5.1f%%" %
                    (active_kind_name, class_id, confidence_percent),
                    color=(255, 255, 255), scale=1)

    print("FPS:%4.1f | KIND:%-6s | C%02d %-22s | %5.1f%%" %
          (clock.fps(), active_kind_name,
           class_id, label_name, confidence_percent))

    gc.collect()

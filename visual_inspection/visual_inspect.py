from machine import UART  # <--- 修改这里：引入 machine 库中的 UART
import sensor, image, time, math
import os, tf, gc

# ==========================================
# 新增：CRC8 校验函数 (依据逐飞协议标准多项式 0x07)
# 这是主控判定数据包是否损坏的唯一标准，必须计算！
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

# ==========================================
# 新增：协议打包函数 (完全依据《视觉通信接口说明》第4点)
# 格式: AA 55 TYPE LEN PAYLOAD CRC8
# ==========================================
def pack_frame(type_byte, payload):
    # 确保 payload 是字节流格式
    if not isinstance(payload, (bytes, bytearray)):
        payload = bytes(payload)
    length = len(payload)
    # body 包含了参与 CRC 校验的部分：TYPE + LEN + PAYLOAD
    body = bytes((type_byte & 0xFF, length & 0xFF)) + payload
    # 最终拼接：帧头(AA 55) + body + CRC8校验码
    return bytes((0xAA, 0x55)) + body + bytes((crc8(body),))


# ==========================================
# 1. 硬件初始化设置
# ==========================================
# 初始化串口 12 (TX=B10, RX=B11) - 请核对 OpenART 的引脚定义
# 智能车比赛常用波特率为 115200
# 【新增批注】：MD文档中写明主控侧接口为 UART1_RX_B13，这里 UART(12) 是 OpenART 端的串口号，
# 只要物理引脚（TX连主控RX，RX连主控TX，GND连GND）接对即可。
uart = UART(12, 115200, timeout_char=1000)

TYPE_RECOG_REQUEST = 0x11
OBJ_KIND_BOX = 0
OBJ_KIND_TARGET = 1
REQUEST_PAYLOAD_LEN = 2

# 主控请求帧接收状态机。只有收到合法的 [obj_kind][request_id] 后才发送分类结果。
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
        if request_rx_len == REQUEST_PAYLOAD_LEN:
            request_rx_state = 4
        else:
            request_rx_state = 0
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
sensor.set_framesize(sensor.QVGA) # 保持 320x240 取景

# 识别框坐标（QVGA 320x240，画面左上角为 (0, 0)）。
# 只需修改下面 4 个数：左上角 (ROI_X1, ROI_Y1)，右下角 (ROI_X2, ROI_Y2)。
# 右上角为 (ROI_X2, ROI_Y1)，左下角为 (ROI_X1, ROI_Y2)。
ROI_X1 = 42
ROI_Y1 = 0
ROI_X2 = 320
ROI_Y2 = 199

if not (0 <= ROI_X1 < ROI_X2 <= 320 and 0 <= ROI_Y1 < ROI_Y2 <= 240):
    raise ValueError("ROI 坐标必须在 320x240 画面内，且右下角必须在左上角的右下方")

RECOGNITION_ROI = (ROI_X1, ROI_Y1, ROI_X2 - ROI_X1, ROI_Y2 - ROI_Y1)

sensor.set_brightness(2000)
sensor.skip_frames(time = 20)
# 保留逐飞历程中防止画面泛白的设置，这对强光环境有一定帮助
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(True, (0, 0x80, 0))
clock = time.clock()
sensor.set_vflip(True)    #上下翻转
sensor.set_auto_exposure(False, exposure_us=950)  #手动设置曝光时间

# ==========================================
# 新增：通信状态维护变量
# ==========================================
vision_seq = 0       # 视觉端自增序号，用于 BOX_CLASS 帧 (0-255循环)
heartbeat_seq = 0    # 心跳包自增序号
last_heartbeat_time = time.ticks_ms() # 记录上次发送心跳的时间

# ==========================================
# 2. 模型与标签加载 (仅加载一次)
# ==========================================
print("正在加载模型到帧缓冲区...")
net_path = "smartcar_mobilenetv2_int8.tflite" # 替换为你下载的模型名称
try:
    net = tf.load(net_path, load_to_fb=True)
    print("模型加载成功！")
except Exception as e:
    print("模型加载失败，请检查 SD 卡内是否有该文件，或是否内存不足！")
    raise e

# 方式 A：从文件加载标签 (推荐，和逐飞历程一致)
# 确保 SD 卡里有一个 labels.txt，里面按训练时的顺序写满 21 个类别的名字
try:
    labels = [line.rstrip() for line in open("/sd/labels.txt")]
except:
    # 方式 B：如果不想建 txt 文件，可以直接在这里把 21 个类别写死
    print("未找到 labels.txt，使用默认标签")
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
        "background",
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

# ==========================================
# 3. 主循环：停车打点识别
# ==========================================
while(True):
    poll_recognition_request()
    clock.tick()
    img = sensor.snapshot()

    # 【新增功能】：定期发送心跳帧 (TYPE=0x10)
    # 依据 MD 文档第 8 点，每 100ms 发送一次心跳，证明视觉端在线
    current_time = time.ticks_ms()
    if time.ticks_diff(current_time, last_heartbeat_time) >= 100:
        heartbeat_payload = bytes([heartbeat_seq])
        uart.write(pack_frame(0x10, heartbeat_payload))
        heartbeat_seq = (heartbeat_seq + 1) % 256 # 0-255 循环
        last_heartbeat_time = current_time

    # 没有主控请求时只维持取景和心跳，不广播无归属的分类结果。
    if active_request_id == 0:
        img.draw_rectangle(RECOGNITION_ROI, color=(255,255,0), thickness=2)
        gc.collect()
        continue

    # 模型只对矩形框内的图像进行分类。
    for obj in tf.classify(net, img, roi=RECOGNITION_ROI):

        # 获取所有类别的预测概率列表
        predictions = obj.output()

        # 只在主控指定的类别集合中找最大值；另一种物体不会参与本次结果竞争。
        if active_request_kind == OBJ_KIND_BOX:
            candidate_start = 0
            candidate_end = 10
        else:
            candidate_start = 11
            candidate_end = 21

        max_index = candidate_start
        for index in range(candidate_start + 1, candidate_end):
            if predictions[index] > predictions[max_index]:
                max_index = index
        max_confidence = predictions[max_index]
        best_label = labels[max_index]

        # 在终端打印，方便连着电脑时肉眼调试
        print("FPS: %5.1f | 识别结果: %s | 置信度: %.2f" % (clock.fps(), best_label, max_confidence))

        # 画框放在分类之后，避免绿色边框影响模型输入。
        img.draw_rectangle(RECOGNITION_ROI, color=(0,255,0), thickness=2)

        # 在屏幕左上角打印结果，方便看 LCD 屏调试
        img.draw_string(10, 10, "%s: %.2f" % (best_label, max_confidence), color=(255,0,0), scale=2)

        # ==========================================
        # 4. 串口通信：发送决策数据给主板 (已全面重构以适配新协议)
        # ==========================================

        mapped_obj_kind = active_request_kind
        mapped_class_id = 0 # 默认 0 表示未识别/背景

        # 指定集合的最佳结果必须同时超过 70% 且强于背景，才算确切识别。
        if max_confidence > 0.70 and max_confidence > predictions[10]:
            if active_request_kind == OBJ_KIND_BOX:
                mapped_class_id = max_index + 1
            else:
                mapped_class_id = (max_index - 11) + 1

        # 【构造 BOX_CLASS 帧】
        # TYPE=0x02，PAYLOAD 回传本次主控 request_id。
        # PAYLOAD = [obj_kind, class_id, vision_seq, request_id]
        data_packet = bytes([mapped_obj_kind, mapped_class_id, vision_seq, active_request_id])

        # 使用 pack_frame 打包并发送 (会自动加上 AA 55、TYPE、LEN、CRC)
        uart.write(pack_frame(0x02, data_packet))

        # 更新视觉帧序号 (0-255 循环)，告诉主控这是一帧新的数据
        vision_seq = (vision_seq + 1) % 256

    # 极限内存回收，防止长期运行死机
    gc.collect()

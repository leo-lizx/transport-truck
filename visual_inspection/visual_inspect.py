from machine import UART  # <--- 修改这里：引入 machine 库中的 UART
import sensor, image, time, math
import os, tf, gc

# ==========================================
# 新增：CRC8 校验函数 (依据逐飞协议标准多项式 0x07)
# 这是主控判定数据包是否损坏的唯一标准，必须计算！
# ==========================================
def crc8(data):
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0x07
            else:
                crc <<= 1
            crc &= 0xFF
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

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA) # 保持 320x240 取景
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

    # 直接将整张图像喂给模型进行分类，不加任何滑动窗口参数！
    # 这时 tf.classify 只会返回一个包含全图分类结果的 obj
    for obj in tf.classify(net, img):

        # 获取所有类别的预测概率列表
        predictions = obj.output()

        # 找到概率最大的那个类别的索引
        max_index = predictions.index(max(predictions))
        max_confidence = predictions[max_index]
        best_label = labels[max_index]

        # 在终端打印，方便连着电脑时肉眼调试
        print("FPS: %5.1f | 识别结果: %s | 置信度: %.2f" % (clock.fps(), best_label, max_confidence))

        # 在屏幕左上角打印结果，方便看 LCD 屏调试（因为不画框了，所以把字写大点）
        img.draw_string(10, 10, "%s: %.2f" % (best_label, max_confidence), color=(255,0,0), scale=2)

        # ==========================================
        # 4. 串口通信：发送决策数据给主板 (已全面重构以适配新协议)
        # ==========================================
        
        # 【极其关键：重新逻辑映射】
        # MD 文档旧版写过 class_id 1..8；当前主控已按 OpenART2 映射扩展到 1..10。
        # 我们利用 obj_kind 字段进行扩容分离：
        # obj_kind = 0 (箱子) 代表前 10 个卡通人物
        # obj_kind = 1 (目标点) 代表后 10 个数字
        mapped_obj_kind = 0
        mapped_class_id = 0 # 默认 0 表示未识别/背景
        
        # 只有当置信度大于 70% 且不是背景时，才进行有效映射
        if max_confidence > 0.70 and best_label != "background":
            if 0 <= max_index <= 9:
                # 识别到卡通人物 (0-9)
                mapped_obj_kind = 0 # 归类为“箱子”
                mapped_class_id = max_index + 1 # 映射为编号 1 到 10
            elif 11 <= max_index <= 20:
                # 识别到数字 (11-20)
                mapped_obj_kind = 1 # 归类为“目标点”
                mapped_class_id = (max_index - 11) + 1 # 同样映射为编号 1 到 10
        else:
            # 置信度不足或识别为背景
            mapped_obj_kind = 0
            mapped_class_id = 0 # class_id=0 主控会判定为 None (未识别)

        # 【构造 BOX_CLASS 帧】
        # 依据 MD 文档第 9 点，TYPE=0x02，LEN=3
        # PAYLOAD = [obj_kind, class_id, vision_seq]
        data_packet = bytes([mapped_obj_kind, mapped_class_id, vision_seq])
        
        # 使用 pack_frame 打包并发送 (会自动加上 AA 55、TYPE、LEN、CRC)
        uart.write(pack_frame(0x02, data_packet))
        
        # 更新视觉帧序号 (0-255 循环)，告诉主控这是一帧新的数据
        vision_seq = (vision_seq + 1) % 256

        # 比赛防抖小技巧：发送完后强制延时一小会儿，防止连续疯狂发包导致主板串口中断卡死
        # 因为你们是停车识别，延时 100-200ms 完全没问题
        # 【新增批注】：MD 文档建议分类摄像头 50ms 一帧 (20Hz)。
        # 因此这里的延时修改为 50ms，以满足主控 1.5s 采样窗口内收集足够票数的需求。
        time.sleep_ms(50) 

    # 极限内存回收，防止长期运行死机
    gc.collect()

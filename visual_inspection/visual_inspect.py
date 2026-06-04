from machine import UART  # <--- 修改这里：引入 machine 库中的 UART
import sensor, image, time, math
import os, tf, gc

# ==========================================
# 1. 硬件初始化设置
# ==========================================
# 初始化串口 12 (TX=B10, RX=B11) - 请核对 OpenART 的引脚定义
# 智能车比赛常用波特率为 115200
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
        # 4. 串口通信：发送决策数据给主板
        # ==========================================
        # 只有当置信度大于 70% 且不是背景时，才认为识别到了有效目标
        if max_confidence > 0.70 and best_label != "background":

            # 【重要】这里设计了一个极简的数据包格式：
            # 帧头 (0xAA) + 类别索引 (max_index) + 帧尾 (0xBB)
            # 例如识别到第 3 类，发送的数据就是 0xAA 0x03 0xBB
            data_packet = bytearray([0xAA, max_index, 0xBB])
            uart.write(data_packet)

            # 比赛防抖小技巧：发送完后强制延时一小会儿，防止连续疯狂发包导致主板串口中断卡死
            # 因为你们是停车识别，延时 100-200ms 完全没问题
            time.sleep_ms(100)

    # 极限内存回收，防止长期运行死机
    gc.collect()

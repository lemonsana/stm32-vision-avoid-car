import gc
import time

import lcd
import sensor
from maix import KPU

# 新增：沿用原实例的板载串口，发送函数直接写在本文件内。
from modules import ybserial

CUSTOM_MODEL_COMMAND = 0x12
SEND_INTERVAL_MS = 100
LOST_DELAY_MS = 500

def make_packet(x=0, y=0, w=0, h=0, class_id=0, confidence=0, found=0):
    """$ length command BB data_len (byte,comma)... checksum #.

    Payload: x/y/w/h as little-endian uint16, class_id, confidence%, found.
    Separators have fixed positions: a data byte of 0x2c remains valid data.
    """
    values = []
    for value in (x, y, w, h):
        value = int(value)
        if not 0 <= value <= 65535:
            raise ValueError("coordinate out of range")
        values.extend((value & 255, (value >> 8) & 255))
    if not 0 <= int(class_id) <= 255:
        raise ValueError("class_id out of range")
    if not 0 <= int(confidence) <= 100 or found not in (0, 1):
        raise ValueError("invalid confidence or found")
    values.extend((int(class_id), int(confidence), int(found)))
    data = []
    for value in values:
        data.extend((value, 0x2c))
    packet = bytearray([0x24, 5 + len(data), CUSTOM_MODEL_COMMAND, 0xbb, len(data)])
    packet.extend(bytearray(data))
    packet.append(sum(packet[1:]) & 255)
    packet.append(0x23)
    return packet


class K210Serial:
    def __init__(self):
        self.port = ybserial()

    def send(self, x=0, y=0, w=0, h=0, class_id=0, confidence=0, found=0):
        packet = make_packet(x, y, w, h, class_id, confidence, found)
        self.port.send_bytearray(list(packet))


serial = K210Serial()


# =========================
# LCD 初始化
# =========================
lcd.init()
lcd.clear(lcd.RED)


# =========================
# 摄像头初始化
# =========================
sensor.reset(dual_buff=True)
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)  # 320×240

# 根据实际摄像头安装方向决定是否开启
#sensor.set_vflip(True)
#sensor.set_hmirror(True)

sensor.skip_frames(time=1000)
clock = time.clock()


# =========================
# 模型配置
# =========================

# 必须按照新模型 label.txt 的顺序填写
labels = ["z"]

# 只使用 anchor.txt 第二行
anchor = (
    0.96, 2.19,
    1.45, 3.32,
    1.80, 4.10,
    2.17, 4.88,
    2.52, 5.72,
)

model_path = "/sd/KPU/train_model_and_use/det.kmodel"

print("Loading model:", model_path)
print("Classes:", len(labels))
print("Anchor number:", len(anchor) // 2)

kpu = KPU()
kpu.load_kmodel(model_path)

kpu.init_yolo2(
    anchor,
    anchor_num=5,
    img_w=320,
    img_h=240,
    net_w=320,
    net_h=240,
    layer_w=10,
    layer_h=8,
    threshold=0.4,
    nms_value=0.3,
    classes=1,
)

print("Model loaded successfully")


# =========================
# 循环检测
# =========================
frame_count = 0
target_present = False
last_send_ms = time.ticks_ms()
last_seen_ms = last_send_ms
# 开机发一帧无目标，STM32重启后也能确认链路（空闲每秒发一次）。
serial.send()

try:
    while True:
        clock.tick()

        # 获取图像
        img = sensor.snapshot()

        # 运行模型
        kpu.run_with_output(img)
        detections = kpu.regionlayer_yolo2()

        # 多目标时只发送置信度最高的一个，屏幕仍绘制全部目标。
        best = None
        if detections:
            for candidate in detections:
                if 0 <= int(candidate[4]) < len(labels):
                    if best is None or candidate[5] > best[5]:
                        best = candidate
        now = time.ticks_ms()
        if best is not None:
            last_seen_ms = now
            if not target_present or time.ticks_diff(now, last_send_ms) >= SEND_INTERVAL_MS:
                serial.send(int(best[0]), int(best[1]), int(best[2]), int(best[3]),
                            int(best[4]), max(0, min(100, int(best[5] * 100))), 1)
                last_send_ms = now
            target_present = True
        elif target_present and time.ticks_diff(now, last_seen_ms) >= LOST_DELAY_MS:
            serial.send()
            last_send_ms = now
            target_present = False
        elif not target_present and time.ticks_diff(now, last_send_ms) >= 1000:
            serial.send()
            last_send_ms = now

        # 绘制检测框
        if detections:
            for detection in detections:
                x = detection[0]
                y = detection[1]
                width = detection[2]
                height = detection[3]
                class_id = detection[4]
                confidence = detection[5]

                if class_id >= 0 and class_id < len(labels):
                    label = labels[class_id]
                else:
                    label = "unknown"

                img.draw_rectangle(
                    x,
                    y,
                    width,
                    height,
                    color=(0, 255, 0),
                    thickness=2,
                )

                info = "%s %.2f" % (label, confidence)

                text_y = y - 20
                if text_y < 20:
                    text_y = 20

                img.draw_string(
                    x,
                    text_y,
                    info,
                    color=(255, 0, 0),
                    scale=2.0,
                )

                # 调试时可以打开；测试帧率时保持注释
                # print(info)

        # 显示帧率
        fps = clock.fps()

        img.draw_string(
            0,
            0,
            "%.1f fps" % fps,
            color=(0, 60, 255),
            scale=2.0,
        )

        lcd.display(img)

        # 每20帧清理一次内存
        frame_count += 1
        if frame_count >= 20:
            gc.collect()
            frame_count = 0

finally:
    kpu.deinit()
    gc.collect()

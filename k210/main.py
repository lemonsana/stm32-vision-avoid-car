import gc
import time

import lcd
import sensor
from maix import KPU
from fpioa_manager import fm
from machine import UART


CUSTOM_MODEL_COMMAND = 0x12
SEND_INTERVAL_MS = 100
YOLO_THRESHOLD = 0.25       # 黑线背景会降低分数；STM32仍连续3帧确认
LOCK_CAMERA_PARAMS = True   # 预热后锁定相机参数，减少黑线导致的亮度变化

# 明确绑定亚博K210外部串口：IO8=TX，IO6=RX，避免固件内置ybserial映射差异。
fm.register(8, fm.fpioa.UART2_TX, force=True)
fm.register(6, fm.fpioa.UART2_RX, force=True)


class Kalman1D:
    """适合K210的标量卡尔曼滤波：不用矩阵库，平滑检测框抖动。"""
    def __init__(self, process_noise=4.0, measure_noise=16.0):
        self.q = process_noise
        self.r = measure_noise
        self.p = 1.0
        self.value = 0.0
        self.ready = False

    def reset(self):
        self.p = 1.0
        self.ready = False

    def update(self, measurement):
        measurement = float(measurement)
        if not self.ready:
            self.value = measurement
            self.ready = True
            return self.value
        self.p += self.q
        gain = self.p / (self.p + self.r)
        self.value += gain * (measurement - self.value)
        self.p *= 1.0 - gain
        return self.value


class BoxKalman:
    def __init__(self):
        self.cx = Kalman1D()
        self.cy = Kalman1D()
        self.w = Kalman1D()
        self.h = Kalman1D()

    def reset(self):
        self.cx.reset()
        self.cy.reset()
        self.w.reset()
        self.h.reset()

    def update(self, target):
        width = max(1, int(self.w.update(target[2])))
        height = max(1, int(self.h.update(target[3])))
        center_x = int(self.cx.update(target[0] + target[2] * 0.5))
        center_y = int(self.cy.update(target[1] + target[3] * 0.5))
        x = max(0, min(319 - width, center_x - width // 2))
        y = max(0, min(239 - height, center_y - height // 2))
        return (x, y, width, height, int(target[4]), float(target[5]))


def make_packet(x=0, y=0, w=0, h=0, class_id=0, confidence=0, found=0):
    values = []
    for value in (x, y, w, h):
        value = int(value)
        values.extend((value & 255, (value >> 8) & 255))
    values.extend((int(class_id), int(confidence), int(found)))
    data = []
    for value in values:
        data.extend((value, 0x2C))
    packet = bytearray([0x24, 5 + len(data), CUSTOM_MODEL_COMMAND, 0xBB, len(data)])
    packet.extend(bytearray(data))
    packet.append(sum(packet[1:]) & 255)
    packet.append(0x23)
    return packet


class K210Serial:
    def __init__(self):
        self.port = UART(UART.UART2, 115200, 8, None, 1,
                         timeout=1000, read_buf_len=256)

    def send(self, target=None):
        if target is None:
            self.port.write(make_packet())
            return
        self.port.write(make_packet(
            int(target[0]), int(target[1]), int(target[2]), int(target[3]),
            int(target[4]), max(0, min(100, int(target[5] * 100))), 1))


serial = K210Serial()
box_filter = BoxKalman()


def lock_camera_params():
    """锁住预热后的曝光、增益和白平衡；固件不支持时自动跳过。"""
    if not LOCK_CAMERA_PARAMS:
        return
    try:
        exposure_us = sensor.get_exposure_us()
        sensor.set_auto_exposure(False, exposure_us=exposure_us)
        print("Exposure locked:", exposure_us)
    except Exception as err:
        print("Exposure lock skipped:", err)
    try:
        gain_db = sensor.get_gain_db()
        sensor.set_auto_gain(False, gain_db=gain_db)
        print("Gain locked:", gain_db)
    except Exception as err:
        print("Gain lock skipped:", err)
    try:
        rgb_gain_db = sensor.get_rgb_gain_db()
        sensor.set_auto_whitebal(False, rgb_gain_db=rgb_gain_db)
        print("White balance locked:", rgb_gain_db)
    except Exception as err:
        print("White balance lock skipped:", err)


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

sensor.skip_frames(time=1500)
lock_camera_params()
sensor.skip_frames(time=200)
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
    threshold=YOLO_THRESHOLD,
    nms_value=0.3,
    classes=1,
)

print("Model loaded successfully")


# =========================
# 循环检测
# =========================
frame_count = 0
last_send_ms = time.ticks_ms()
tx_count = 0
serial.send()
tx_count += 1

try:
    while True:
        clock.tick()

        # 获取图像
        img = sensor.snapshot()

        # 运行模型
        kpu.run_with_output(img)
        detections = kpu.regionlayer_yolo2()

        # 不用置信度和高度作触发门槛；多目标时只选面积最大的前方障碍。
        best = None
        for candidate in detections:
            x = int(candidate[0])
            y = int(candidate[1])
            w = int(candidate[2])
            h = int(candidate[3])
            if int(candidate[4]) != 0:
                continue
            if x < 0 or y < 0 or w <= 0 or h <= 0 or x + w > 320 or y + h > 240:
                continue
            if x >= 240 or x + w <= 80:
                continue
            if best is None or w * h > int(best[2]) * int(best[3]):
                best = candidate

        if best is None:
            box_filter.reset()
        else:
            best = box_filter.update(best)

        # 有无障碍都每100ms发送，STM32可区分“无目标”和“通信断开”。
        now = time.ticks_ms()
        if time.ticks_diff(now, last_send_ms) >= SEND_INTERVAL_MS:
            serial.send(best)
            tx_count += 1
            last_send_ms = now

        # 绘制检测框
        if len(detections) > 0:
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

                info = "%s" % label

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
        img.draw_string(0, 22, "TX:%d" % tx_count,
                        color=(0, 60, 255), scale=2.0)

        lcd.display(img)

        # 每20帧清理一次内存
        frame_count += 1
        if frame_count >= 20:
            gc.collect()
            frame_count = 0

finally:
    kpu.deinit()
    gc.collect()

import gc
import time

import lcd
import sensor
from maix import KPU
from fpioa_manager import fm
from machine import UART

CUSTOM_MODEL_COMMAND = 0x12
SEND_INTERVAL_MS = 50       # 最快 50ms 一帧；K210 实际 8~12fps 时等于每帧都发
LCD_PREVIEW = False         # True = LCD 显示画面和框（调试用，会拖慢 3~5fps）
# 与 SYSTEM/vision/vision_avoid.h 的 VA_ROI_LEFT / VA_ROI_RIGHT 保持一致
ROI_LEFT = 60
ROI_RIGHT = 260
MIN_BOX_PX = 12             # 降阈值后 12px 以下的小噪声框直接丢
FREEZE_EXPOSURE = True      # 固定曝光/增益，原因见摄像头初始化处

fm.register(8, fm.fpioa.UART2_TX, force=True)
fm.register(6, fm.fpioa.UART2_RX, force=True)


class Kalman1D:
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
        self.port = UART(UART.UART2, 115200, 8, None, 1,
                         timeout=1000, read_buf_len=256)

    def send(self, x=0, y=0, w=0, h=0, class_id=0, confidence=0, found=0):
        packet = make_packet(x, y, w, h, class_id, confidence, found)
        self.port.write(packet)


serial = K210Serial()
box_filter = BoxKalman()


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

# 固定曝光/增益：
# 黑线在画面下半部占一大块，自动曝光(AEC)/自动增益(AGC)会为了照顾黑线把白纸推过曝，
# 障碍物暗面被压平、噪声变大，YOLO 的分数就掉到 threshold 附近甚至以下 ——
# 这是"地上有黑线就不灵敏"的头号原因。
# 用法：先把车放在赛道上（黑线在视野里）再上电，让 AE 在赛道亮度上收敛，然后冻结。
# 现场偏暗/偏亮时改成固定值，例如 sensor.set_auto_exposure(False, exposure_us=12000)
if FREEZE_EXPOSURE:
    try:
        sensor.set_auto_exposure(False)
        sensor.set_auto_gain(False)
        sensor.skip_frames(time=300)
    except Exception as e:
        print("freeze exposure failed:", e)

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
    threshold=0.3,          # 0.4 时低对比度的远处障碍会被直接筛掉；防误检交给 STM32 的滑窗
    nms_value=0.3,
    classes=1,
)

print("Model loaded successfully")
print("LCD preview:", LCD_PREVIEW, " freeze exposure:", FREEZE_EXPOSURE,
      " send interval:", SEND_INTERVAL_MS, "ms")


# =========================
# 循环检测
# =========================
frame_count = 0
last_send_ms = time.ticks_ms()
# 开机发一帧无目标；运行期间有无目标均定期发送。
serial.send()

try:
    while True:
        clock.tick()

        # 获取图像
        img = sensor.snapshot()

        # 运行模型
        kpu.run_with_output(img)
        detections = kpu.regionlayer_yolo2()

        # 不用置信度和高度作触发门槛；多目标时选面积最大的前方障碍。
        best = None
        for candidate in detections:
            x, y, w, h = [int(candidate[i]) for i in range(4)]
            if int(candidate[4]) != 0:
                continue
            if x < 0 or y < 0 or w <= 0 or h <= 0 or x+w > 320 or y+h > 240:
                continue
            if w < MIN_BOX_PX or h < MIN_BOX_PX:
                continue
            if x >= ROI_RIGHT or x+w <= ROI_LEFT:
                continue
            if best is None or w * h > int(best[2]) * int(best[3]):
                best = candidate
        if best is None:
            box_filter.reset()
        else:
            best = box_filter.update(best)
        now = time.ticks_ms()
        # 每 SEND_INTERVAL_MS 报告当前画面，无目标也发心跳；不延迟 500ms 才报告丢失。
        if time.ticks_diff(now, last_send_ms) >= SEND_INTERVAL_MS:
            if best is None:
                serial.send()
            else:
                serial.send(int(best[0]), int(best[1]), int(best[2]), int(best[3]),
                            int(best[4]), max(0, min(100, int(best[5] * 100))), 1)
            last_send_ms = now

        # 绘制/显示只在调试时开：lcd.display + 每帧画字符串要吃掉 3~5fps，
        # 帧率低会直接拉长 STM32 的确认时间。
        fps = clock.fps()
        if LCD_PREVIEW:
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

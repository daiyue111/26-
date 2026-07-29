import sensor
import struct
import time
from pyb import UART

# Calibrate these values with the final rigid camera mount.
ROI = (10, 80, 300, 80)
BALL_THRESHOLD = (0, 85)
PIXEL_AT_NEG_100MM = 50
PIXEL_AT_POS_100MM = 270


def crc8(data):
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else \
                (crc << 1) & 0xFF
    return crc


def pixel_to_tenth_mm(pixel_x):
    span = PIXEL_AT_POS_100MM - PIXEL_AT_NEG_100MM
    value = -1000 + ((pixel_x - PIXEL_AT_NEG_100MM) * 2000) // span
    return max(-1250, min(1250, value))


def send_sample(uart, position, confidence, frame_id):
    payload = struct.pack("<hBB", position, confidence, frame_id)
    uart.write(b"\xAA\x55" + payload + bytes((crc8(payload),)))


sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=1200)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(False, exposure_us=3000)

uart = UART(3, 115200, timeout_char=2)
clock = time.clock()
frame_id = 0
last_position = 0

while True:
    clock.tick()
    image = sensor.snapshot()
    blobs = image.find_blobs([BALL_THRESHOLD], roi=ROI,
                             pixels_threshold=35,
                             area_threshold=35, merge=True)
    best = None
    best_score = -1
    for blob in blobs:
        width = max(1, blob.w())
        height = max(1, blob.h())
        aspect_error = abs(width - height)
        score = blob.pixels() - 3 * aspect_error
        if score > best_score:
            best = blob
            best_score = score

    if best is None:
        send_sample(uart, last_position, 0, frame_id)
    else:
        last_position = pixel_to_tenth_mm(best.cx())
        confidence = max(1, min(100, best.pixels() * 100 // 180))
        send_sample(uart, last_position, confidence, frame_id)
        image.draw_rectangle(best.rect(), color=255)
        image.draw_cross(best.cx(), best.cy(), color=255)

    frame_id = (frame_id + 1) & 0xFF

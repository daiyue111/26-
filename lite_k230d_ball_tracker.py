from machine import FPIOA, UART
from media.sensor import Sensor
from media.display import Display
from media.media import MediaManager
import gc
import os
import sys
import time
import ustruct


# Lite-K230D / CanMV first-stage tracker for the 10 mm steel ball.
# Tune the ROI and circle parameters with the final rigid camera mount.
WIDTH = 320
HEIGHT = 240
ROI = (10, 80, 300, 80)

UART_TX_PIN = 3
UART_RX_PIN = 4
UART_BAUD = 115200

CIRCLE_THRESHOLD = 2200
CIRCLE_X_MARGIN = 8
CIRCLE_Y_MARGIN = 8
CIRCLE_R_MARGIN = 4
CIRCLE_R_MIN = 4
CIRCLE_R_MAX = 20
CIRCLE_R_STEP = 2
MAX_POSITION_JUMP_PX = 55
TRACK_RESET_MISSES = 5
LOCK_FRAMES_REQUIRED = 2

# Measure these two pixels after the camera and pipe are fixed.
PIXEL_AT_NEG_100MM = 50
PIXEL_AT_POS_100MM = 270


def clamp(value, minimum, maximum):
    if value < minimum:
        return minimum
    if value > maximum:
        return maximum
    return value


def crc8(data):
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def pixel_to_tenth_mm(pixel_x):
    span = PIXEL_AT_POS_100MM - PIXEL_AT_NEG_100MM
    if span == 0:
        return 0
    position = -1000 + ((pixel_x - PIXEL_AT_NEG_100MM) * 2000) // span
    return clamp(position, -1250, 1250)


def send_sample(uart, position_tenth_mm, confidence, frame_id):
    payload = ustruct.pack(
        "<hBB",
        int(position_tenth_mm),
        int(confidence),
        int(frame_id),
    )
    uart.write(b"\xAA\x55" + payload + bytes((crc8(payload),)))


def select_circle(circles, previous_x):
    best = None
    best_score = None
    roi_center_y = ROI[1] + ROI[3] // 2

    for circle in circles:
        if previous_x is not None:
            jump = abs(circle.x() - previous_x)
            if jump > MAX_POSITION_JUMP_PX:
                continue
        else:
            jump = 0

        score = (
            int(circle.magnitude())
            - 8 * abs(circle.y() - roi_center_y)
            - 4 * jump
        )
        if best is None or score > best_score:
            best = circle
            best_score = score
    return best


def main():
    os.exitpoint(os.EXITPOINT_ENABLE)
    sensor = None
    uart = None
    display_started = False
    media_started = False

    try:
        fpioa = FPIOA()
        fpioa.set_function(UART_TX_PIN, fpioa.UART1_TXD)
        fpioa.set_function(UART_RX_PIN, fpioa.UART1_RXD)
        uart = UART(
            UART.UART1,
            baudrate=UART_BAUD,
            bits=UART.EIGHTBITS,
            parity=UART.PARITY_NONE,
            stop=UART.STOPBITS_ONE,
        )

        sensor = Sensor()
        sensor.reset()
        sensor.set_framesize(width=WIDTH, height=HEIGHT)
        sensor.set_pixformat(Sensor.RGB565)
        Display.init(Display.VIRT, width=WIDTH, height=HEIGHT, to_ide=True)
        display_started = True
        MediaManager.init()
        media_started = True
        sensor.run()

        frame_id = 0
        frame_count = 0
        lock_frames = 0
        missed_frames = 0
        previous_x = None
        last_position = 0
        clock = time.clock()
        print("Lite-K230D ball tracker running")
        print("UART1 TX=IO3 RX=IO4 at", UART_BAUD)

        while True:
            os.exitpoint()
            clock.tick()
            img = sensor.snapshot()
            circles = img.find_circles(
                roi=ROI,
                threshold=CIRCLE_THRESHOLD,
                x_margin=CIRCLE_X_MARGIN,
                y_margin=CIRCLE_Y_MARGIN,
                r_margin=CIRCLE_R_MARGIN,
                r_min=CIRCLE_R_MIN,
                r_max=CIRCLE_R_MAX,
                r_step=CIRCLE_R_STEP,
            )
            ball = select_circle(circles, previous_x)

            img.draw_rectangle(ROI, color=(0, 255, 0), thickness=1)
            zero_x = (PIXEL_AT_NEG_100MM + PIXEL_AT_POS_100MM) // 2
            img.draw_line(zero_x, ROI[1], zero_x, ROI[1] + ROI[3],
                          color=(0, 0, 255), thickness=1)

            if ball is None:
                missed_frames += 1
                lock_frames = 0
                send_sample(uart, last_position, 0, frame_id)
                if missed_frames >= TRACK_RESET_MISSES:
                    previous_x = None
            else:
                missed_frames = 0
                lock_frames = min(LOCK_FRAMES_REQUIRED, lock_frames + 1)
                previous_x = ball.x()
                last_position = pixel_to_tenth_mm(ball.x())
                confidence = 40 if lock_frames < LOCK_FRAMES_REQUIRED else 85
                send_sample(uart, last_position, confidence, frame_id)
                img.draw_circle(
                    ball.x(), ball.y(), ball.r(),
                    color=(255, 0, 0), thickness=2,
                )
                img.draw_cross(
                    ball.x(), ball.y(), color=(255, 255, 0), size=8,
                )

            Display.show_image(img)
            frame_id = (frame_id + 1) & 0xFF
            frame_count += 1
            if frame_count % 15 == 0:
                print(
                    "x:", previous_x,
                    "pos_0.1mm:", last_position,
                    "circles:", len(circles),
                    "fps:", round(clock.fps(), 1),
                )
                gc.collect()
    except BaseException as exc:
        if "IDE interrupt" in str(exc):
            print("Ball tracker stopped")
        else:
            print("BALL TRACKER FAIL")
            sys.print_exception(exc)
    finally:
        if sensor is not None:
            sensor.stop()
        if display_started:
            Display.deinit()
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)
        if media_started:
            MediaManager.deinit()


main()

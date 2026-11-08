from machine import Pin, PWM, time_pulse_us
import time
import json
import socket
import network
import gc

# ============================================================
# 1. CAU HINH
# ============================================================

# --- WiFi / Web ---
WIFI_SSID = "LoVuong"
WIFI_PASSWORD = "phuancut"
AP_SSID = "RobotArm"            # Neu khong vao duoc WiFi -> tu phat WiFi nay
AP_PASSWORD = "12345678"        # >= 8 ky tu
WEB_PORT = 80
AUTO_MODE_DEFAULT = True        # True: tu dong phan loai ngay khi bat nguon
SPEED_MIN = 50                  # Gioi han toc do bang tai (Hz) tren web
SPEED_MAX = 800

# A4988 - NEMA17
STEP_PIN = 18
DIR_PIN = 19
EN_PIN = 21
STEP_FREQUENCY = 333
STEP_DUTY = 32768
BELT_DIRECTION = 1

# HC-SR04
TRIG_PIN = 32
ECHO_PIN = 14

# LM393
LIGHT_PIN = 13
LIGHT_ACTIVE_LEVEL = 0
SENSOR_DEBOUNCE_MS = 30
SENSOR_CLEAR_MS = 250

# Do cao
BASE_DISTANCE = 16.52
# True : luon dung BASE_DISTANCE trong code, bo qua gia tri cu trong settings.json
# (bam "Hieu chuan day" tren web se tu chuyen sang False va luu gia tri do duoc)
FORCE_BASE = True
HEIGHT_THRESHOLD = 6.0
MIN_OBJECT_HEIGHT = 0.5           # cao <= gia tri nay (cm) = dao dong/nhieu, KHONG tinh la vat

# Servo: Base, Shoulder, Elbow, Gripper
SERVO_BASE_PIN = 27
SERVO_SHOULDER_PIN = 26
SERVO_ELBOW_PIN = 25
SERVO_GRIPPER_PIN = 33

SERVO_FREQ = 50
SERVO_MIN_US = 500
SERVO_MAX_US = 2500
SERVO_STEP_DELAY_MS = 20
WAYPOINT_WAIT_MS = 150

# ============================================================
# 2. KHOI TAO PHAN CUNG
# ============================================================

dir_pin = Pin(DIR_PIN, Pin.OUT)
en_pin = Pin(EN_PIN, Pin.OUT)

dir_pin.value(BELT_DIRECTION)
en_pin.value(1)

# PWM phan cung cho A4988
step_pwm = PWM(Pin(STEP_PIN))
step_pwm.freq(STEP_FREQUENCY)
step_pwm.duty_u16(0)

trig = Pin(TRIG_PIN, Pin.OUT)
echo = Pin(ECHO_PIN, Pin.IN)
trig.value(0)

light_sensor = Pin(LIGHT_PIN, Pin.IN)

servo_base = PWM(Pin(SERVO_BASE_PIN))
servo_shoulder = PWM(Pin(SERVO_SHOULDER_PIN))
servo_elbow = PWM(Pin(SERVO_ELBOW_PIN))
servo_gripper = PWM(Pin(SERVO_GRIPPER_PIN))

servos = [
    servo_base,
    servo_shoulder,
    servo_elbow,
    servo_gripper
]

for s in servos:
    s.freq(SERVO_FREQ)

# ============================================================
# 2B. NHAT KY + THONG KE (cho web)
# ============================================================

log_buf = []
log_id = 0

# muc: i = thong tin, o = tot, w = canh bao, e = loi
def log(msg, lvl="i"):
    global log_id
    print(msg)
    log_id += 1
    log_buf.append((log_id, time.ticks_ms() // 1000, msg, lvl))
    if len(log_buf) > 40:
        log_buf.pop(0)


stats = {"total": 0, "high": 0, "low": 0, "err": 0}

# Lich su do: (id, giay, khoang_cach, chieu_cao, nguong, loai)
# loai: L = thap, H = cao, E = loi, T = do thu
hist_buf = []
hist_id = 0

def record(d, h, cls):
    global hist_id
    hist_id += 1
    hist_buf.append((
        hist_id,
        time.ticks_ms() // 1000,
        None if d is None else round(d, 2),
        None if h is None else round(h, 2),
        HEIGHT_THRESHOLD,
        cls
    ))
    if len(hist_buf) > 50:
        hist_buf.pop(0)

auto_mode = AUTO_MODE_DEFAULT
estop = False                 # dung khan cap
manual_moving = False         # robot dang di chuyen theo lenh tay tu web
SETTINGS_FILE = "settings.json"
live_distance = None
ip_addr = "0.0.0.0"

# ============================================================
# 3. CAC VI TRI ROBOT
# ============================================================

HOME = [90, 30, 60, 80]
P1 = [90, 45, 90, 60]
P2 = [90, 60, 120, 130]
P3 = [90, 80, 90, 130]
P4 = [140, 80, 60, 130]
P5 = [140, 80, 60, 60]

current_angles = HOME[:]
display_angles = HOME[:]      # goc thuc te dang xuat ra servo (de hien thi tren web)


def set_servo_angle(servo, angle):
    angle = max(0, min(180, angle))

    pulse_us = (
        SERVO_MIN_US
        + (angle / 180)
        * (SERVO_MAX_US - SERVO_MIN_US)
    )

    duty = int(pulse_us * 65535 / 20000)
    servo.duty_u16(duty)


def set_all_servos(angles):
    for i in range(4):
        set_servo_angle(servos[i], angles[i])
        display_angles[i] = angles[i]

# ============================================================
# 4. DIEU KHIEN BANG TAI BANG PWM
# ============================================================

belt_running = False

def belt_start():
    global belt_running

    dir_pin.value(BELT_DIRECTION)
    en_pin.value(0)

    step_pwm.freq(STEP_FREQUENCY)
    step_pwm.duty_u16(STEP_DUTY)

    belt_running = True

    log("Băng tải chạy")


def belt_stop():
    global belt_running

    step_pwm.duty_u16(0)

    belt_running = False

    # Giu moment dong co
    en_pin.value(0)

    log("Băng tải dừng")

# ============================================================
# 5. CAM BIEN
# ============================================================

def object_detected():
    return light_sensor.value() == LIGHT_ACTIVE_LEVEL


def object_detected_stable():
    if not object_detected():
        return False

    time.sleep_ms(SENSOR_DEBOUNCE_MS)

    return object_detected()


def read_distance():
    trig.value(0)
    time.sleep_us(2)

    trig.value(1)
    time.sleep_us(10)

    trig.value(0)

    try:
        duration = time_pulse_us(echo, 1, 30000)

        if duration < 0:
            return None

        distance = duration * 0.0343 / 2

        if 2 <= distance <= 400:
            return distance

    except OSError:
        pass

    return None


def read_average(samples=7):
    values = []

    for _ in range(samples):
        d = read_distance()

        if d is not None:
            values.append(d)

        time.sleep_ms(60)

    if len(values) < 3:
        return None

    values.sort()

    return values[len(values) // 2]


SAMPLES = 9              # so lan do moi san pham
MAX_ATTEMPTS = 15        # toi da so lan thu (bu cho lan doc loi)
OUTLIER_CM = 1.0         # loai gia tri lech qua xa trung vi
MIN_KEPT = 5             # so mau hop le toi thieu sau khi loc


def read_mean9():
    # Do lien tiep 9 lan -> loai mau nhieu -> lay trung binh
    values = []
    attempts = 0

    while len(values) < SAMPLES and attempts < MAX_ATTEMPTS:
        attempts += 1

        d = read_distance()

        if d is not None:
            values.append(d)

        time.sleep_ms(60)

    if len(values) < MIN_KEPT:
        return None

    values.sort()

    median = values[len(values) // 2]

    kept = [v for v in values if abs(v - median) <= OUTLIER_CM]

    if len(kept) < MIN_KEPT:
        # Cac mau qua phan tan: dung trung vi cho an toan
        return median

    return sum(kept) / len(kept)


def measure_height():
    global live_distance

    distance = read_mean9()

    if distance is None:
        live_distance = None
        return None, None

    live_distance = distance
    height = max(0, BASE_DISTANCE - distance)

    return distance, height

# ============================================================
# 6. DIEU KHIEN ROBOT KHONG CHAN CHUONG TRINH
# ============================================================

robot_busy = False
robot_stage = 0
robot_wait_until = 0

move_start = None
move_target = None
move_max_steps = 0
move_step = 0
move_last_time = 0


def begin_robot_move(target):
    global move_start
    global move_target
    global move_max_steps
    global move_step
    global move_last_time

    move_start = display_angles[:]
    move_target = target[:]

    move_max_steps = max(
        abs(target[i] - move_start[i])
        for i in range(4)
    )

    move_step = 0
    move_last_time = time.ticks_ms()


def robot_start():
    global robot_busy, robot_stage

    if robot_busy or estop or manual_moving:
        return

    log("Robot bắt đầu gắp", "o")

    robot_busy = True
    robot_stage = 0

    begin_robot_move(P1)


def robot_move_service():
    global current_angles
    global move_step
    global move_last_time

    if move_max_steps == 0:
        current_angles = move_target[:]
        return True

    now = time.ticks_ms()

    if time.ticks_diff(
        now,
        move_last_time
    ) < SERVO_STEP_DELAY_MS:
        return False

    move_last_time = now
    move_step += 1

    new_angles = []

    for i in range(4):

        diff = move_target[i] - move_start[i]

        angle = (
            move_start[i]
            + diff * move_step / move_max_steps
        )

        new_angles.append(int(round(angle)))

    set_all_servos(new_angles)

    if move_step >= move_max_steps:
        current_angles = move_target[:]
        return True

    return False


def robot_service():
    global robot_stage
    global robot_wait_until
    global robot_busy
    global move_target

    if not robot_busy:
        return

    # Dang di chuyen
    if move_target is not None:

        if not robot_move_service():
            return

        robot_wait_until = time.ticks_add(
            time.ticks_ms(),
            WAYPOINT_WAIT_MS
        )

        completed_stage = robot_stage
        move_target = None

        if completed_stage == 0:
            robot_stage = 10

        elif completed_stage == 1:
            robot_stage = 11

        elif completed_stage == 2:
            robot_stage = 12

        elif completed_stage == 4:
            robot_stage = 14

        elif completed_stage == 5:
            robot_stage = 15

        elif completed_stage == 6:
            robot_stage = 16

        return

    # Cho giua cac diem
    if time.ticks_diff(
        time.ticks_ms(),
        robot_wait_until
    ) < 0:
        return

    if robot_stage == 10:

        robot_stage = 1
        begin_robot_move(P2)

    elif robot_stage == 11:

        robot_stage = 2
        begin_robot_move(P3)

    elif robot_stage == 12:

        robot_stage = 3

        log("Robot đã gắp - chờ cảm biến trống")

    elif robot_stage == 3:
        return

    elif robot_stage == 14:

        robot_stage = 5
        begin_robot_move(P5)

    elif robot_stage == 15:

        robot_stage = 6
        begin_robot_move(HOME)

    elif robot_stage == 16:

        log("Robot đã về HOME", "o")

        robot_busy = False
        robot_stage = 7

# ============================================================
# 7. TRANG THAI HE THONG
# ============================================================

waiting_clear_low = False
low_clear_start = None

pending_high = False

waiting_clear_pick = False
pick_clear_start = None

sensor_armed = True


def sensor_clear_service():
    global low_clear_start
    global waiting_clear_low
    global sensor_armed

    global pick_clear_start
    global waiting_clear_pick

    global robot_stage

    if object_detected():

        low_clear_start = None
        pick_clear_start = None

        return

    now = time.ticks_ms()

    # Vat thap da di qua
    if waiting_clear_low:

        if low_clear_start is None:
            low_clear_start = now

        elif time.ticks_diff(
            now,
            low_clear_start
        ) >= SENSOR_CLEAR_MS:

            waiting_clear_low = False
            low_clear_start = None
            sensor_armed = True

            log("Sản phẩm thấp đã qua cảm biến")

    # Robot da nhac vat ra khoi vung cam bien
    if waiting_clear_pick:

        if pick_clear_start is None:
            pick_clear_start = now

        elif time.ticks_diff(
            now,
            pick_clear_start
        ) >= SENSOR_CLEAR_MS:

            waiting_clear_pick = False
            pick_clear_start = None
            sensor_armed = True

            log("Đã nhấc vật ra - băng tải chạy tiếp")

            belt_start()

            robot_stage = 4
            begin_robot_move(P4)

# ============================================================
# 8. WEB SERVER (khong chan, chay chung vong lap chinh)
# ============================================================

# stage noi bo -> buoc hien thi tren web (1..7)
STAGE_TO_STEP = {
    0: 1, 10: 1,
    1: 2, 11: 2,
    2: 3, 12: 3, 3: 3,
    4: 5, 14: 5,
    5: 6, 15: 6,
    6: 7, 16: 7,
}


def wifi_connect():
    # Tat AP truoc de tranh xung dot trang thai WiFi
    ap = network.WLAN(network.AP_IF)
    try:
        ap.active(False)
    except OSError:
        pass

    sta = network.WLAN(network.STA_IF)

    for attempt in range(3):
        try:
            # Reset han giao dien WiFi de tranh "Wifi Internal State Error"
            sta.active(False)
            time.sleep_ms(300)
            sta.active(True)
            time.sleep_ms(300)

            try:
                sta.disconnect()
            except OSError:
                pass

            print("Dang ket noi WiFi:", WIFI_SSID, "(lan", attempt + 1, ")")
            sta.connect(WIFI_SSID, WIFI_PASSWORD)

            t0 = time.ticks_ms()
            while (not sta.isconnected()
                   and time.ticks_diff(time.ticks_ms(), t0) < 15000):
                time.sleep_ms(200)

            if sta.isconnected():
                return sta.ifconfig()[0]

        except OSError as e:
            print("Loi WiFi:", e)
            time.sleep_ms(500)

    # Khong vao duoc WiFi -> tu phat WiFi rieng
    print("Khong ket noi duoc WiFi - chuyen sang che do AP")

    try:
        sta.active(False)
        time.sleep_ms(300)
        ap.active(True)
        time.sleep_ms(300)
        ap.config(essid=AP_SSID, password=AP_PASSWORD, authmode=3)
        return ap.ifconfig()[0]
    except OSError as e:
        print("Loi AP:", e)
        return "0.0.0.0"


def build_state(since, hsince):
    d = live_distance
    h = None if d is None else max(0, BASE_DISTANCE - d)

    logs = [[i, t, m, l] for (i, t, m, l) in log_buf if i > since]

    return json.dumps({
        "ip": ip_addr,
        "up": time.ticks_ms() // 1000,
        "auto": auto_mode,
        "belt": belt_running,
        "hz": STEP_FREQUENCY,
        "smin": SPEED_MIN,
        "smax": SPEED_MAX,
        "obj": object_detected(),
        "d": None if d is None else round(d, 2),
        "h": None if h is None else round(h, 2),
        "base": BASE_DISTANCE,
        "thr": HEIGHT_THRESHOLD,
        "busy": robot_busy,
        "stg": robot_stage,
        "step": STAGE_TO_STEP.get(robot_stage, 0) if robot_busy else 0,
        "ang": display_angles,
        "wp": {"HOME": HOME, "P1": P1, "P2": P2, "P3": P3, "P4": P4, "P5": P5},
        "dir": BELT_DIRECTION,
        "sd": SERVO_STEP_DELAY_MS,
        "lvl": LIGHT_ACTIVE_LEVEL,
        "estop": estop,
        "man": manual_moving,
        "pend": pending_high,
        "st": stats,
        "log": logs,
        "lid": log_id,
        "hist": [list(r) for r in hist_buf if r[0] > hsince],
        "hid": hist_id,
    })


def cmd_belt(run):
    global sensor_armed, waiting_clear_low, waiting_clear_pick
    global low_clear_start, pick_clear_start

    if run:
        if estop:
            log("Đang dừng khẩn cấp - nhấn Tiếp tục trước", "w")
            return

        if robot_busy:
            log("Robot đang gắp - chưa thể chạy băng tải", "w")
            return

        # Nguoi dung chu dong chay -> go bo trang thai cho / loi
        sensor_armed = True
        waiting_clear_low = False
        waiting_clear_pick = False
        low_clear_start = None
        pick_clear_start = None

        log("Web: chạy băng tải")
        belt_start()
    else:
        log("Web: dừng băng tải", "w")
        belt_stop()


def cmd_speed(hz):
    global STEP_FREQUENCY

    hz = max(SPEED_MIN, min(SPEED_MAX, hz))
    STEP_FREQUENCY = hz

    if belt_running:
        step_pwm.freq(STEP_FREQUENCY)

    log("Web: tốc độ băng tải " + str(hz) + " Hz")
    save_settings()


def cmd_auto(on):
    global auto_mode, sensor_armed

    auto_mode = on

    if on:
        log("Chế độ tự động: BẬT", "o")
        if (not belt_running) and (not robot_busy):
            cmd_belt(True)
        sensor_armed = True
    else:
        log("Chế độ tự động: TẮT", "w")

    save_settings()


def cmd_threshold(cm):
    global HEIGHT_THRESHOLD

    HEIGHT_THRESHOLD = max(1.0, min(20.0, cm))
    log("Web: ngưỡng phân loại " + str(HEIGHT_THRESHOLD) + " cm")
    save_settings()


def cmd_pick():
    if robot_busy:
        log("Robot đang bận", "w")
        return

    log("Web: chạy thử chu trình gắp", "o")
    robot_start()


# ------------------ lenh nang cao ------------------

WP_NAMES = ("HOME", "P1", "P2", "P3", "P4", "P5")


def wp_get(name):
    return {"HOME": HOME, "P1": P1, "P2": P2,
            "P3": P3, "P4": P4, "P5": P5}.get(name)


def save_settings():
    try:
        with open(SETTINGS_FILE, "w") as f:
            json.dump({
                "thr": HEIGHT_THRESHOLD, "hz": STEP_FREQUENCY,
                "base": BASE_DISTANCE, "fb": FORCE_BASE,
                "dir": BELT_DIRECTION,
                "auto": auto_mode, "sd": SERVO_STEP_DELAY_MS,
                "lvl": LIGHT_ACTIVE_LEVEL,
                "wp": {n: wp_get(n) for n in WP_NAMES},
            }, f)
    except Exception as e:
        log("Không lưu được cài đặt: " + str(e), "e")


def load_settings():
    global HEIGHT_THRESHOLD, STEP_FREQUENCY, BASE_DISTANCE, BELT_DIRECTION
    global auto_mode, SERVO_STEP_DELAY_MS, LIGHT_ACTIVE_LEVEL, FORCE_BASE

    try:
        with open(SETTINGS_FILE) as f:
            c = json.load(f)
    except Exception:
        return

    HEIGHT_THRESHOLD = c.get("thr", HEIGHT_THRESHOLD)
    STEP_FREQUENCY = c.get("hz", STEP_FREQUENCY)
    BELT_DIRECTION = c.get("dir", BELT_DIRECTION)
    auto_mode = c.get("auto", auto_mode)
    SERVO_STEP_DELAY_MS = c.get("sd", SERVO_STEP_DELAY_MS)
    LIGHT_ACTIVE_LEVEL = c.get("lvl", LIGHT_ACTIVE_LEVEL)

    # File cu khong co "fb" -> giu FORCE_BASE=True -> dung BASE_DISTANCE trong code.
    # Sau khi bam "Hieu chuan day", fb=False duoc luu -> dung gia tri da hieu chuan.
    FORCE_BASE = c.get("fb", FORCE_BASE)
    if not FORCE_BASE:
        BASE_DISTANCE = c.get("base", BASE_DISTANCE)

    for n, v in c.get("wp", {}).items():
        t = wp_get(n)
        if t is not None and len(v) == 4:
            t[:] = v


def cmd_estop():
    global estop, belt_running, robot_busy, robot_stage, pending_high
    global waiting_clear_low, waiting_clear_pick, manual_moving
    global move_target, current_angles

    estop = True

    step_pwm.duty_u16(0)
    en_pin.value(1)                 # nha dong co bang tai
    belt_running = False

    robot_busy = False
    robot_stage = 0
    pending_high = False
    waiting_clear_low = False
    waiting_clear_pick = False
    manual_moving = False
    move_target = None
    current_angles = display_angles[:]

    log("DỪNG KHẨN CẤP - đã ngắt băng tải và robot", "e")


def cmd_resume():
    global estop, sensor_armed

    estop = False
    sensor_armed = True
    en_pin.value(0)

    log("Đã tiếp tục - nhấn Chạy hoặc Về HOME", "o")


def cmd_dir(v):
    global BELT_DIRECTION

    BELT_DIRECTION = 1 if v else 0
    dir_pin.value(BELT_DIRECTION)

    log("Chiều băng tải: " + ("thuận" if BELT_DIRECTION else "ngược"))
    save_settings()


def manual_goto(target):
    global manual_moving

    if estop:
        log("Đang dừng khẩn cấp", "w")
        return

    if robot_busy:
        log("Robot đang chạy chu trình - không điều khiển tay được", "w")
        return

    begin_robot_move(target)
    manual_moving = True


def cmd_servo(i, a):
    if not 0 <= i <= 3:
        return

    base = move_target if (manual_moving and move_target) else display_angles
    t = list(base)
    t[i] = max(0, min(180, a))

    manual_goto(t)


def cmd_goto(name):
    t = wp_get(name)

    if t is None:
        return

    log("Web: robot tới " + name)
    manual_goto(t[:])


def cmd_save_wp(name):
    t = wp_get(name)

    if t is None or robot_busy or manual_moving:
        log("Chưa lưu được - robot đang chuyển động", "w")
        return

    t[:] = display_angles
    log("Đã lưu " + name + " = " + str(list(display_angles)), "o")
    save_settings()


def cmd_sdelay(ms):
    global SERVO_STEP_DELAY_MS

    SERVO_STEP_DELAY_MS = max(5, min(60, ms))

    log("Web: tốc độ servo " + str(SERVO_STEP_DELAY_MS) + " ms/độ")
    save_settings()


def cmd_level(v):
    global LIGHT_ACTIVE_LEVEL

    LIGHT_ACTIVE_LEVEL = 1 if v else 0

    log("LM393 kích hoạt mức " + ("CAO" if v else "THẤP"))
    save_settings()


def cmd_measure():
    if robot_busy:
        log("Robot đang gắp - không đo thử", "w")
        return

    d, h = measure_height()

    if h is None:
        record(None, None, "E")
        log("Đo thử: không đọc được HC-SR04", "e")
    else:
        record(d, h, "T")
        log("Đo thử: khoảng cách " + str(round(d, 1))
            + " cm | cao " + str(round(h, 1)) + " cm", "o")


def cmd_calibrate():
    global BASE_DISTANCE, FORCE_BASE

    if object_detected():
        log("Có vật trên băng tải - lấy vật ra trước khi hiệu chuẩn", "w")
        return

    d = read_average(9)

    if d is None:
        log("Hiệu chuẩn thất bại - không đọc được HC-SR04", "e")
        return

    BASE_DISTANCE = round(d, 2)
    FORCE_BASE = False          # tu nay dung gia tri hieu chuan, ke ca sau khi reset

    log("Hiệu chuẩn đáy = " + str(BASE_DISTANCE) + " cm", "o")
    save_settings()


def cmd_reset_stats():
    for k in stats:
        stats[k] = 0

    log("Đã đặt lại thống kê")


def cmd_clear_hist():
    del hist_buf[:]


def cmd_clear_log():
    del log_buf[:]


def handle_api(path, q):
    try:
        if path == "/api/belt":
            cmd_belt(q.get("run") == "1")

        elif path == "/api/speed":
            cmd_speed(int(float(q.get("hz", STEP_FREQUENCY))))

        elif path == "/api/auto":
            cmd_auto(q.get("on") == "1")

        elif path == "/api/thr":
            cmd_threshold(float(q.get("cm", HEIGHT_THRESHOLD)))

        elif path == "/api/pick":
            cmd_pick()

        elif path == "/api/estop":
            cmd_estop()

        elif path == "/api/resume":
            cmd_resume()

        elif path == "/api/dir":
            cmd_dir(q.get("v") == "1")

        elif path == "/api/servo":
            cmd_servo(int(q.get("i", "0")), int(float(q.get("a", "90"))))

        elif path == "/api/goto":
            cmd_goto(q.get("p", ""))

        elif path == "/api/savewp":
            cmd_save_wp(q.get("p", ""))

        elif path == "/api/sdelay":
            cmd_sdelay(int(float(q.get("ms", SERVO_STEP_DELAY_MS))))

        elif path == "/api/level":
            cmd_level(q.get("v") == "1")

        elif path == "/api/measure":
            cmd_measure()

        elif path == "/api/calib":
            cmd_calibrate()

        elif path == "/api/resetstats":
            cmd_reset_stats()

        elif path == "/api/clearhist":
            cmd_clear_hist()

        elif path == "/api/clearlog":
            cmd_clear_log()

    except Exception as e:
        log("Lỗi lệnh web: " + str(e), "e")


def send_all(cl, data):
    mv = memoryview(data)
    while len(mv):
        try:
            n = cl.send(mv)
        except OSError as e:
            if e.args and e.args[0] == 11:   # EAGAIN
                time.sleep_ms(2)
                continue
            raise
        if n:
            mv = mv[n:]


def send_text(cl, status, ctype, body):
    if isinstance(body, str):
        body = body.encode()
    head = (
        "HTTP/1.0 " + status + "\r\n"
        "Content-Type: " + ctype + "\r\n"
        "Cache-Control: no-store\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Connection: close\r\n\r\n"
    )
    send_all(cl, head.encode())
    send_all(cl, body)


def send_file(cl, name):
    try:
        f = open(name, "rb")
    except OSError:
        send_text(cl, "404 Not Found", "text/plain",
                  "Thieu file index.html tren ESP32")
        return

    send_all(cl, (
        "HTTP/1.0 200 OK\r\n"
        "Content-Type: text/html; charset=utf-8\r\n"
        "Cache-Control: no-cache\r\n"
        "Connection: close\r\n\r\n"
    ).encode())

    buf = bytearray(1024)
    mv = memoryview(buf)

    while True:
        n = f.readinto(buf)
        if not n:
            break
        send_all(cl, mv[:n])

    f.close()


def handle_client(cl):
    cl.settimeout(2)

    req = cl.recv(1024)
    if not req:
        return

    line = req.split(b"\r\n", 1)[0].decode()
    parts = line.split(" ")
    if len(parts) < 2:
        return

    path, _, qs = parts[1].partition("?")

    q = {}
    for kv in qs.split("&"):
        if kv:
            k, _, v = kv.partition("=")
            q[k] = v

    if path == "/" or path == "/index.html":
        send_file(cl, "index.html")

    elif path.startswith("/api/"):
        handle_api(path, q)

        try:
            since = int(q.get("since", "0"))
        except ValueError:
            since = 0

        try:
            hsince = int(q.get("hs", "0"))
        except ValueError:
            hsince = 0

        send_text(cl, "200 OK", "application/json",
                  build_state(since, hsince))

    else:
        send_text(cl, "404 Not Found", "text/plain", "Not found")


server = None

def web_start():
    global server

    server = socket.socket()
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("0.0.0.0", WEB_PORT))
    server.listen(4)
    server.setblocking(False)

    print("Web: http://" + ip_addr + ":" + str(WEB_PORT))


def web_service():
    try:
        cl, _ = server.accept()
    except OSError:
        return

    try:
        handle_client(cl)
    except Exception as e:
        print("Web loi:", e)
    finally:
        try:
            cl.close()
        except Exception:
            pass

    gc.collect()

# ============================================================
# 9. KHOI DONG
# ============================================================

print("--------------------------------")
print("HE THONG PHAN LOAI SAN PHAM")
print("--------------------------------")

load_settings()
dir_pin.value(BELT_DIRECTION)

set_all_servos(HOME)

current_angles = HOME[:]

time.sleep_ms(1000)

ip_addr = wifi_connect()
web_start()

log("Hệ thống khởi động - web tại " + ip_addr, "o")
log("Đáy đang dùng = " + str(BASE_DISTANCE) + " cm", "o")

if auto_mode:
    belt_start()

log("Hệ thống sẵn sàng", "o")

print(
    "LM393 DO =",
    light_sensor.value(),
    "| muc kich hoat =",
    LIGHT_ACTIVE_LEVEL
)

# ============================================================
# 10. VONG LAP CHINH
# ============================================================

last_sensor_check = 0

while True:

    now = time.ticks_ms()

    # Phuc vu web (khong chan)
    web_service()

    # PWM phan cung tu tao xung STEP
    robot_service()

    # Robot di chuyen theo lenh tay tu web
    if manual_moving and not robot_busy:
        if robot_move_service():
            manual_moving = False
            move_target = None

    # Cap nhat cam bien
    sensor_clear_service()

    # Robot ve HOME, co vat cao dang cho
    if (not robot_busy) and pending_high and not estop:

        pending_high = False

        log("Robot sẵn sàng - gắp sản phẩm cao tiếp theo")

        belt_stop()
        robot_start()

    # Kiem tra san pham moi (chi khi che do tu dong BAT)
    if (
        auto_mode
        and not estop
        and belt_running
        and sensor_armed
        and not waiting_clear_low
        and not waiting_clear_pick
    ):

        if time.ticks_diff(
            now,
            last_sensor_check
        ) >= 10:

            last_sensor_check = now

            if object_detected_stable():

                sensor_armed = False

                belt_stop()

                log("Phát hiện sản phẩm - đang đo chiều cao")

                distance, height = measure_height()

                if height is not None and height <= MIN_OBJECT_HEIGHT:

                    # Do cao chi dao dong <= 0.5 cm -> khong phai vat, bo qua
                    log(
                        "Cao " + str(round(height, 2))
                        + " cm <= " + str(MIN_OBJECT_HEIGHT)
                        + " cm - không phải vật, bỏ qua",
                        "w"
                    )

                    waiting_clear_low = True
                    low_clear_start = None

                    belt_start()

                elif height is None:

                    stats["total"] += 1
                    stats["err"] += 1
                    record(None, None, "E")

                    log(
                        "LỖI HC-SR04 - dừng an toàn, "
                        "kiểm tra cảm biến (nhấn Chạy để tiếp tục)",
                        "e"
                    )

                else:

                    stats["total"] += 1

                    log(
                        "Đo 9 lần (đã lọc nhiễu): khoảng cách " + str(round(distance, 2))
                        + " cm | cao " + str(round(height, 2)) + " cm"
                    )

                    if height <= HEIGHT_THRESHOLD:

                        log("Sản phẩm thấp - cho đi qua")

                        stats["low"] += 1
                        record(distance, height, "L")

                        waiting_clear_low = True
                        low_clear_start = None

                        belt_start()

                    else:

                        log("Sản phẩm cao", "w")

                        stats["high"] += 1
                        record(distance, height, "H")

                        if not robot_busy:

                            robot_start()

                        else:

                            pending_high = True

                            log(
                                "Robot đang bận - "
                                "sản phẩm cao chờ lượt sau",
                                "w"
                            )

    # Robot tai P3, cho LM393 bao vat da duoc nhac ra
    if (
        robot_busy
        and robot_stage == 3
        and waiting_clear_pick
    ):
        pass

    elif (
        robot_busy
        and robot_stage == 3
        and not waiting_clear_pick
    ):

        waiting_clear_pick = True
        pick_clear_start = None

        log("Chờ sản phẩm rời khỏi cảm biến")

    time.sleep_ms(1)
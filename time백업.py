import time
import cv2
import threading
from gpiozero import DigitalOutputDevice, PWMOutputDevice, Button, DistanceSensor, TonalBuzzer
from gpiozero.pins.pigpio import PiGPIOFactory
from picamera2 import Picamera2

# ======================
# pigpio 설정
# ======================
factory = PiGPIOFactory()

# ======================
# 설정값
# ======================
SAFE_DISTANCE = 20          # 장애물 감지 거리 (cm)
forward_speed = 0.6
backward_speed = 0.4
turn_speed = 0.6
TURN_TIME = 0.3
TIMEOUT = 10                # 회피 반복 제한 시간 (초)

# ======================
# 버튼 설정
# ======================
SW1 = Button(5, pull_up=False)
SW2 = Button(6, pull_up=False)

# ======================
# 모터 핀 설정
# ======================
PWMA = PWMOutputDevice(18)
AIN1 = DigitalOutputDevice(22)
AIN2 = DigitalOutputDevice(27)
PWMB = PWMOutputDevice(23)
BIN1 = DigitalOutputDevice(25)
BIN2 = DigitalOutputDevice(24)

# ======================
# 부저
# ======================
BUZZER = TonalBuzzer(12)

# ======================
# 초음파 센서 (전방)
# ======================
sensor = DistanceSensor(
    echo=7,
    trigger=8,
    pin_factory=factory,
    max_distance=3.0,
    queue_len=5
)

# ======================
# 모터 상태 관리
# ======================
current_state = "stop"  # "forward", "left", "stop"

def forward(speed=forward_speed):
    global current_state
    if current_state != "forward":
        print("➡️ 직진")
        AIN1.value = 0
        AIN2.value = 1
        PWMA.value = 0.5
        BIN1.value = 0
        BIN2.value = 1
        PWMB.value = 0.5
        current_state = "forward"

def backward(speed=backward_speed):
    global current_state
    print("⬅️ 후진")
    AIN1.value, AIN2.value = 1, 0
    BIN1.value, BIN2.value = 1, 0
    PWMA.value = PWMB.value = speed
    current_state = "backward"

def left_turn(speed=turn_speed, t=TURN_TIME):
    global current_state
    print(f"↩️ 좌회전 (시간: {t}s)")
    AIN1.value, AIN2.value = 0, 1
    BIN1.value, BIN2.value = 1, 0
    PWMA.value = PWMB.value = speed
    current_state = "left"
    time.sleep(t)
    stop()

def stop():
    global current_state
    if current_state != "stop":
        AIN1.value = AIN2.value = BIN1.value = BIN2.value = 0
        PWMA.value = PWMB.value = 0
        current_state = "stop"
        print("🛑 정지")

# ======================
# 주행 상태 관리
# ======================
driving = False
rear_alert = False

def toggle_drive():
    global driving
    driving = not driving
    print(f"🚗 {'주행 시작' if driving else '주행 정지'}")
    if not driving:
        stop()

SW1.when_pressed = toggle_drive
SW2.when_pressed = stop

# ======================
# 후방 카메라 감지 스레드
# ======================
rear_alert_lock = threading.Lock()

def rear_camera_thread():
    global rear_alert
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"format":"XRGB8888","size":(640,480)})
    picam2.configure(config)
    picam2.start()

    fgbg = cv2.createBackgroundSubtractorMOG2(history=50, varThreshold=50, detectShadows=False)

    while True:
        frame = picam2.capture_array()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        fgmask = fgbg.apply(gray)
        count = cv2.countNonZero(fgmask)

        with rear_alert_lock:
            rear_alert = count > 5000  # 후방에 물체 있으면 True

        time.sleep(0.1)

# ======================
# 메인 루프
# ======================
try:
    threading.Thread(target=rear_camera_thread, daemon=True).start()

    while True:
        if driving:
            front_dist = sensor.distance * 100
            print(f"📏 전방 거리: {front_dist:.1f} cm")

            start_time = time.time()
            if front_dist < SAFE_DISTANCE:
                # 전방 물체 가까움 → 후진 + 좌회전 회피
                stop()
                time.sleep(0.2)
                while sensor.distance * 100 < SAFE_DISTANCE:
                    with rear_alert_lock:
                        if rear_alert:
                            print("⚠️ 후방 막힘 → 정지 + 부저")
                            BUZZER.play(391)
                            time.sleep(1)
                            BUZZER.stop()
                            stop()
                            break
                        else:
                            print("✅ 후방 안전 → 후진 + 좌회전")
                            backward(backward_speed)
                            time.sleep(0.5)
                            left_turn(turn_speed, t=TURN_TIME)
                            stop()

                    if time.time() - start_time > TIMEOUT:
                        print("⏱️ 회피 시간 초과 → 정지 + 부저")
                        BUZZER.play(391)
                        time.sleep(1)
                        BUZZER.stop()
                        stop()
                        break
                    time.sleep(0.05)
            else:
                # 전방 안전 → 제자리 좌회전 반복
                while sensor.distance * 100 < SAFE_DISTANCE:
                    print("↩️ 제자리 좌회전 (전방 확보 중)")
                    left_turn(turn_speed, t=TURN_TIME)
                    if time.time() - start_time > TIMEOUT:
                        print("⏱️ 회피 시간 초과 → 정지 + 부저")
                        BUZZER.play(391)
                        time.sleep(1)
                        BUZZER.stop()
                        stop()
                        break

                forward(forward_speed)

        time.sleep(0.05)

except KeyboardInterrupt:
    stop()
    BUZZER.stop()
    print("프로그램 종료")

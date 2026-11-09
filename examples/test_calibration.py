import json
import time
from f32c import F32CBus, Mode

with open("camera_calibration.json", "r", encoding="utf-8") as f:
    pitch_vertical = json.load(f)["pitch_vertical"]

with F32CBus("COM8") as bus:
    pitch = bus.motor(2)

    current = pitch.read_angle()
    target = pitch_vertical

    # 防止角度标定失效后出现意外的大角度旋转
    if abs(target - current) > 120:
        raise RuntimeError(
            f"当前角度 {current:.1f}°，垂直标定角度 {target:.1f}°，"
            "两者相差过大，请重新检查标定，不自动运动。"
        )

    pitch.enable()
    pitch.set_mode(Mode.MULTI_POSITION_DIRECT)
    pitch.set_speed(5)

    # 先保持当前位置，再平滑移动到垂直角度
    pitch.set_angle(current)
    time.sleep(0.1)

    duration = max(2.0, abs(target - current) / 10.0)
    start = time.monotonic()

    while True:
        progress = min(1.0, (time.monotonic() - start) / duration)
        smooth = progress * progress * (3.0 - 2.0 * progress)
        pitch.set_angle(current + (target - current) * smooth)
        if progress >= 1.0:
            break
        time.sleep(0.02)

    # 到位后继续保持，不失能
    while True:
        pitch.set_angle(target)
        time.sleep(0.02)
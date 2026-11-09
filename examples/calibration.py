import json
from f32c import F32CBus

with F32CBus("COM8") as bus:
    pitch = bus.motor(2)  # 上方电机地址为 2

    # 此时先手动把相机调整到垂直姿态
    pitch_vertical = pitch.read_angle()

with open("camera_calibration.json", "w", encoding="utf-8") as f:
    json.dump({"pitch_vertical": pitch_vertical}, f, indent=2)

print("垂直姿态标定角度：", pitch_vertical)
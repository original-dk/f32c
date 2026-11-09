import json
from f32c import F32CBus

with F32CBus("COM8") as bus:
    yaw = bus.motor(1)  # 下方电机地址为 1
    pitch = bus.motor(2)  # 上方电机地址为 2

    yaw_init = yaw.read_angle()
    pitch_init = pitch.read_angle()

with open("calibration.json", "w", encoding="utf-8") as f:
    json.dump({"yaw_init": yaw_init, "pitch_init": pitch_init}, f, indent=2)

print("yaw标定角度：", yaw_init, "pitch标定角度：", pitch_init)
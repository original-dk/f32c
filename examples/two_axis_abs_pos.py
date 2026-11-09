"""双轴云台直通位置与保持示例；必须显式添加 --confirm 才会使能电机。

示例：python examples/two_axis_position.py --port COM5 \
          --yaw-delta 10 --pitch-delta -5 --speed 20 --hold-time 5 --confirm

角度变化量相对于程序启动时读到的多圈绝对角度。与 F32C 云台绘图例程一致，
本示例使用多圈位置直通模式（0x0003），并周期重发目标位置。到位后继续
保持 --hold-time 秒，然后退出并尝试失能两轴；失能后云台可能因重力下落。
"""

import argparse
import json
import time

from f32c import F32CBus, Mode


def update_hold_phase(in_tolerance, now, deadline, reached_at, hold_time):
    """推进“等待首次到位→保持”状态，返回 (首次到位时间, 事件)。

    一旦 reached_at 不为 None，就锁定在保持状态：后续反馈波动不会重新
    启用启动超时，也不会再次产生 reached 事件。
    """
    if reached_at is None:
        if in_tolerance:
            return now, "reached"
        if now >= deadline:
            return None, "timeout"
        return None, "waiting"
    if hold_time > 0 and now - reached_at >= hold_time:
        return reached_at, "finished"
    return reached_at, "holding"


def main() -> int:
    parser = argparse.ArgumentParser(description="F32C 双轴直通位置与保持示例")
    parser.add_argument("--port", required=True, help="串口，例如 COM5 或 /dev/ttyUSB0")
    parser.add_argument("--yaw-address", type=int, default=1)
    parser.add_argument("--pitch-address", type=int, default=2)
    parser.add_argument("--yaw-delta", type=float, default=0.0, help="yaw 相对角度，单位：度")
    parser.add_argument("--pitch-delta", type=float, default=0.0, help="pitch 相对角度，单位：度")
    parser.add_argument("--speed", type=int, default=20, help="位置模式运行速度，单位：RPM")
    parser.add_argument("--tolerance", type=float, default=0.3, help="到位容差，单位：度")
    parser.add_argument("--timeout", type=float, default=10.0, help="最长等待首次到位时间，单位：秒")
    parser.add_argument("--startup-delay", type=float, default=1.5, help="上电启动等待时间")
    parser.add_argument(
        "--hold-time",
        type=float,
        default=5.0,
        help="到位后继续保持的秒数；0 表示一直保持直到 Ctrl+C，默认 5 秒",
    )
    parser.add_argument(
        "--command-period",
        type=float,
        default=0.02,
        help="目标位置重发周期，单位：秒，默认 0.02",
    )
    parser.add_argument("--confirm", action="store_true", help="确认接线及机械安全，允许使能电机")
    args = parser.parse_args()

    if not args.confirm:
        parser.error("本示例会使能并移动电机；确认安全后必须显式添加 --confirm")
    if args.yaw_address == args.pitch_address:
        parser.error("两轴不能使用相同的电机地址")
    if args.speed <= 0 or args.tolerance <= 0 or args.timeout <= 0:
        parser.error("speed、tolerance 和 timeout 必须大于 0")
    if args.hold_time < 0 or args.command_period <= 0:
        parser.error("hold-time 不能小于 0，command-period 必须大于 0")

    with open("calibration.json", "r", encoding="utf-8") as f:
        data = json.load(f)
        yaw_init = data["yaw_init"]
        pitch_init = data["pitch_init"]

    with F32CBus(args.port) as bus:
        yaw = bus.motor(args.yaw_address)
        pitch = bus.motor(args.pitch_address)
        try:
            bus.wake()
            time.sleep(max(0.0, args.startup_delay))

            yaw_start = yaw_init
            pitch_start = pitch_init
            yaw_target = yaw_start + args.yaw_delta
            pitch_target = pitch_start + args.pitch_delta
            print(f"yaw：{yaw_start:.1f}° → {yaw_target:.1f}°")
            print(f"pitch：{pitch_start:.1f}° → {pitch_target:.1f}°")
            print("模式：多圈位置直通（0x0003）；目标将周期重发。")

            for motor in (yaw, pitch):
                motor.enable()
                motor.set_mode(Mode.MULTI_POSITION_DIRECT)
                motor.set_speed(args.speed)

            # 先以当前位置为目标，避免切换到直通模式后出现不必要的初始跳动。
            yaw.set_angle(yaw_start)
            pitch.set_angle(pitch_start)
            time.sleep(0.1)

            deadline = time.monotonic() + args.timeout
            reached_at = None
            next_feedback = 0.0
            while True:
                yaw.set_angle(yaw_target)
                pitch.set_angle(pitch_target)
                now = time.monotonic()

                if now >= next_feedback:
                    yaw_now = yaw.read_angle()
                    pitch_now = pitch.read_angle()
                    now = time.monotonic()
                    print(f"当前：yaw={yaw_now:.1f}°，pitch={pitch_now:.1f}°")
                    next_feedback = now + 0.2

                    in_tolerance = (
                        abs(yaw_now - yaw_target) <= args.tolerance
                        and abs(pitch_now - pitch_target) <= args.tolerance
                    )
                    reached_at, event = update_hold_phase(
                        in_tolerance=in_tolerance,
                        now=now,
                        deadline=deadline,
                        reached_at=reached_at,
                        hold_time=args.hold_time,
                    )
                    if event == "reached":
                        if args.hold_time == 0:
                            print("两轴均已首次到位；将持续保持，按 Ctrl+C 退出并失能。")
                        else:
                            print(
                                f"两轴均已首次到位；继续保持 {args.hold_time:g} 秒，"
                                "随后退出并失能。"
                            )
                    elif event == "timeout":
                        print("等待首次到位超时；即将尝试失能两轴。")
                        return 1
                    elif event == "finished":
                        print("保持时间结束；即将退出并失能两轴。")
                        return 0
                time.sleep(args.command_period)
        except KeyboardInterrupt:
            print("\n收到 Ctrl+C；即将尝试失能两轴。")
            return 0
        finally:
            for motor in (yaw, pitch):
                try:
                    motor.disable()
                    print("\n使能已经失效。")
                except Exception as exc:  # 尽量继续失能另一台电机
                    print(f"地址 0x{motor.address:02X} 失能命令发送失败：{exc}")


if __name__ == "__main__":
    raise SystemExit(main())

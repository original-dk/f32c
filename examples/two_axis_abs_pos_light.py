"""双轴云台多圈位置、平滑轨迹与保持示例；必须显式添加 --confirm。

直通模式（默认）：Python 用 smoothstep 周期发送位置目标。
示例：python examples/two_axis_abs_pos.py --port COM8 \
          --yaw-delta 5 --pitch-delta 5 --speed 50 --move-time 3 --hold-time 0 --confirm

规划模式（--planned）：电机内部生成轨迹，稳定后最终目标只发送一次。
示例：python examples/two_axis_abs_pos.py --port COM8 \
          --yaw-delta 5 --pitch-delta 5 --speed 50 --planned --hold-time 0 --confirm

最终目标 = calibration.json 中的标定参考角度 + 命令行增量。
实际起点 = 本次启动时通过 read_angle() 读到的两轴多圈绝对角度。

--speed 是电机内部位置速度参数；--move-time 只控制直通模式的 Python 平滑轨迹时长。
二者相互独立：可以用较稳定的速度参数，同时让激光笔缓慢转动。
到位后继续保持 --hold-time 秒，然后退出并尝试失能两轴；失能后机构可能下落。
"""

import argparse
import json
import time

from f32c import F32CBus, Mode


def update_hold_phase(in_tolerance, now, deadline, reached_at, hold_time):
    """推进“等待首次到位→保持”状态，返回 (首次到位时间, 事件)。"""
    if reached_at is None:
        if in_tolerance:
            return now, "reached"
        if now >= deadline:
            return None, "timeout"
        return None, "waiting"
    if hold_time > 0 and now - reached_at >= hold_time:
        return reached_at, "finished"
    return reached_at, "holding"


def smoothstep01(progress):
    """把进度限制到 0～1，并返回起止斜率为零的 smoothstep 插值系数。"""
    u = min(1.0, max(0.0, progress))
    return u * u * (3.0 - 2.0 * u)


def main() -> int:
    parser = argparse.ArgumentParser(description="F32C 双轴多圈位置、平滑轨迹与保持示例")
    parser.add_argument("--port", required=True, help="串口，例如 COM5 或 /dev/ttyUSB0")
    parser.add_argument("--yaw-address", type=int, default=1)
    parser.add_argument("--pitch-address", type=int, default=2)
    parser.add_argument("--yaw-delta", type=float, default=0.0, help="yaw 相对标定参考的增量，单位：度")
    parser.add_argument("--pitch-delta", type=float, default=0.0, help="pitch 相对标定参考的增量，单位：度")
    parser.add_argument("--speed", type=int, default=20, help="电机内部位置速度参数，单位：RPM")
    parser.add_argument(
        "--hold-speed",
        type=int,
        default=0,
        help="首次到位后的保持速度参数，单位：RPM；0 表示沿用 --speed",
    )
    parser.add_argument(
        "--move-time",
        type=float,
        default=0.0,
        help="直通模式的 Python 平滑轨迹时长，单位：秒；0 表示直接发送最终目标",
    )
    parser.add_argument(
        "--planned",
        action="store_true",
        help="使用电机内部多圈位置规划模式；最终目标只发送一次，忽略 move-time",
    )
    parser.add_argument("--tolerance", type=float, default=0.3, help="到位容差，单位：度")
    parser.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        help="直通轨迹结束或规划目标发送后，最长等待首次到位的时间，单位：秒",
    )
    parser.add_argument("--startup-delay", type=float, default=1.5, help="上电启动等待时间")
    parser.add_argument(
        "--settle-time",
        type=float,
        default=0.5,
        help="使能后先保持实际起点的稳定时间，单位：秒，默认 0.5",
    )
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
        help="直通目标重发及主循环周期，单位：秒，默认 0.02",
    )
    parser.add_argument("--confirm", action="store_true", help="确认接线及机械安全，允许使能电机")
    args = parser.parse_args()

    if not args.confirm:
        parser.error("本示例会使能并移动电机；确认安全后必须显式添加 --confirm")
    if args.yaw_address == args.pitch_address:
        parser.error("两轴不能使用相同的电机地址")
    if args.speed <= 0 or args.tolerance <= 0 or args.timeout <= 0:
        parser.error("speed、tolerance 和 timeout 必须大于 0")
    if args.hold_speed < 0:
        parser.error("hold-speed 不能小于 0；0 表示沿用 speed")
    if args.move_time < 0 or args.settle_time < 0:
        parser.error("move-time 和 settle-time 不能小于 0")
    if args.hold_time < 0 or args.command_period <= 0:
        parser.error("hold-time 不能小于 0，command-period 必须大于 0")

    with open("calibration.json", "r", encoding="utf-8") as f:
        data = json.load(f)
        yaw_reference = float(data["yaw_init"])
        pitch_reference = float(data["pitch_init"])

    yaw_target = yaw_reference + args.yaw_delta
    pitch_target = pitch_reference + args.pitch_delta

    with F32CBus(args.port) as bus:
        yaw = bus.motor(args.yaw_address)
        pitch = bus.motor(args.pitch_address)
        try:
            bus.wake()
            time.sleep(max(0.0, args.startup_delay))

            # 标定参考用于计算最终目标；实际起点必须来自本次电机反馈。
            yaw_start = yaw.read_angle()
            pitch_start = pitch.read_angle()
            print(f"标定参考：yaw={yaw_reference:.1f}°，pitch={pitch_reference:.1f}°")
            print(f"实际起点：yaw={yaw_start:.1f}°，pitch={pitch_start:.1f}°")
            print(f"最终目标：yaw={yaw_target:.1f}°，pitch={pitch_target:.1f}°")
            selected_mode = Mode.MULTI_POSITION if args.planned else Mode.MULTI_POSITION_DIRECT
            if args.planned:
                motion_description = "多圈位置规划（0x0001），最终目标只发送一次"
            else:
                motion_description = f"多圈位置直通（0x0003），Python 平滑时间={args.move_time:g} 秒"
            print(f"模式：{motion_description}；内部速度参数={args.speed} RPM。")

            # 失能状态下先设置模式和速度，并预装载真实当前位置，避免使能后沿用旧目标。
            for motor in (yaw, pitch):
                motor.set_mode(selected_mode)
                motor.set_speed(args.speed)
            yaw.set_angle(yaw_start)
            pitch.set_angle(pitch_start)

            # 两轴目标都已准备好后再使能，并立即重发当前位置。
            for motor in (yaw, pitch):
                motor.enable()

            settle_deadline = time.monotonic() + args.settle_time
            while True:
                yaw.set_angle(yaw_start)
                pitch.set_angle(pitch_start)
                if time.monotonic() >= settle_deadline:
                    break
                time.sleep(args.command_period)

            # 使能后可能有很小的负载沉降；以稳定后的真实位置重新开始平滑轨迹。
            yaw_start = yaw.read_angle()
            pitch_start = pitch.read_angle()
            yaw.set_angle(yaw_start)
            pitch.set_angle(pitch_start)
            print(f"稳定后起点：yaw={yaw_start:.1f}°，pitch={pitch_start:.1f}°")
            time.sleep(0.1)

            if args.planned:
                # 稳定后只发送一次最终目标；之后不再用 Python 周期重发位置命令。
                yaw.set_angle(yaw_target)
                pitch.set_angle(pitch_target)
                print("最终目标已发送；后续由电机内部规划，不再周期重发位置目标。")

            # 规划模式的等待超时从最终目标发送后开始计算。
            trajectory_started = time.monotonic()
            trajectory_duration = 0.0 if args.planned else args.move_time
            deadline = trajectory_started + trajectory_duration + args.timeout
            reached_at = None
            next_feedback = 0.0

            while True:
                now = time.monotonic()
                if args.planned:
                    progress = 1.0
                else:
                    if args.move_time > 0:
                        progress = (now - trajectory_started) / args.move_time
                    else:
                        progress = 1.0
                    blend = smoothstep01(progress)

                    yaw_command = yaw_start + (yaw_target - yaw_start) * blend
                    pitch_command = pitch_start + (pitch_target - pitch_start) * blend
                    yaw.set_angle(yaw_command)
                    pitch.set_angle(pitch_command)

                if now >= next_feedback:
                    yaw_now = yaw.read_angle()
                    pitch_now = pitch.read_angle()
                    now = time.monotonic()
                    if args.planned:
                        motion_status = "电机内部规划"
                    else:
                        motion_status = f"轨迹进度={min(1.0, max(0.0, progress)) * 100:.0f}%"
                    print(
                        f"当前：yaw={yaw_now:.1f}°，pitch={pitch_now:.1f}°；"
                        f"{motion_status}"
                    )
                    next_feedback = now + 0.2

                    # 直通轨迹尚未结束时，即使中途经过最终目标，也不提前进入保持阶段。
                    trajectory_done = args.planned or progress >= 1.0
                    in_tolerance = (
                        trajectory_done
                        and abs(yaw_now - yaw_target) <= args.tolerance
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
                        if args.hold_speed > 0 and args.hold_speed != args.speed:
                            for motor in (yaw, pitch):
                                motor.set_speed(args.hold_speed)
                            print(
                                f"已切换保持速度参数：{args.speed} → {args.hold_speed} RPM；"
                                "目标角度、模式和使能状态不变。"
                            )
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
"""双轴多圈位置规划与保持示例；必须显式添加 --confirm 才会使能电机。

示例：
# python examples/two_axis_abs_pos_light.py --port COM8 --yaw-delta 0 --pitch-delta 0 --speed 50 --planned --hold-time 0 --confirm
角度变化量相对于 calibration.json 中记录的多圈绝对角度。

本示例使用带 T 型轨迹规划的多圈位置模式（0x0001）。
默认按照官方 C 位置例程，周期重发相同的最终目标；
添加 --send-once 可切换为只发送一次最终目标。

主机只负责发送最终目标和读取反馈，中间轨迹由电机内部规划。
到位后继续保持 --hold-time 秒，然后退出并尝试失能两轴。
"""

import argparse
import json
import time

from f32c import F32CBus, FeedbackTimeoutError, Mode


def update_hold_phase(
    in_tolerance,
    now,
    deadline,
    reached_at,
    hold_time,
):
    """推进“等待首次到位→保持”状态，返回 (首次到位时间, 事件)。

    一旦 reached_at 不为 None，就锁定在保持状态：
    后续反馈波动不会重新启用启动超时，也不会再次产生 reached 事件。
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
    parser = argparse.ArgumentParser(
        description="F32C 双轴多圈位置规划与保持示例"
    )

    parser.add_argument(
        "--port",
        required=True,
        help="串口，例如 COM5、COM8 或 /dev/ttyUSB0",
    )
    parser.add_argument(
        "--yaw-address",
        type=int,
        default=1,
        help="yaw 电机地址，默认 1",
    )
    parser.add_argument(
        "--pitch-address",
        type=int,
        default=2,
        help="pitch 电机地址，默认 2",
    )
    parser.add_argument(
        "--yaw-delta",
        type=float,
        default=0.0,
        help="yaw 相对角度，单位：度",
    )
    parser.add_argument(
        "--pitch-delta",
        type=float,
        default=0.0,
        help="pitch 相对角度，单位：度",
    )
    parser.add_argument(
        "--speed",
        type=int,
        default=20,
        help="位置模式运行速度，单位：RPM",
    )
    parser.add_argument(
        "--tolerance",
        type=float,
        default=0.3,
        help="到位容差，单位：度",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        help="最长等待首次到位时间，单位：秒",
    )
    parser.add_argument(
        "--startup-delay",
        type=float,
        default=1.5,
        help="上电启动等待时间，单位：秒",
    )
    parser.add_argument(
        "--hold-time",
        type=float,
        default=5.0,
        help="到位后继续保持的秒数；0 表示一直保持直到 Ctrl+C",
    )
    parser.add_argument(
        "--feedback-period",
        type=float,
        default=0.05,
        help="反馈读取及目标重发间隔，单位：秒，默认 0.05",
    )
    parser.add_argument(
        "--feedback-timeout",
        type=float,
        default=0.5,
        help="单轴位置反馈请求超时，单位：秒，默认 0.5",
    )
    parser.add_argument(
        "--send-once",
        action="store_true",
        help=(
            "只发送一次最终目标；默认按照官方 C 例程，"
            "在后续循环中重发相同目标"
        ),
    )
    parser.add_argument(
        "--confirm",
        action="store_true",
        help="确认接线及机械安全，允许使能电机",
    )

    args = parser.parse_args()

    if not args.confirm:
        parser.error(
            "本示例会使能并移动电机；"
            "确认安全后必须显式添加 --confirm"
        )

    if args.yaw_address == args.pitch_address:
        parser.error("两轴不能使用相同的电机地址")

    if (
        args.speed <= 0
        or args.tolerance <= 0
        or args.timeout <= 0
    ):
        parser.error(
            "speed、tolerance 和 timeout 必须大于 0"
        )

    if (
        args.hold_time < 0
        or args.feedback_period <= 0
        or args.feedback_timeout <= 0
    ):
        parser.error(
            "hold-time 不能小于 0，"
            "feedback-period 和 feedback-timeout 必须大于 0"
        )

    with open(
        "calibration.json",
        "r",
        encoding="utf-8",
    ) as f:
        data = json.load(f)
        yaw_init = data["yaw_init"]
        pitch_init = data["pitch_init"]

    # 官方 C 位置例程在位置命令和反馈请求之间使用 10 ms 延时。
    # 这里将 SDK 默认的 2 ms 命令间隔提高到 10 ms。
    with F32CBus(
        args.port,
        command_interval=0.01,
    ) as bus:
        yaw = bus.motor(args.yaw_address)
        pitch = bus.motor(args.pitch_address)

        try:
            bus.wake()
            time.sleep(max(0.0, args.startup_delay))

            yaw_start = yaw_init
            pitch_start = pitch_init

            yaw_target = yaw_start + args.yaw_delta
            pitch_target = pitch_start + args.pitch_delta

            print(
                f"yaw：{yaw_start:.1f}° → "
                f"{yaw_target:.1f}°"
            )
            print(
                f"pitch：{pitch_start:.1f}° → "
                f"{pitch_target:.1f}°"
            )
            print("模式：多圈位置规划（0x0001）。")

            if args.send_once:
                print("目标策略：最终目标只发送一次。")
            else:
                print(
                    "目标策略：按照官方 C 例程，"
                    "周期重发相同的最终目标。"
                )

            for motor in (yaw, pitch):
                motor.enable()
                motor.set_mode(Mode.MULTI_POSITION)
                motor.set_speed(args.speed)

            # 先以记录的位置为目标，降低切换模式后初始跳动的风险。
            yaw.set_angle(yaw_start)
            pitch.set_angle(pitch_start)
            time.sleep(0.1)

            # 发送最终运动目标。中间轨迹由电机内部规划。
            yaw.set_angle(yaw_target)
            pitch.set_angle(pitch_target)

            # 给电机留出处理目标位置和开始轨迹规划的时间。
            time.sleep(0.05)

            deadline = time.monotonic() + args.timeout
            reached_at = None

            while True:
                try:
                    yaw_now = bus.read_multi_angle(
                        yaw.address,
                        timeout=args.feedback_timeout,
                    )
                    pitch_now = bus.read_multi_angle(
                        pitch.address,
                        timeout=args.feedback_timeout,
                    )
                except FeedbackTimeoutError as exc:
                    print(f"\n位置反馈读取失败：{exc}")
                    print(
                        "为避免通信异常时电机继续运动，"
                        "即将尝试失能两轴。"
                    )
                    return 1

                now = time.monotonic()

                print(
                    f"当前：yaw={yaw_now:.1f}°，"
                    f"pitch={pitch_now:.1f}°"
                )

                in_tolerance = (
                    abs(yaw_now - yaw_target)
                    <= args.tolerance
                    and abs(pitch_now - pitch_target)
                    <= args.tolerance
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
                        print(
                            "两轴均已首次到位；将持续保持，"
                            "按 Ctrl+C 退出并失能。"
                        )
                    else:
                        print(
                            "两轴均已首次到位；继续保持 "
                            f"{args.hold_time:g} 秒，"
                            "随后退出并失能。"
                        )

                elif event == "timeout":
                    print(
                        "等待首次到位超时；"
                        "即将尝试失能两轴。"
                    )
                    return 1

                elif event == "finished":
                    print(
                        "保持时间结束；"
                        "即将退出并失能两轴。"
                    )
                    return 0

                # 默认模拟官方 C 位置例程，继续重发相同目标。
                # 这不是主机轨迹插补，T 型规划仍由电机完成。
                if not args.send_once:
                    yaw.set_angle(yaw_target)
                    pitch.set_angle(pitch_target)

                time.sleep(args.feedback_period)

        except KeyboardInterrupt:
            print("\n收到 Ctrl+C；即将尝试失能两轴。")
            return 0

        finally:
            for motor in (yaw, pitch):
                try:
                    motor.disable()
                    print("\n使能已经失效。")
                except Exception as exc:
                    # 一台失能失败时，仍继续尝试另一台。
                    print(
                        f"地址 0x{motor.address:02X} "
                        f"失能命令发送失败：{exc}"
                    )


if __name__ == "__main__":
    raise SystemExit(main())
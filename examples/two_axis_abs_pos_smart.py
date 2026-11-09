"""双轴云台多圈位置直通、平滑轨迹与保持示例。

必须显式添加 --confirm 才会使能电机。

示例：
python examples/two_axis_abs_pos.py --port COM8 --yaw-delta 180 --pitch-delta 180 --speed 50 --move-time 1 --hold-time 0 --confirm

最终目标 = calibration.json 中的标定参考角度 + 命令行增量。
实际起点 = 本次启动时通过 read_angle() 读到的两轴多圈绝对角度。

本示例固定使用多圈位置直通模式（0x0003）。
Python 使用 smoothstep 生成中间绝对位置，并周期发送给两轴。

--speed 是电机内部位置速度参数；
--move-time 是 Python 平滑轨迹时长。

smoothstep 会使目标轨迹在起点和终点处的速度为 0。
轨迹结束且两轴位置都进入容差后，程序产生 reached 事件。

到位后继续保持 --hold-time 秒，然后退出并尝试失能两轴；
失能后机构可能因重力下落。
"""

import argparse
import json
import time

from f32c import F32CBus, Mode


def update_hold_phase(
    in_tolerance,
    now,
    deadline,
    reached_at,
    hold_time,
):
    """推进“等待首次到位→保持”状态。

    返回：
        (首次到位时间, 事件)

    事件包括：
        waiting：等待到位；
        reached：首次到位；
        timeout：等待到位超时；
        holding：正在保持；
        finished：保持时间结束。
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


def smoothstep01(progress):
    """把进度限制到 0～1，并返回起止斜率为零的插值系数。

    s(u) = 3u² - 2u³

    其导数为：
        s'(u) = 6u(1-u)

    因此 u=0 和 u=1 时，目标轨迹速度均为 0。
    """
    u = min(1.0, max(0.0, progress))
    return u * u * (3.0 - 2.0 * u)


def initialize_direct_motors(
    yaw,
    pitch,
    speed,
    settle_time,
    command_period,
):
    """初始化双轴直通位置模式，并返回稳定后的实际起点。

    初始化过程：

    1. 读取两轴真实当前位置；
    2. 在失能状态下设置直通模式和速度；
    3. 预装载真实当前位置，避免使能后沿用旧目标；
    4. 使能两轴；
    5. 在原位保持一段时间，等待负载和机构稳定；
    6. 重新读取稳定后的实际位置；
    7. 以新位置作为后续平滑轨迹的起点。

    返回：
        (yaw_start, pitch_start)
    """
    # 第一次读取：获得使能前的真实位置。
    yaw_start = yaw.read_angle()
    pitch_start = pitch.read_angle()

    print(
        f"使能前实际位置：yaw={yaw_start:.1f}°，"
        f"pitch={pitch_start:.1f}°"
    )

    # 失能状态下先设置模式和速度。
    for motor in (yaw, pitch):
        motor.set_mode(Mode.MULTI_POSITION_DIRECT)
        motor.set_speed(speed)

    # 第一次发送当前位置：预装载目标，降低使能跳动风险。
    yaw.set_angle(yaw_start)
    pitch.set_angle(pitch_start)

    # 两轴目标准备好后再使能。
    for motor in (yaw, pitch):
        motor.enable()

    # 第二次发送当前位置：使能后持续保持原位。
    settle_deadline = time.monotonic() + settle_time

    while True:
        yaw.set_angle(yaw_start)
        pitch.set_angle(pitch_start)

        if time.monotonic() >= settle_deadline:
            break

        time.sleep(command_period)

    # 第二次读取：使能后可能存在负载沉降，重新获取稳定起点。
    yaw_start = yaw.read_angle()
    pitch_start = pitch.read_angle()

    # 第三次发送当前位置：以稳定后的位置作为新的保持目标。
    yaw.set_angle(yaw_start)
    pitch.set_angle(pitch_start)

    print(
        f"稳定后轨迹起点：yaw={yaw_start:.1f}°，"
        f"pitch={pitch_start:.1f}°"
    )

    # 给最后一次原位目标留出执行时间。
    time.sleep(0.1)

    return yaw_start, pitch_start


def main() -> int:
    parser = argparse.ArgumentParser(
        description="F32C 双轴多圈位置直通、平滑轨迹与保持示例"
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
        help="yaw 相对标定参考的增量，单位：度",
    )
    parser.add_argument(
        "--pitch-delta",
        type=float,
        default=0.0,
        help="pitch 相对标定参考的增量，单位：度",
    )
    parser.add_argument(
        "--speed",
        type=int,
        default=20,
        help="电机内部位置速度参数，单位：RPM",
    )
    parser.add_argument(
        "--move-time",
        type=float,
        default=0.0,
        help=(
            "Python 平滑轨迹时长，单位：秒；"
            "0 表示直接发送最终目标"
        ),
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
        help=(
            "直通轨迹结束后，"
            "最长等待首次到位的时间，单位：秒"
        ),
    )
    parser.add_argument(
        "--startup-delay",
        type=float,
        default=1.5,
        help="上电启动等待时间，单位：秒",
    )
    parser.add_argument(
        "--settle-time",
        type=float,
        default=0.5,
        help=(
            "使能后先保持实际起点的稳定时间，"
            "单位：秒，默认 0.5"
        ),
    )
    parser.add_argument(
        "--hold-time",
        type=float,
        default=5.0,
        help=(
            "到位后继续保持的秒数；"
            "0 表示一直保持直到 Ctrl+C，默认 5 秒"
        ),
    )
    parser.add_argument(
        "--command-period",
        type=float,
        default=0.02,
        help=(
            "目标重发及主循环休眠间隔，"
            "单位：秒，默认 0.02"
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

    if args.move_time < 0 or args.settle_time < 0:
        parser.error(
            "move-time 和 settle-time 不能小于 0"
        )

    if args.hold_time < 0 or args.command_period <= 0:
        parser.error(
            "hold-time 不能小于 0，"
            "command-period 必须大于 0"
        )

    with open(
        "calibration.json",
        "r",
        encoding="utf-8",
    ) as f:
        data = json.load(f)
        yaw_reference = float(data["yaw_init"])
        pitch_reference = float(data["pitch_init"])

    # 最终目标始终相对于标定参考角度。
    yaw_target = yaw_reference + args.yaw_delta
    pitch_target = pitch_reference + args.pitch_delta

    with F32CBus(args.port) as bus:
        yaw = bus.motor(args.yaw_address)
        pitch = bus.motor(args.pitch_address)

        try:
            bus.wake()
            time.sleep(max(0.0, args.startup_delay))

            print(
                f"标定参考：yaw={yaw_reference:.1f}°，"
                f"pitch={pitch_reference:.1f}°"
            )
            print(
                f"最终目标：yaw={yaw_target:.1f}°，"
                f"pitch={pitch_target:.1f}°"
            )
            print(
                "模式：多圈位置直通（0x0003）；"
                f"内部速度参数={args.speed} RPM；"
                f"Python 平滑时间={args.move_time:g} 秒。"
            )

            # 初始化、使能、原位稳定，并获得真正的轨迹起点。
            yaw_start, pitch_start = initialize_direct_motors(
                yaw=yaw,
                pitch=pitch,
                speed=args.speed,
                settle_time=args.settle_time,
                command_period=args.command_period,
            )

            trajectory_started = time.monotonic()

            # 直通轨迹结束后，再允许等待 timeout 秒用于到位。
            deadline = (
                trajectory_started
                + args.move_time
                + args.timeout
            )

            reached_at = None
            next_feedback = 0.0

            while True:
                now = time.monotonic()

                if args.move_time > 0:
                    progress = (
                        (now - trajectory_started)
                        / args.move_time
                    )
                else:
                    progress = 1.0

                blend = smoothstep01(progress)

                # 两轴共用相同的 blend，因此按相同的
                # 归一化进度同时开始、同时结束。
                yaw_command = (
                    yaw_start
                    + (yaw_target - yaw_start) * blend
                )
                pitch_command = (
                    pitch_start
                    + (pitch_target - pitch_start) * blend
                )

                # 发送的是绝对目标角度，不是相对增量。
                yaw.set_angle(yaw_command)
                pitch.set_angle(pitch_command)

                if now >= next_feedback:
                    yaw_now = yaw.read_angle()
                    pitch_now = pitch.read_angle()

                    now = time.monotonic()

                    print(
                        f"当前：yaw={yaw_now:.1f}°，"
                        f"pitch={pitch_now:.1f}°；"
                        f"轨迹进度="
                        f"{min(1.0, max(0.0, progress)) * 100:.0f}%"
                    )

                    next_feedback = now + 0.2

                    # 直通轨迹尚未结束时，即使中途进入
                    # 最终位置容差，也不提前进入保持阶段。
                    trajectory_done = progress >= 1.0

                    position_reached = (
                        abs(yaw_now - yaw_target)
                        <= args.tolerance
                        and abs(pitch_now - pitch_target)
                        <= args.tolerance
                    )

                    # 轨迹结束且两轴位置都进入容差，才允许 reached。
                    in_tolerance = (
                        trajectory_done
                        and position_reached
                    )

                    reached_at, event = update_hold_phase(
                        in_tolerance=in_tolerance,
                        now=now,
                        deadline=deadline,
                        reached_at=reached_at,
                        hold_time=args.hold_time,
                    )

                    if event == "reached":
                        # 不再修改速度参数，继续使用 args.speed。
                        if args.hold_time == 0:
                            print(
                                "两轴均已首次到位；"
                                "将持续保持，"
                                "按 Ctrl+C 退出并失能。"
                            )
                        else:
                            print(
                                "两轴均已首次到位；"
                                f"继续保持 {args.hold_time:g} 秒，"
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

                time.sleep(args.command_period)

        except KeyboardInterrupt:
            print(
                "\n收到 Ctrl+C；即将尝试失能两轴。"
            )
            return 0

        finally:
            for motor in (yaw, pitch):
                try:
                    motor.disable()
                    print("\n使能已经失效。")
                except Exception as exc:
                    # 尽量继续失能另一台电机。
                    print(
                        f"地址 0x{motor.address:02X} "
                        f"失能命令发送失败：{exc}"
                    )


if __name__ == "__main__":
    raise SystemExit(main())
"""双轴云台随机角度测试、平滑轨迹、数据录入与保持示例。

必须显式添加 --confirm 才会使能电机。

示例：
python examples/two_axis_random_test.py --port COM8 --yaw-min -16 --yaw-max 8 --pitch-min -19 --pitch-max 5.5 --count 20 --speed 50 --move-time 2 --easing power --ease-power 3 --ease-peak 0.35 --confirm

随机角度：
    在 yaw_min～yaw_max、pitch_min～pitch_max 范围内随机生成。
    每组最终绝对目标 = calibration.json 标定参考 + 随机增量。

每组测试流程：
    1. 从当前实际位置运动到随机目标；
    2. 到位后保持使能，并继续重发最终目标；
    3. 等待命令行输入 x y；
    4. 输入 x y 后回车，保存 CSV 并进入下一组；
    5. 直接回车也会保存，但 x、y 留空；
    6. 输入 q 或按 Ctrl+C，取消测试，不保存当前组。

CSV 格式：
    yaw_delta,pitch_delta,x,y

每次运行都会创建一个新 CSV 文件。
即使中途取消、反馈超时或运动超时，也不会覆盖之前的文件。
所有测试结束或退出后，程序会尝试失能两轴。
"""

import argparse
import csv
import json
import queue
import random
import threading
import time
from datetime import datetime
from pathlib import Path

from f32c import F32CBus, Mode


def smoothstep01(progress):
    """三次平滑曲线，起止目标速度为 0。"""
    u = min(1.0, max(0.0, progress))
    return u * u * (3.0 - 2.0 * u)


def smootherstep01(progress):
    """五次平滑曲线，起止目标速度和加速度均为 0。"""
    u = min(1.0, max(0.0, progress))
    return u * u * u * (u * (6.0 * u - 15.0) + 10.0)


def power_ease_in_out01(
    progress,
    power,
    peak=0.5,
):
    """可调节强度和峰值位置的缓入缓出曲线。

    power：
        曲线强度，必须大于或等于 1。

    peak：
        目标速度峰值所在的时间比例。

        0.50：前 50% 加速，后 50% 减速；
        0.35：前 35% 加速，后 65% 减速。
    """
    if power < 1.0:
        raise ValueError("power 必须大于或等于 1")

    if not 0.0 < peak < 1.0:
        raise ValueError("peak 必须位于 0～1 之间")

    u = min(1.0, max(0.0, progress))

    if u <= peak:
        return peak * (u / peak) ** power

    remaining = (1.0 - u) / (1.0 - peak)

    return 1.0 - (
        (1.0 - peak) * remaining ** power
    )


def trajectory_blend(
    progress,
    easing,
    ease_power,
    ease_peak,
):
    """根据选择的轨迹曲线计算插值系数。"""
    if easing == "smootherstep":
        return smootherstep01(progress)

    if easing == "power":
        return power_ease_in_out01(
            progress=progress,
            power=ease_power,
            peak=ease_peak,
        )

    return smoothstep01(progress)


def initialize_direct_motors(
    yaw,
    pitch,
    speed,
    settle_time,
    command_period,
):
    """初始化双轴直通位置模式，并返回稳定后的实际起点。"""
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

    # 预装载真实当前位置，降低使能后沿用旧目标的风险。
    yaw.set_angle(yaw_start)
    pitch.set_angle(pitch_start)

    # 两轴目标准备好后再使能。
    for motor in (yaw, pitch):
        motor.enable()

    # 使能后先保持原位，等待机构和负载稳定。
    settle_deadline = time.monotonic() + settle_time

    while True:
        yaw.set_angle(yaw_start)
        pitch.set_angle(pitch_start)

        if time.monotonic() >= settle_deadline:
            break

        time.sleep(command_period)

    # 使能后可能存在负载沉降，重新读取稳定起点。
    yaw_start = yaw.read_angle()
    pitch_start = pitch.read_angle()

    yaw.set_angle(yaw_start)
    pitch.set_angle(pitch_start)

    print(
        f"稳定后初始位置：yaw={yaw_start:.1f}°，"
        f"pitch={pitch_start:.1f}°"
    )

    time.sleep(0.1)

    return yaw_start, pitch_start


def move_to_target(
    yaw,
    pitch,
    yaw_start,
    pitch_start,
    yaw_target,
    pitch_target,
    move_time,
    easing,
    ease_power,
    ease_peak,
    tolerance,
    timeout,
    command_period,
):
    """执行一次双轴平滑运动，直到两轴都进入位置容差。

    中间轨迹不等待逐点到位。
    只有轨迹结束且两轴都进入最终目标容差后才返回。

    如果轨迹结束后的 timeout 秒内仍未到位，
    则抛出 TimeoutError。
    """
    trajectory_started = time.monotonic()

    deadline = (
        trajectory_started
        + move_time
        + timeout
    )

    next_feedback = 0.0

    while True:
        now = time.monotonic()

        if move_time > 0:
            progress = (
                (now - trajectory_started)
                / move_time
            )
        else:
            progress = 1.0

        blend = trajectory_blend(
            progress=progress,
            easing=easing,
            ease_power=ease_power,
            ease_peak=ease_peak,
        )

        # 两轴使用相同的归一化轨迹进度。
        yaw_command = (
            yaw_start
            + (yaw_target - yaw_start) * blend
        )
        pitch_command = (
            pitch_start
            + (pitch_target - pitch_start) * blend
        )

        # 发送的是多圈绝对目标位置，不是相对增量。
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

            trajectory_done = progress >= 1.0

            position_reached = (
                abs(yaw_now - yaw_target) <= tolerance
                and abs(pitch_now - pitch_target) <= tolerance
            )

            if trajectory_done and position_reached:
                # 进入等待输入前，再明确发送一次最终目标。
                yaw.set_angle(yaw_target)
                pitch.set_angle(pitch_target)

                print(
                    f"已到位：yaw={yaw_now:.1f}°，"
                    f"pitch={pitch_now:.1f}°"
                )
                return

        if now >= deadline:
            raise TimeoutError(
                "等待双轴到位超时："
                f"yaw_target={yaw_target:.1f}°，"
                f"pitch_target={pitch_target:.1f}°"
            )

        time.sleep(command_period)


def start_async_input(prompt):
    """在后台线程读取一行输入，避免阻塞电机保持循环。"""
    result_queue = queue.Queue(maxsize=1)

    def input_worker():
        try:
            value = input(prompt)
            result_queue.put(("ok", value))
        except EOFError:
            result_queue.put(("cancel", ""))
        except Exception as exc:
            result_queue.put(("error", str(exc)))

    thread = threading.Thread(
        target=input_worker,
        daemon=True,
    )
    thread.start()

    return result_queue


def wait_for_xy_while_holding(
    yaw,
    pitch,
    yaw_target,
    pitch_target,
    command_period,
):
    """保持最终目标并等待用户输入 x y。

    等待期间继续周期重发最终位置，电机保持使能。

    返回：
        (x, y)：输入有效；
        ("", "")：用户直接回车；
        None：用户取消或输入失败。
    """
    prompt = (
        "请输入 x y 后回车保存并进入下一组；"
        "直接回车保存空 x,y；输入 q 取消："
    )

    input_queue = start_async_input(prompt)

    while True:
        # 等待输入期间继续保持最终位置。
        yaw.set_angle(yaw_target)
        pitch.set_angle(pitch_target)

        try:
            status, value = input_queue.get_nowait()
        except queue.Empty:
            time.sleep(command_period)
            continue

        if status != "ok":
            return None

        text = value.strip()

        if text.lower() in {"q", "quit", "exit"}:
            return None

        # 直接回车：保存空 x、y，然后进入下一组。
        if not text:
            return "", ""

        parts = text.split()

        if len(parts) == 2:
            try:
                float(parts[0])
                float(parts[1])
            except ValueError:
                pass
            else:
                return parts[0], parts[1]

        print(
            "输入格式错误，请输入两个数值，"
            "例如：12.5 -8.3；"
            "或者直接回车保存空 x,y。"
        )

        # 输入无效时不保存、不进入下一组，继续等待。
        input_queue = start_async_input(
            "请重新输入 x y，或输入 q 取消："
        )


def create_new_csv_file():
    """独占创建一个新 CSV 文件，绝不覆盖已有文件。

    返回：
        (csv_path, csv_file)
    """
    timestamp = datetime.now().strftime(
        "%Y%m%d_%H%M%S"
    )

    for index in range(10000):
        if index == 0:
            suffix = ""
        else:
            suffix = f"_{index:04d}"

        csv_path = (
            Path.cwd()
            / f"角度随机测试_{timestamp}{suffix}.csv"
        )

        try:
            # "x" 表示独占创建：
            # 文件已经存在时抛出 FileExistsError。
            csv_file = csv_path.open(
                mode="x",
                newline="",
                encoding="utf-8-sig",
            )
        except FileExistsError:
            continue

        return csv_path, csv_file

    raise RuntimeError(
        "无法创建新的 CSV 文件：同名文件数量过多"
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "F32C 双轴随机角度测试、"
            "平滑轨迹与 CSV 数据录入示例"
        )
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
        "--yaw-min",
        type=float,
        required=True,
        help="随机 yaw_delta 最小值，单位：度",
    )
    parser.add_argument(
        "--yaw-max",
        type=float,
        required=True,
        help="随机 yaw_delta 最大值，单位：度",
    )
    parser.add_argument(
        "--pitch-min",
        type=float,
        required=True,
        help="随机 pitch_delta 最小值，单位：度",
    )
    parser.add_argument(
        "--pitch-max",
        type=float,
        required=True,
        help="随机 pitch_delta 最大值，单位：度",
    )
    parser.add_argument(
        "--count",
        type=int,
        default=10,
        help="本次随机测试的角度组数，默认 10",
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
            "每组 Python 平滑轨迹时长，单位：秒；"
            "0 表示直接发送最终目标"
        ),
    )

    parser.add_argument(
        "--easing",
        choices=(
            "smoothstep",
            "smootherstep",
            "power",
        ),
        default="smoothstep",
        help="位置轨迹曲线，默认 smoothstep",
    )
    parser.add_argument(
        "--ease-power",
        type=float,
        default=2.0,
        help=(
            "power 曲线强度；"
            "1 为匀速，2～3 为较明显"
        ),
    )
    parser.add_argument(
        "--ease-peak",
        type=float,
        default=0.5,
        help=(
            "power 曲线目标速度峰值位置；"
            "0.35 表示前 35% 加速、后 65% 减速"
        ),
    )

    parser.add_argument(
        "--tolerance",
        type=float,
        default=0.3,
        help="每组目标的位置到位容差，单位：度",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        help=(
            "每组轨迹结束后，"
            "最长等待到位时间，单位：秒"
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
        help="使能后的初始原位稳定时间，单位：秒",
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

    if args.yaw_min > args.yaw_max:
        parser.error("yaw-min 不能大于 yaw-max")

    if args.pitch_min > args.pitch_max:
        parser.error("pitch-min 不能大于 pitch-max")

    if args.count <= 0:
        parser.error("count 必须大于 0")

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

    if args.command_period <= 0:
        parser.error("command-period 必须大于 0")

    if args.ease_power < 1.0:
        parser.error("ease-power 必须大于或等于 1")

    if not 0.0 < args.ease_peak < 1.0:
        parser.error(
            "ease-peak 必须大于 0 且小于 1"
        )

    # 程序启动时一次性生成本次测试的全部随机角度。
    # 保留到 0.1°，与电机位置命令分辨率一致。
    samples = [
        (
            round(
                random.uniform(
                    args.yaw_min,
                    args.yaw_max,
                ),
                1,
            ),
            round(
                random.uniform(
                    args.pitch_min,
                    args.pitch_max,
                ),
                1,
            ),
        )
        for _ in range(args.count)
    ]

    with open(
        "calibration.json",
        "r",
        encoding="utf-8",
    ) as f:
        data = json.load(f)
        yaw_reference = float(data["yaw_init"])
        pitch_reference = float(data["pitch_init"])

    # 在使能电机前先创建 CSV，避免文件创建失败后电机动作。
    csv_path, csv_file = create_new_csv_file()

    print(f"本次 CSV 文件：{csv_path}")
    print(f"本次随机测试组数：{len(samples)}")

    with csv_file:
        writer = csv.writer(csv_file)

        writer.writerow(
            [
                "yaw_delta",
                "pitch_delta",
                "x",
                "y",
            ]
        )
        csv_file.flush()

        with F32CBus(args.port) as bus:
            yaw = bus.motor(args.yaw_address)
            pitch = bus.motor(args.pitch_address)

            try:
                bus.wake()
                time.sleep(
                    max(0.0, args.startup_delay)
                )

                print(
                    f"标定参考：yaw={yaw_reference:.1f}°，"
                    f"pitch={pitch_reference:.1f}°"
                )
                print(
                    "模式：多圈位置直通（0x0003）；"
                    f"内部速度参数={args.speed} RPM；"
                    f"平滑时间={args.move_time:g} 秒；"
                    f"曲线={args.easing}。"
                )

                # 整个测试过程只初始化、使能一次。
                # 各组之间不会因为输入数据而失能。
                yaw_start, pitch_start = (
                    initialize_direct_motors(
                        yaw=yaw,
                        pitch=pitch,
                        speed=args.speed,
                        settle_time=args.settle_time,
                        command_period=args.command_period,
                    )
                )

                for index, (
                    yaw_delta,
                    pitch_delta,
                ) in enumerate(samples, start=1):
                    # 从第二组开始，每次运动前重新读取实际位置，
                    # 避免人工录入期间的小幅扰动影响下一段轨迹。
                    if index > 1:
                        yaw_start = yaw.read_angle()
                        pitch_start = pitch.read_angle()

                    # 随机增量相对于标定参考，不是相对当前位置累加。
                    yaw_target = (
                        yaw_reference + yaw_delta
                    )
                    pitch_target = (
                        pitch_reference + pitch_delta
                    )

                    print(
                        f"\n[{index}/{len(samples)}] "
                        f"yaw_delta={yaw_delta:.1f}°，"
                        f"pitch_delta={pitch_delta:.1f}°"
                    )
                    print(
                        f"绝对目标：yaw={yaw_target:.1f}°，"
                        f"pitch={pitch_target:.1f}°"
                    )

                    # 运动到本组目标；超时由外层捕获。
                    move_to_target(
                        yaw=yaw,
                        pitch=pitch,
                        yaw_start=yaw_start,
                        pitch_start=pitch_start,
                        yaw_target=yaw_target,
                        pitch_target=pitch_target,
                        move_time=args.move_time,
                        easing=args.easing,
                        ease_power=args.ease_power,
                        ease_peak=args.ease_peak,
                        tolerance=args.tolerance,
                        timeout=args.timeout,
                        command_period=args.command_period,
                    )

                    # 到位后保持使能并等待录入。
                    # 等待期间继续重发最终位置。
                    xy = wait_for_xy_while_holding(
                        yaw=yaw,
                        pitch=pitch,
                        yaw_target=yaw_target,
                        pitch_target=pitch_target,
                        command_period=args.command_period,
                    )

                    if xy is None:
                        print(
                            "\n本次测试已取消；"
                            "当前组未保存。"
                        )
                        print(
                            f"此前保存的数据仍保留在：{csv_path}"
                        )
                        return 0

                    x, y = xy

                    # 每次回车保存一行。
                    # 直接回车时 x、y 是空字符串。
                    writer.writerow(
                        [
                            f"{yaw_delta:.1f}",
                            f"{pitch_delta:.1f}",
                            x,
                            y,
                        ]
                    )

                    # 立即刷新，避免取消或超时丢失已录入的数据。
                    csv_file.flush()

                    print(
                        f"已保存第 {index} 组："
                        f"{yaw_delta:.1f},"
                        f"{pitch_delta:.1f},"
                        f"{x},{y}"
                    )

                print(
                    "\n全部随机角度测试完成。"
                )
                print(f"CSV 已保存：{csv_path}")
                return 0

            except TimeoutError as exc:
                # 同时覆盖运动 TimeoutError 和
                # SDK 的 FeedbackTimeoutError。
                print(
                    f"\n运动或电机反馈超时：{exc}"
                )
                print(
                    "当前组未保存；"
                    "此前保存的数据不会丢失。"
                )
                print(f"CSV 文件：{csv_path}")
                return 1

            except KeyboardInterrupt:
                print(
                    "\n收到 Ctrl+C；本次测试已取消。"
                )
                print(
                    f"此前保存的数据仍保留在：{csv_path}"
                )
                return 0

            finally:
                # 正常完成、取消、超时或其他异常时，
                # 都尝试失能两轴。
                for motor in (yaw, pitch):
                    try:
                        motor.disable()
                        print("\n使能已经失效。")
                    except Exception as exc:
                        print(
                            f"地址 0x{motor.address:02X} "
                            f"失能命令发送失败：{exc}"
                        )


if __name__ == "__main__":
    raise SystemExit(main())
"""只读取电机角度，不使能电机，也不发送运动命令。

示例：python examples/read_angles.py --port COM5 --address 1 2
"""

import argparse

from f32c import F32CBus, FeedbackTimeoutError


def main() -> int:
    parser = argparse.ArgumentParser(description="只读 F32C 多圈角度反馈")
    parser.add_argument("--port", required=True, help="串口，例如 COM5 或 /dev/ttyUSB0")
    parser.add_argument("--address", type=int, nargs="+", default=[1, 2], help="电机地址")
    args = parser.parse_args()

    failed = False
    with F32CBus(args.port) as bus:
        print(f"已打开 {bus.port}，115200 8N1；本程序不会使能电机。")
        for address in args.address:
            try:
                angle = bus.read_multi_angle(address)
                voltage = bus.read_voltage(address)
            except FeedbackTimeoutError as exc:
                failed = True
                print(f"地址 0x{address:02X}：{exc}")
            else:
                print(f"地址 0x{address:02X}：多圈角度={angle:.1f}°，母线电压={voltage:.2f}V")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())

"""F32C TTL 串口协议的纯 Python 实现。

本模块不访问硬件，可独立测试。协议依据用户提供的 WHEELTEC STM32F1
位置/速度闭环例程及 TI 云台例程整理。
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from functools import reduce
from operator import xor
from typing import Iterator

HEADER = 0x7A
TAIL = 0x7B
FEEDBACK_FRAME_SIZE = 9


class Command(IntEnum):
    MODE = 0x00
    SPEED = 0x01
    MULTI_POSITION = 0x02
    SINGLE_POSITION = 0x03
    DISABLE = 0x05
    ENABLE = 0x06
    ACCELERATION = 0x07
    SAVE_PARAMETERS = 0x08
    CLEAR_MULTI_ANGLE = 0x09
    SET_SINGLE_ANGLE_ZERO = 0x0A
    FACTORY_RESET = 0x0B
    SET_ADDRESS = 0x0D
    FEEDBACK = 0x0E


class Mode(IntEnum):
    SPEED = 0x0000
    MULTI_POSITION = 0x0001       # 多圈位置，带 T 型轨迹规划
    SINGLE_POSITION = 0x0002      # 单圈位置，带 T 型轨迹规划
    MULTI_POSITION_DIRECT = 0x0003  # 多圈位置，直通
    SINGLE_POSITION_DIRECT = 0x0004  # 单圈位置，直通


class FeedbackType(IntEnum):
    SPEED = 0x00
    MULTI_ANGLE = 0x01
    SINGLE_ANGLE = 0x02
    ACCELERATION = 0x03
    VOLTAGE = 0x04


@dataclass(frozen=True)
class Feedback:
    """一条已校验的电机反馈。"""

    address: int
    kind: FeedbackType
    raw_value: int
    value: float
    unit: str


class ProtocolError(ValueError):
    """帧格式、校验或反馈内容不正确。"""


def calculate_bcc(data: bytes) -> int:
    """计算从帧头开始、校验字节之前所有字节的异或值。"""
    if not data:
        return 0
    return reduce(xor, data)


def _check_address(address: int) -> None:
    if not 1 <= address <= 0xFF:
        raise ValueError("电机地址必须是 1～255 的整数")


def build_frame(address: int, command: int | Command, payload: bytes = b"") -> bytes:
    """生成 ``7A 地址 命令 数据 BCC 7B`` 格式的命令帧。"""
    _check_address(address)
    body = bytes((HEADER, address, int(command))) + bytes(payload)
    return body + bytes((calculate_bcc(body), TAIL))


def build_feedback_frame_for_test(
    address: int, kind: int | FeedbackType, payload: bytes
) -> bytes:
    """构造 9 字节反馈帧，仅供模拟器与测试使用。"""
    if len(payload) != 4:
        raise ValueError("反馈帧的数据区必须为 4 字节")
    return build_frame(address, int(kind), payload)


def parse_feedback_frame(frame: bytes) -> Feedback:
    """解析一条 9 字节速度、角度、加速度或电压反馈帧。"""
    if len(frame) != FEEDBACK_FRAME_SIZE:
        raise ProtocolError("反馈帧长度必须为 9 字节")
    if frame[0] != HEADER or frame[-1] != TAIL:
        raise ProtocolError("反馈帧帧头或帧尾错误")
    if calculate_bcc(frame[:7]) != frame[7]:
        raise ProtocolError("反馈帧 BCC 校验失败")

    address = frame[1]
    _check_address(address)
    try:
        kind = FeedbackType(frame[2])
    except ValueError as exc:
        raise ProtocolError(f"未知反馈类型：0x{frame[2]:02X}") from exc

    raw32 = int.from_bytes(frame[3:7], byteorder="big", signed=False)
    low16 = raw32 & 0xFFFF
    signed_low16 = low16 - 0x10000 if low16 & 0x8000 else low16
    signed32 = raw32 - 0x100000000 if raw32 & 0x80000000 else raw32

    if kind is FeedbackType.SPEED:
        return Feedback(address, kind, signed_low16, float(signed_low16), "rpm")
    if kind is FeedbackType.MULTI_ANGLE:
        return Feedback(address, kind, signed32, signed32 / 10.0, "deg")
    if kind is FeedbackType.SINGLE_ANGLE:
        return Feedback(address, kind, low16, low16 / 10.0, "deg")
    if kind is FeedbackType.ACCELERATION:
        return Feedback(address, kind, signed_low16, float(signed_low16), "rev/s^2")
    return Feedback(address, kind, low16, low16 / 100.0, "V")


class FeedbackParser:
    """可从任意分片、噪声或连续字节流中恢复反馈帧的增量解析器。"""

    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, data: bytes) -> Iterator[Feedback]:
        self._buffer.extend(data)
        while True:
            try:
                start = self._buffer.index(HEADER)
            except ValueError:
                self._buffer.clear()
                return
            if start:
                del self._buffer[:start]
            if len(self._buffer) < FEEDBACK_FRAME_SIZE:
                return

            candidate = bytes(self._buffer[:FEEDBACK_FRAME_SIZE])
            try:
                feedback = parse_feedback_frame(candidate)
            except ProtocolError:
                # 只丢弃一个帧头，再搜索下一个 0x7A，避免错帧后永久失步。
                del self._buffer[0]
                continue
            del self._buffer[:FEEDBACK_FRAME_SIZE]
            yield feedback

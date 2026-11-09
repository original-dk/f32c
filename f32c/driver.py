"""基于 pyserial 的 F32C 电机总线驱动。"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from threading import RLock
from typing import Optional

import serial

from .protocol import (
    Command,
    Feedback,
    FeedbackParser,
    FeedbackType,
    Mode,
    build_frame,
)


class FeedbackTimeoutError(TimeoutError):
    """在指定时间内没有收到目标电机的目标类型反馈。"""


def _checked_int(value: int, name: str, minimum: int, maximum: int) -> int:
    if not isinstance(value, int) or isinstance(value, bool):
        raise TypeError(f"{name} 必须是整数")
    if not minimum <= value <= maximum:
        raise ValueError(f"{name} 必须位于 {minimum}～{maximum}")
    return value


class F32CBus:
    """一条 TTL 总线上的一个或多个 F32C 电机。

    通常电脑通过 USB 转 TTL 模块连接电机，端口在 Windows 上类似 ``COM5``，
    在 Linux 上类似 ``/dev/ttyUSB0``。所有写操作及反馈请求均在锁内执行，
    防止同一串口上的命令帧相互穿插。
    """

    def __init__(
        self,
        port: Optional[str] = None,
        *,
        baudrate: int = 115200,
        timeout: float = 0.02,
        command_interval: float = 0.002,
        serial_instance=None,
    ) -> None:
        if command_interval < 0:
            raise ValueError("command_interval 不能为负数")
        if serial_instance is None:
            if not port:
                raise ValueError("必须提供串口名称 port")
            serial_instance = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=timeout,
                write_timeout=timeout,
            )
        self._serial = serial_instance
        self._command_interval = command_interval
        self._last_write = 0.0
        self._parser = FeedbackParser()
        self._latest: dict[tuple[int, FeedbackType], Feedback] = {}
        self._lock = RLock()

    @property
    def port(self) -> str:
        return str(self._serial.port)

    @property
    def is_open(self) -> bool:
        return bool(self._serial.is_open)

    def close(self) -> None:
        with self._lock:
            if self._serial.is_open:
                self._serial.close()

    def __enter__(self) -> "F32CBus":
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.close()

    def motor(self, address: int) -> "F32CMotor":
        return F32CMotor(self, address)

    def _write(self, data: bytes) -> None:
        if not self._serial.is_open:
            raise serial.SerialException("串口尚未打开")
        wait = self._command_interval - (time.monotonic() - self._last_write)
        if wait > 0:
            time.sleep(wait)
        written = self._serial.write(data)
        self._last_write = time.monotonic()
        if written is not None and written != len(data):
            raise serial.SerialException(f"串口仅写入 {written}/{len(data)} 字节")

    def send_command(self, address: int, command: int | Command, payload: bytes = b"") -> None:
        frame = build_frame(address, command, payload)
        with self._lock:
            self._write(frame)

    def wake(self) -> None:
        """发送例程使用的 0x00 唤醒字节；不等同于电机使能。"""
        with self._lock:
            self._write(b"\x00")

    def enable(self, address: int) -> None:
        self.send_command(address, Command.ENABLE)

    def disable(self, address: int) -> None:
        self.send_command(address, Command.DISABLE)

    def set_mode(self, address: int, mode: int | Mode) -> None:
        mode_value = _checked_int(int(mode), "模式", 0, 0xFFFF)
        self.send_command(address, Command.MODE, mode_value.to_bytes(2, "big"))

    def set_speed_rpm(self, address: int, rpm: int) -> None:
        rpm = _checked_int(rpm, "转速", -32768, 32767)
        self.send_command(address, Command.SPEED, rpm.to_bytes(2, "big", signed=True))

    def set_multi_angle_raw(self, address: int, angle_x10: int) -> None:
        angle_x10 = _checked_int(angle_x10, "多圈角度原始值", -2147483648, 2147483647)
        self.send_command(
            address, Command.MULTI_POSITION, angle_x10.to_bytes(4, "big", signed=True)
        )

    def set_multi_angle(self, address: int, angle_deg: float) -> None:
        if not math.isfinite(angle_deg):
            raise ValueError("角度必须是有限数值")
        self.set_multi_angle_raw(address, round(angle_deg * 10.0))

    def set_single_angle(self, address: int, angle_deg: float) -> None:
        if not math.isfinite(angle_deg) or not 0.0 <= angle_deg <= 359.9:
            raise ValueError("单圈角度必须位于 0.0～359.9 度")
        angle_x10 = round(angle_deg * 10.0)
        self.send_command(
            address, Command.SINGLE_POSITION, angle_x10.to_bytes(2, "big", signed=False)
        )

    def set_acceleration(self, address: int, rev_per_s2: int) -> None:
        rev_per_s2 = _checked_int(rev_per_s2, "加速度", 0, 65535)
        self.send_command(address, Command.ACCELERATION, rev_per_s2.to_bytes(2, "big"))

    def save_parameters(self, address: int) -> None:
        self.send_command(address, Command.SAVE_PARAMETERS)

    def clear_multi_angle(self, address: int) -> None:
        self.send_command(address, Command.CLEAR_MULTI_ANGLE)

    def set_single_angle_zero(self, address: int) -> None:
        self.send_command(address, Command.SET_SINGLE_ANGLE_ZERO)

    def factory_reset(self, address: int) -> None:
        """恢复目标电机出厂设置；调用前必须确认地址及风险。"""
        self.send_command(address, Command.FACTORY_RESET)

    def set_address(self, address: int, new_address: int) -> None:
        new_address = _checked_int(new_address, "新地址", 1, 255)
        self.send_command(address, Command.SET_ADDRESS, bytes((new_address,)))

    def _store(self, feedback: Feedback) -> None:
        self._latest[(feedback.address, feedback.kind)] = feedback

    def poll(self) -> list[Feedback]:
        """非阻塞读取当前已有数据，更新并返回本次收到的反馈。"""
        with self._lock:
            count = max(0, int(self._serial.in_waiting))
            if count == 0:
                return []
            results = list(self._parser.feed(self._serial.read(count)))
            for item in results:
                self._store(item)
            return results

    def get_latest(self, address: int, kind: FeedbackType) -> Optional[Feedback]:
        return self._latest.get((address, kind))

    def request_feedback(
        self,
        address: int,
        kind: FeedbackType,
        *,
        timeout: float = 0.25,
    ) -> Feedback:
        """发送反馈请求并等待匹配的响应，忽略其他电机或其他类型反馈。"""
        if timeout <= 0:
            raise ValueError("timeout 必须大于 0")
        kind = FeedbackType(kind)
        with self._lock:
            # 防止把上一次请求遗留的响应当成本次结果。
            self._serial.reset_input_buffer()
            self._parser = FeedbackParser()
            self._write(build_frame(address, Command.FEEDBACK, bytes((kind,))))
            deadline = time.monotonic() + timeout
            while True:
                count = max(1, int(self._serial.in_waiting))
                data = self._serial.read(count)
                for item in self._parser.feed(data):
                    self._store(item)
                    if item.address == address and item.kind is kind:
                        return item
                if time.monotonic() >= deadline:
                    raise FeedbackTimeoutError(
                        f"等待地址 0x{address:02X} 的 {kind.name} 反馈超时"
                    )

    def read_speed_rpm(self, address: int, *, timeout: float = 0.25) -> int:
        return int(self.request_feedback(address, FeedbackType.SPEED, timeout=timeout).raw_value)

    def read_multi_angle(self, address: int, *, timeout: float = 0.25) -> float:
        """读取多圈绝对角度，单位为度。"""
        return self.request_feedback(address, FeedbackType.MULTI_ANGLE, timeout=timeout).value

    def read_single_angle(self, address: int, *, timeout: float = 0.25) -> float:
        return self.request_feedback(address, FeedbackType.SINGLE_ANGLE, timeout=timeout).value

    def read_acceleration(self, address: int, *, timeout: float = 0.25) -> int:
        return int(
            self.request_feedback(address, FeedbackType.ACCELERATION, timeout=timeout).raw_value
        )

    def read_voltage(self, address: int, *, timeout: float = 0.25) -> float:
        return self.request_feedback(address, FeedbackType.VOLTAGE, timeout=timeout).value


@dataclass(frozen=True)
class F32CMotor:
    """绑定地址的便捷电机对象，多个对象可共享同一个 :class:`F32CBus`。"""

    bus: F32CBus
    address: int

    def enable(self) -> None:
        self.bus.enable(self.address)

    def disable(self) -> None:
        self.bus.disable(self.address)

    def set_mode(self, mode: int | Mode) -> None:
        self.bus.set_mode(self.address, mode)

    def set_speed(self, rpm: int) -> None:
        self.bus.set_speed_rpm(self.address, rpm)

    def set_angle(self, angle_deg: float) -> None:
        self.bus.set_multi_angle(self.address, angle_deg)

    def read_speed(self) -> int:
        return self.bus.read_speed_rpm(self.address)

    def read_angle(self) -> float:
        return self.bus.read_multi_angle(self.address)

    def read_voltage(self) -> float:
        return self.bus.read_voltage(self.address)

"""WHEELTEC F32C TTL 电机的 Python 串口驱动。"""

from .driver import F32CBus, F32CMotor, FeedbackTimeoutError
from .protocol import (
    Command,
    Feedback,
    FeedbackParser,
    FeedbackType,
    Mode,
    ProtocolError,
    build_frame,
    calculate_bcc,
    parse_feedback_frame,
)

__all__ = [
    "Command",
    "F32CBus",
    "F32CMotor",
    "Feedback",
    "FeedbackParser",
    "FeedbackTimeoutError",
    "FeedbackType",
    "Mode",
    "ProtocolError",
    "build_frame",
    "calculate_bcc",
    "parse_feedback_frame",
]

__version__ = "0.1.0"

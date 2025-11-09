"""lilsim Python SDK for autonomous racing simulation."""

from .client import LilsimClient
from .messages_pb2 import (
    AdminCommandType,
    MarkerType,
    FrameId,
    StateUpdate,
    ControlRequest,
    ControlReply,
    AdminCommand,
    AdminReply,
    MarkerArray,
    Marker,
)

__version__ = "0.1.0"
__all__ = [
    "LilsimClient",
    "AdminCommandType",
    "MarkerType",
    "FrameId",
    "StateUpdate",
    "ControlRequest",
    "ControlReply",
    "AdminCommand",
    "AdminReply",
    "MarkerArray",
    "Marker",
]


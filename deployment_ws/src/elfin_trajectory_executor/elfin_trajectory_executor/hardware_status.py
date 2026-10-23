"""Compact CPS / ServoJ status for the loading-console page.

The executor holds the only CPS TCP client. This snapshot is what the
console can show without a second CPS connection.
"""
from __future__ import annotations

from typing import Any, Mapping, Optional
from .controller_state import LABELS, FAULTS, MOVING, JOGGING, STOPPING, HOLDING

HARDWARE_STATUS_TOPIC = "/elfin/hardware_status"
SCHEMA = "elfin_hardware_status/v1"
EVENT_LIMIT = 16

FSM_LABELS = LABELS

HRIF_HINTS = {
    0: "成功",
    20018: "当前 FSM 禁止该命令",
    40025: "不在 StandBy，不能 InitServoEsJ",
    40071: "ServoEsJ 缓冲未初始化",
    40083: "交融/到位查询失败",
}

NOTABLE_OK = frozenset({"InitServoEsJ", "StartServo", "GrpStop", "GrpReset", "GrpEnable"})


def fsm_label(fsm_id: Optional[int]) -> str:
    if fsm_id is None:
        return "未知"
    if int(fsm_id) in FSM_LABELS:
        return FSM_LABELS[int(fsm_id)]
    return "FSM %s" % fsm_id


def fsm_severity(fsm_id: Optional[int]) -> str:
    if fsm_id in FAULTS:
        return "fault"
    if fsm_id == 33:
        return "ok"
    if fsm_id in (MOVING, JOGGING, STOPPING, HOLDING):
        return "moving"
    if fsm_id in (23, 24, 28, 34, 35, 36, 37, 38):
        return "warn"
    return "unknown"


def hrif_hint(code: Optional[int]) -> str:
    if code is None:
        return ""
    return HRIF_HINTS.get(int(code), "HRIF %s" % code)


def hrif_severity(code: Optional[int]) -> str:
    if code in (None, 0):
        return "ok"
    if int(code) in (40025, 40071, 40083, 20018):
        return "fault"
    return "warn"


def make_hrif_event(
    name: str,
    code: int,
    message: str = "",
    *,
    monotonic_sec: float = 0.0,
    extra: Optional[Mapping[str, Any]] = None,
) -> dict[str, Any]:
    payload = {
        "name": str(name),
        "code": int(code),
        "message": str(message or ""),
        "hint": hrif_hint(code),
        "severity": hrif_severity(code),
        "monotonic_sec": float(monotonic_sec),
    }
    if extra:
        payload.update(dict(extra))
    return payload


def should_keep_event(name: str, code: int) -> bool:
    return int(code) != 0 or str(name) in NOTABLE_OK


def append_event(events: list, event: Mapping[str, Any], limit: int = EVENT_LIMIT) -> list:
    out = list(events)
    out.append(dict(event))
    if len(out) > limit:
        out = out[-limit:]
    return out


def build_snapshot(
    *,
    connection: str = "",
    fsm_id: Optional[int] = None,
    fsm_desc: str = "",
    last_hrif: Optional[Mapping[str, Any]] = None,
    last_fault: Optional[Mapping[str, Any]] = None,
    events: Optional[list] = None,
    joints_deg: Optional[list] = None,
    last_error_string: str = "",
    stamp_ns: int = 0,
    monotonic_sec: float = 0.0,
) -> dict[str, Any]:
    return {
        "schema": SCHEMA,
        "available": True,
        "stamp_ns": int(stamp_ns),
        "monotonic_sec": float(monotonic_sec),
        "connection": str(connection or ""),
        "fsm_id": fsm_id,
        "fsm_desc": str(fsm_desc or ""),
        "fsm_label": fsm_label(fsm_id),
        "fsm_severity": fsm_severity(fsm_id),
        "ready_for_servo": fsm_id == 33,
        "current_fault": fsm_id in FAULTS or connection == "ERROR",
        "last_hrif": dict(last_hrif) if last_hrif else None,
        "last_fault": dict(last_fault) if last_fault else None,
        "events": [dict(item) for item in (events or [])],
        "joints_deg": [float(v) for v in (joints_deg or [])],
        "last_error_string": str(last_error_string or ""),
    }

from __future__ import annotations

import json
import math
import os
import time
from pathlib import Path
from typing import Any


SKILL_NAME_TO_ID = {
    "idle": 0,
    "walk": 1,
    "walk_skill": 1,
    "climb": 2,
    "push": 3,
    "push_box": 3,
    "nav": 4,
    "navigation": 4,
    "nav_climb": 5,
    "navigation_climb": 5,
}
SKILL_ID_TO_NAME = {
    0: "idle",
    1: "walk_skill",
    2: "climb",
    3: "push",
    4: "nav",
    5: "nav_climb",
}
DEFAULT_BOX_OBJECT = {
    "id": "box",
    "type": "box",
    "size": [0.6, 0.8, 0.24],
    "movable": True,
}


def coerce_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return value != 0
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on", "start"}
    return False


def coerce_vector(value: Any, length: int = 3) -> list[float] | None:
    if not isinstance(value, (list, tuple)) or len(value) < length:
        return None
    try:
        return [float(value[i]) for i in range(length)]
    except (TypeError, ValueError):
        return None


def quat_wxyz_to_xyzw(quat: list[float]) -> tuple[float, float, float, float]:
    return float(quat[1]), float(quat[2]), float(quat[3]), float(quat[0])


def rotate_body_to_world(vector: list[float], quat_wxyz: list[float]) -> list[float]:
    w, x, y, z = [float(item) for item in quat_wxyz[:4]]
    vx, vy, vz = [float(item) for item in vector[:3]]
    return [
        (1.0 - 2.0 * (y * y + z * z)) * vx + 2.0 * (x * y - z * w) * vy + 2.0 * (x * z + y * w) * vz,
        2.0 * (x * y + z * w) * vx + (1.0 - 2.0 * (x * x + z * z)) * vy + 2.0 * (y * z - x * w) * vz,
        2.0 * (x * z - y * w) * vx + 2.0 * (y * z + x * w) * vy + (1.0 - 2.0 * (x * x + y * y)) * vz,
    ]


def box_world_position_from_snapshot(snap: dict) -> list[float] | None:
    pos = coerce_vector(snap.get("robot_pos"))
    quat = coerce_vector(snap.get("robot_quat"), 4)
    obs = snap.get("push_box_obs")
    if pos is None or quat is None or not isinstance(obs, (list, tuple)) or len(obs) < 9:
        return None
    box_in_robot = coerce_vector(obs[6:9])
    if box_in_robot is None:
        return None
    box_offset_world = rotate_body_to_world(box_in_robot, quat)
    return [round(pos[i] + box_offset_world[i], 6) for i in range(3)]


def build_scene_objects(snap: dict, static_objects: list[dict] | None = None) -> list[dict]:
    objects = []
    box_pos = box_world_position_from_snapshot(snap)
    if box_pos is not None:
        box = dict(DEFAULT_BOX_OBJECT)
        box["center"] = box_pos
        objects.append(box)
    objects.extend(static_objects or [])
    return objects


def parse_scene_objects_config(value: str | None) -> list[dict]:
    if not value:
        return []
    source = value.strip()
    if not source:
        return []
    if source.startswith("@"):
        source = source[1:]
    path = Path(source).expanduser()
    if path.exists():
        data = json.loads(path.read_text(encoding="utf-8"))
    else:
        data = json.loads(source)
    if not isinstance(data, list):
        raise ValueError("static scene objects must be a JSON list")
    return [item for item in data if isinstance(item, dict)]


def write_text(file_path: str | Path, text: str) -> None:
    path = Path(file_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text.strip() + "\n", encoding="utf-8")


def distance_xy(a: list[float], b: list[float]) -> float:
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


class ControlFileWriter:
    def __init__(self, control_dir: str):
        self.control_dir = Path(control_dir).expanduser() if control_dir else None

    def write_file(self, name: str, text: str) -> None:
        if self.control_dir is None:
            return
        write_text(self.control_dir / name, text)

    def write_skill_command(self, payload: dict[str, Any]) -> None:
        if self.control_dir is None:
            return
        self.control_dir.mkdir(parents=True, exist_ok=True)
        self.write_file("skill_command.json", json.dumps(payload, ensure_ascii=False))
        if payload.get("model_use") is not None:
            self.write_file("model_use.txt", str(payload["model_use"]))
        if payload.get("skill") is not None:
            self.write_file("skill.txt", str(payload["skill"]))
        if payload.get("start") is not None:
            self.write_file("start.txt", "1" if coerce_bool(payload["start"]) else "0")
        velocity = coerce_vector(payload.get("velocity"))
        if velocity is not None:
            self.write_file("velocity.txt", f"{velocity[0]} {velocity[1]} {velocity[2]}")
        goal = coerce_vector(payload.get("goal"))
        if goal is not None:
            self.write_file("goal.txt", f"{goal[0]} {goal[1]} {goal[2]}")


class SkillStatusTracker:
    def __init__(self, goal_tolerance: float = 0.08):
        self.goal_tolerance = float(goal_tolerance)
        self._active: dict[str, Any] | None = None
        self._last_status: dict[str, Any] = {
            "timestamp": 0.0,
            "model_use": 0,
            "skill": "idle",
            "start": False,
        }

    def apply_command(self, payload: dict[str, Any], timestamp: float | None = None) -> None:
        now = float(timestamp if timestamp is not None else time.time())
        model_use = int(payload.get("model_use") or SKILL_NAME_TO_ID.get(str(payload.get("skill", "")).lower(), 0))
        skill = SKILL_ID_TO_NAME.get(model_use, str(payload.get("skill") or "unknown"))
        velocity = coerce_vector(payload.get("velocity"))
        goal = coerce_vector(payload.get("goal"))
        start = coerce_bool(payload.get("start"))
        if not start or payload.get("reset") is not None or self._is_stop_command(model_use, velocity):
            self._active = None
            self._last_status = {
                "timestamp": now,
                "model_use": 0,
                "skill": "idle",
                "start": False,
            }
            return

        self._active = {
            "timestamp": now,
            "model_use": model_use,
            "skill": skill,
            "start": True,
        }
        if velocity is not None:
            self._active["velocity"] = velocity
        if goal is not None:
            self._active["goal"] = goal
        self._last_status = dict(self._active)

    def status(self, snap: dict) -> dict:
        timestamp = float(snap.get("sim_time") or time.time())
        if self._active is None:
            status = dict(self._last_status)
            status["timestamp"] = timestamp
            return status
        status = dict(self._active)
        status["timestamp"] = timestamp
        if self._target_reached(status, snap):
            status["start"] = False
            self._active = None
            self._last_status = dict(status)
        return status

    @staticmethod
    def _is_stop_command(model_use: int, velocity: list[float] | None) -> bool:
        return model_use == 1 and velocity is not None and all(abs(item) <= 1e-6 for item in velocity)

    def _target_reached(self, status: dict, snap: dict) -> bool:
        goal = status.get("goal")
        if not isinstance(goal, list):
            return False
        model_use = int(status.get("model_use") or 0)
        if model_use == 3:
            box_pos = box_world_position_from_snapshot(snap)
            return box_pos is not None and distance_xy(box_pos, goal) <= self.goal_tolerance
        if model_use in {4, 5}:
            robot_pos = coerce_vector(snap.get("robot_pos"))
            return robot_pos is not None and distance_xy(robot_pos, goal) <= self.goal_tolerance
        return False

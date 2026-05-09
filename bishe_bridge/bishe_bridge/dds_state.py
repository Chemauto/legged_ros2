from __future__ import annotations

import threading


try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import HeightMap_, LowState_, SportModeState_

    DDS_AVAILABLE = True
except Exception:
    ChannelFactoryInitialize = None
    ChannelSubscriber = None
    HeightMap_ = LowState_ = SportModeState_ = None
    DDS_AVAILABLE = False


class DdsStateCache:
    def __init__(self):
        self._lock = threading.Lock()
        self._robot_pos = None
        self._robot_vel = None
        self._robot_quat = None
        self._push_box_obs = None
        self._sim_time = 0.0

    def update_sportstate(self, msg) -> None:
        with self._lock:
            self._robot_pos = [float(msg.position[0]), float(msg.position[1]), float(msg.position[2])]
            self._robot_vel = [float(msg.velocity[0]), float(msg.velocity[1]), float(msg.velocity[2])]

    def update_lowstate(self, msg) -> None:
        with self._lock:
            self._robot_quat = list(msg.imu_state.quaternion)

    def update_push_box_obs(self, msg) -> None:
        with self._lock:
            self._push_box_obs = list(msg.data)
            self._sim_time = float(getattr(msg, "stamp", 0.0))

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "robot_pos": list(self._robot_pos) if self._robot_pos else None,
                "robot_vel": list(self._robot_vel) if self._robot_vel else None,
                "robot_quat": list(self._robot_quat) if self._robot_quat else None,
                "push_box_obs": list(self._push_box_obs) if self._push_box_obs else None,
                "sim_time": self._sim_time,
            }


def start_dds_subscribers(cache: DdsStateCache, domain_id: int, interface: str, *, need_box: bool = True) -> threading.Thread:
    if not DDS_AVAILABLE:
        raise RuntimeError("unitree_sdk2py is not available. Check PYTHONPATH before starting bishe_bridge.")

    def run() -> None:
        ChannelFactoryInitialize(int(domain_id), str(interface))
        ChannelSubscriber("rt/sportmodestate", SportModeState_).Init(cache.update_sportstate, 10)
        ChannelSubscriber("rt/lowstate", LowState_).Init(cache.update_lowstate, 10)
        if need_box:
            ChannelSubscriber("rt/push_box_obs", HeightMap_).Init(cache.update_push_box_obs, 10)

    thread = threading.Thread(target=run, daemon=True)
    thread.start()
    return thread

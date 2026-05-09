from __future__ import annotations


def declare_common_dds_params(node) -> tuple[int, str]:
    node.declare_parameter("domain_id", 0)
    node.declare_parameter("interface", "lo")
    domain_id = int(node.get_parameter("domain_id").value)
    interface = str(node.get_parameter("interface").value)
    return domain_id, interface


def declare_publish_hz(node, default: float = 10.0) -> float:
    node.declare_parameter("publish_hz", float(default))
    return max(float(node.get_parameter("publish_hz").value), 0.1)

#!/usr/bin/env python3
"""Normalize Isaac Lab IO descriptors for the legged ROS 2 RL controller."""

from __future__ import annotations

import argparse
from pathlib import Path

import yaml


def _flatten_singleton_row(value):
    if isinstance(value, list) and len(value) == 1 and isinstance(value[0], list):
        return value[0]
    return value


def _term_dim(term):
    dim = 1
    for item in term.get("shape", []):
        dim *= int(item)
    return dim


def _expand_scalar_or_singleton(value, dim):
    value = _flatten_singleton_row(value)
    if isinstance(value, (int, float)):
        return [float(value)] * dim
    if isinstance(value, list) and len(value) == 1 and isinstance(value[0], (int, float)) and dim != 1:
        return [float(value[0])] * dim
    return value


def normalize_io_descriptors(data):
    for action in data.get("actions", []):
        action_dim = _term_dim(action)
        for key in ("scale", "offset"):
            if key in action:
                action[key] = _expand_scalar_or_singleton(action[key], action_dim)
        if "clip" in action:
            action["clip"] = _flatten_singleton_row(action["clip"])

    for obs_group in data.get("observations", {}).values():
        for obs_term in obs_group:
            overloads = obs_term.get("overloads")
            if not isinstance(overloads, dict):
                continue
            obs_dim = _term_dim(obs_term)
            if "scale" in overloads:
                overloads["scale"] = _expand_scalar_or_singleton(overloads["scale"], obs_dim)
            if "clip" in overloads:
                overloads["clip"] = _flatten_singleton_row(overloads["clip"])

    return data


def main():
    parser = argparse.ArgumentParser(
        description="Convert an Isaac Lab IO_descriptors YAML into the format expected by legged_rl_controller."
    )
    parser.add_argument("input", type=Path, help="Exported IO descriptor YAML from Isaac Lab.")
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=Path(__file__).with_name("IO_descriptors.yaml"),
        help="Output YAML path. Defaults to IO_descriptors.yaml next to this script.",
    )
    args = parser.parse_args()

    data = yaml.safe_load(args.input.read_text())
    data = normalize_io_descriptors(data)
    args.output.write_text(yaml.safe_dump(data, sort_keys=False))
    print(f"Wrote normalized IO descriptors to {args.output}")


if __name__ == "__main__":
    main()

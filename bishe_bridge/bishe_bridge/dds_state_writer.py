#!/usr/bin/env python3
from __future__ import annotations

import argparse
import signal
import time

from bishe_bridge.dds_state import DdsStateCache, start_dds_subscribers
from bishe_bridge.state_file import write_state_file


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Write Unitree DDS state snapshots to a JSON file.")
    parser.add_argument("--domain-id", type=int, default=0)
    parser.add_argument("--interface", default="lo")
    parser.add_argument("--state-file", default="/tmp/bishe_bridge_state.json")
    parser.add_argument("--write-hz", type=float, default=50.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    running = True

    def stop(_signum, _frame) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    cache = DdsStateCache()
    start_dds_subscribers(cache, args.domain_id, args.interface, need_box=True)
    period = 1.0 / max(float(args.write_hz), 0.1)
    while running:
        write_state_file(args.state_file, cache.snapshot())
        time.sleep(period)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

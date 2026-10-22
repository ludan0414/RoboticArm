"""Ten-box replay: right ground feeder, raised observation, original packing.

First invocation generates all ten motions before opening the viewer.
Subsequent invocations replay the saved recording. New bundles are checked by
default; --preview explicitly opts into unvalidated kinematic animation.
"""
import argparse
import json
import math
from pathlib import Path
from demo_contract import write_json
from ground_feeder_bridge import prepare_bridge
from replay_prepared_demo import replay


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base", type=Path, default=Path(__file__).parent/"demo_data/site55_greedy10")
    parser.add_argument("--bundle", type=Path, default=Path(__file__).parent/"demo_data/site55_ground_greedy10")
    parser.add_argument("--feeder-x", type=float, default=-1.2)
    parser.add_argument("--feeder-y", type=float, default=0.0)
    parser.add_argument("--feeder-world", type=float, nargs=3, metavar=("X", "Y", "BOTTOM_Z"),
                        help="explicit world-frame ground pickup; bottom Z must be zero")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--prepare-only", action="store_true")
    checks = parser.add_mutually_exclusive_group()
    checks.add_argument("--validate", action="store_true", help="require a checked bundle, including when replaying")
    checks.add_argument("--preview", action="store_true", help="explicitly skip collision checking for a NEW animation bundle")
    parser.add_argument("--auto", action="store_true", help="play continuously without the per-box Pick button")
    args = parser.parse_args()
    if args.feeder_world is not None and (not all(map(math.isfinite, args.feeder_world))
                                         or abs(args.feeder_world[2]) > 1e-6):
        parser.error("--feeder-world requires finite coordinates and bottom Z=0")
    certified = args.bundle/"READY.json"
    preview = args.bundle/"MOTION_READY.json"
    if not certified.is_file() and not preview.is_file():
        if args.bundle.exists():
            parser.error("incomplete output retained; choose a new --bundle directory")
        prepare_bridge(args.base, args.bundle, [args.feeder_x, args.feeder_y],
                       validate=not args.preview, observe_at_base=True,
                       feeder_world_xy=args.feeder_world[:2] if args.feeder_world is not None else None)
    else:
        plan = json.loads((args.bundle/"plan.json").read_text(encoding="utf-8"))
        requested_xy = args.feeder_world[:2] if args.feeder_world is not None else [args.feeder_x, args.feeder_y]
        requested_frame = "world" if args.feeder_world is not None else "elfin_base"
        if (len(plan["sequence"]) != 10 or plan["feeder"]["xy_m"] != requested_xy
                or plan["feeder"]["frame"] != requested_frame):
            parser.error("recording is not the requested ten-box ground-feeder demo")
        if args.validate and not certified.is_file():
            parser.error("existing preview has not been checked; choose a new --bundle to validate")
    if not args.prepare_only:
        report = replay(args.bundle, fps=args.fps, unvalidated_preview=not certified.is_file(), manual_pick=not args.auto)
        write_json(args.bundle/"replay_result.json", report)


if __name__ == "__main__":
    main()

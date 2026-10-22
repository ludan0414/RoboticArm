"""Ten-box replay: right ground feeder, raised observation, original packing.

First invocation generates all ten motions before opening the viewer.
Subsequent invocations replay the saved recording. No validation campaign.
"""
import argparse
import json
from pathlib import Path
from demo_contract import write_json
from ground_feeder_bridge import prepare_bridge
from replay_prepared_demo import replay


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base", type=Path, default=Path(__file__).parent/"demo_data/greedy10")
    parser.add_argument("--bundle", type=Path, default=Path(__file__).parent/"demo_data/ground_greedy10_observe")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--prepare-only", action="store_true")
    parser.add_argument("--auto", action="store_true", help="play continuously without the per-box Pick button")
    args = parser.parse_args()
    marker = args.bundle/"MOTION_READY.json"
    if not marker.is_file():
        if args.bundle.exists():
            parser.error("incomplete output retained; choose a new --bundle directory")
        prepare_bridge(args.base, args.bundle, [1.2, 0.0], validate=False, observe_at_base=True)
    else:
        stored = json.loads(marker.read_text(encoding="utf-8"))
        if stored["boxes"] != 10 or stored["feeder_base_xy_m"] != [1.2, 0.0]:
            parser.error("recording is not the requested ten-box ground-feeder demo")
    if not args.prepare_only:
        report = replay(args.bundle, fps=args.fps, unvalidated_preview=True, manual_pick=not args.auto)
        write_json(args.bundle/"replay_result.json", report)


if __name__ == "__main__":
    main()

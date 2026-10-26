"""Replay four-color loading: boxes at base(-1.6,0), EEF waits at(-1.2,0,0).

One Pick button click runs observe, pick, place and return for one box.
"""
import argparse
from pathlib import Path

from ground_feeder_bridge import prepare_bridge
from replay_prepared_demo import replay


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    root = Path(__file__).parent / "demo_data"
    parser.add_argument("--base", type=Path, default=root / "site_four_colors_inside_clearance5")
    parser.add_argument("--bundle", type=Path, default=root / "site_four_colors_right160_clearance5")
    parser.add_argument("--prepare-only", action="store_true")
    parser.add_argument("--auto", action="store_true")
    args = parser.parse_args()
    if not (args.bundle / "READY.json").is_file():
        prepare_bridge(args.base, args.bundle, [-1.6, 0.0], validate=True, observe_at_base=True,
                       observation_base_xy=[-1.2, 0.0])
    if not args.prepare_only:
        replay(args.bundle, manual_pick=not args.auto, show_excursion=True)


if __name__ == "__main__":
    main()

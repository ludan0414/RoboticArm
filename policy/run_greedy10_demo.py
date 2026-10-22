"""Prepare everything first, then replay the frozen ten-box greedy demo."""
import argparse
import json
from pathlib import Path
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", type=Path, default=Path("demo_data/greedy10"))
    parser.add_argument("--seed", type=int, default=20261022)
    parser.add_argument("--prepare-only", action="store_true")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--fps", type=int, default=30)
    args = parser.parse_args()
    root = Path(__file__).resolve().parent
    if (args.bundle / "READY.json").is_file():
        source = json.loads((args.bundle / "input.json").read_text(encoding="utf-8"))
        if source.get("seed") != args.seed:
            parser.error("bundle contains a different seed; choose a new --bundle path")
    if not (args.bundle / "READY.json").is_file():
        if args.bundle.exists():
            parser.error("incomplete bundle exists; preserve it and choose a new --bundle path")
        subprocess.run([sys.executable, str(root / "prepare_greedy_demo.py"),
                        "--output", str(args.bundle), "--seed", str(args.seed)], check=True)
    if not args.prepare_only:
        command = [sys.executable, str(root / "replay_prepared_demo.py"),
                   "--bundle", str(args.bundle), "--fps", str(args.fps)]
        if args.headless:
            command.append("--headless")
        return subprocess.run(command).returncode
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

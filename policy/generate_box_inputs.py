"""Create reproducible ten-box JSON fixtures without Newton or a viewer."""
import argparse
from pathlib import Path
from demo_contract import generate_boxes, write_json


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("demo_data/inputs"))
    parser.add_argument("--seeds", type=int, nargs="+", default=[20261022, 20261023, 20261024])
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    for seed in args.seeds:
        path = args.output / f"boxes_seed_{seed}.json"
        if path.exists():
            parser.error("fixture already exists; choose a new output folder: " + str(path))
        write_json(path, generate_boxes(seed))
        print(path)

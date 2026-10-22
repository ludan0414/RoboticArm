"""Create ten reproducible cuboid envelopes from published suitcase dimensions.

This produces INPUT order, not a placement plan or a hardware execution list.
Dimensions are exterior, unexpanded, wheels included, trolley retracted.
For flat pickup, standing suitcase height maps to local X and depth to local Z.
"""
import argparse
import json
from pathlib import Path
import random
import math


CATALOG = {
    "20": dict(model="Samsonite ENWRAP SPINNER 55/20 EXP",
        exterior_hwd_cm=[55.0, 36.0, 24.0],
        source_url="https://www.samsonite.com.ph/samsonite/enwrap/spinner-55/20-exp/ss-125647-4386.html"),
    "23": dict(model="Samsonite ZENPOD SPINNER 63/23",
        exterior_hwd_cm=[63.0, 37.0, 28.0],
        source_url="https://www.samsonite.co.th/en/samsonite/zenpod/spinner-63/23/ss-154737-1259.html"),
    "28": dict(model="Samsonite ENWRAP SPINNER 75/28 EXP",
        exterior_hwd_cm=[75.0, 51.0, 32.0],
        source_url="https://www.samsonite.com.ph/luggage/enwrap-spinner-exp.html?dwvar_enwrap-spinner-exp_color=4386&dwvar_enwrap-spinner-exp_productSize=75+cm"),
}


def generate_suitcases(seed=20261022):
    labels = ["20"] * 4 + ["23"] * 3 + ["28"] * 3
    random.Random(seed).shuffle(labels)
    boxes = [[value / 100 for value in CATALOG[label]["exterior_hwd_cm"]]
             for label in labels]
    return dict(schema_version=1, seed=seed, units="m", boxes=boxes,
        fixture_kind="published_suitcase_cuboid_envelopes",
        sequence_kind="input_order_not_execution_order",
        geometry=dict(shape="solid_cuboid", pose="lying_flat",
            dimension_order=["standing_height_as_length_x", "width_y", "depth_as_height_z"],
            wheels_included=True, trolley_handle="retracted", expansion="closed",
            added_clearance_m=0.0),
        distribution=dict(counts_by_nominal_inches={"20": 4, "23": 3, "28": 3},
                          randomization="fixed counts, shuffled order; no dimension jitter"),
        sources_checked_on="2026-10-22", catalog=CATALOG,
        items=[dict(id="B%d" % (index + 1), input_index=index,
                    nominal_inches=int(label), catalog_key=label, dimensions_m=boxes[index])
               for index, label in enumerate(labels)])


def generate_identical_boxes(dimensions_cm):
    dims = [float(value) / 100 for value in dimensions_cm]
    if len(dims) != 3 or not all(math.isfinite(v) and v > 0 for v in dims):
        raise ValueError("fixed size requires three finite positive dimensions")
    return dict(schema_version=1, units="m", boxes=[list(dims) for _ in range(10)],
                fixture_kind="user_supplied_identical_cuboids",
                sequence_kind="input_order_not_execution_order",
                geometry=dict(shape="solid_cuboid", pose="lying_flat",
                              dimension_order=["length_x", "width_y", "height_z"],
                              added_clearance_m=0.0),
                provenance="User-supplied physical case: 55 x 35 x 20 cm" if dims == [.55, .35, .2]
                           else "User-supplied physical case dimensions",
                items=[dict(id="B%d" % (i + 1), input_index=i, dimensions_m=list(dims))
                       for i in range(10)])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=20261022)
    parser.add_argument("--fixed-size-cm", type=float, nargs=3, metavar=("L", "W", "H"),
                        help="ten identical cuboids from user measurements, no randomization")
    parser.add_argument("--output", type=Path,
                        default=Path(__file__).parent / "demo_data/inputs/suitcases10_seed_20261022.json")
    args = parser.parse_args()
    fixture = (generate_identical_boxes(args.fixed_size_cm) if args.fixed_size_cm
               else generate_suitcases(args.seed))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    # Never silently overwrite an input already used by a plan or recording.
    with args.output.open("x", encoding="utf-8", newline="\n") as stream:
        json.dump(fixture, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
    print(args.output)
    print("input order:", [item.get("nominal_inches", item["dimensions_m"])
                           for item in fixture["items"]])


if __name__ == "__main__":
    main()

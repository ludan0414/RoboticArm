"""Export commissioning defaults assuming the hardware cell matches the demo.

Only poses/planning intent are exported, never executable joint recordings.
Run from policy/ using the Newton environment (only numpy is needed here).
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def pose_dict(pose):
    pose = np.asarray(pose, dtype=float)
    if pose.shape != (7,) or not np.isfinite(pose).all():
        raise ValueError("invalid recorded pose")
    norm = np.linalg.norm(pose[3:])
    if norm < 1e-6:
        raise ValueError("invalid recorded quaternion")
    return dict(position_m=pose[:3].tolist(),
                orientation_xyzw=(pose[3:] / norm).tolist())


def sample_poses(poses):
    """Keep endpoints and bounded translation/rotation spacing along the trace.

    The resulting Cartesian chords are checked again by live PlanMotion;
    sampling a demo is not a collision certificate for those chords.
    """
    selected = [poses[0]]
    for pose in poses[1:]:
        previous = selected[-1]
        dot = abs(float(np.dot(pose[3:], previous[3:])))
        angle = 2 * np.arccos(np.clip(dot, 0, 1))
        if np.linalg.norm(pose[:3] - previous[:3]) >= .04 or angle >= .06:
            selected.append(pose)
    if not np.array_equal(selected[-1], poses[-1]):
        selected.append(poses[-1])
    return [pose_dict(pose) for pose in selected]


def export_profile(bundle):
    bundle = Path(bundle)
    plan_path, trace_path = bundle / "plan.json", bundle / "trajectory.npz"
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    if plan["feeder"]["xy_m"] != [1.2, 0.0]:
        raise ValueError("expected the base-frame +1.2m ground feeder demo")
    cycles = []
    with np.load(trace_path, allow_pickle=False) as trace:
        first_id = plan["sequence"][0]["id"]
        observe_indices = np.flatnonzero((trace["box_id"] == first_id)
                                        & (trace["stage"] == "OBSERVE"))
        observe = pose_dict(trace["tool_pose"][observe_indices[-1]])
        for row in plan["sequence"]:
            this_box = trace["box_id"] == row["id"]
            carrying = np.flatnonzero(this_box & (trace["stage"] == "CARRY"))
            retreat = np.flatnonzero(this_box & (trace["stage"] == "RETREAT"))
            returning = np.flatnonzero(this_box & (trace["stage"] == "RETURN"))
            if not len(carrying) or not len(retreat) or not len(returning):
                raise ValueError("incomplete demo stages for " + row["id"])
            # Start the empty route at the original high retreat pose: the real
            # driver first retreats only clearance_m after release. This first
            # exit segment lifts it to the recording's exit height.
            exit_indices = np.r_[retreat[-1], returning]
            cycles.append(dict(
                demo_box_id=row["id"],
                expected_demo_input_dims_m=row["input_dims_m"],
                place_target=dict(frame_id="world", pose_reference="box_center",
                    position_m=row["position_world_m"],
                    orientation_xyzw=row["orientation_world_xyzw"],
                    clearance_m=.15,
                    via_box_poses=sample_poses(trace["box_pose"][carrying])),
                exit_tool_poses=sample_poses(trace["tool_pose"][exit_indices])))
    hashes = {name: hashlib.sha256((bundle / name).read_bytes()).hexdigest()
              for name in ("plan.json", "trajectory.npz", "scene_tf.yaml")}
    return dict(profile="site_commissioning_v1", frame_id="world",
                tool_frame="suction_contact_frame",
                assumptions=dict(
                    calibration_source="simulation_layout_assumed_equal_to_hardware",
                    source_bundle="policy/demo_data/" + bundle.name,
                    source_sha256=hashes,
                    feeder_frame="elfin_base_link", feeder_xy_m=[1.2, 0.0],
                    ground_world_z_m=0.0,
                    base_world_translation_m=[0.0, 0.0, .86],
                    base_world_yaw_rad=1.5708,
                    hardware_motion_validated=False,
                    expected_dims_are_detection_inputs=False),
                observe_tool_pose=observe, cycles=cycles)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", type=Path,
                        default=Path(__file__).parent / "demo_data/ground_greedy10_observe")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()
    profile = export_profile(args.bundle)
    with args.output.open("w" if args.force else "x", encoding="utf-8", newline="\n") as stream:
        json.dump(profile, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
    print("Exported %d commissioning cycles to %s" % (len(profile["cycles"]), args.output))


if __name__ == "__main__":
    main()

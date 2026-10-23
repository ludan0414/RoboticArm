"""Export commissioning defaults assuming the hardware cell matches the demo.

Only poses/planning intent are exported, never executable joint recordings.
Run from policy/ using the Newton environment (numpy, scipy and PyYAML).
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from demo_contract import check_bundle
from demo_pose_sampling import sample_pose_indices


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
    return [pose_dict(poses[index]) for index in sample_pose_indices(poses)]


def post_place_exit_indices(trace, box_id):
    """Exclude pre-spawn positioning tagged with this same box ID."""
    this_box = trace["box_id"] == box_id
    retreat = np.flatnonzero(this_box & (trace["stage"] == "RETREAT"))
    if not len(retreat):
        raise ValueError("missing retreat for " + box_id)
    returning = np.flatnonzero(this_box & (trace["stage"] == "RETURN"))
    returning = returning[returning > retreat[-1]]
    if not len(returning):
        raise ValueError("missing post-place return for " + box_id)
    return np.r_[retreat[-1], returning]


def export_profile(bundle):
    bundle = Path(bundle)
    check_bundle(bundle, allow_partial=True)
    plan_path, trace_path = bundle / "plan.json", bundle / "trajectory.npz"
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    if plan["feeder"]["frame"] != "elfin_base" or plan["feeder"]["xy_m"] != [-1.6, 0.0]:
        raise ValueError("expected the reviewed base-frame -1.6m right-hand feeder")
    if plan.get("ground_carry_clearance", {}).get("pedestal_margin_m") != .05:
        raise ValueError("reviewed high-clearance route required")
    scene = yaml.safe_load((bundle / "scene_tf.yaml").read_text(encoding="utf-8"))
    pedestal = scene["pedestal"]
    base_rotation = (Rotation.from_euler("xyz", pedestal["rotation_rpy"])
                     * Rotation.from_euler("xyz", scene["robot"]["rotation_rpy"]))
    base_position = (np.asarray(pedestal["translation"]) + Rotation.from_euler(
        "xyz", pedestal["rotation_rpy"]).apply([0, 0, pedestal["size"][2]]))
    container = next(pose for pose in scene["static_transforms"] if pose["child"] == "container_link")
    cycles = []
    with np.load(trace_path, allow_pickle=False) as trace:
        first_id = plan["sequence"][0]["id"]
        observe_indices = np.flatnonzero((trace["box_id"] == first_id)
                                        & (trace["stage"] == "OBSERVE"))
        observe = pose_dict(trace["tool_pose"][observe_indices[-1]])
        for row in plan["sequence"]:
            this_box = trace["box_id"] == row["id"]
            carrying = np.flatnonzero(this_box & (trace["stage"] == "CARRY"))
            if not len(carrying):
                raise ValueError("incomplete demo stages for " + row["id"])
            # Start the empty route at the original high retreat pose: the real
            # driver first retreats only clearance_m after release. This first
            # exit segment lifts it to the recording's exit height.
            exit_indices = post_place_exit_indices(trace, row["id"])
            cycles.append(dict(
                demo_box_id=row["id"],
                box_label=row.get("label", row["id"]),
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
                payload_geometry_source="configured_cycle",
                return_pose_adjustment=dict(offset_base_m=[.8, 0., -.2],
                    camera_optical_frame="d555_color_optical_frame"),
                assumptions=dict(
                    calibration_source="user_provided_scene_tf_yaml",
                    source_bundle="policy/demo_data/" + bundle.name,
                    source_sha256=hashes,
                    feeder_frame="elfin_base_link", feeder_xy_m=[-1.6, 0.0],
                    observe_base_xy_m=plan.get("observation", {}).get("base_xy_m"),
                    ground_world_z_m=0.0,
                    base_world_translation_m=base_position.tolist(),
                    base_world_yaw_rad=float(base_rotation.as_euler("xyz")[2]),
                    hardware_motion_validated=False,
                    expected_dims_are_detection_inputs=True),
                motion_safety=dict(schema_version=1,
                    expected_base_world_pose=pose_dict(np.r_[base_position, base_rotation.as_quat()]),
                    expected_container_world_pose=pose_dict(np.r_[container["translation"],
                        Rotation.from_euler("xyz", container["rotation_rpy"]).as_quat()]),
                    feeder_base_xy_m=[-1.6, 0.0], ground_base_z_m=-float(base_position[2]),
                    pickup_position_source="vision", max_segment_translation_m=.50,
                    max_segment_rotation_rad=float(np.pi / 2),
                    enforce_segment_heuristics=False,
                    axis_test_step_m=.02),
                carry_entry=dict(strategy="outside_retract_high_lift_v1",
                    pedestal_center_world_pose=pose_dict(np.r_[
                        np.asarray(pedestal["translation"]) + [0, 0, pedestal["size"][2]/2],
                        Rotation.from_euler("z", pedestal["rotation_rpy"][2]).as_quat()]),
                    pedestal_size_m=pedestal["size"], pedestal_margin_m=.05,
                    high_bottom_above_pedestal_m=.15, outer_lift_tool_world_z_m=float(base_position[2]),
                    staging_radius_m=1.25),
                observe_tool_pose=observe, cycles=cycles)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", type=Path,
                        default=Path(__file__).parent / "demo_data/site_four_colors_right160_observe120")
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

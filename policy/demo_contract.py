"""Portable data and integrity contracts for prepared Newton demos."""
import hashlib
import json
import math
from pathlib import Path
import random


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def recorded_urdf_digest(urdf, source_root, recorded_source_root):
    """Compare identical assets after relocating a Windows/Linux checkout.

    Flattened URDF mesh filenames are absolute. Substitute only that known
    asset-root prefix for hashing; mesh contents retain independent hashes.
    """
    current = Path(source_root).resolve().as_posix()
    recorded = str(recorded_source_root).replace("\\", "/")
    return hashlib.sha256(urdf.replace(current, recorded).encode()).hexdigest()


def text_digest_matches(path, expected):
    """Accept CRLF/LF checkout differences without accepting text changes."""
    original = Path(path).read_bytes()
    content = original.replace(b"\r\n", b"\n")
    return expected in {hashlib.sha256(original).hexdigest(), hashlib.sha256(content).hexdigest(),
                        hashlib.sha256(content.replace(b"\n", b"\r\n")).hexdigest()}


def generate_boxes(seed, count=10):
    rng = random.Random(seed)
    boxes = []
    for _ in range(count):
        # Correlated footprints permit fully-supported stacks without
        # rejection-sampling successful box lists. Every seed stays fixed.
        scale = rng.uniform(0.86, 1.0)
        boxes.append([round(.50 * scale, 3), round(.40 * scale, 3), rng.choice([.25, .30])])
    return dict(schema_version=1, seed=seed, units="m", boxes=boxes,
                distribution=dict(footprint_scale_uniform=[.86, 1.0],
                                  base_footprint_m=[.50, .40], height_choices_m=[.25, .30]))


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, ensure_ascii=False), encoding="utf-8")


def load_box_input(path):
    value = json.loads(Path(path).read_text(encoding="utf-8"))
    result = dict(value) if isinstance(value, dict) else dict(boxes=value)
    boxes = result.get("boxes")
    if not isinstance(boxes, list) or len(boxes) != 10:
        raise ValueError("demo input must contain exactly 10 boxes")
    normalized = []
    for dims in boxes:
        if not isinstance(dims, (list, tuple)) or len(dims) != 3:
            raise ValueError("each box needs L,W,H")
        dims = [float(v) for v in dims]
        if not all(math.isfinite(v) and v > 0 for v in dims):
            raise ValueError("box dimensions must be finite and positive")
        normalized.append(dims)
    result.update(boxes=normalized, units="m")
    return result


def freeze_bundle(folder):
    folder = Path(folder)
    names = ["input.json", "domain.json", "plan.json", "trajectory.npz", "scene_tf.yaml",
             "validation.json", "model_manifest.json"]
    manifest = dict(schema_version=1, ready=True,
                    files={name: digest(folder / name) for name in names})
    write_json(folder / "READY.json", manifest)
    return manifest


def check_bundle(folder):
    folder = Path(folder)
    ready = json.loads((folder / "READY.json").read_text(encoding="utf-8"))
    if ready.get("ready") is not True:
        raise ValueError("bundle is not ready")
    for name, expected in ready["files"].items():
        if Path(name).name != name or digest(folder / name) != expected:
            raise ValueError("bundle integrity mismatch: " + name)
    plan = json.loads((folder / "plan.json").read_text(encoding="utf-8"))
    inputs = json.loads((folder / "input.json").read_text(encoding="utf-8"))
    if len(plan["sequence"]) != len(inputs["boxes"]) or len(plan["sequence"]) != 10:
        raise ValueError("demo requires all 10 input boxes; partial plans are rejected")
    ids = [row["input_index"] for row in plan["sequence"]]
    if sorted(ids) != list(range(10)):
        raise ValueError("plan must contain each input box exactly once")
    return plan

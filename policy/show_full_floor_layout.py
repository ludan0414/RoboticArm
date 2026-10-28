"""Display the finished 32-box geometry layout; no motion or reachability claim."""
import argparse
import math
from pathlib import Path
import time

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from container_demo import make_ground_grid
import newton
import warp as wp
from full_floor_layout import full_floor_cells
from online_packing_demo import Box, collides, fully_supported
from online_packing_real import Hull
from online_packing_newton import Pose, world_pose, make_model
from roboticarm_newton import DEFAULT_SOURCE


def build_layout(hull, count):
    cells = full_floor_cells(hull)
    if not 1 <= count <= len(cells):
        raise ValueError('count must be between 1 and %d' % len(cells))
    placed = []
    for index, cell in enumerate(cells[:count]):
        dims = (.55,.35,.20) if abs(cell.yaw) < 1e-6 else (.35,.55,.20)
        box = Box('B%d' % (index+1), dims,
                  (cell.x-dims[0]/2, cell.y-dims[1]/2, round(cell.top-dims[2],6)))
        if (not hull.fits(box) or any(collides(box, other) for other in placed)
                or not (hull.floor_supports(box) or fully_supported(box, placed))):
            raise ValueError('invalid support or overlap at ' + box.id)
        placed.append(box)
    return cells[:count], placed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene-config', type=Path, default=Path('../deployment_ws/config/scene_tf.loading_site.yaml'))
    parser.add_argument('--count', type=int, default=32)
    parser.add_argument('--device', default='cuda:0')
    args = parser.parse_args()
    scene = args.scene_config.resolve()
    hull = Hull.from_config(yaml.safe_load(scene.read_text(encoding='utf-8')))
    cells, boxes = build_layout(hull, args.count)
    sample = dict(container=hull.as_dict(), steps=[dict(incoming=dict(id=b.id, dims=[.55,.35,.20])) for b in boxes])
    wp.set_device(args.device)
    model, ids, container, _ = make_model(sample, DEFAULT_SOURCE.resolve(), scene)
    state = model.state()
    newton.eval_fk(model, model.joint_q, model.joint_qd, model)
    transforms = state.body_q.numpy()
    transforms[:] = model.body_q.numpy()
    for cell, box in zip(cells, boxes):
        local = Pose(np.array([cell.x,cell.y,cell.top-.1]), Rotation.from_euler('z',cell.yaw).as_quat())
        pose = world_pose(local, container, hull)
        transforms[ids[box.id]] = np.r_[pose.center,pose.quat]
    state.body_q.assign(transforms)
    from newton import viewer as newton_viewer
    viewer = newton_viewer.ViewerGL(width=1280, height=800)
    viewer.set_model(model)
    viewer.set_camera(pos=wp.vec3(3.7,-5.8,3.2), pitch=-12, yaw=136)
    viewer.renderer.set_title('Newton | %d-box finished layout | MOTION NOT VALIDATED' % len(boxes))
    grid = make_ground_grid(z=.003)
    print('FINISHED_LAYOUT %d boxes; support checked; arm movement NOT validated' % len(boxes),flush=True)
    try:
        while viewer.is_running():
            viewer.begin_frame(0.)
            viewer.log_state(state)
            viewer.log_lines('ground_grid', *grid)
            viewer.end_frame()
            time.sleep(1/60)
    finally:
        viewer.close()

if __name__ == '__main__':
    main()

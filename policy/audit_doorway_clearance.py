"""Independent oriented-payload clearance audit of a frozen recording.

Uses vertices of OBB intersected with the full physical front-frame X slab,
not only TCP positions or waypoint endpoints. This is a discrete-frame audit.
"""
import argparse
import itertools
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

from demo_contract import check_bundle,write_json
from doorway_scene import DoorwayGeometry


def audit(bundle):
    bundle=Path(bundle)
    plan=check_bundle(bundle)
    scene=yaml.safe_load((bundle/'scene_tf.yaml').read_text(encoding='utf-8'))
    door=DoorwayGeometry.from_scene(scene)
    transform=next(t for t in scene['static_transforms'] if t['child']=='container_link')
    inverse=Rotation.from_euler('xyz',transform['rotation_rpy']).inv()
    origin=np.asarray(transform['translation'])
    x,y0,y1,z0,z1=door.bounds
    planes=(-door.outer_size[0]/2,x)
    signs=np.asarray(list(itertools.product((-1,1),repeat=3)))
    edges=[(i,j) for i in range(8) for j in range(i+1,8) if np.count_nonzero(signs[i]!=signs[j])==1]
    reports=[]
    with np.load(bundle/'trajectory.npz') as data:
        for row in plan['sequence']:
            best=None
            count=0
            indices=np.flatnonzero((data['box_id']==row['id']) & data['attached'])
            for frame in indices:
                pose=data['box_pose'][frame]
                center=inverse.apply(pose[:3]-origin)
                rotation=inverse*Rotation.from_quat(pose[3:])
                vertices=center+rotation.apply(signs*np.asarray(row['input_dims_m'])/2)
                points=[v for v in vertices if planes[0]<=v[0]<=planes[1]]
                for plane in planes:
                    for a,b in edges:
                        denominator=vertices[b,0]-vertices[a,0]
                        if abs(denominator)<1e-12:
                            continue
                        fraction=(plane-vertices[a,0])/denominator
                        if 0<=fraction<=1:
                            points.append(vertices[a]+fraction*(vertices[b]-vertices[a]))
                if not points:
                    continue
                count+=1
                points=np.asarray(points)
                gaps=dict(left=float(y1-points[:,1].max()),right=float(points[:,1].min()-y0),
                          top=float(z1-points[:,2].max()),bottom=float(points[:,2].min()-z0))
                if best is None or min(gaps.values())<best['minimum_m']:
                    best=dict(frame=int(frame),stage=str(data['stage'][frame]),minimum_m=min(gaps.values()),gaps_m=gaps)
            reports.append(dict(box=row['id'],label=row.get('label'),crossing_frames=count,worst=best))
    success=all(row['crossing_frames']>0 and row['worst']['minimum_m']>=door.clearance-1e-6 for row in reports)
    return dict(success=success,required_clearance_m=door.clearance,boxes=reports,
                minimum_m=min(row['worst']['minimum_m'] for row in reports if row['worst']),
                scope='all recorded attached frames, full configured front slab; no continuous/hardware claim')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bundle',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    result=audit(args.bundle)
    write_json(args.output,result)
    print(json.dumps(result),flush=True)
    if not result['success']:
        raise SystemExit(1)


if __name__=='__main__':
    main()

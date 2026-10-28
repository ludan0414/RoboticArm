"""Offline reproducible multi-seed probes of deep top-contact poses."""
import json
from pathlib import Path
import numpy as np
import warp as wp
import yaml
from demo_workspace import CertifiedWorkspace
from online_packing_real import Hull
from roboticarm_newton import DEFAULT_SOURCE
out = Path('../docs/status/evidence/2026-10-28-deep-seeds')
out.mkdir(parents=True, exist_ok=False)
scene = Path('../deployment_ws/config/scene_tf.loading_site.yaml').resolve()
hull = Hull.from_config(yaml.safe_load(scene.read_text(encoding='utf-8')))
wp.set_device('cuda:0')
rng = np.random.default_rng(20261028)
with (out/'boundary.jsonl').open('w', encoding='utf-8') as trace:
    w = CertifiedWorkspace(DEFAULT_SOURCE.resolve(), scene, device='cuda:0', box_size=[.55,.35,.2],trace=trace)
    lo, hi = w.model.joint_limit_lower.numpy()[:6],w.model.joint_limit_upper.numpy()[:6]
    results=[]
    for local in ([.965,.275,.2], [1.315,.275,.2], [.965,.825,.4], [1.315,.825,.4]):
        pos,yaw=w.local_contact_world(local,(0,0,0),-np.pi/2,hull)
        orientation=w.tool_orientation(yaw)
        attempts=[]
        for n in range(24):
            q=w.q_default.copy()
            q[:6]=rng.uniform(lo,hi) if n else w.q_default[:6]
            ok,solution=w._solve_once(pos,orientation,q,True)
            attempts.append(dict(seed=q.tolist(),ok=bool(ok),diagnostics=dict(w.last_diagnostics),collision_pairs=w.last_collision_pairs))
            print('SEED local=%s trial=%d ok=%s error=%.3f' % (local,n,ok,w.last_diagnostics['position_error_m']),flush=True)
            if ok: break
        results.append(dict(local=local, attempts=attempts, passed=any(a['ok'] for a in attempts)))
        (out/'results.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
print('EXPERIMENT_COMPLETE',flush=True)

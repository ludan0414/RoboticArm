"""Deterministic IK, bidirectional RRT and Newton checking of frozen box routes."""
from dataclasses import replace
from types import SimpleNamespace as NS
import math
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
from luggage_planning.pose import Pose,Point,Quaternion
from luggage_planning.policy_place import PlaceTarget,build_place_segments
from luggage_planning.placement_payload import release_above_support


def xyz(p):return np.array([getattr(p.position,k) for k in 'xyz'],dtype=float)
def quat(p):return np.array([getattr(p.orientation,k) for k in 'xyzw'],dtype=float)
def pose_data(p):return dict(position_m=xyz(p).tolist(),orientation_xyzw=quat(p).tolist())
def pose_array(a):return Pose(Point(*map(float,a[:3])),Quaternion(*map(float,a[3:])))


def checked_pose(run,goal,reference,extra_seeds=()):
    from luggage_planning.ik_search import nearby_ik_seeds
    from luggage_planning.wrist_check import measure_wrist
    from joint_approach_path import nearest_revolute_solution
    attempts=[]
    for seed in [*nearby_ik_seeds(np.asarray(reference).tolist()),*extra_seeds]:
        ok,q=run.ik._solve_once(xyz(goal),quat(goal),np.asarray(seed,dtype=np.float32),True)
        reason='ik'
        if ok:
            q=nearest_revolute_solution(q,reference,run.model.joint_limit_lower.numpy()[:6],run.model.joint_limit_upper.numpy()[:6])
            wrist=measure_wrist([reference,q]);reason='controller_wrist_branch'
            if wrist['ok']:
                reason='robot_or_payload_collision'
                if run.valid(q):
                    run.ik.q_last=q.astype(np.float32);return q,attempts
        attempts.append(dict(reason=reason,seed=np.asarray(seed).tolist(),joint_q=q.tolist(),
            diagnostics=dict(run.ik.last_diagnostics),
            payload_contacts=run.payload_contact_failures() if reason=='robot_or_payload_collision' else []))
    return None,attempts


def checked_tool_path(run,goals,start_q):
    """Plan only; every IK sample checks the full arm, held box and prior cargo."""
    run.refresh(start_q);last=pose_array(run.state.body_q.numpy()[run.contact_body])
    run.ik.q_last=np.asarray(start_q,dtype=np.float32)
    from luggage_planning.wrist_check import measure_wrist
    path=[];run.last_path_rejection=None;previous=np.asarray(start_q)
    for goal in goals:
        count=max(2,math.ceil(np.linalg.norm(xyz(goal)-xyz(last))/.015),
                  math.ceil((Rotation.from_quat(quat(last)).inv()*Rotation.from_quat(quat(goal))).magnitude()/math.radians(2)))
        rotations=Slerp([0,1],Rotation.from_quat([quat(last),quat(goal)]))
        for alpha in np.linspace(0,1,count+1)[1:]:
            target=xyz(last)+alpha*(xyz(goal)-xyz(last))
            ok,q=run.ik.solve_world_pose(target,0.,orientation_xyzw=rotations(alpha).as_quat())
            valid=ok and measure_wrist([previous,q])['ok'] and run.valid(q)
            if not valid:
                # A cached/local IK failure is not a global reachability proof.
                # Retry direct solver starts on the actual current branch.
                endpoint=getattr(run,'route_end_seed',None)
                extra=() if endpoint is None else (endpoint,)
                q,_=checked_pose(run,Pose(Point(*target),Quaternion(*rotations(alpha).as_quat())),previous,extra)
                valid=q is not None
            if not valid:
                run.last_path_rejection=dict(target_world_m=target.tolist(),
                    target_quaternion_xyzw=rotations(alpha).as_quat().tolist(),
                    joint_q=None if q is None else q.tolist(),
                    diagnostics=dict(run.ik.last_diagnostics),
                    robot_contacts=run.ik.last_collision_pairs,
                    payload_contacts=[] if not ok else run.payload_contact_failures())
                return None
            edges=max(1,math.ceil(float(np.max(np.abs(q-previous)))/.035))
            for fraction in np.linspace(0,1,edges+1)[1:-1]:
                intermediate=previous+(q-previous)*fraction
                if not run.valid(intermediate):
                    run.last_path_rejection=dict(reason='interpolated_joint_edge_collision',joint_q=intermediate.tolist())
                    return None
                path.append(intermediate.copy())
            path.append(q.copy())
            previous=q.copy()
        last=goal
    return path


def freeze_samples(run,target,path):
    from luggage_planning.placement_payload import RELEASE_SUPPORT_GAP_M
    dense=[]
    for q in path[:-1]:
        run.refresh(q)
        pose=pose_array(run.state.body_q.numpy()[run.body_ids[run.box_id]])
        raw=xyz(pose)-Rotation.from_quat(quat(pose)).apply([0,0,RELEASE_SUPPORT_GAP_M])
        dense.append(Pose(Point(*raw),pose.orientation))
    return replace(target,via=tuple(dense),route_source='offline_frozen_v1')


def find_joint_route(run,target,grasp,release_q,output,write_json):
    """Classical bidirectional RRT; no box-name or sequence-index exceptions."""
    from joint_approach_path import plan_joint_approach
    from luggage_planning.wrist_check import measure_wrist
    start=run.q.copy()
    segments=build_place_segments(release_above_support(target),
        NS(frame='world',yaw_valid=True),NS(frame='world'),measured_grasp=grasp)
    run.stage='PLACE'
    # Solve the short vertical approach outward from its certified release
    # solution, retaining that branch instead of jumping at the last waypoint.
    up=checked_tool_path(run,[segments[-3].target_pose],release_q)
    if not up:
        write_json(output/'joint_search.json',dict(success=False,phase='vertical_approach',rejection=run.last_path_rejection))
        run.q=start;run.refresh(start);return None
    goal=up[-1]
    capture=dict(start=start.tolist(),goal=goal.tolist(),dimensions=run.current_box_size,
        placed={name:pose.tolist() for name,pose in run.placed.items()},
        valid_calls=0,rejected=0,rejection_samples=[],algorithm='bidirectional_rrt_seed7',max_seconds=45)
    write_json(output/'joint_search.json',capture)
    def valid(q):
        capture['valid_calls']+=1
        wrist=measure_wrist([start,q]);ok=wrist['ok'] and run.valid(q)
        if not ok:
            capture['rejected']+=1
            if len(capture['rejection_samples'])<12:
                capture['rejection_samples'].append(dict(q=q.tolist(),wrist=wrist,
                    robot=run.ik.last_collision_pairs,
                    payload=run.payload_contact_failures() if wrist['ok'] else []))
        if capture['valid_calls']%100==0:write_json(output/'joint_search.json',capture)
        return ok
    run.stage='CARRY'
    try:
        knots=plan_joint_approach(start,goal,run.model.joint_limit_lower.numpy()[:6],
            run.model.joint_limit_upper.numpy()[:6],valid,resolution=.05,max_seconds=45)
        carry=[]
        for a,b in zip(knots,knots[1:]):
            count=max(2,math.ceil(float(np.max(np.abs(b-a)))/.025))
            for alpha in np.linspace(0,1,count+1)[1:]:
                q=a+alpha*(b-a)
                if not valid(q):raise RuntimeError('dense RRT edge collision')
                carry.append(q.copy())
        frozen=freeze_samples(run,target,carry)
        descend=list(reversed([release_q,*up[:-1]]))
        capture.update(success=True,knots=[q.tolist() for q in knots],frames=len(carry))
        write_json(output/'joint_search.json',capture)
        return frozen,carry,descend
    except RuntimeError as exc:
        capture.update(success=False,error=str(exc));write_json(output/'joint_search.json',capture)
        return None
    finally:
        run.q=start;run.refresh(start)

"""Recover incomplete Cartesian requests by reaching every requested pose.

Fraction measures Cartesian planner coverage, not feasibility of every other
planner. Discard its partial prefix. Use complete collision-checked IK/joint
plans in route order, retaining attached-payload and wrist checks. A successful
subgoal is a real motion; if a later one fails, stop with vacuum unchanged.
"""
from copy import deepcopy

from luggage_planning.motion_boundary import robot_traj_to_dict


def replan_complete_route(executor, segment, feedback, timeout, current_joints, fraction,
                         partial_plan=None):
    targets = [*getattr(segment, 'waypoints', ()), segment.target_pose]
    trace = executor._cartesian_recovery = []
    executor._cartesian_partial = robot_traj_to_dict(getattr(partial_plan, 'solution', None))
    seed = current_joints
    reached_boundary = False
    for index, target in enumerate(targets):
        cancel = getattr(executor, '_cancel_check', None)
        if cancel is not None and cancel():
            result = executor._result(False, '%s: cancelled before replan goal %d/%d' %
                                      (segment.name, index+1, len(targets)), fraction)
            break
        hop = deepcopy(segment)
        hop.type, hop.target_pose, hop.waypoints = 'pose_target', deepcopy(target), []
        executor._notify(feedback, 'planning', segment.name, fraction,
                         'Cartesian incomplete; full-goal replan %d/%d' % (index+1, len(targets)))
        executor._planned_traj = None
        projection_count = len(getattr(executor, '_reach_projections', ()))
        result = executor._run_pose_target(
            hop, feedback, timeout, seed,
            allow_reach_projection=not reached_boundary and index+1 < len(targets))
        trajectory = getattr(executor, '_planned_traj', None)
        item = dict(index=index, success=bool(result.success), message=result.message,
                    trajectory=robot_traj_to_dict(trajectory))
        trace.append(item)
        if not result.success:
            if (reached_boundary and index+1 < len(targets)
                    and getattr(executor, '_last_ik_code', None) == -31
                    and result.message.startswith('IK failed error_code=-31')):
                # At the reach edge, lower in place before trying to reach a
                # deeper lane. An exact IK and a full plan are still required.
                current_pose = executor._fk_pose(seed)
                bridge = deepcopy(hop)
                if current_pose is not None:
                    bridge.target_pose.position.x = current_pose.position.x
                    bridge.target_pose.position.y = current_pose.position.y
                if (current_pose is not None and
                        target.position.z < current_pose.position.z - .01):
                    result = executor._run_pose_target(
                        bridge, feedback, timeout, seed, allow_reach_projection=False)
                    trajectory = getattr(executor, '_planned_traj', None)
                    item.update(success=bool(result.success), message=result.message,
                                trajectory=robot_traj_to_dict(trajectory),
                                boundary_lowering=True)
                    if not result.success and not (
                            getattr(executor, '_last_ik_code', None) == -31 and
                            result.message.startswith('IK failed error_code=-31')):
                        result.message = '%s: boundary lowering failed: %s' % (segment.name, result.message)
                        break
                if not result.success:
                    item['skipped_unreachable_after_boundary'] = True
                    continue
            else:
                result.message = '%s: full-goal replan %d/%d failed: %s' % (
                    segment.name, index+1, len(targets), result.message)
                break
        if len(getattr(executor, '_reach_projections', ())) > projection_count:
            reached_boundary = True
        if index+1 < len(targets) or getattr(executor, '_reach_projections', None):
            joint = getattr(trajectory, 'joint_trajectory', None)
            if joint is None or not joint.points:
                result = executor._result(False, '%s: replan endpoint joints missing' % segment.name, fraction)
                break
            seed = list(joint.points[-1].positions)
            if getattr(executor, '_reach_projections', None):
                from luggage_planning.reach_limit import pose_dict
                pose = executor._fk_pose(seed)
                if pose is None:
                    result = executor._result(False, '%s: achieved TCP unavailable after reach projection' % segment.name, fraction)
                    break
                item['achieved_pose'] = pose_dict(pose)
                # Earlier hops had exact IK goals; preserve them too, so the
                # client never records unexecuted projected requests on return.
                for previous in trace:
                    if previous['success'] and 'achieved_pose' not in previous:
                        previous['achieved_pose'] = pose_dict(targets[previous['index']])
    else:
        result.message = '%s: completed %d transit goals by replan (Cartesian fraction %.3f)' % (
            segment.name, len(targets), fraction)
    result.fraction = fraction
    result.used_ompl_fallback = True
    return result

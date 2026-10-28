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
        result = executor._run_pose_target(hop, feedback, timeout, seed)
        trajectory = getattr(executor, '_planned_traj', None)
        trace.append(dict(index=index, success=bool(result.success), message=result.message,
                          trajectory=robot_traj_to_dict(trajectory)))
        if not result.success:
            result.message = '%s: full-goal replan %d/%d failed: %s' % (
                segment.name, index+1, len(targets), result.message)
            break
        if index+1 < len(targets):
            joint = getattr(trajectory, 'joint_trajectory', None)
            if joint is None or not joint.points:
                result = executor._result(False, '%s: replan endpoint joints missing' % segment.name, fraction)
                break
            seed = list(joint.points[-1].positions)
    else:
        result.message = '%s: reached all %d poses by full-goal replan (Cartesian fraction %.3f)' % (
            segment.name, len(targets), fraction)
    result.fraction = fraction
    result.used_ompl_fallback = True
    return result

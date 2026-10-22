"""ROS-free measured-payload entry to the reviewed high carry route.

No joint replay. Every generated segment still needs full live Cartesian
planning with the attached measured payload and the live collision scene.
"""
import math
from luggage_planning.pose import Point, Pose, Quaternion, MotionSegment
from luggage_planning.policy_place import _pose, _vector, grasp_transform, connect_place_from_pose
from luggage_planning.suction_candidate_waypoints import quat_rotate, quat_mul, quat_conjugate


def dot(a, b):
    return sum(x*y for x, y in zip(a, b))


def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def axes(q):
    return [quat_rotate(q, v) for v in ((1, 0, 0), (0, 1, 0), (0, 0, 1))]


class CarryConfig:
    def __init__(self, settings):
        if settings.get('strategy') != 'outside_retract_high_lift_v1':
            raise ValueError('unsupported carry entry strategy')
        self.pedestal = _pose(settings['pedestal_center_world_pose'])
        self.size = _vector(settings['pedestal_size_m'], 3, 'pedestal_size_m')
        self.margin = float(settings['pedestal_margin_m'])
        self.clearance = float(settings['high_bottom_above_pedestal_m'])
        self.outer_z = float(settings['outer_lift_tool_world_z_m'])
        self.radius = float(settings['staging_radius_m'])
        if (min(self.size) <= 0 or not .05 <= self.margin <= .20
                or not .15 <= self.clearance <= .4 or not .8 <= self.radius <= 1.4
                or not math.isfinite(self.outer_z)):
            raise ValueError('invalid carry entry geometry/clearance')


class CarryEntry:
    def __init__(self, config, detection, candidate):
        self.config = config
        self.size = _vector(detection.box_size, 3, 'measured box_size')
        if min(self.size) <= 0:
            raise ValueError('positive measured payload dimensions required')
        self.offset, self.relative_q = grasp_transform(detection, candidate)

    def box_pose(self, tool):
        q = quat_mul(tool.orientation, quat_conjugate(self.relative_q))
        offset = quat_rotate(q, self.offset)
        return Pose(Point(*(a-b for a, b in zip(
            (tool.position.x, tool.position.y, tool.position.z), offset))), q)

    def bottom(self, box):
        return box.position.z - sum(abs(axis[2])*size/2 for axis, size in zip(axes(box.orientation), self.size))

    def pedestal_clear(self, box):
        config = self.config
        pa, ba = axes(config.pedestal.orientation), axes(box.orientation)
        delta = tuple(getattr(box.position, key)-getattr(config.pedestal.position, key) for key in ('x','y','z'))
        for axis in pa + ba + [cross(a,b) for a in pa for b in ba]:
            if dot(axis, axis) < 1e-12:
                continue
            span = sum(abs(dot(a, axis))*(s/2+config.margin) for a,s in zip(pa,config.size))
            span += sum(abs(dot(a, axis))*s/2 for a,s in zip(ba,self.size))
            if abs(dot(delta, axis)) > span:
                return True
        return False

    def connect(self, actual, segments):
        box = self.box_pose(actual)
        config = self.config
        delta = (box.position.x-config.pedestal.position.x, box.position.y-config.pedestal.position.y)
        distance = math.hypot(*delta)
        if distance < 1e-6:
            raise ValueError('no outside pedestal staging direction')
        direction = (delta[0]/distance, delta[1]/distance, 0)
        support = sum(abs(dot(a,direction))*s/2 for a,s in zip(axes(config.pedestal.orientation),config.size))
        outside = support + math.hypot(*self.size[:2])/2 + .15
        if distance < outside:
            raise ValueError('measured pickup lacks outside pedestal staging clearance')
        radius = min(distance, max(config.radius, outside))
        sx = config.pedestal.position.x + direction[0]*radius
        sy = config.pedestal.position.y + direction[1]*radius
        outer_z = max(actual.position.z, config.outer_z)
        pedestal_top = config.pedestal.position.z + sum(abs(a[2])*s/2 for a,s in zip(axes(config.pedestal.orientation),config.size))
        high_z = max(outer_z, actual.position.z + pedestal_top + config.clearance + .005 - self.bottom(box))
        poses = [Pose(Point(actual.position.x, actual.position.y, outer_z), actual.orientation),
                 Pose(Point(actual.position.x+sx-box.position.x, actual.position.y+sy-box.position.y, outer_z), actual.orientation),
                 Pose(Point(actual.position.x+sx-box.position.x, actual.position.y+sy-box.position.y, high_z), actual.orientation)]
        result, previous = [], actual
        for name, target in zip(('outer_lift', 'outside_retract', 'high_lift'), poses):
            goal = MotionSegment(name='ground_entry_'+name, type='cartesian', target_pose=target,
                                 required_cartesian_fraction=1.0, allow_ompl_fallback=False)
            pieces = connect_place_from_pose(previous, [goal])
            for index, piece in enumerate(pieces[:-1]):
                piece.name = goal.name+'_%d' % index
            result.extend(pieces)
            previous = target
        return result + connect_place_from_pose(previous, segments)

    def check_segment(self, actual, segment):
        # These names all execute before release. Retreat and return are empty.
        if not (segment.name.startswith(('ground_entry_', 'place_via_', 'place_pickup_connect_'))
                or segment.name in ('pre_place', 'place_descend')):
            return True, 'not a carrying segment'
        samples = connect_place_from_pose(actual, [segment], .01, math.radians(2))
        for pose in [actual] + [s.target_pose for s in samples]:
            if not self.pedestal_clear(self.box_pose(pose)):
                return False, 'CARRY_PEDESTAL_CLEARANCE: measured payload needs %.0fcm margin' % (100*self.config.margin)
        return True, 'payload/pedestal margin checked'

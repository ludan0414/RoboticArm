"""Centre the entire payload before crossing the door, then select its aisle.

This changes commissioning box waypoints, not joint commands or slot selection.
All poses still require live attached-payload MoveIt planning. Configured earlier
boxes supply conservative transit height only; they are not measured occupancy.
"""
from dataclasses import asdict, replace
import math

from luggage_description.doorway_clearance import rotation_matrix, rotate
from .pose import MotionSegment, Point, Pose
from .policy_place import connect_place_from_pose

EXTRA_FRAME_DISTANCE_M = .10
CARGO_OVERFLIGHT_GAP_M = .05
ENTRY_SAMPLE_M = .045
MAX_TRANSIT_LOWERING_M = .08
CENTRE_OFFSETS_M = (0., .05, -.05, .10, -.10)
HORIZONTAL_CARGO_CLEARANCE_M = .03
P2_FORWARD_CENTRE_STEP_M = .01


def xyz(pose):
    return tuple(getattr(pose.position, key) for key in ('x', 'y', 'z'))


def extent(pose, size, inverse=None):
    matrix = rotation_matrix(tuple(getattr(pose.orientation, k) for k in ('x', 'y', 'z', 'w')))
    if inverse is not None:
        matrix = tuple(tuple(sum(inverse[i][k]*matrix[k][j] for k in range(3))
                             for j in range(3)) for i in range(3))
    return tuple(sum(abs(row[j])*size[j]/2 for j in range(3)) for row in matrix)


def crosses_footprint(start, end, cargo, cargo_size, payload_extent, margin):
    """Conservative segment/AABB Minkowski test in world XY, including tilt."""
    half = extent(cargo, cargo_size)
    lower, upper = 0., 1.
    for axis in (0, 1):
        center = xyz(cargo)[axis]
        radius = half[axis]+payload_extent[axis]+margin
        delta = end[axis]-start[axis]
        if abs(delta) < 1e-12:
            if not center-radius <= start[axis] <= center+radius:
                return False
        else:
            a, b = sorted(((center-radius-start[axis])/delta,
                           (center+radius-start[axis])/delta))
            lower, upper = max(lower, a), min(upper, b)
            if lower > upper:
                return False
    return True


class CenteredDoorwayEntry:
    strategy = 'p2_centre_enter_then_lane_v3'

    def __init__(self, geometry, carry_config=None):
        self.geometry, self.carry_config = geometry, carry_config

    def build(self, target, size, container_pose, prior_targets=(), start_box_pose=None):
        if len(size) != 3 or any(not math.isfinite(v) or v <= 0 for v in size):
            raise ValueError('DOORWAY_ENTRY: positive finite payload dimensions required')
        origin = xyz(container_pose)
        rotation = rotation_matrix(tuple(getattr(container_pose.orientation, k)
                                         for k in ('x', 'y', 'z', 'w')))
        inverse = tuple(zip(*rotation))
        def local(pose):
            return rotate(inverse, tuple(v-o for v, o in zip(xyz(pose), origin)))
        def world(point):
            offset = rotate(rotation, point)
            return tuple(o+v for o, v in zip(origin, offset))

        door_x, y0, y1, z0, z1 = self.geometry.bounds
        margin = self.geometry.clearance + EXTRA_FRAME_DISTANCE_M
        # Replace the old side-biased route, including its descending suffix.
        # A suffix that drops while still between columns can hit earlier cargo.
        lane = replace(target.box_pose, position=Point(
            target.box_pose.position.x, target.box_pose.position.y,
            target.box_pose.position.z+target.clearance_m))
        if local(lane)[0]-extent(lane, size, inverse)[0] < door_x+margin:
            raise ValueError('DOORWAY_ENTRY: slot approach is not fully-inside')
        local_extent = extent(lane, size, inverse)
        if y1-y0 < 2*(local_extent[1]+margin):
            raise ValueError('DOORWAY_ENTRY: payload too wide for centred crossing')
        nominal_y = (y0+y1)/2
        inside_x = door_x+margin+local_extent[0]
        # Circumscribed radius protects the entire box during outside rotation,
        # including the measured pickup heading before it matches the route.
        radius = math.sqrt(sum(v*v for v in size))/2
        outside_x = -self.geometry.outer_size[0]/2-margin-radius
        if start_box_pose is not None:
            # The taught P2 has already reached the door. Reusing the old
            # nominal outside point made pre_place reverse toward the robot.
            # Keep the measured box centre's longitudinal coordinate while
            # centring laterally, then move monotonically into the container.
            # Advance 1cm during the lateral/height alignment. With the
            # measured container pitch, holding 3-D local X exactly fixed
            # would look like a small outward move in world XY.
            outside_x = local(start_box_pose)[0] + P2_FORWARD_CENTRE_STEP_M
            if not outside_x < door_x:
                raise ValueError('DOORWAY_ENTRY: P2 box centre must be outside door')
        pedestal_bottom = 0.
        config = self.carry_config
        if config is not None:
            pedestal_bottom = (config.pedestal.position.z + extent(config.pedestal, config.size)[2]
                               + config.clearance + .005)
        cargo = [p for p in prior_targets if p is not None and p.reference_box_size_m]
        tops = [p.box_pose.position.z + extent(p.box_pose, p.reference_box_size_m)[2] for p in cargo]
        baseline_bottom = max([pedestal_bottom, *(top+CARGO_OVERFLIGHT_GAP_M for top in tops)])
        payload_extent = extent(lane, size)
        if rotation[2][2] < .9:
            raise ValueError('DOORWAY_ENTRY: unsupported container tilt')
        def local_height(bottom, y):
            minimum_z = max(lane.position.z, bottom+payload_extent[2])
            return max(local(lane)[2], *[
                (minimum_z-origin[2]-rotation[2][0]*x-rotation[2][1]*y)/rotation[2][2]
                for x in (outside_x, inside_x)])

        baseline_z = local_height(baseline_bottom, nominal_y)
        # Search only the middle 20cm of the aperture. Avoiding a tall adjacent
        # stack by a few cm can lower the arm without flying over that stack.
        # The existing all-cargo baseline limits this update to 8cm lowering.
        tilt_slack = max(abs(rotation[0][2]), abs(rotation[1][2]))*MAX_TRANSIT_LOWERING_M/rotation[2][2]
        choices = []
        for offset in CENTRE_OFFSETS_M:
            y = nominal_y+offset
            if not y0+margin+local_extent[1] <= y <= y1-margin-local_extent[1]:
                continue
            points = ([xyz(start_box_pose)] if start_box_pose is not None else []) + [
                world((outside_x,y,baseline_z)), world((inside_x,y,baseline_z)), xyz(lane)]
            bottom = pedestal_bottom
            relevant = []
            for previous, top in zip(cargo, tops):
                if any(crosses_footprint(a,b,previous.box_pose,previous.reference_box_size_m,
                                        payload_extent,HORIZONTAL_CARGO_CLEARANCE_M+tilt_slack)
                       for a,b in zip(points,points[1:])):
                    bottom = max(bottom,top+CARGO_OVERFLIGHT_GAP_M)
                    relevant.append(prior_targets.index(previous)+1)
            bottom = max(bottom,baseline_bottom-MAX_TRANSIT_LOWERING_M)
            height = local_height(bottom,y)
            if z0+margin+local_extent[2] <= height <= z1-margin-local_extent[2]:
                # Compare required world height, not sub-mm changes caused by
                # container roll: keep the exact centre if it is equally low.
                required_world_z = max(lane.position.z,bottom+payload_extent[2])
                choices.append((required_world_z,abs(offset),offset,height,bottom,relevant))
        if not choices:
            raise ValueError('DOORWAY_ENTRY: insufficient vertical doorway clearance')
        _, _, center_offset, local_z, bottom, relevant = min(choices)
        y = nominal_y+center_offset
        outside = Pose(Point(*world((outside_x,y,local_z))), lane.orientation)
        inside = Pose(Point(*world((inside_x,y,local_z))), lane.orientation)
        high_lane = replace(lane, position=Point(lane.position.x, lane.position.y,
                                                max(lane.position.z, inside.position.z)))
        # Explicit dense waypoints preserve the reviewed corner at each phase.
        route = [start_box_pose, outside] if start_box_pose is not None else [outside]
        for goal in [inside, high_lane, lane]:
            pieces = connect_place_from_pose(route[-1], [MotionSegment(
                name='doorway_entry', type='cartesian', target_pose=goal)],
                ENTRY_SAMPLE_M, math.radians(5))
            route.extend(piece.target_pose for piece in pieces)
        return replace(target, via=tuple(route)), dict(
            strategy=self.strategy,
            start_source='measured_p2' if start_box_pose is not None else 'nominal_outside',
            centre_local_y_m=y, frame_clearance_m=margin,
            nominal_centre_local_y_m=nominal_y, centre_offset_m=center_offset,
            lowered_from_all_cargo_baseline_m=(baseline_z-local_z)*rotation[2][2],
            height_relevant_prior_boxes=relevant,
            outside_box_pose=asdict(outside), inside_box_pose=asdict(inside),
            replaced_via_poses=len(target.via), via_count=len(route),
            minimum_transit_box_bottom_world_m=bottom)

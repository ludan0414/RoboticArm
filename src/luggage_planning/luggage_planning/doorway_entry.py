"""Select an entry aisle using the target and already loaded cargo.

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
FIRST_BOX_INSIDE_DROP_M = .12
THIRD_BOX_INSIDE_DROP_M = .08
THIRD_BOX_PRE_PLACE_CLEARANCE_M = .075
FOURTH_BOX_DOOR_LOWER_M = .09


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
    strategy = 'p2_occupied_side_aware_entry_v6'

    def __init__(self, geometry, carry_config=None):
        self.geometry, self.carry_config = geometry, carry_config

    def build(self, target, size, container_pose, prior_targets=(), start_box_pose=None):
        if len(size) != 3 or any(not math.isfinite(v) or v <= 0 for v in size):
            raise ValueError('DOORWAY_ENTRY: positive finite payload dimensions required')
        # Third-box pre-place at 15 cm clearance was beyond the arm's reach
        # with J3=-16°. The same site target is IK-feasible 8 cm lower; keep
        # a 7.5 cm release approach above the configured support.
        if len(prior_targets) == 2 and start_box_pose is not None:
            target = replace(target, clearance_m=min(
                target.clearance_m, THIRD_BOX_PRE_PLACE_CLEARANCE_M))
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
        # When loading the opposite stack, enter toward its own aisle rather
        # than first drifting back toward the occupied stack. Door bounds
        # still constrain the whole oriented payload, not just its centre.
        target_y = local(lane)[1]
        target_aisle = start_box_pose is not None and any(
            (local(p.box_pose)[1]-nominal_y)*(target_y-nominal_y) < 0
            for p in cargo)
        entry_y = min(max(target_y,y0+margin+local_extent[1]+1e-6),
                      y1-margin-local_extent[1]-1e-6)
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
        # Keep the existing centre search for the original-side approach.
        # An opposite-side aisle uses only cargo actually crossed by its path;
        # an unrelated tall stack must not force a high, unreachable transit.
        tilt_slack = max(abs(rotation[0][2]), abs(rotation[1][2]))*MAX_TRANSIT_LOWERING_M/rotation[2][2]
        choices = []
        offsets = (entry_y-nominal_y,) if target_aisle else CENTRE_OFFSETS_M
        for offset in offsets:
            # Fourth box enters on the left/central side of the opening.
            # The formerly selected +10 cm offset grazed link5 on the top
            # guard while aiming toward the right-side stack.
            if not target_aisle and len(cargo) == 3 and start_box_pose is not None and offset != 0.:
                continue
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
            if not target_aisle:
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
        # At the central/left entry, cargo overflight demands a higher route.
        # Cross the doorway 9 cm below that overflight height; lift only once fully inside.
        door_lower_m = FOURTH_BOX_DOOR_LOWER_M if not target_aisle and len(cargo) == 3 and start_box_pose is not None else 0.
        y = nominal_y+center_offset
        doorway_local_z = local_z-door_lower_m/rotation[2][2]
        outside = Pose(Point(*world((outside_x,y,doorway_local_z))), lane.orientation)
        inside = Pose(Point(*world((inside_x,y,doorway_local_z))), lane.orientation)
        raised_inside = replace(inside, position=Point(
            inside.position.x, inside.position.y,
            inside.position.z+door_lower_m))
        # Boxes one and three reached J3 near full extension at the high
        # far-lane corner. Lower only after the entire payload clears the
        # doorway, while retaining configured earlier-cargo clearance.
        inside_drop_m = (
            FIRST_BOX_INSIDE_DROP_M if not cargo else
            THIRD_BOX_INSIDE_DROP_M if len(cargo) == 2 else 0.
        ) if start_box_pose is not None else 0.
        early_drop = inside_drop_m > 0.
        cargo_floor = max((tops[index-1] for index in relevant), default=float('-inf'))
        transit_z = (max(lane.position.z, inside.position.z-inside_drop_m,
                         cargo_floor+CARGO_OVERFLIGHT_GAP_M+payload_extent[2]) if early_drop
                     else raised_inside.position.z if door_lower_m else inside.position.z)
        lowered_inside = replace(inside, position=Point(
            inside.position.x, inside.position.y, transit_z))
        high_lane = replace(lane, position=Point(lane.position.x, lane.position.y,
                                                max(lane.position.z, transit_z)))
        # Explicit dense waypoints preserve the reviewed corner at each phase.
        route = [start_box_pose, outside] if start_box_pose is not None else [outside]
        goals = ([inside, raised_inside, high_lane, lane] if door_lower_m else
                 [inside, lowered_inside, high_lane, lane] if early_drop else
                 [inside, high_lane, lane])
        for goal in goals:
            pieces = connect_place_from_pose(route[-1], [MotionSegment(
                name='doorway_entry', type='cartesian', target_pose=goal)],
                ENTRY_SAMPLE_M, math.radians(5))
            route.extend(piece.target_pose for piece in pieces)
        return replace(target, via=tuple(route)), dict(
            strategy=self.strategy,
            entry_mode='target_aisle' if target_aisle else 'centre',
            start_source='measured_p2' if start_box_pose is not None else 'nominal_outside',
            centre_local_y_m=y, frame_clearance_m=margin,
            nominal_centre_local_y_m=nominal_y, centre_offset_m=center_offset,
            lowered_from_all_cargo_baseline_m=(baseline_z-local_z)*rotation[2][2],
            height_relevant_prior_boxes=relevant,
            outside_box_pose=asdict(outside), inside_box_pose=asdict(inside),
            inside_drop_m=(inside.position.z-transit_z if early_drop else 0.),
            door_lower_m=door_lower_m,
            replaced_via_poses=len(target.via), via_count=len(route),
            minimum_transit_box_bottom_world_m=bottom)

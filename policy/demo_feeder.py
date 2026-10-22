"""One fixed box-centre feeder outside the doorway for the prepared demo."""
import numpy as np
from online_packing_real import path_clear, route_to

# Door-floor coordinates: x increases into the container. This centre stays
# fixed for every box; top-contact height still depends on the box height.
FEEDER_CENTER_M = (-.3, .5, .55)


def feeder_route(box, placed, hull, workspace, source_dims, feeder_center=FEEDER_CENTER_M):
    dims = box.dims
    start = tuple(feeder_center[k] - dims[k] / 2 for k in range(3))
    if start[0] + dims[0] > -.05 + 1e-6:
        return None  # Keep the whole payload outside while aligning it.
    yaw = workspace.box_yaw(source_dims, dims)
    for height in sorted({.4, .5, .6, round(hull.height - dims[2], 3)}):
        incoming = route_to(box, placed, hull, min_entry_z=height)
        if incoming is None:
            continue
        entry = incoming[0]
        # Long cases extend back into link1 when the original fully-outside
        # rail sweeps across the pedestal. Enter the open doorway corridor
        # before lateral alignment; path_clear checks its aperture and hull.
        # Keep the legacy rail for the original <=0.5m fixtures.
        rail_x = max(entry[0], -.40) if max(source_dims[:2]) > .5 else entry[0]
        # Move away from the robot to the insertion rail before sweeping Y.
        # Sweeping beside the pedestal at the feeder's X can hit link1/link2
        # even though both endpoints and the cargo corridor are clear.
        rail_entry = (rail_x, entry[1], entry[2])
        connectors = [start, (rail_x, start[1], start[2]),
                      (rail_x, start[1], entry[2]), rail_entry]
        route = []
        for point in connectors + incoming[1:]:
            if not route or not np.allclose(point, route[-1], atol=1e-8, rtol=0):
                route.append(tuple(point))
        if not path_clear(dims, route, placed, hull):
            continue
        if (workspace.route_is_feasible(route, dims, yaw, hull)
                and workspace.route_is_feasible(list(reversed(route)), dims, yaw, hull)):
            return route
    return None

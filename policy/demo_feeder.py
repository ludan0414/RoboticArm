"""One fixed box-centre feeder outside the doorway for the prepared demo."""
import numpy as np
from online_packing_real import path_clear, route_to

# Door-floor coordinates: x increases into the container. This centre stays
# fixed for every box; top-contact height still depends on the box height.
FEEDER_CENTER_M = (-.3, .5, .55)


def feeder_route(box, placed, hull, workspace, source_dims):
    dims = box.dims
    start = tuple(FEEDER_CENTER_M[k] - dims[k] / 2 for k in range(3))
    if start[0] + dims[0] > -.05 + 1e-6:
        return None  # Keep the whole payload outside while aligning it.
    yaw = workspace.box_yaw(source_dims, dims)
    for height in sorted({.4, .5, .6, round(hull.height - dims[2], 3)}):
        incoming = route_to(box, placed, hull, min_entry_z=height)
        if incoming is None:
            continue
        entry = incoming[0]
        # Move away from the robot to the insertion rail before sweeping Y.
        # Sweeping beside the pedestal at the feeder's X can hit link1/link2
        # even though both endpoints and the cargo corridor are clear.
        connectors = [start, (entry[0], start[1], start[2]),
                      (entry[0], start[1], entry[2]), entry]
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

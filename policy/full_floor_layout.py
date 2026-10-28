"""Generate structured, full-flat-floor candidates for a 32-case Newton demo.

Candidate geometry is only a layout hypothesis. IK, payload routing and full
trajectory compilation must succeed before a replay bundle is marked ready.
"""
from dataclasses import dataclass
import math

@dataclass(frozen=True)
class Cell:
    x: float
    y: float
    top: float
    yaw: float


def full_floor_cells(hull, layers=7, dims=(.55, .35, .20)):
    """Interlock rows by orientation, flush at the cargo back and flat side.

    Two long-width rows contain four 35 cm deep boxes each, followed by a
    narrow 35 cm row with two 55 cm deep boxes.  The remaining strip cannot
    fit another box in this orientation. A cell isn't a reachability proof.
    """
    length, width, height = map(float, dims)
    if length < width or layers < 1:
        raise ValueError("expected length >= width and at least one layer")
    cells = []
    rows = [(0., length, width, -math.pi / 2),
            (length, length, width, -math.pi / 2),
            (2 * length, width, length, 0.)]
    for layer in range(layers):
        top = round((layer + 1) * height, 6)
        if top > hull.height + 1e-7:
            break
        for y0, row_width, depth, yaw in rows:
            if y0 + row_width > hull.flat_y + 1e-7:
                continue
            count = int((hull.length + 1e-8) // depth)
            for i in range(count):
                x = hull.length - (i + .5) * depth
                cells.append(Cell(round(x, 6), round(y0 + row_width / 2, 6), top, yaw))
    return cells

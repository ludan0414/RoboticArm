import unittest
from full_floor_layout import full_floor_cells
from online_packing_demo import Box, collides, fully_supported
from online_packing_real import Hull

class FullFloorLayoutTest(unittest.TestCase):
    def test_candidates_are_supported_flat_and_nonoverlapping(self):
        hull = Hull(1.49, 1.97, 1.48, 1.535, .37, .057, 1.383, .067, 1.479, .53, -.745, -.985)
        cells = full_floor_cells(hull)
        self.assertEqual(len(cells), 70)
        placed = []
        for cell in cells:
            dims = (.55, .35, .20) if cell.yaw == 0 else (.35, .55, .20)
            box = Box('box', dims, (cell.x-dims[0]/2, cell.y-dims[1]/2, round(cell.top-.2, 6)))
            self.assertTrue(hull.fits(box))
            self.assertFalse(any(collides(box, other) for other in placed))
            self.assertTrue(hull.floor_supports(box) if box.pos[2] == 0 else fully_supported(box, placed))
            self.assertLessEqual(box.pos[1]+box.dims[1], hull.flat_y+1e-7)
            placed.append(box)
    def test_low_ceiling_excludes_top_layer(self):
        hull = Hull(1.49, 1.97, .6, 1.535, .37, .057, 1.383, .067, .6, .53, -.745, -.985)
        self.assertEqual(len(full_floor_cells(hull)), 30)

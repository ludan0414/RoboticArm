"""A box's pre-spawn positioning must never become its post-place route."""
import unittest
import numpy as np
from export_hardware_loading_profile import post_place_exit_indices


class ExitRouteTests(unittest.TestCase):
    def test_only_returns_after_the_last_retreat_are_exported(self):
        trace = dict(stage=np.array(["RETURN", "RETURN", "SPAWN", "PLACE", "RETREAT", "RETREAT", "RETURN", "RETURN", "RETURN"]),
                     box_id=np.array(["B2"]*8 + ["B3"]))
        np.testing.assert_array_equal(post_place_exit_indices(trace, "B2"), [5, 6, 7])

    def test_early_return_cannot_substitute_for_missing_post_place_return(self):
        trace = dict(stage=np.array(["RETURN", "SPAWN", "RETREAT"]), box_id=np.array(["B2"]*3))
        with self.assertRaisesRegex(ValueError, "post-place return"):
            post_place_exit_indices(trace, "B2")


if __name__ == "__main__":
    unittest.main()

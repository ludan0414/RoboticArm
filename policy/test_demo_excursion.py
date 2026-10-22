import unittest
from pathlib import Path
import numpy as np
from demo_excursion import ExcursionDisplay


class ExcursionDisplayTests(unittest.TestCase):
    def test_reviewed_recording_preserves_real_exceedances(self):
        folder=Path(__file__).parent/'demo_data/site55_right160_one'
        with np.load(folder/'trajectory.npz') as data:
            display=ExcursionDisplay(data)
        self.assertEqual(display.summary,dict(segments=188,over_3=4,over_4=2,maximum_rad_per_m=4.641))
        carry=[s for s in display.segments if s['stage']=='CARRY']
        self.assertEqual(max(s['metric']['rad_per_m'] for s in carry),4.417)
        peak=max(display.segments,key=lambda s:s['metric']['rad_per_m'])
        self.assertIs(display.by_frame[peak['end_frame']],peak)

    def test_orientation_only_motion_reports_ratio_without_translation(self):
        poses=np.asarray([[0,0,0,0,0,0,1],[0,0,0,0,0,2**-.5,2**-.5]],dtype=float)
        data=dict(stage=np.asarray(['CARRY']*2),box_id=np.asarray(['B1']*2),
                  box_pose=poses,tool_pose=poses,joint_q=np.asarray([[0]*6,[.4,0,0,0,0,0]]))
        display=ExcursionDisplay(data)
        self.assertEqual(display.summary['maximum_rad_per_m'],8)
        self.assertEqual(display.segments[0]['metric']['path_len_m'],0)
        self.assertEqual(display.segments[0]['metric']['excursion_rad'],.4)

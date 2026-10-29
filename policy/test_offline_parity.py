"""Compare the real console consumer against the checked Newton recording."""
import json,os,unittest
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
from scipy.spatial.transform import Rotation
from luggage_planning.hardware_loading_cycle import parse_profile
from luggage_planning.pose import Pose,Point,Quaternion
from test_hardware_loading_ports import load_console


class OfflineParityTests(unittest.TestCase):
    def test_real_console_uses_the_same_nominal_tool_waypoints_as_newton(self):
        folder=Path(os.environ.get('OFFLINE_SAMPLE_BUNDLE',
            str(Path(__file__).parent/'demo_data/offline_sample55')))
        profile=json.loads((folder/'hardware.json').read_text(encoding='utf-8'))
        _,cycles=parse_profile(profile)
        module=load_console()
        with np.load(folder/'trajectory.npz') as data:
            for row,(target,_) in zip(profile['cycles'],cycles):
                selected=np.flatnonzero((data['box_id']==row['demo_box_id'])&(data['stage']=='CARRY'))
                count=len(target.via)+1
                actual=data['tool_pose'][selected[-count-1]]
                driver=module.CommissioningDriver();driver._args=NS(place_target_data=target)
                driver._measured_grasp=((0,0,row['expected_demo_input_dims_m'][2]/2),Quaternion(1,0,0,0))
                driver.lookup_pose=lambda *a,**kw:Pose(Point(*actual[:3]),Quaternion(*actual[3:]))
                def forbidden(*a,**kw):raise AssertionError('offline route was regenerated')
                driver.doorway_guard=NS(align_target=forbidden);driver.doorway_entry=NS(build=forbidden)
                driver.get_logger=lambda:NS(info=lambda *a:None)
                segments=driver.prepare_place_segments([],NS(frame='world',yaw_valid=True,
                    box_size=row['expected_demo_input_dims_m']),NS(frame='world'))
                carry=next(segment for segment in segments if segment.name=='pre_place')
                poses=[*carry.waypoints,carry.target_pose]
                self.assertEqual(len(poses),count)
                for pose,expected in zip(poses,data['tool_pose'][selected[-count:]]):
                    self.assertLess(np.linalg.norm(np.array([pose.position.x,pose.position.y,pose.position.z])-expected[:3]),2e-5)
                    actual_rotation=Rotation.from_quat([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w])
                    self.assertLess((actual_rotation.inv()*Rotation.from_quat(expected[3:])).magnitude(),2e-5)


if __name__=='__main__':unittest.main()

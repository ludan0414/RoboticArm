from types import SimpleNamespace as NS
from unittest.mock import patch
import unittest
from luggage_planning.pose import Pose,Point,Quaternion
from luggage_planning.policy_place import PlaceTarget
from test_hardware_loading_ports import load_console


class FrozenRouteTests(unittest.TestCase):
    def test_console_does_not_regenerate_frozen_box_waypoints(self):
        module=load_console();driver=module.CommissioningDriver()
        via=Pose(Point(-.6,.2,1.2),Quaternion())
        target=PlaceTarget(Pose(Point(-1.,-.2,.7)),.075,(via,),(.55,.35,.2),'offline_frozen_v1')
        driver._args=NS(place_target_data=target)
        driver._measured_grasp=((0,0,.1),Quaternion(1,0,0,0))
        driver.lookup_pose=lambda *a,**kw:Pose(Point(-.6,.2,1.3),Quaternion(1,0,0,0))
        def forbidden(*a,**kw):raise AssertionError('frozen route was regenerated')
        driver.doorway_guard=NS(align_target=forbidden);driver.doorway_entry=NS(build=forbidden)
        driver.get_logger=lambda:NS(info=lambda *a:None)
        with patch.object(module,'nearest_cuboid_heading',side_effect=forbidden):
            segments=driver.prepare_place_segments([],NS(frame='world',yaw_valid=True,box_size=(.55,.35,.2)),NS(frame='world'))
        carry=next(s for s in segments if s.name=='pre_place')
        poses=[*carry.waypoints,carry.target_pose]
        self.assertTrue(any(abs(p.position.x-via.position.x)<1e-9 and
                            abs(p.position.y-via.position.y)<1e-9 for p in poses))
        self.assertEqual(driver._doorway_entry_strategy,'offline_frozen_v1')
        self.assertTrue(all(s.type=='offline_frozen_cartesian' for s in segments))
        self.assertEqual(driver._settled_box_pose,target.box_pose)


if __name__=='__main__':unittest.main()

from dataclasses import asdict
import json
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace as NS
import unittest

from luggage_planning.recovery_checkpoint import RecoveryCheckpoint
from luggage_planning.hardware_recovery import HardwareRecovery
from luggage_planning.hardware_loading_cycle import LoadingCycle
from luggage_planning.pose import Pose, Point, Quaternion, MotionSegment
from luggage_planning.policy_place import PlaceTarget
from luggage_planning.pick_return_path import PickReturnPath


class RecoveryTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name)/'loading.checkpoint.json'
        self.store = RecoveryCheckpoint(self.path, 'profile-sha')
        self.calls = []
        self.driver = NS(_scene_box=[[0,0,1],[0,0,0,1],[.5,.3,.2]],
            _measured_grasp=([0,0,.1],Quaternion()), _pick_return_path=PickReturnPath(),
            _effective_place_target=PlaceTarget(Pose(Point(1,2,3.005)), .15, (), (.5,.3,.2)),
            _settled_box_pose=Pose(Point(1,2,3)), placed_proxy_id='expected_session_1',
            carry_entry_config=None, release_touch_links=('tool',),
            _args=NS(release_low_hold_sec=0.,release_deadline_sec=2.),
            ensure_doorway_clearance=lambda:(True,'ok'),
            lookup_pose=lambda *args,**kwargs:Pose(Point(0,1,1)),
            validate_place_pose=lambda:(True,'aligned'), report_stage=lambda stage:self.calls.append(stage))
        self.driver._pick_return_path.poses = [Pose(Point(0,1,1)),Pose(Point(1,2,3.15))]
        self.scene = NS(add_collision_box=lambda *args:(True,'ok'),
                        set_acm_pairs=lambda *args:(True,'ok'))
        self.driver._scene_client = lambda:self.scene
        self.ports = NS(driver=self.driver, cycles=[(None,()),(PlaceTarget(Pose(),.15),())],
            record=lambda event:self.calls.append(event), recover_controller=lambda:self.calls.append('recover'))
        owner = self
        class Native:
            def __init__(self,*args): self.clock=0.
            def execute_segment(self,segment,*args):
                owner.calls.append(segment.name)
                return NS(ok=True,message='complete')
            def close(self): pass
            def graph_health(self): return True,'ok'
            def di0(self): return owner.driver._recovery_io['DI0']
            def log_event(self,event): pass
            def now(self): self.clock+=.1;return self.clock
            def sleep(self,dt): pass
            def validate_place_pose(self): return owner.driver.validate_place_pose()
            def place_checkpoint(self,phase,index): owner.recovery.progress(phase,index)
            def vacuum_release(self,*args):
                owner.calls.append('release')
                owner.io(False)
                return True,'released'
            def scene_remove(self):
                owner.calls.append('detach')
                owner.driver._scene_box=None
                return True,'detached'
        self.native = Native
        self.recovery = HardwareRecovery(self.ports,self.store,Native)
        self.driver.recovery = self.recovery
        self.io(True)
        self.store.begin_pick(1)
        templates = [MotionSegment(name=name,type='cartesian',target_pose=Pose(Point(1,2,3)))
                     for name in ('pre_place','place_descend','place_retreat')]
        self.recovery.begin_place(templates)
        self.driver.prepare_place_segments = lambda *args:self.recovery.begin_place(templates)
        def transfer_after_pick(native, start=0):
            from luggage_planning.manual_carry_transfer import transfer_segments, execute_transfer
            segments = transfer_segments()
            if start == 0:
                self.recovery.begin_transfer(segments)
            return execute_transfer(native, segments, start, self.recovery.transfer_progress)
        self.driver.transfer_after_pick = transfer_after_pick

    def io(self,held):
        self.driver._recovery_io=dict(DI0=int(held),DO0=int(held),DO1=0)
        self.driver._recovery_io_received_at=time.monotonic()

    def test_checkpoint_reload_is_passive_and_rejects_wrong_profile(self):
        loaded=RecoveryCheckpoint(self.path,'profile-sha')
        self.assertEqual(loaded.snapshot()['phase'],'CARRYING')
        self.assertEqual(self.calls,[])
        with self.assertRaises(ValueError):RecoveryCheckpoint(self.path,'different')
        with self.assertRaises(RuntimeError):loaded.begin_pick(0)

    def test_resume_failed_descent_does_not_repeat_pick_or_completed_carry(self):
        self.recovery.progress('CARRYING',1)
        self.calls.clear()
        self.recovery.resume_place(1)
        self.assertNotIn('pre_place',self.calls)
        self.assertEqual([c for c in self.calls if isinstance(c,str)],
                         ['recover','place_descend','release','detach','place_retreat'])
        self.assertEqual(self.store.snapshot()['phase'],'RETURN_PENDING')

    def test_held_home_keeps_suction_and_policy_index_then_resume_replays_route(self):
        self.recovery.carry_home()
        self.assertIn('return_pick_stop_0',self.calls)
        self.assertIn('held_home',self.calls)
        self.assertNotIn('release',self.calls)
        self.assertEqual(self.store.snapshot()['phase'],'HELD_HOME')
        self.assertEqual(self.store.snapshot()['box_index'],1)
        self.assertEqual(self.store.snapshot()['segments'],[])
        self.assertEqual(self.driver._recovery_io['DO0'],1)
        self.calls.clear()
        self.recovery.resume_place(1)
        self.assertLess(self.calls.index('carry_p1'), self.calls.index('carry_p2'))
        self.assertLess(self.calls.index('carry_p2'), self.calls.index('pre_place'))
        self.assertIn('pre_place',self.calls)
        self.assertEqual(self.calls.count('release'),1)

    def test_released_checkpoint_only_retreats_and_never_releases_again(self):
        self.recovery.progress('PLACED',2)
        self.io(False)
        self.recovery._reconcile_released_scene=lambda data:self.calls.append('reconcile')
        self.recovery.resume_place(1)
        self.assertEqual([c for c in self.calls if isinstance(c,str)],['recover','reconcile','place_retreat'])
        self.assertNotIn('release',self.calls)

    def test_interrupted_transfer_resumes_p2_then_plans_from_arrived_pose(self):
        from luggage_planning.manual_carry_transfer import transfer_segments
        self.recovery.begin_transfer(transfer_segments())
        self.recovery.transfer_progress(1)
        self.calls.clear()
        self.recovery.resume_place(1)
        self.assertNotIn('carry_p1', self.calls)
        self.assertLess(self.calls.index('carry_p2'), self.calls.index('pre_place'))
        self.assertEqual(self.calls.count('release'), 1)

    def test_missing_suction_or_stale_feedback_cannot_move_held_box(self):
        for stale in (False,True):
            self.io(False if not stale else True)
            if stale:self.driver._recovery_io_received_at=time.monotonic()-3
            with self.assertRaises(RuntimeError):self.recovery.carry_home()
        self.assertFalse(any(name in self.calls for name in ('held_home','return_pick_stop_0','release')))

    def test_interrupted_return_must_finish_before_policy_resumption(self):
        self.store.update(phase='CARRY_RETURN')
        with self.assertRaisesRegex(RuntimeError,'finish carry-home'):self.recovery.resume_place(1)
        self.assertNotIn('release',self.calls)

    def test_release_ambiguity_with_moved_tool_stays_blocked(self):
        self.recovery.progress('RELEASE_REQUESTED',2)
        self.io(False)
        self.driver.validate_place_pose=lambda:(False,'tool moved')
        with self.assertRaisesRegex(RuntimeError,'ambiguous release'):self.recovery.resume_place(1)
        self.assertNotIn('release',self.calls)
        self.assertNotIn('place_retreat',self.calls)

    def test_checkpoint_write_failure_cannot_replace_last_durable_progress(self):
        from unittest.mock import patch
        before=self.path.read_bytes()
        with patch('luggage_planning.recovery_checkpoint.os.replace',side_effect=OSError('disk full')):
            with self.assertRaises(OSError):self.store.update(phase='RELEASE_CONFIRMED')
        self.assertEqual(self.path.read_bytes(),before)
        self.assertEqual(self.store.snapshot()['phase'],'CARRYING')

    def test_cycle_carry_home_and_resume_advance_exactly_one_box(self):
        calls=[]
        ports=NS(record=lambda event:None,carry_home=lambda:calls.append('home'),
            resume_place=lambda index:calls.append(('resume',index)),
            return_observe=lambda index:calls.append(('return',index)))
        cycle=LoadingCycle(ports,3)
        cycle.restore_progress(self.store.snapshot())
        for command in ('carry_home','resume'):
            self.assertTrue(cycle.submit(command,cycle.request_id,1,True))
            cycle.worker.join(2)
            self.assertFalse(cycle.worker.is_alive())
            if command=='carry_home':
                self.assertEqual(cycle.index,1)
                self.assertEqual(cycle.stage,'HELD_AT_HOME')
        self.assertEqual(cycle.index,2)
        self.assertEqual(calls,['home',('resume',1),('return',1)])

    def test_completed_policy_cursor_survives_restart_without_effects(self):
        self.store.update(phase='DONE',box_index=1,start_box=1)
        cycle=LoadingCycle(NS(record=lambda event:None),3)
        cycle.restore_progress(self.store.snapshot())
        self.assertEqual(cycle.snapshot()['next_box'],3)
        self.assertEqual(cycle.state,'WAIT_START')

    def test_aborted_box_cursor_survives_restart_without_repeating_previous_boxes(self):
        self.store.update(phase='ABORTED',box_index=1,start_box=1)
        cycle=LoadingCycle(NS(record=lambda event:None),3)
        cycle.restore_progress(self.store.snapshot())
        self.assertEqual(cycle.snapshot()['next_box'],2)
        self.assertEqual(cycle.state,'WAIT_START')

    def test_held_unconfirmed_pick_never_repeats_pick(self):
        self.store.update(phase='PICKING')
        self.ports.pick_place=lambda index:self.fail('must not repick a physically held box')
        with self.assertRaisesRegex(RuntimeError,'released-box recovery'):
            self.recovery.resume_place(1)
        self.assertNotIn('recover',self.calls)
    def test_legacy_frozen_carry_cannot_bypass_new_centred_entry(self):
        self.driver.doorway_entry=NS(strategy='outside_centre_enter_then_lane_v1')
        with self.assertRaisesRegex(RuntimeError,'carry-home then resume'):
            self.recovery.resume_place(1)
        self.assertNotIn('pre_place',self.calls)
        self.assertNotIn('recover',self.calls)

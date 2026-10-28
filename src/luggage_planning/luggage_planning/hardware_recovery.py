"""Node-layer recovery adapter; uses existing ports, never a second CPS client.

Checkpoints authorize no startup effects. Every resume/held-home action needs
an operator command and fresh physical IO. Release ambiguity is reconciled
separately from held-box motion, so a completed placement is never repicked.
"""
from dataclasses import asdict
import time
from types import SimpleNamespace

from .commissioning_home import HOME_SEGMENT_TYPE
from .manual_carry_transfer import TRANSFER_TYPES, transfer_segments
from .pose import MotionSegment, Quaternion
from .pick_return_path import PickReturnPath
from .recovery_checkpoint import (pose_from_record, segment_from_record, target_from_record)
from .policy_place import run_place


class HardwareRecovery:
    def __init__(self, ports, store, ports_factory):
        self.ports, self.driver, self.store, self.ports_factory = ports, ports.driver, store, ports_factory
        self.offset = 0

    def snapshot(self):
        data = self.store.snapshot()
        return dict(available=self.store.pending, phase=data['phase'],
                    box=data['box_index']+1, next_segment=data['next_segment'],
                    checkpoint=str(self.store.path), error=data.get('error',''))

    def context(self):
        driver = self.driver
        previous = self.store.snapshot()['context']
        grasp = getattr(driver, '_measured_grasp', None)
        geometry = getattr(driver, '_scene_box', None)
        return dict(scene_box=geometry if geometry is not None else previous.get('scene_box'),
            grasp=dict(offset=list(grasp[0]), quaternion=asdict(grasp[1])) if grasp else previous.get('grasp'),
            return_poses=[asdict(p) for p in getattr(driver, '_pick_return_path', PickReturnPath()).poses],
            effective_target=asdict(driver._effective_place_target) if hasattr(driver, '_effective_place_target')
                else previous.get('effective_target'),
            settled_pose=asdict(driver._settled_box_pose) if hasattr(driver, '_settled_box_pose')
                else previous.get('settled_pose'),
            doorway_entry_strategy=getattr(driver, '_doorway_entry_strategy',
                                           previous.get('doorway_entry_strategy', '')),
            proxy_id=getattr(driver, 'placed_proxy_id', previous.get('proxy_id')))

    def sealed(self):
        self.store.update(phase='HELD_PICK', context=self.context())

    def begin_place(self, segments):
        self.offset = 0
        self.store.update(phase='CARRYING', segments=[asdict(s) for s in segments],
                          next_segment=0, context=self.context())

    def begin_transfer(self, segments):
        self.store.update(phase='HELD_TRANSFER', segments=[asdict(s) for s in segments],
                          next_segment=0, context=self.context())

    def transfer_progress(self, next_segment):
        self.store.update(phase='HELD_TRANSFER', next_segment=next_segment,
                          context=self.context())
        self.ports.record(dict(event='recovery_checkpoint', phase='HELD_TRANSFER',
                               next_segment=next_segment))

    def progress(self, phase, next_segment):
        self.store.update(phase=phase, next_segment=self.offset+next_segment, context=self.context())
        self.ports.record(dict(event='recovery_checkpoint', phase=phase,
                               next_segment=self.offset+next_segment))

    def note_placed(self, identity, pose, size):
        placed = [row for row in self.store.snapshot()['placed'] if row['id']!=identity]
        placed.append(dict(id=identity, pose=asdict(pose), size=list(size)))
        self.store.update(placed=placed)

    def _io(self):
        io = getattr(self.driver, '_recovery_io', None)
        stamp = getattr(self.driver, '_recovery_io_received_at', None)
        if not io or stamp is None or time.monotonic()-stamp > 2.:
            raise RuntimeError('recovery requires fresh physical vacuum DI0/DO0/DO1')
        return io

    def require_held(self):
        io = self._io()
        if (io.get('DI0'), io.get('DO0'), io.get('DO1')) != (1,1,0):
            raise RuntimeError('held-box recovery requires DI0=1, pump DO0=1, release DO1=0')
        ok, reason = self.driver.ensure_doorway_clearance()
        if not ok:
            raise RuntimeError(reason)

    def require_empty(self):
        io = self._io()
        if io.get('DI0') != 0 or io.get('DO0') != 0:
            raise RuntimeError('released-box recovery requires DI0=0 and pump DO0=0')

    def restore(self):
        data = self.store.snapshot()
        index, context = data['box_index'], data['context']
        if not 0 <= index < len(self.ports.cycles) or not context.get('scene_box') or not context.get('grasp'):
            raise RuntimeError('sealed grasp/geometry checkpoint missing; automatic recovery unavailable')
        driver = self.driver
        driver._args.place_target_data = self.ports.cycles[index][0]
        driver._prior_place_targets = tuple(target for target, _ in self.ports.cycles[:index])
        driver._configured_payload_size = tuple(context['scene_box'][2])
        driver._scene_box = context['scene_box']
        driver._measured_grasp = (context['grasp']['offset'], Quaternion(**context['grasp']['quaternion']))
        driver._pick_return_path = PickReturnPath()
        driver._pick_return_path.poses = [pose_from_record(p) for p in context['return_poses']]
        driver.placed_proxy_id = context['proxy_id']
        driver._doorway_entry_strategy = context.get('doorway_entry_strategy', '')
        if context.get('effective_target'):
            driver._effective_place_target = target_from_record(context['effective_target'])
        if context.get('settled_pose'):
            driver._settled_box_pose = pose_from_record(context['settled_pose'])
        driver._doorway_carrying = data['phase'] not in ('RELEASE_CONFIRMED','PLACED','RETURN_PENDING')
        if driver.carry_entry_config is not None:
            from .carry_entry import CarryEntry
            driver._carry_entry = CarryEntry(driver.carry_entry_config,
                SimpleNamespace(box_size=driver._configured_payload_size), None, driver._measured_grasp)
        # Recreate only this checkpoint's expected obstacles if MoveIt restarted.
        self.restore_placed()
        return data

    def restore_placed(self):
        scene = self.driver._scene_client()
        for row in self.store.snapshot()['placed']:
            pose = pose_from_record(row['pose'])
            ok, reason = scene.add_collision_box(row['id'], list(asdict(pose.position).values()),
                                                 list(asdict(pose.orientation).values()), row['size'])
            if not ok:
                raise RuntimeError('restore expected placement: '+reason)

    def _ensure_plan(self):
        data = self.store.snapshot()
        if not data['segments']:
            if not self.driver._pick_return_path.poses:
                self.driver._pick_return_path.poses = [self.driver.lookup_pose(
                    'world','suction_contact_frame',fresh=True)]
            detect = SimpleNamespace(frame='world', yaw_valid=True, box_size=self.driver._configured_payload_size)
            self.driver.prepare_place_segments([], detect, SimpleNamespace(frame='world'))
            data = self.store.snapshot()
        return data

    def carry_home(self):
        if not self.store.pending:
            raise RuntimeError('no interrupted held-box checkpoint')
        data = self.restore()
        if data['phase'] not in ('HELD_PICK','HELD_TRANSFER','CARRYING','HELD_HOME','CARRY_RETURN'):
            raise RuntimeError('release already started; reconcile placement instead of carrying home')
        self.require_held()
        self.ports.recover_controller()
        self.require_held()
        if data['phase'] != 'HELD_TRANSFER':
            self._ensure_plan()
        self.store.update(phase='CARRY_RETURN', context=self.context())
        native = self.ports_factory(self.driver, self.driver)
        try:
            if self.driver._pick_return_path.poses:
                segment = self.driver._pick_return_path.continuous_segment()
                self.driver.report_stage('HELD_REVERSE_ROUTE')
                self.require_held()
                result = native.execute_segment(segment, 'held_return')
                if not result.ok:
                    raise RuntimeError('held reverse return: '+result.message)
            self.require_held()
            self.driver.report_stage('HELD_HOME')
            result = native.execute_segment(MotionSegment(name='held_home', type=HOME_SEGMENT_TYPE), 'held_return')
            if not result.ok:
                raise RuntimeError('held home: '+result.message)
            self.require_held()
            home = self.driver.lookup_pose('world','suction_contact_frame',fresh=True)
            self.driver._pick_return_path.poses = [home]
            # Rebuild from actual home on resume, so upgraded entry rules and
            # the changed connector start are both applied to the new route.
            self.store.update(phase='HELD_HOME', next_segment=0, segments=[],
                              context=self.context(), error='')
        finally:
            native.close()

    def resume_place(self, index):
        if not self.store.pending or self.store.snapshot()['box_index'] != index:
            raise RuntimeError('no matching interrupted box')
        if self.store.snapshot()['phase']=='PICKING':
            # No confirmed seal was checkpointed. Only an empty, unpowered
            # suction tool may retry Pick; a held/ambiguous box never does.
            self.require_empty()
            self.ports.recover_controller()
            self.store.update(phase='ABORTED')
            self.ports.pick_place(index)
            return
        data = self.restore()
        if data['phase'] == 'CARRY_RETURN':
            raise RuntimeError('held return interrupted; finish carry-home before resuming policy')
        entry = getattr(self.driver, 'doorway_entry', None)
        if (entry is not None and data['phase']=='CARRYING'
                and data['context'].get('doorway_entry_strategy') != entry.strategy):
            raise RuntimeError('saved carry route uses different doorway rules; carry-home then resume '
                               'to rebuild, or explicitly drop/reset')
        self.ports.recover_controller()
        if data['phase'] == 'RETURN_PENDING':
            self.require_empty()
            self._clear_release_touch()
            return
        native = self.ports_factory(self.driver, self.driver)
        try:
            io = self._io()
            released = data['phase'] in ('RELEASE_CONFIRMED','PLACED') or (
                data['phase']=='RELEASE_REQUESTED' and io.get('DI0')==0 and io.get('DO0')==0)
            if released:
                self.require_empty()
                if data['phase']=='RELEASE_REQUESTED':
                    ok, reason = self.driver.validate_place_pose()
                    if not ok:
                        raise RuntimeError('ambiguous release position: '+reason)
                segments = [segment_from_record(s) for s in data['segments']]
                if not segments:
                    raise RuntimeError('release checkpoint has no retreat target')
                self._reconcile_released_scene(data)
                self.store.update(phase='PLACED')
                result = native.execute_segment(segments[-1], 'resume_retreat')
                if not result.ok:
                    raise RuntimeError('resume retreat: '+result.message)
                self.store.update(phase='RETURN_PENDING', context=self.context())
            else:
                self.require_held()
                if data['phase'] in ('HELD_PICK', 'HELD_TRANSFER', 'HELD_HOME'):
                    start_transfer = (data['next_segment'] if data['phase']=='HELD_TRANSFER' else 0)
                    if data['phase']=='HELD_TRANSFER':
                        saved = [segment_from_record(s) for s in data['segments']]
                        if [s.type for s in saved] != list(TRANSFER_TYPES):
                            raise RuntimeError('saved carry transfer differs from current P1/P2')
                    ok, reason = self.driver.transfer_after_pick(native, start_transfer)
                    if not ok:
                        raise RuntimeError('resume transfer: '+reason)
                    detect = SimpleNamespace(frame='world', yaw_valid=True,
                                             box_size=self.driver._configured_payload_size)
                    self.driver.prepare_place_segments([], detect, SimpleNamespace(frame='world'))
                    data = self.store.snapshot()
                data = self._ensure_plan()
                segments = [segment_from_record(s) for s in data['segments']]
                start = 0 if data['phase']=='HELD_HOME' else data['next_segment']
                if not 0 <= start < len(segments):
                    raise RuntimeError('invalid resume segment index')
                self.offset = start
                code, reason = run_place(native, segments[start:],
                    low_hold_sec=self.driver._args.release_low_hold_sec,
                    release_deadline_sec=self.driver._args.release_deadline_sec)
                if code:
                    raise RuntimeError('resume exit %d: %s' % (code, reason))
        finally:
            self.offset = 0
            native.close()
        self._clear_release_touch()

    def _clear_release_touch(self):
        ok, reason = self.driver._scene_client().set_acm_pairs(
            [(self.driver.placed_proxy_id, link) for link in self.driver.release_touch_links], False)
        if not ok:
            raise RuntimeError('resume clear released touch: '+reason)

    def _reconcile_released_scene(self, data):
        from moveit_msgs.msg import PlanningSceneComponents
        scene = self.driver._scene_client().get_scene(
            PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
        if scene is None:
            raise RuntimeError('cannot reconcile released attachment')
        attached = any(a.object.id=='pickup_box' for a in scene.robot_state.attached_collision_objects)
        if attached:
            ok, reason = self.driver.remove_scene_box()
        else:
            ok, reason = self.driver.add_expected_placed_box(data['context']['scene_box'][2])
            self.driver._scene_box = None
        if not ok:
            raise RuntimeError('release scene reconciliation: '+reason)

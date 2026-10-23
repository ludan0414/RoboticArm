#!/usr/bin/env python3
"""Vacuum backend abstraction (no ROS): attach state machine + sim follow.

Three layers per the plan:
- ``VacuumBackend`` ABC - the hardware/sim contract.
- ``HardwareVacuumBackend`` - box DO0/DO1 + DI0 (injected IO, no ROS).
- ``SimVacuumBackend`` - kinematic follow via an injected ``GzPoseClient``
  (so the state machine is unit-testable with a mock) plus an injected
  planning-scene attacher.
- ``StubVacuumBackend`` - state-only, for environments without Gazebo.

The follow loop itself (timer, TF lookups) lives in the node layer; this
module owns the *decisions*: what attach/detach do to state, how the box
pose is computed from the locked offset.
"""

from __future__ import division

import math


def compose_pose(panel_xyz, panel_quat, offset_xyz, offset_quat):
    """Box pose in world = panel pose * locked relative offset.

    ``offset_*`` is the box pose expressed in the panel frame (captured at
    attach time). Uses the same quaternion math as vacuum_attach_utils.
    """
    from luggage_planning.vacuum_attach_utils import (
        compose_transform,
    )
    return compose_transform(panel_xyz, panel_quat, offset_xyz, offset_quat)


def invert_pose(xyz, quat):
    from luggage_planning.vacuum_attach_utils import invert_transform
    return invert_transform(xyz, quat)


def relative_offset(panel_xyz, panel_quat, box_xyz, box_quat):
    """Offset of the box in the panel frame (captured at attach)."""
    inv_xyz, inv_quat = invert_pose(panel_xyz, panel_quat)
    return compose_pose(inv_xyz, inv_quat, box_xyz, box_quat)


class VacuumBackend(object):
    """Contract for attach/detach implementations."""

    def attach(self, context):
        """Bind the box. ``context`` carries model name, current panel/box
        poses and the planning-scene handle. Returns (ok, message)."""
        raise NotImplementedError

    def detach(self, context):
        """Release the box. Returns (ok, message)."""
        raise NotImplementedError

    def is_attached(self):
        raise NotImplementedError

    def follow_step(self, panel_xyz, panel_quat):
        """Optional kinematic follow. Stub/hardware default is a no-op."""
        del panel_xyz, panel_quat
        return True, ""


class HardwareVacuumBackend(VacuumBackend):
    """实机代码 (site / real-cell). Not part of the Gazebo simulation stack.

    Real suction cup: box DO0=pump, DO1=blow-off, DI0=sealed.
    Measured 2026-09-09: only DO0=1/DO1=0 seals (DI0 rise ~6.3 s).
    Both-on never seals. Release is DO0=0 then DO1 pulse; DI0 falls in
    tens of milliseconds with blow-off.

    ``io`` must provide ``set_do(bit, value) -> (ok, msg)``, ``di0()``,
    ``sleep(dt)``, and ``now()``.
    """

    def __init__(
            self, io, seal_timeout_sec=8.0, seal_poll_sec=0.1,
            release_timeout_sec=1.0, blowoff_hold_sec=0.1):
        self._io = io
        self.seal_timeout_sec = float(seal_timeout_sec)
        self.seal_poll_sec = float(seal_poll_sec)
        self.release_timeout_sec = float(release_timeout_sec)
        self.blowoff_hold_sec = float(blowoff_hold_sec)
        self._attached = False
        self.last_error = ""

    def is_attached(self):
        return self._attached

    def follow_step(self, panel_xyz, panel_quat):
        del panel_xyz, panel_quat
        di0 = self._io.di0()
        if di0 is None:
            return False, "VACUUM_IO_STALE: holding state unknown"
        if di0 is not None:
            self._attached = bool(di0)
        return True, ""

    def attach(self, context):
        del context
        if self._io.di0() is None:
            return False, "VACUUM_IO_STALE: no fresh DI0 before attach"
        ok, message = self._set_grasp()
        if not ok:
            self.last_error = message
            self._release()
            return False, "VACUUM_BACKEND_ERROR: %s" % message
        t0 = self._io.now()
        while (self._io.now() - t0) < self.seal_timeout_sec:
            if self._io.di0() is None:
                self.last_error = "VACUUM_IO_STALE"
                return False, "VACUUM_IO_STALE during seal; inspect pump state"
            if self._io.di0() == 1:
                self._attached = True
                elapsed = self._io.now() - t0
                return True, "sealed in %.2fs" % elapsed
            self._io.sleep(self.seal_poll_sec)
        self.last_error = "VACUUM_SEAL_TIMEOUT"
        self._release()
        self._attached = False
        return False, "VACUUM_SEAL_TIMEOUT after %.1fs" % self.seal_timeout_sec

    def detach(self, context):
        del context
        ok, message = self._release()
        if ok:
            self._attached = False
        if not ok:
            self.last_error = message
            return False, "VACUUM_BACKEND_ERROR: %s" % message
        return True, message or "released"

    def _set_grasp(self):
        # Never 11: blow-off off first, then hold pump.
        ok, message = self._io.set_do(1, 0)
        if not ok:
            return False, message
        return self._io.set_do(0, 1)

    def _release(self):
        ok, message = self._io.set_do(0, 0)
        if not ok:
            return False, message
        ok, message = self._io.set_do(1, 1)
        if not ok:
            return False, message
        t0 = self._io.now()
        confirmed = False
        while (self._io.now() - t0) < self.release_timeout_sec:
            if self._io.di0() == 0:
                confirmed = True
                break
            self._io.sleep(min(self.seal_poll_sec, 0.05))
        self._io.sleep(self.blowoff_hold_sec)
        ok, message = self._io.set_do(1, 0)
        if not ok:
            return False, message
        if not confirmed:
            return False, "VACUUM_RELEASE_UNCONFIRMED: DI0 not low/fresh"
        return True, "released; DI0 low confirmed"


class StubVacuumBackend(VacuumBackend):
    """State-only backend (tests, no-Gazebo environments)."""

    def __init__(self):
        self._attached = False

    def attach(self, context):
        self._attached = True
        return True, "stub attached"

    def detach(self, context):
        self._attached = False
        return True, "stub detached"

    def is_attached(self):
        return self._attached


class SimVacuumBackend(VacuumBackend):
    """Simulation backend: gz kinematic follow + planning-scene attach.

    ``gz_client`` must expose ``set_model_pose(model_name, xyz, quat)``
    (the node wraps the bridged /world/<w>/set_pose service).
    ``scene`` must expose ``attach_pickup_box(...)`` /
    ``detach_and_remove(...)`` (planning_scene_client). Both are injected
    so the state machine runs under plain pytest with mocks.

    Privilege boundary: ``context["scene_box_size"]`` is the MoveIt attach
    geometry and MUST be the perception-measured size — never the GT spawn
    size (docs/architecture/privilege_boundary.md). The GT box pose feeds
    only the kinematic follow offset.
    """

    def __init__(self, gz_client, scene, follow_rate_hz=30.0):
        self._gz = gz_client
        self._scene = scene
        self.follow_rate_hz = float(follow_rate_hz)
        self._attached = False
        self._model_name = None
        self._offset_xyz = None
        self._offset_quat = None
        self.last_error = ""

    def is_attached(self):
        return self._attached

    def attach(self, context):
        if self._attached:
            return True, "already attached"
        try:
            self._model_name = context["model_name"]
            self._offset_xyz, self._offset_quat = relative_offset(
                context["panel_xyz"], context["panel_quat"],
                context["box_xyz"], context["box_quat"])
            scene_ok, scene_msg = self._scene.attach_pickup_box(
                context["model_name"], context["box_xyz"],
                context["box_quat"], context["scene_box_size"],
                context.get("attach_link", "suction_contact_frame"))
            if not scene_ok:
                self.last_error = scene_msg
                return False, "VACUUM_BACKEND_ERROR: scene %s" % scene_msg
        except Exception as exc:  # noqa: BLE001 - backend boundary
            self.last_error = str(exc)
            return False, "VACUUM_BACKEND_ERROR: %s" % exc
        self._attached = True
        return True, "attached %s (follow %.1f Hz)" % (
            self._model_name, self.follow_rate_hz)

    def follow_step(self, panel_xyz, panel_quat):
        """One follow tick: recompute the box pose and push it to gz.

        Called by the node's timer at ``follow_rate_hz``. No-op unless
        attached. Returns (ok, message).
        """
        if not self._attached or self._model_name is None:
            return True, ""
        try:
            xyz, quat = compose_pose(
                panel_xyz, panel_quat, self._offset_xyz, self._offset_quat)
            ok, msg = self._gz.set_model_pose(self._model_name, xyz, quat)
            if not ok:
                return False, msg
        except Exception as exc:  # noqa: BLE001
            return False, "follow error: %s" % exc
        return True, ""

    def detach(self, context):
        if not self._attached:
            return True, "not attached"
        model = self._model_name
        try:
            scene_ok, scene_msg = self._scene.detach_and_remove(model)
        except Exception as exc:  # noqa: BLE001
            self.last_error = str(exc)
            return False, "VACUUM_BACKEND_ERROR: detach %s" % exc
        if not scene_ok:
            self.last_error = scene_msg
            return False, "VACUUM_BACKEND_ERROR: detach %s" % scene_msg
        self._attached = False
        self._model_name = None
        self._offset_xyz = None
        self._offset_quat = None
        # Leave the box where it is; physics takes over.
        return True, "detached %s" % model

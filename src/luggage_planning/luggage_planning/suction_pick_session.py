#!/usr/bin/env python3
"""Port-injected suction pick session (ROS-free).

``PickSession`` owns the candidate loop, the bounded retry and the
vacuum failure recovery for one operator pick request. All IO — ROS
services/actions, clocks, sleeps, logging — goes through the ports
object, so gates C2-C6 run without a ROS graph and the retry decisions
live entirely in :mod:`luggage_planning.suction_retry_contracts`.

Ports interface (duck-typed; see ``RosPickSessionPorts`` in the driver
for the ROS implementation):

======================  =====================================================
``now()``               monotonic clock seconds
``sleep(dt)``           block *dt* seconds (fake in tests)
``detect()``            one ``DetectionView`` (retries are port-side)
``probe(segment, cid)`` plan-only probe: ``{ik_ok, fraction,
                        moveit_error_code}``
``execute_segment(s,``  ``SegmentOutcome`` (plan + execute + settle)
``cid)`` ``vacuum_enable(cid)``       ``(ok, message)``; backend self-releases on
                        seal timeout
``vacuum_release(cid)`` ``(ok, message)``
``di0()``               ``0`` low, ``1`` high, ``None`` unknown
``scene_add_box(box)``  ``(ok, message)``
``scene_attach()``      ``(ok, message)``
``scene_remove()``      ``(ok, message)``
``set_pickup_touch(b)`` ``(ok, message)``
``identity_latest()``   ``(instance_id, generation) | None``
``graph_health()``      ``(ok, reason)``
``log_event(record)``   JSONL trace sink
======================  =====================================================
"""

from collections import deque
from dataclasses import dataclass, replace
from typing import Tuple

from luggage_planning.suction_candidate_selection import (
    MAX_CANDIDATE_ATTEMPTS,
    REQUIRED_CARTESIAN_FRACTION,
    SUCTION_CANDIDATES_EXHAUSTED,
    SUCTION_CONTACT_MODEL_MISMATCH,
    ProbeRecord,
    SelectionState,
    contact_model_matches,
    cross_candidate_identity_consistent,
    judge_probe,
    next_candidate,
    observation_identity_consistent,
    rank_candidates,
    record_attempt,
    record_rejection,
)
from luggage_planning.suction_candidate_waypoints import (
    REVERSE_MIN_M,
    build_candidate_pick_segments,
    retry_reverse_segment,
    reverse_distance_m,
)
from luggage_planning.suction_retry_contracts import (
    REQUEST_DEADLINE_SEC,
    RELEASE_DEADLINE_SEC,
    RELEASE_LOW_HOLD_SEC,
    RetryEffectType,
    RetryEvent,
    RetryEventType,
    RetryState,
    audit_trace,
    initial_retry_model,
    is_terminal,
    reduce_retry_event,
)

#: driver exit codes (1-6 preserve the legacy meanings)
EXIT_OK = 0
EXIT_GRAPH = 1
EXIT_OBSERVE = 2
EXIT_DETECT = 3
EXIT_BUILD_SCENE = 4
EXIT_SEGMENT = 5
EXIT_VACUUM = 6
EXIT_RETRY_RECOVERY = 7
EXIT_CANDIDATES_EXHAUSTED = 8
EXIT_IDENTITY = 9
#: carry fault: vacuum PRESERVED; explicit recovery required (C6)
EXIT_CARRY_FAULT = 10

DETECT_NO_SEALABLE_PATCH = "DETECT_NO_SEALABLE_PATCH"
DETECT_TOP_SURFACE_INVALID = "DETECT_TOP_SURFACE_INVALID"
VACUUM_SEAL_TIMEOUT = "VACUUM_SEAL_TIMEOUT"


class SessionConfigError(ValueError):
    pass


@dataclass(frozen=True)
class SessionConfig:
    max_candidates: int = MAX_CANDIDATE_ATTEMPTS
    release_low_hold_sec: float = RELEASE_LOW_HOLD_SEC
    release_deadline_sec: float = RELEASE_DEADLINE_SEC
    reverse_min_m: float = REVERSE_MIN_M
    required_cartesian_fraction: float = REQUIRED_CARTESIAN_FRACTION
    request_deadline_sec: float = REQUEST_DEADLINE_SEC
    di0_poll_sec: float = 0.05
    use_scene: bool = True
    use_vacuum: bool = True
    planning_frame: str = "world"
    contact_model_version: int = 0
    contact_model_hash: str = ""
    dry_run: bool = False
    dry_run_count: int = 10

    def validate(self) -> None:
        positive = (
            self.release_low_hold_sec, self.release_deadline_sec,
            self.reverse_min_m, self.request_deadline_sec,
            self.di0_poll_sec,
        )
        if any(value <= 0.0 for value in positive):
            raise SessionConfigError("time/distance budgets must be positive")
        if self.release_low_hold_sec > self.release_deadline_sec:
            raise SessionConfigError(
                "release_low_hold_sec must fit inside release_deadline_sec")
        if self.di0_poll_sec > self.release_low_hold_sec * 0.5:
            raise SessionConfigError(
                "di0_poll_sec too coarse for release_low_hold_sec")
        if not 0.95 < self.required_cartesian_fraction <= 1.0:
            raise SessionConfigError(
                "required_cartesian_fraction must be in (0.95, 1.0]")
        if self.reverse_min_m < 0.05:
            raise SessionConfigError("reverse_min_m below 0.05 m")
        if not 1 <= self.max_candidates <= 5:
            raise SessionConfigError("max_candidates must be in [1, 5]")
        if not self.use_vacuum and not self.dry_run:
            # the seal (DI0) is the only attach confirmation in the
            # candidate contract; a vacuum-less pick cannot be authorized
            raise SessionConfigError(
                "candidate pick requires vacuum (dry_run exempt)")


@dataclass(frozen=True)
class SegmentOutcome:
    ok: bool
    message: str = ""
    fraction: float = -1.0
    settle_json: str = ""


@dataclass(frozen=True)
class DetectionView:
    """Plain observation view; built from DetectedLuggage port-side."""
    stamp: float
    frame: str
    candidates: Tuple = ()
    top_surface_valid: bool = True
    detection_yaw: float = 0.0
    yaw_valid: bool = False
    box_xyz: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    box_quat: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    box_size: Tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class SessionResult:
    reason_code: str
    detail: str
    exit_code: int
    completed: bool
    selected_candidate_id: str
    rejections: Tuple[Tuple[str, str, str], ...] = ()
    trace: Tuple[dict, ...] = ()


class PickSession:
    """One operator pick request against injected ports."""

    def __init__(self, ports, config=None):
        self._ports = ports
        self._config = config or SessionConfig()
        self._config.validate()
        self._model = initial_retry_model(self._config.max_candidates)
        self._selection = SelectionState(
            max_attempts=self._config.max_candidates)
        self._candidates = {}
        self._segments = {}
        self._recorded_approach = None
        self._trace = []
        self._pending = deque()
        self._request_started = 0.0
        self._detection_yaw = 0.0
        self._detection_yaw_valid = False
        self._detection_instance = ""
        self._detection_generation = 0

    # -- public API ----------------------------------------------------------

    def run_request(self, detection: DetectionView) -> SessionResult:
        self._request_started = self._ports.now()
        rejection = self._gate(detection)
        if rejection is not None:
            reason, detail, exit_code = rejection
            self._feed(RetryEvent(RetryEventType.REQUEST_REJECTED,
                                  reason_code=reason, detail=detail))
            return self._result(exit_code)

        self._candidates = {
            candidate.candidate_id: candidate
            for candidate in detection.candidates}
        ranked = rank_candidates(detection.candidates)
        self._selection = SelectionState(
            ranked_ids=tuple(c.candidate_id for c in ranked),
            max_attempts=self._config.max_candidates)
        self._detection_yaw = detection.detection_yaw
        self._detection_yaw_valid = detection.yaw_valid
        self._detection_instance = ranked[0].instance_id
        self._detection_generation = ranked[0].generation

        if self._config.use_scene:
            ok, message = self._ports.scene_add_box(detection)
            self._emit("io", "scene_add_box", ok=ok, message=message)
            if not ok:
                self._feed(RetryEvent(
                    RetryEventType.REQUEST_REJECTED,
                    reason_code="SCENE_ADD_FAILED", detail=message))
                return self._result(EXIT_BUILD_SCENE)
            ok, message = self._ports.set_pickup_touch(True)
            self._emit("io", "set_pickup_touch", ok=ok, message=message)
            if not ok:
                self._feed(RetryEvent(
                    RetryEventType.REQUEST_REJECTED,
                    reason_code="SCENE_TOUCH_FAILED", detail=message))
                return self._result(EXIT_BUILD_SCENE)

        self._feed(RetryEvent(
            RetryEventType.REQUEST_START, t=self._request_started,
            ranked_ids=self._selection.ranked_ids))

        while not is_terminal(self._model.state):
            if self._pending:
                effect = self._pending.popleft()
                if self._carry_pressure_lost():
                    # C6: any pressure loss after DI0 confirmation enters the
                    # carry fault boundary before the next motion dispatch
                    self._feed(RetryEvent(RetryEventType.DI0_SAMPLE,
                                          t=self._ports.now(), di0=0))
                    continue
                events = self._execute_effect(effect)
                for event in events:
                    self._feed(event)
                continue
            state = self._model.state
            if state == RetryState.SELECTING:
                self._select_step()
            elif state == RetryState.RELEASE_LOW_WAIT:
                self._poll_release_window()
            elif state == RetryState.RECOVERY_VERIFY:
                self._verify_recovery()
            else:
                raise RuntimeError(
                    "state %s has no activity and no pending effect"
                    % state.value)
        return self._result(self._exit_code())

    def run_dry_run(self) -> SessionResult:
        """C7 protocol: detect and print ranked candidates, zero actions."""
        if not self._config.dry_run:
            raise RuntimeError("run_dry_run requires config.dry_run")
        for _ in range(max(1, int(self._config.dry_run_count))):
            detection = self._ports.detect()
            self._emit("io", "dry_run_detection",
                       count=len(detection.candidates))
            for candidate in rank_candidates(detection.candidates):
                self._emit(
                    "io", "dry_run_candidate",
                    candidate_id=candidate.candidate_id,
                    rank=candidate.rank,
                    contact=(candidate.contact.position.x,
                             candidate.contact.position.y,
                             candidate.contact.position.z),
                    model_version=candidate.model_version,
                    model_hash=candidate.model_hash,
                    score=candidate.score)
        return SessionResult(
            reason_code="DRY_RUN_COMPLETE", detail="", exit_code=EXIT_OK,
            completed=True, selected_candidate_id="",
            trace=tuple(self._trace))

    @property
    def trace(self) -> Tuple[dict, ...]:
        return tuple(self._trace)

    @property
    def audit(self):
        return audit_trace(self._trace)

    # -- gates ---------------------------------------------------------------

    def _gate(self, detection):
        """Fail-closed request gates: ``(reason, detail, exit)`` or None."""
        if not detection.top_surface_valid:
            return (DETECT_TOP_SURFACE_INVALID, "", EXIT_DETECT)
        if not detection.candidates:
            return (DETECT_NO_SEALABLE_PATCH, "", EXIT_DETECT)
        mismatch = observation_identity_consistent(
            detection.candidates, detection.stamp, detection.frame,
            detection.candidates[0].instance_id,
            detection.candidates[0].generation)
        if mismatch is not None:
            return (mismatch[0], mismatch[1], EXIT_IDENTITY)
        mismatch = cross_candidate_identity_consistent(detection.candidates)
        if mismatch is not None:
            return (mismatch[0], mismatch[1], EXIT_IDENTITY)
        for candidate in detection.candidates:
            if not contact_model_matches(
                    candidate, self._config.contact_model_version,
                    self._config.contact_model_hash):
                return (SUCTION_CONTACT_MODEL_MISMATCH,
                        "candidate %s model %s/%s" % (
                            candidate.candidate_id, candidate.model_version,
                            candidate.model_hash),
                        EXIT_IDENTITY)
        return None

    # -- reducer plumbing ------------------------------------------------------

    def _feed(self, event: RetryEvent):
        # machine-readable event record: `suction_retry_replay` re-feeds
        # these through the pure reducer and must reach the same state
        record = {
            "t": self._ports.now(),
            "kind": "retry_event",
            "name": event.event_type.value,
            "candidate_id": event.candidate_id,
            "segment": event.segment,
            "ok": bool(event.ok),
            "reason_code": event.reason_code,
            "detail": event.detail,
            "di0": event.di0,
            "fraction": event.fraction,
            "ranked_ids": list(event.ranked_ids),
        }
        self._trace.append(record)
        self._ports.log_event(record)
        transition = reduce_retry_event(self._model, event)
        self._apply(transition)
        if (not is_terminal(self._model.state)
                and event.event_type != RetryEventType.REQUEST_START
                and self._ports.now() - self._request_started
                > self._config.request_deadline_sec):
            watchdog = reduce_retry_event(
                self._model,
                RetryEvent(RetryEventType.BUDGET_EXPIRED,
                           t=self._ports.now()))
            self._apply(watchdog)

    def _apply(self, transition):
        previous, self._model = self._model, transition.model
        if self._model.state != previous.state:
            self._emit("state", self._model.state.value,
                       reason_code=self._model.reason_code,
                       detail=self._model.detail)
        if is_terminal(self._model.state):
            self._pending.clear()   # no effect is authorized past a terminal
            return
        for effect in transition.effects:
            if effect.effect_type == RetryEffectType.PUBLISH_STATUS:
                # status effects become the canonical named trace events
                self._emit(
                    "event",
                    effect.payload_dict().get("reason_code", "status"),
                    state=self._model.state.value)
            else:
                self._pending.append(effect)

    def _carry_pressure_lost(self) -> bool:
        """Sample DI0 before dispatching motion while carrying (C6)."""
        if not self._model.di0_confirmed:
            return False
        if self._model.state not in (RetryState.SCENE_ATTACH,
                                     RetryState.PICK_RETREAT_MOTION):
            return False
        return self._ports.di0() != 1

    def _emit(self, kind: str, name: str, **payload):
        record = {
            "t": self._ports.now(),
            "kind": kind,
            "name": name,
            "candidate_id": self._model.active_candidate,
            "state": self._model.state.value,
            "payload": payload,
        }
        self._trace.append(record)
        self._ports.log_event(record)

    # -- selection ---------------------------------------------------------------

    def _select_step(self):
        """Probe candidates in rank order until one passes; feed the reducer."""
        while not is_terminal(self._model.state):
            candidate_id = next_candidate(self._selection)
            if candidate_id == SUCTION_CANDIDATES_EXHAUSTED:
                raise RuntimeError(
                    "selection exhausted but reducer state is %s"
                    % self._model.state.value)
            candidate = self._candidates[candidate_id]
            segments = build_candidate_pick_segments(
                candidate,
                detection_yaw=self._detection_yaw,
                yaw_valid=self._detection_yaw_valid)
            self._emit("io", "plan", candidate_id=candidate_id,
                       segments=[s.name for s in segments])
            records = []
            for segment in segments:
                if segment.name not in ("approach", "attach"):
                    continue
                probe = self._ports.probe(segment, candidate_id)
                records.append(ProbeRecord(
                    candidate_id=candidate_id,
                    segment_name=segment.name,
                    ik_ok=bool(probe.get("ik_ok", False)),
                    fraction=float(probe.get("fraction", -1.0)),
                    moveit_error_code=int(
                        probe.get("moveit_error_code", 0)),
                ))
            reason = judge_probe(
                candidate, records,
                required_fraction=self._config.required_cartesian_fraction,
                planning_frame=self._config.planning_frame)
            if reason is None:
                self._selection = record_attempt(self._selection, candidate_id)
                self._segments[candidate_id] = segments
                self._emit("io", "candidate_selected",
                           candidate_id=candidate_id)
                self._feed(RetryEvent(RetryEventType.CANDIDATE_SELECTED,
                                      candidate_id=candidate_id))
                return
            detail = "; ".join(
                "%s ik=%s frac=%.3f code=%d" % (
                    record.segment_name, record.ik_ok, record.fraction,
                    record.moveit_error_code)
                for record in records)
            self._selection = record_rejection(
                self._selection, candidate_id, reason, detail)
            self._emit("io", "candidate_rejected", candidate_id=candidate_id,
                       reason=reason)
            self._feed(RetryEvent(
                RetryEventType.CANDIDATE_REJECTED, candidate_id=candidate_id,
                reason_code=reason, detail=detail))

    # -- effect execution ----------------------------------------------------------

    def _execute_effect(self, effect):
        """Run one capability effect through the ports; return outcome events."""
        record = {
            "t": self._ports.now(),
            "kind": "effect",
            "name": effect.name,
            "effect_type": effect.effect_type.value,
            "candidate_id": self._model.active_candidate,
            "state": self._model.state.value,
            "motion": effect.motion,
            "vacuum": effect.vacuum,
            "detect": effect.detect,
            "scene": effect.scene,
            "lateral": effect.lateral,
            "payload": effect.payload_dict(),
        }
        self._trace.append(record)
        self._ports.log_event(record)
        if effect.effect_type == RetryEffectType.EXECUTE_SEGMENT:
            return self._execute_segment_effect(effect)
        if effect.effect_type == RetryEffectType.VACUUM_ENABLE:
            return self._execute_vacuum_enable()
        if effect.effect_type == RetryEffectType.SCENE_ATTACH:
            ok, message = self._ports.scene_attach()
            self._emit("io", "scene_attach", ok=ok, message=message)
            event_type = (RetryEventType.SCENE_ATTACH_SUCCEEDED if ok
                          else RetryEventType.SCENE_ATTACH_FAILED)
            return [RetryEvent(event_type,
                               reason_code="" if ok else "scene_attach_failed",
                               detail=message)]
        return []

    def _segment_by_name(self, name: str):
        if name == "retry_reverse":
            return retry_reverse_segment(
                self._candidates[self._model.active_candidate],
                self._recorded_approach)
        for segment in self._segments[self._model.active_candidate]:
            if segment.name == name:
                return segment
        raise KeyError("segment %s not built for %s"
                       % (name, self._model.active_candidate))

    def _execute_segment_effect(self, effect):
        segment_name = effect.payload_dict()["segment"]
        segment = self._segment_by_name(segment_name)
        outcome = self._ports.execute_segment(
            segment, effect.payload_dict().get("candidate_id", ""))
        self._emit("io", "segment_result", segment=segment_name,
                   ok=outcome.ok, message=outcome.message,
                   fraction=outcome.fraction)
        if segment_name == "approach" and outcome.ok:
            self._recorded_approach = segment.target_pose
        if segment_name == "retry_reverse" and outcome.ok:
            distance = reverse_distance_m(
                self._candidates[self._model.active_candidate],
                self._segment_by_name("attach").target_pose,
                self._recorded_approach)
            self._emit("event", "settled", reverse_distance_m=distance)
        if outcome.ok:
            return [RetryEvent(RetryEventType.SEGMENT_SUCCEEDED,
                               segment=segment_name,
                               fraction=outcome.fraction)]
        return [RetryEvent(
            RetryEventType.SEGMENT_FAILED, segment=segment_name,
            reason_code=outcome.message or "segment_failed",
            detail=segment_name, fraction=outcome.fraction)]

    def _execute_vacuum_enable(self):
        ok, message = self._ports.vacuum_enable(
            self._model.active_candidate)
        self._emit("io", "vacuum_enable", ok=ok, message=message)
        if ok:
            return [RetryEvent(RetryEventType.VACUUM_SEALED,
                               t=self._ports.now())]
        if VACUUM_SEAL_TIMEOUT in message:
            # backend completed DO0=0 + blow-off inside the timed-out attach
            self._emit("event", "release_backend", message=message)
            return [RetryEvent(RetryEventType.VACUUM_SEAL_TIMEOUT,
                               t=self._ports.now())]
        return [RetryEvent(RetryEventType.VACUUM_SERVICE_ERROR,
                           reason_code=message)]

    # -- state activities ------------------------------------------------------------

    def _poll_release_window(self):
        """Feed DI0 samples until the window resolves."""
        while self._model.state == RetryState.RELEASE_LOW_WAIT:
            value = self._ports.di0()
            self._feed(RetryEvent(RetryEventType.DI0_SAMPLE,
                                  t=self._ports.now(), di0=value))
            if self._model.state == RetryState.RELEASE_LOW_WAIT:
                self._ports.sleep(self._config.di0_poll_sec)

    def _verify_recovery(self):
        identity = self._ports.identity_latest()
        if identity is None:
            self._emit("io", "identity_unavailable")
            self._feed(RetryEvent(
                RetryEventType.IDENTITY_CHANGED,
                detail="identity_latest unavailable (fail closed)"))
            return
        if (str(identity[0]) != str(self._detection_instance)
                or int(identity[1]) != int(self._detection_generation)):
            self._emit("io", "identity_changed", identity=list(identity))
            self._feed(RetryEvent(RetryEventType.IDENTITY_CHANGED,
                                  detail=str(identity)))
            return
        self._emit("io", "identity_ok")
        ok, reason = self._ports.graph_health()
        if not ok:
            self._emit("io", "graph_lost", reason=reason)
            self._feed(RetryEvent(RetryEventType.GRAPH_LOST, detail=reason))
            return
        self._emit("io", "graph_ok")
        self._feed(RetryEvent(RetryEventType.RECOVERY_VERIFIED))

    # -- helpers -------------------------------------------------------------------

    def _result(self, exit_code: int) -> SessionResult:
        model = self._model
        if (model.state == RetryState.CANDIDATES_EXHAUSTED
                and not model.reason_code):
            model = replace(model, reason_code=SUCTION_CANDIDATES_EXHAUSTED)
        return SessionResult(
            reason_code=model.reason_code,
            detail=model.detail,
            exit_code=exit_code,
            completed=model.state == RetryState.COMPLETED,
            selected_candidate_id=model.sealed_candidate
            or model.active_candidate,
            rejections=model.rejections,
            trace=tuple(self._trace))

    def _exit_code(self) -> int:
        state = self._model.state
        if state == RetryState.COMPLETED:
            return EXIT_OK
        if state == RetryState.CANDIDATES_EXHAUSTED:
            return EXIT_CANDIDATES_EXHAUSTED
        if state == RetryState.RETRY_RECOVERY_FAILED:
            return EXIT_RETRY_RECOVERY
        if state == RetryState.MOTION_FAILED:
            return EXIT_SEGMENT
        if state == RetryState.CARRY_FAULT:
            return EXIT_CARRY_FAULT
        if state == RetryState.REJECTED:
            return EXIT_DETECT
        return EXIT_OK

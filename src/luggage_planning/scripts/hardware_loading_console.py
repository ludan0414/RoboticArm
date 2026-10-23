#!/usr/bin/env python3
"""Persistent local commissioning console. Startup never authorizes motion."""
import argparse
from dataclasses import replace
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import secrets
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor

from hardware_pick_driver import (
    HardwarePickDriver, RosPickSessionPorts, build_argument_parser,
)
from luggage_planning.hardware_loading_cycle import LoadingCycle, parse_profile, loading_box_prompt
from luggage_planning.pose import MotionSegment
from luggage_planning.release_confirmation import wait_for_release
from luggage_planning.return_pose import (adjusted_return_pose, DEFAULT_OFFSET_BASE_M,
                                         DEFAULT_CAMERA_FRAME)
from luggage_planning.placement_approach import lower_final_approach
from luggage_planning.placement_payload import release_above_support
from luggage_planning.pick_return_path import PickReturnPath
from luggage_planning.hardware_motion_guard import (HardwareMotionGuard, axis_test_segments,
                                                    quaternion_angle)
from luggage_planning.pose import Point, Pose, Quaternion


HARDWARE_STATUS_TOPIC = "/elfin/hardware_status"
EVENTS_TOPIC = "/trajectory_executor/events"


def _latched_qos():
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    return QoSProfile(
        depth=16,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
    )


def vacuum_view(driver):
    received = getattr(driver, "_vacuum_received_at", None)
    age = None if received is None else time.monotonic() - received
    state = dict(getattr(driver, "_vacuum_state", {}) or {})
    state["age_sec"] = None if age is None else round(age, 2)
    return state


def hardware_view(driver):
    state = dict(getattr(driver, "_console_hardware", {}) or {})
    received = getattr(driver, "_console_hardware_received_at", None)
    age = None if received is None else time.monotonic() - received
    state["age_sec"] = age
    if age is None or age > 2.0:
        state.update(available=False, stale=True)
    return state


def merge_console_status(snapshot, next_box_prompt, vacuum, hardware, executor_event):
    """Operator page payload: cycle + live CPS/ServoJ, not the site profile."""
    return dict(
        snapshot,
        next_box_prompt=next_box_prompt,
        vacuum=vacuum if isinstance(vacuum, dict) else {},
        hardware=hardware if isinstance(hardware, dict) else {"available": False},
        executor_event=executor_event if isinstance(executor_event, dict) else {},
    )


PAGE = """<!doctype html><meta charset="utf-8"><title>Hardware loading</title>
<style>
body{font:20px/1.4 sans-serif;max-width:980px;margin:28px auto;padding:0 16px}
button{font-size:22px;padding:14px 16px;margin:4px 4px 4px 0}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:12px}
.card{border:1px solid #bbb;border-radius:8px;padding:12px 14px}
.card h2{margin:0 0 8px;font-size:18px}
.ok{background:#e7f6e7}.warn{background:#fff6d4}.fault{background:#fde8e8}
.moving{background:#e7f0ff}.unknown{background:#f4f4f4}
.kv{display:flex;justify-content:space-between;gap:10px;margin:5px 0}
.big{font-size:28px;font-weight:700}
table{width:100%;border-collapse:collapse;font-size:15px}
td,th{text-align:left;padding:3px 5px;border-bottom:1px solid #eee;vertical-align:top}
pre{white-space:pre-wrap;font-size:13px}
#page-error{color:#a40000}
@media(max-width:800px){.grid{grid-template-columns:1fr}}
</style>
<h1>真机逐箱操作台 · 现场调试模式</h1>
<p>初始化可选：归到配置的观察位，或跳过并从当前位置开始。每次 Pick 自动抓取、放置、松吸、退舱、归位。</p>
<p>尚无自动扫舱、视觉放置验收或地图提交。占位仅供避障，并非实测货舱地图。</p>
<div class="grid">
<div id="cycle-card" class="card unknown"><h2>任务</h2>
<div class="kv"><span>状态</span><span id="cycle-state" class="big"></span></div>
<div class="kv"><span>阶段</span><span id="cycle-stage"></span></div>
<div class="kv"><span>箱序</span><span id="cycle-box"></span></div>
<div class="kv"><span>故障</span><span id="cycle-error"></span></div>
</div>
<div id="robot-card" class="card unknown"><h2>机器人 CPS</h2>
<div class="kv"><span>FSM</span><span id="robot-fsm" class="big"></span></div>
<div class="kv"><span>含义</span><span id="robot-fsm-label"></span></div>
<div class="kv"><span>连接</span><span id="robot-conn"></span></div>
<div class="kv"><span>执行器</span><span id="robot-exec"></span></div>
<div class="kv"><span>关节 °</span><span id="robot-joints"></span></div>
<p id="robot-missing"></p>
</div>
<div id="hrif-card" class="card unknown"><h2>最近 HRIF / ServoJ 记录</h2>
<div class="kv"><span>命令</span><span id="hrif-name"></span></div>
<div class="kv"><span>错误码</span><span id="hrif-code" class="big"></span></div>
<div class="kv"><span>含义</span><span id="hrif-hint"></span></div>
<div class="kv"><span>原文</span><span id="hrif-message"></span></div>
<div class="kv"><span>历史最近故障（不代表当前状态）</span><span id="hrif-fault"></span></div>
</div>
<div id="vac-card" class="card unknown"><h2>真空 / 执行器事件</h2>
<div class="kv"><span>吸附</span><span id="vac-attached"></span></div>
<div class="kv"><span>泵</span><span id="vac-on"></span></div>
<div class="kv"><span>年龄</span><span id="vac-age"></span></div>
<div class="kv"><span>动作事件</span><span id="exec-event"></span></div>
<div class="kv"><span>动作错误</span><span id="exec-error"></span></div>
</div>
</div>
<h2>最近 CPS 事件</h2>
<table><thead><tr><th>秒</th><th>命令</th><th>码</th><th>含义</th></tr></thead><tbody id="event-log"></tbody></table>
<label><input id="ack" type="checkbox">已检查场景、上一箱放置和上料区域，人员已离开运动范围</label>
<p><button id="axes" disabled>坐标方向测试（可选，每次 2cm 并复位）</button></p>
<p>测试顺序：+X、-X、+Y、-Y、+Z、-Z，每次都复位。请现场确认：+X 向左，+Y 向后，+Z 向上。完成动作不代表已自动验证物理方向。</p>
<p><button id="start" disabled>初始化归位（可选）</button> <button id="skip" disabled>跳过初始化，从当前位置开始</button></p>
<p><button id="pick" disabled>开始 Pick</button></p>
<p><button id="recover" disabled>停止并恢复待机（不松吸、不回位）</button></p>
<p id="recover-result"></p>
<p><button id="reset" disabled>松吸掉箱并回位（保留箱序）</button></p>
<p id="reset-target"></p>
<p>掉箱复位会停止机械臂、关闭吸盘，允许当前箱子掉落，确认松吸后再规划回位。请确认掉落区域和回位路径。掉落箱体位置未测量；复位不清除舱内占位，也不猜测本箱放置成功。</p>
<h2 id="next-box"></h2>
<p id="page-error"></p>
<p>故障后锁定，不自动松吸或重试。此网页不提供急停，请使用现场实体急停。</p>
<details><summary>原始 JSON</summary><pre id="status"></pre></details>
<script>
let current=null, pending=false;
const ack=document.getElementById('ack'), a=document.getElementById('start'), s=document.getElementById('skip'), b=document.getElementById('pick'), t=document.getElementById('axes'), r=document.getElementById('reset'), restore=document.getElementById('recover');
function enable(){a.disabled=s.disabled=pending||!ack.checked||current?.state!=='WAIT_START'||current?.return_pending;b.disabled=pending||!ack.checked||current?.state!=='WAIT_PICKUP_READY'||current?.return_pending;t.disabled=pending||!ack.checked||current?.return_pending||!['WAIT_START','WAIT_PICKUP_READY'].includes(current?.state);r.disabled=restore.disabled=pending||!ack.checked||!['WAIT_START','WAIT_PICKUP_READY','FAULT'].includes(current?.state);}
function txt(id,v){document.getElementById(id).textContent=v==null||v===''?'—':String(v);}
function paint(id,sev){document.getElementById(id).className='card '+(sev||'unknown');}
function ago(mono,now){if(!mono)return '—';const d=now-mono;if(d<0)return '0.0s';return d.toFixed(1)+'s 前';}
function render(){
  const hw=current.hardware||{};
  const ev=current.executor_event||{};
  const vac=current.vacuum||{};
  const now=hw.monotonic_sec||0;
  txt('next-box',current.next_box_prompt);
  const target=current.reset_target||{}, p=target.position||{};
  txt('reset-target',target.frame_id?'回位目标：'+target.frame_id+'，XYZ (m) = '+[p.x,p.y,p.z].map(v=>Number(v).toFixed(3)).join(', '):'回位目标待 TF 解析；相对原观察位的底座偏移 (m)：'+(current.reset_offset_base_m||[]).join(', '));
  const recovery=hw.recovery||{};
  txt('recover-result',recovery.stage?('最近恢复：'+recovery.stage+' · '+(recovery.message||'')):'');
  txt('cycle-state',current.state);
  txt('cycle-stage',current.stage);
  txt('cycle-box',(current.completed||0)+' / '+(current.total||0));
  txt('cycle-error',current.error||'无');
  paint('cycle-card',current.state==='FAULT'?'fault':current.state==='BUSY'?'moving':'ok');
  if(hw.available){
    txt('robot-fsm',hw.fsm_id==null?'?':hw.fsm_id);
    txt('robot-fsm-label',hw.fsm_label||hw.fsm_desc||'');
    txt('robot-conn',hw.connection||'');
    txt('robot-exec',(hw.executor||'')+(hw.ready_for_servo?' · 可开 ServoJ':''));
    txt('robot-joints',(hw.joints_deg||[]).map(v=>Number(v).toFixed(1)).join('  ')||'—');
    txt('robot-missing','');
    paint('robot-card',hw.fsm_severity||'unknown');
  }else{
    txt('robot-fsm','无数据');
    txt('robot-fsm-label',hw.stale?'控制器状态数据缺失或已过期':'执行器未发布 /elfin/hardware_status');
    txt('robot-conn','—');txt('robot-exec','—');txt('robot-joints','—');
    txt('robot-missing','重启 trajectory_executor 后才会有 FSM / 40071');
    paint('robot-card','unknown');
  }
  const last=hw.last_hrif||{};
  txt('hrif-name',last.name||'—');
  txt('hrif-code',last.code==null?'—':last.code);
  txt('hrif-hint',last.hint||'');
  txt('hrif-message',last.message||'');
  const fault=hw.last_fault||{};
  txt('hrif-fault',fault.name?fault.name+' '+fault.code+' · '+ (fault.hint||fault.message||'')+' · '+ago(fault.monotonic_sec,now):'无');
  paint('hrif-card',hw.available?(hw.current_fault?'fault':'ok'):'unknown');
  txt('vac-attached',vac.attached==null?'—':vac.attached);
  txt('vac-on',vac.vacuum_on==null?'—':vac.vacuum_on);
  txt('vac-age',vac.age_sec==null?'无':' '+vac.age_sec+'s');
  txt('exec-event',ev.event||'—');
  txt('exec-error',ev.error_string||'—');
  paint('vac-card',vac.attached||vac.vacuum_on?'moving':'ok');
  const rows=(hw.events||[]).slice().reverse();
  document.getElementById('event-log').innerHTML=rows.length?rows.map(e=>'<tr class="'+ (e.severity||'')+'"><td>'+ago(e.monotonic_sec,now)+'</td><td>'+ (e.name||'')+'</td><td>'+ (e.code==null?'':e.code)+'</td><td>'+ (e.hint||e.message||'')+(e.index==null?'':' #'+e.index)+'</td></tr>').join(''):'<tr><td colspan="4">尚无 Init/Start/40071/40025 记录</td></tr>';
  document.getElementById('status').textContent=JSON.stringify(current,null,2);
}
async function refresh(){try{current=await(await fetch('/status')).json();render();enable();}catch(e){a.disabled=s.disabled=b.disabled=t.disabled=r.disabled=restore.disabled=true;}}
async function send(command){pending=true;enable();try{const r=await fetch('/command',{method:'POST',headers:{'Content-Type':'application/json','X-Console-Token':'TOKEN'},body:JSON.stringify({command,request_id:current.request_id,schema_version:1,scene_confirmed:ack.checked})});if(!r.ok)throw Error(await r.text());ack.checked=false;document.getElementById('page-error').textContent='';}catch(e){document.getElementById('page-error').textContent=e.message;}finally{pending=false;await refresh();}}
a.onclick=()=>send('initialize');s.onclick=()=>send('skip_initialize');b.onclick=()=>send('pick');t.onclick=()=>send('axis_test');r.onclick=()=>send('reset');restore.onclick=()=>send('recover');ack.onchange=enable;setInterval(refresh,400);refresh();
</script>"""


class CommissioningDriver(HardwarePickDriver):
    # Rigid sealed-grasp estimate must reach these bounds before release.
    place_position_tolerance_m = .01
    place_orientation_tolerance_rad = math.radians(2)

    def _detection_view(self, box):
        """Commissioning only: operator-configured dimensions, visual top/yaw.

        Preserve the base driver's observation authorization and candidates.
        Re-centre the configured box below the observed top, so the collision
        top and the suction contact describe the same surface.
        """
        detection = super()._detection_view(box)
        size = getattr(self, '_configured_payload_size', None)
        if not size:
            raise RuntimeError('configured cycle payload dimensions missing')
        centre = (detection.box_xyz[0], detection.box_xyz[1],
                  float(box.top_surface_pose.position.z) - size[2]/2)
        self.get_logger().info('payload geometry source=configured_cycle size=%s visual_size=%s' % (
            size, detection.box_size))
        return replace(detection, box_xyz=centre, box_size=tuple(size))

    def record_completed_motion(self, segment):
        if not (segment.name == 'pick_retreat' or segment.name == 'pre_place'
                or segment.name.startswith(('ground_entry_', 'place_pickup_connect_', 'place_via_'))):
            return
        if not hasattr(self, '_pick_return_path'):
            self._pick_return_path = PickReturnPath()
        pose = self.lookup_pose('world', 'suction_contact_frame', fresh=True)
        self._pick_return_path.record(segment.name, pose)
    def attach_scene_box(self):
        """Freeze actual sealed-contact TF before the successful pick retreat."""
        from luggage_planning.policy_place import measured_grasp_transform
        self._measured_grasp = None
        self._sealed_grasp_record = None
        if self._scene_box is None:
            return False, "measured scene box missing at seal"
        xyz, quat, size = self._scene_box
        tool = self.lookup_pose("world", "suction_contact_frame", fresh=True)
        grasp = measured_grasp_transform(Pose(Point(*xyz), Quaternion(*quat)), tool)
        ok, message = super().attach_scene_box()
        if ok:
            self._measured_grasp = grasp
            self._sealed_grasp_record = dict(
                box_pose=dict(position=xyz, orientation=quat),
                collision_box_size_m=size, geometry_source='configured_cycle',
                tool_pose=dict(position=vars(tool.position), orientation=vars(tool.orientation)),
                offset_box_m=grasp[0], relative_orientation=vars(grasp[1]))
        return ok, message

    def validate_place_pose(self):
        from luggage_planning.policy_place import box_pose_from_grasp
        grasp = getattr(self, "_measured_grasp", None)
        if grasp is None:
            return False, "sealed grasp unavailable"
        tool = self.lookup_pose("world", "suction_contact_frame", fresh=True)
        estimated = box_pose_from_grasp(tool, grasp)
        target = getattr(self, '_effective_place_target', self._args.place_target_data).box_pose
        position_error = math.dist(tuple(vars(estimated.position).values()),
                                   tuple(vars(target.position).values()))
        orientation_error = quaternion_angle(estimated.orientation, target.orientation)
        self._last_place_pose_check = dict(
            source="estimated_from_sealed_grasp", position=vars(estimated.position),
            orientation=vars(estimated.orientation), position_error_m=position_error,
            orientation_error_rad=orientation_error)
        ok = (position_error <= self.place_position_tolerance_m
              and orientation_error <= self.place_orientation_tolerance_rad)
        return ok, "sealed-grasp box error %.4fm / %.2fdeg" % (
            position_error, math.degrees(orientation_error))

    def recover_controller(self):
        """Use the existing executor's CPS connection; never open a second one."""
        from std_srvs.srv import Trigger
        if not hasattr(self, "_recovery_client"):
            self._recovery_client = self.create_client(Trigger, "/elfin/recover_standby")
        if not self._recovery_client.wait_for_service(timeout_sec=2.0):
            return False, "executor recovery service unavailable; restart updated executor"
        response = self._call_srv(self._recovery_client, Trigger.Request(), 20.0)
        if response is None:
            return False, "recovery RPC timed out; physical result unknown"
        return bool(response.success), str(response.message)

    """Keep nominal placed-box collision proxies separate from cargo mapping."""
    release_touch_links = ("suction_panel", "suction_contact_frame")
    exclusive_trace = True

    def lookup_pose(self, target, source, fresh=False):
        from rclpy.time import Time
        stamped = self.tf_buffer.lookup_transform(target, source, Time())
        if fresh:
            age = (self.get_clock().now().nanoseconds
                   - Time.from_msg(stamped.header.stamp).nanoseconds) / 1e9
            if not -.1 <= age <= 2.0:
                raise RuntimeError("stale tool TF")
        transform = stamped.transform
        return Pose(Point(transform.translation.x, transform.translation.y, transform.translation.z),
                    Quaternion(transform.rotation.x, transform.rotation.y,
                               transform.rotation.z, transform.rotation.w))

    def validate_scene_frame(self):
        ok, message = self.motion_guard.check_base(self.lookup_pose("world", "elfin_base_link"))
        if not ok:
            raise RuntimeError(message)
        ok, message = self.motion_guard.check_container(self.lookup_pose("world", "container_link"))
        if not ok:
            raise RuntimeError(message)

    def guard_detection(self, detection):
        self.validate_scene_frame()
        return self.motion_guard.check_pickup(detection, None)

    def prepare_place_segments(self, segments, detection=None, candidate=None):
        from luggage_planning.policy_place import connect_place_from_pose, build_place_segments
        actual = self.lookup_pose("world", "suction_contact_frame", fresh=True)
        grasp = getattr(self, "_measured_grasp", None)
        if detection is not None and candidate is not None:
            if grasp is None:
                raise RuntimeError("actual sealed-contact TF missing; refusing nominal grasp placement")
            target = release_above_support(self._args.place_target_data)
            self._effective_place_target = target
            self.get_logger().info('placement payload height: configured=%s reference=%s target=%s' % (
                detection.box_size, target.reference_box_size_m, vars(target.box_pose.position)))
            target = lower_final_approach(target)
            segments = build_place_segments(target, detection, candidate, measured_grasp=grasp)
        config = getattr(self, "carry_entry_config", None)
        if config is not None:
            from luggage_planning.carry_entry import CarryEntry
            if detection is None or candidate is None:
                raise RuntimeError("measured successful grasp required for high carry entry")
            self._carry_entry = CarryEntry(config, detection, candidate, measured_grasp=grasp)
            return self._carry_entry.connect(actual, segments)
        return connect_place_from_pose(actual, segments)

    def guard_segment(self, segment):
        self.validate_scene_frame()
        actual = self.lookup_pose("world", "suction_contact_frame", fresh=True)
        ok, message = self.motion_guard.check_segment(actual, segment.target_pose, segment.name)
        if not ok:
            return ok, message
        if "_WARNING:" in message:
            self.get_logger().warning("ADVISORY ONLY: " + message)
        entry = getattr(self, "_carry_entry", None)
        if entry is not None:
            return entry.check_segment(actual, segment)
        return ok, message

    def remove_scene_box(self):
        geometry = self._scene_box
        ok, message = super().remove_scene_box()
        if not ok or geometry is None:
            return False, message or "missing measured payload geometry"
        target = getattr(self, '_effective_place_target', self._args.place_target_data).box_pose
        # This is an EXPECTED placement, never a measured cargo-map commit.
        scene = self._scene_client()
        # Only suction contact links may initially touch the released proxy.
        # Clear this temporary permission after place_retreat, before returning.
        ok, message = scene.set_acm_pairs(
            [(self.placed_proxy_id, link) for link in self.release_touch_links], True)
        if not ok:
            return ok, message
        return scene.add_collision_box(
            self.placed_proxy_id,
            [target.position.x, target.position.y, target.position.z],
            [target.orientation.x, target.orientation.y,
             target.orientation.z, target.orientation.w], geometry[2])


class HardwarePorts:
    def __init__(self, driver, observe, cycles, journal, session, return_settings=None):
        self.driver, self.observe, self.cycles = driver, observe, cycles
        self.journal, self.session = journal, session
        settings = return_settings or {}
        from luggage_planning.policy_place import _vector
        self.nominal_observe = observe
        self.return_offset = _vector(settings.get("offset_base_m", DEFAULT_OFFSET_BASE_M), 3,
                                     "return offset_base_m")
        self.return_camera_frame = settings.get("camera_optical_frame", DEFAULT_CAMERA_FRAME)
        if not isinstance(self.return_camera_frame, str) or not self.return_camera_frame:
            raise ValueError("return camera_optical_frame must be a nonempty frame name")
        self.return_resolved = False

    def resolve_return_pose(self):
        base = self.driver.lookup_pose("world", "elfin_base_link")
        camera = self.driver.lookup_pose("suction_contact_frame", self.return_camera_frame)
        self.observe = adjusted_return_pose(self.nominal_observe, base, camera, self.return_offset)
        self.return_resolved = True
        self.record(dict(event="return_target_resolved", offset_base_m=self.return_offset,
            base_pose=dict(position=vars(base.position), orientation=vars(base.orientation)),
            camera_pose=dict(position=vars(camera.position), orientation=vars(camera.orientation)),
            target=dict(position=vars(self.observe.position), orientation=vars(self.observe.orientation))))
        return self.observe

    def record(self, event):
        record = {"time": time.time(), "session": self.session, **event}
        self.driver.get_logger().info("loading_event: " + json.dumps(record, default=str))
        self.journal.write(json.dumps(record) + "\n")
        self.journal.flush()
        os.fsync(self.journal.fileno())

    def axis_test(self):
        self.prepare_current()
        origin = self.driver.lookup_pose("world", "suction_contact_frame", fresh=True)
        base = self.driver.lookup_pose("world", "elfin_base_link")
        for segment in axis_test_segments(origin, base):
            self.execute([segment])
            actual = self.driver.lookup_pose("world", "suction_contact_frame", fresh=True)
            target = segment.target_pose
            distance = math.dist((actual.position.x, actual.position.y, actual.position.z),
                                 (target.position.x, target.position.y, target.position.z))
            if distance > .005 or quaternion_angle(actual.orientation, target.orientation) > math.radians(2):
                raise RuntimeError("AXIS_TEST_NOT_SETTLED: " + segment.name)
            self.record(dict(event="axis_test_settled", segment=segment.name,
                             target=vars(target.position), actual=vars(actual.position),
                             error_m=distance))

    def execute(self, segments):
        ports = RosPickSessionPorts(self.driver, self.driver)
        try:
            self.require_empty_tool(ports)
            for segment in segments:
                self.require_empty_tool(ports)
                self.driver.report_stage(segment.name.upper())
                healthy, reason = ports.graph_health()
                if not healthy:
                    raise RuntimeError(reason)
                outcome = ports.execute_segment(segment, "return")
                if not outcome.ok:
                    raise RuntimeError("%s: %s" % (segment.name, outcome.message))
        finally:
            ports.close()

    def require_empty_tool(self, ports):
        received = self.driver._vacuum_received_at
        if (received is None or time.monotonic() - received > getattr(self.driver._args, "vacuum_state_max_age_sec", 2.0)
                or ports.di0() != 0
                or self.driver._vacuum_state.get("attached", True)
                or self.driver._vacuum_state.get("vacuum_on", True)):
            raise RuntimeError("fresh empty tool / vacuum off not confirmed")

    def recover_controller(self):
        self.driver.report_stage("RECOVERING_CONTROLLER")
        self.record(dict(event="controller_recovery_requested"))
        ok, message = self.driver.recover_controller()
        self.record(dict(event="controller_recovery_result", ok=ok, message=message))
        if not ok:
            raise RuntimeError("RESET_CONTROLLER: " + message)

    def reset_observe(self):
        """Operator-authorized payload drop, then configured MoveIt return."""
        self.recover_controller()
        ports = RosPickSessionPorts(self.driver, self.driver)
        try:
            received = self.driver._vacuum_received_at
            state = dict(self.driver._vacuum_state)
            maximum = getattr(self.driver._args, "vacuum_state_max_age_sec", 2.0)
            self.record(dict(event="reset_preflight", vacuum=state,
                vacuum_age_sec=None if received is None else time.monotonic()-received,
                di0=ports.di0()))
            self.driver.report_stage("RESET_RELEASE")
            commanded_at = time.monotonic()
            ok, message = self.driver.vacuum(False)
            self.record(dict(event="reset_release_result", ok=ok, message=message))
            if not ok:
                raise RuntimeError("RESET_PUMP: " + message)
            wait_for_release(
                lambda: (self.driver._vacuum_received_at, dict(self.driver._vacuum_state)),
                ports.di0, commanded_at, self.driver._args.release_deadline_sec, maximum)
            # A dropped payload is not a placement at the frozen target. Remove
            # only the pickup attachment, never create a placed-target proxy.
            ok, message = self.driver._scene_client().detach_and_remove()
            if not ok:
                raise RuntimeError("RESET_DETACH: " + message)
            self.driver._scene_box = None
            ok, message = self.driver.set_pickup_touch(False)
            if not ok:
                raise RuntimeError("RESET_TOUCH: " + message)
        finally:
            ports.close()
        self.prepare()
        self.record(dict(event="reset_complete", target=vars(self.observe.position)))

    def prepare(self):
        self.prepare_current()
        self.resolve_return_pose()
        self.return_to_pick_stop()
        self.execute([MotionSegment(name="initialize_observe", type="pose_target",
                                    target_pose=self.observe)])

    def return_to_pick_stop(self):
        path = getattr(self.driver, '_pick_return_path', None)
        if path is not None and path.poses:
            while path.poses:
                segment = path.next_segment()
                lookup = getattr(self.driver, 'lookup_pose', None)
                actual = lookup('world', 'suction_contact_frame', fresh=True) if lookup else None
                at_target = (actual is not None and math.dist(
                    tuple(vars(actual.position).values()),
                    tuple(vars(segment.target_pose.position).values())) <= .003
                    and quaternion_angle(actual.orientation, segment.target_pose.orientation) <= math.radians(1))
                if not at_target:
                    self.execute([segment])
                path.settled()
        else:
            # Before a pick has completed, use the original configured pickup
            # waiting pose as the staging waypoint, before the adjusted camera pose.
            self.execute([MotionSegment(name='return_pick_wait', type='pose_target',
                                        target_pose=self.nominal_observe)])

    def prepare_current(self):
        """Readiness checks only: no observe move, detection or vacuum command."""
        ready, reason = self.driver.wait_graph()
        if not ready:
            raise RuntimeError(reason)
        self.driver.validate_scene_frame()
        scene = self.driver._scene_client()
        if not scene.wait_ready(timeout_sec=5.0):
            raise RuntimeError("planning scene unavailable")
        self.execute([])

    def pick_place(self, index):
        self.driver.validate_scene_frame()
        self.execute([])  # A held box or unknown vacuum state blocks a new pick.
        self.driver._args.place_target_data = self.cycles[index][0]
        self.driver._configured_payload_size = self.cycles[index][0].reference_box_size_m
        if not self.driver._configured_payload_size:
            raise RuntimeError('cycle needs expected_demo_input_dims_m for configured collision geometry')
        self.driver._pick_return_path = PickReturnPath()
        if hasattr(self.driver, '_effective_place_target'):
            del self.driver._effective_place_target
        self.driver._measured_grasp = None
        self.driver.placed_proxy_id = "expected_placed_%s_%d" % (self.session, index)
        # Separate immutable trace per box, never overwrite a previous fault.
        self.driver._args.trace_out = "%s.box%03d.jsonl" % (self.journal.name, index + 1)
        code = self.driver.run()
        if code != 0:
            raise RuntimeError("pick/place exit %d; explicit recovery required" % code)
        ok, message = self.driver._scene_client().set_acm_pairs(
            [(self.driver.placed_proxy_id, link)
             for link in self.driver.release_touch_links], False)
        if not ok:
            raise RuntimeError("placed collision permissions: " + message)

    def return_observe(self, index):
        target = self.resolve_return_pose()
        path = getattr(self.driver, '_pick_return_path', None)
        if path is not None and path.poses:
            self.return_to_pick_stop()
        else:
            # Legacy sessions without measured breadcrumbs retain their explicit
            # exit route, then visit the pickup waiting pose before observation.
            self.execute(list(self.cycles[index][1])[:-1])
            self.return_to_pick_stop()
        self.execute([MotionSegment(name='return_observe', type='pose_target', target_pose=target)])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--site-profile", required=True)
    parser.add_argument("--journal", required=True, help="new JSONL file; existing files refused")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--runtime-config", default="", help="site pick runtime YAML")
    cli = parser.parse_args(argv)
    # The console runs on the Linux Jazzy laptop. Hold this lock until exit;
    # a second console must never submit independent hardware goals.
    import fcntl
    cell_lock = open("/tmp/roboticarm-hardware-loading.lock", "a")
    fcntl.flock(cell_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    raw_profile = Path(cli.site_profile).read_bytes()
    observe, cycles = parse_profile(json.loads(raw_profile))
    motion_guard = HardwareMotionGuard(json.loads(raw_profile))
    from luggage_planning.carry_entry import CarryConfig
    entry_settings = json.loads(raw_profile).get("carry_entry")
    carry_config = CarryConfig(entry_settings) if entry_settings is not None else None
    # Fixed execution contract: commissioning cannot disable scene/vacuum or
    # accidentally invoke a preview/pick-only path through forwarded CLI flags.
    args = build_argument_parser(cli.runtime_config or None).parse_args([])
    args.skip_observe = True  # We return to the explicit TCP pose via MoveIt.
    args.place_target_data = None
    token, session = secrets.token_hex(32), secrets.token_hex(8)
    journal = open(cli.journal, "x", encoding="utf-8")
    rclpy.init()
    driver = CommissioningDriver(args)
    from tf2_ros import Buffer, TransformListener
    driver.tf_buffer = Buffer()
    driver.tf_listener = TransformListener(driver.tf_buffer, driver)
    driver.motion_guard = motion_guard
    driver.carry_entry_config = carry_config
    driver.segment_guard = driver.guard_segment
    driver.detection_guard = driver.guard_detection
    ports = HardwarePorts(driver, observe, cycles, journal, session,
        json.loads(raw_profile).get("return_pose_adjustment"))
    ports.record(dict(event="session", profile_sha256=hashlib.sha256(raw_profile).hexdigest(),
                      profile=json.loads(raw_profile), session=session))
    cycle = LoadingCycle(ports, len(cycles))
    driver.stage_callback = cycle.stage_changed
    driver._console_hardware = {"available": False}
    driver._console_event = {}

    def _on_hardware(msg):
        try:
            state = json.loads(msg.data)
            if isinstance(state, dict):
                driver._console_hardware = state
                driver._console_hardware_received_at = time.monotonic()
        except (TypeError, ValueError, json.JSONDecodeError):
            pass

    def _on_executor_event(msg):
        try:
            driver._console_event = json.loads(msg.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            pass

    from std_msgs.msg import String
    qos = _latched_qos()
    driver.create_subscription(String, HARDWARE_STATUS_TOPIC, _on_hardware, qos)
    driver.create_subscription(String, EVENTS_TOPIC, _on_executor_event, qos)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(driver)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()

    class Handler(BaseHTTPRequestHandler):
        def reply(self, code, body, kind="application/json"):
            payload = body.encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", kind + "; charset=utf-8")
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(payload)

        def do_GET(self):
            if self.headers.get("Host") not in (
                    "127.0.0.1:%d" % cli.port, "localhost:%d" % cli.port):
                self.reply(403, '{}')
                return
            if self.path == "/":
                self.reply(200, PAGE.replace("TOKEN", token), "text/html")
            elif self.path == "/status":
                snapshot = cycle.snapshot()
                snapshot["reset_target"] = (dict(frame_id="world",
                    position=vars(ports.observe.position), orientation=vars(ports.observe.orientation))
                    if ports.return_resolved else None)
                snapshot["reset_offset_base_m"] = list(ports.return_offset)
                profile = json.loads(raw_profile)
                self.reply(200, json.dumps(merge_console_status(
                    snapshot,
                    loading_box_prompt(profile, snapshot),
                    vacuum_view(driver),
                    hardware_view(driver),
                    getattr(driver, "_console_event", {}),
                )))
            else:
                self.reply(404, '{}')

        def do_POST(self):
            if self.path != "/command" or self.headers.get("X-Console-Token") != token:
                self.reply(403, '{}')
                return
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if not 0 < size <= 2048:
                    raise ValueError("invalid request length")
                body = json.loads(self.rfile.read(size))
                accepted = cycle.submit(body.get("command"), body.get("request_id"),
                                        body.get("schema_version"), body.get("scene_confirmed"))
                self.reply(202 if accepted else 409, json.dumps({"accepted": accepted}))
            except (ValueError, TypeError, AttributeError):
                self.reply(400, '{}')
            except OSError:
                self.reply(503, '{"error":"journal unavailable; no authorization"}')

        def log_message(self, *_args):
            pass

    server = ThreadingHTTPServer(("127.0.0.1", cli.port), Handler)
    print("Hardware console: http://127.0.0.1:%d (WAIT_START)" % cli.port, flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        # Never release suction or initiate a return on shutdown. Hardware e-stop
        # remains the stop mechanism; HTTP and Ctrl-C are not an emergency stop.
        server.server_close()
        if cycle.worker is not None and cycle.worker.is_alive():
            print("Active cycle interrupted: use physical stop and explicit recovery.", flush=True)
            os._exit(2)
        executor.shutdown()
        driver.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        journal.close()


if __name__ == "__main__":
    main()

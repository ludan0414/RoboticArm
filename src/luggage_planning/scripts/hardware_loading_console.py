#!/usr/bin/env python3
"""Persistent local commissioning console. Startup never authorizes motion."""
import argparse
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
<div id="hrif-card" class="card unknown"><h2>最近 HRIF / ServoJ</h2>
<div class="kv"><span>命令</span><span id="hrif-name"></span></div>
<div class="kv"><span>错误码</span><span id="hrif-code" class="big"></span></div>
<div class="kv"><span>含义</span><span id="hrif-hint"></span></div>
<div class="kv"><span>原文</span><span id="hrif-message"></span></div>
<div class="kv"><span>上次故障</span><span id="hrif-fault"></span></div>
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
<p><button id="reset" disabled>空载复位到观察位（保留箱序）</button></p>
<p>复位前确认机械臂已停止、工具空载。本箱如已实际放入但系统尚未记为完成，先人工处理该箱；复位不会猜测放置成功或清除舱内占位。</p>
<h2 id="next-box"></h2>
<p id="page-error"></p>
<p>故障后锁定，不自动松吸或重试。此网页不提供急停，请使用现场实体急停。</p>
<details><summary>原始 JSON</summary><pre id="status"></pre></details>
<script>
let current=null, pending=false;
const ack=document.getElementById('ack'), a=document.getElementById('start'), s=document.getElementById('skip'), b=document.getElementById('pick'), t=document.getElementById('axes'), r=document.getElementById('reset');
function enable(){a.disabled=s.disabled=pending||!ack.checked||current?.state!=='WAIT_START';b.disabled=pending||!ack.checked||current?.state!=='WAIT_PICKUP_READY';t.disabled=pending||!ack.checked||!['WAIT_START','WAIT_PICKUP_READY'].includes(current?.state);r.disabled=pending||!ack.checked||!['WAIT_START','WAIT_PICKUP_READY','FAULT'].includes(current?.state);}
function txt(id,v){document.getElementById(id).textContent=v==null||v===''?'—':String(v);}
function paint(id,sev){document.getElementById(id).className='card '+(sev||'unknown');}
function ago(mono,now){if(!mono)return '—';const d=now-mono;if(d<0)return '0.0s';return d.toFixed(1)+'s 前';}
function render(){
  const hw=current.hardware||{};
  const ev=current.executor_event||{};
  const vac=current.vacuum||{};
  const now=hw.monotonic_sec||0;
  txt('next-box',current.next_box_prompt);
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
    txt('robot-fsm-label','执行器未发布 /elfin/hardware_status');
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
  paint('hrif-card',last.severity|| (fault.severity||'ok'));
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
async function refresh(){try{current=await(await fetch('/status')).json();render();enable();}catch(e){a.disabled=s.disabled=b.disabled=t.disabled=r.disabled=true;}}
async function send(command){pending=true;enable();try{const r=await fetch('/command',{method:'POST',headers:{'Content-Type':'application/json','X-Console-Token':'TOKEN'},body:JSON.stringify({command,request_id:current.request_id,schema_version:1,scene_confirmed:ack.checked})});if(!r.ok)throw Error(await r.text());ack.checked=false;document.getElementById('page-error').textContent='';}catch(e){document.getElementById('page-error').textContent=e.message;}finally{pending=false;await refresh();}}
a.onclick=()=>send('initialize');s.onclick=()=>send('skip_initialize');b.onclick=()=>send('pick');t.onclick=()=>send('axis_test');r.onclick=()=>send('reset');ack.onchange=enable;setInterval(refresh,400);refresh();
</script>"""


class CommissioningDriver(HardwarePickDriver):
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
        from luggage_planning.policy_place import connect_place_from_pose
        actual = self.lookup_pose("world", "suction_contact_frame", fresh=True)
        config = getattr(self, "carry_entry_config", None)
        if config is not None:
            from luggage_planning.carry_entry import CarryEntry
            if detection is None or candidate is None:
                raise RuntimeError("measured successful grasp required for high carry entry")
            self._carry_entry = CarryEntry(config, detection, candidate)
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
        target = self._args.place_target_data.box_pose
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
    def __init__(self, driver, observe, cycles, journal, session):
        self.driver, self.observe, self.cycles = driver, observe, cycles
        self.journal, self.session = journal, session

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
                or self.driver._vacuum_state.get("vacuum_on", True)):
            raise RuntimeError("fresh empty tool / vacuum off not confirmed")

    def reset_observe(self):
        """Explicit empty-tool recovery; never release a held/unknown box."""
        ready, reason = self.driver.wait_graph()
        if not ready:
            raise RuntimeError("RESET_GRAPH: " + reason)
        ports = RosPickSessionPorts(self.driver, self.driver)
        try:
            received = self.driver._vacuum_received_at
            state = dict(self.driver._vacuum_state)
            maximum = getattr(self.driver._args, "vacuum_state_max_age_sec", 2.0)
            self.record(dict(event="reset_preflight", vacuum=state,
                vacuum_age_sec=None if received is None else time.monotonic()-received,
                di0=ports.di0()))
            if (received is None or time.monotonic()-received > maximum
                    or ports.di0() != 0 or state.get("attached", True)):
                raise RuntimeError("RESET_REFUSED: empty tool not confirmed; suction preserved")
            if state.get("vacuum_on", True):
                ok, message = self.driver.vacuum(False)
                if not ok:
                    raise RuntimeError("RESET_PUMP: " + message)
                deadline = time.monotonic() + self.driver._args.release_deadline_sec
                while self.driver._vacuum_state.get("vacuum_on", True) and time.monotonic() < deadline:
                    time.sleep(.05)
        finally:
            ports.close()
        self.prepare()
        self.record(dict(event="reset_complete", target=vars(self.observe.position)))

    def prepare(self):
        self.prepare_current()
        self.execute([MotionSegment(name="initialize_observe", type="pose_target",
                                    target_pose=self.observe)])

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
        self.execute(self.cycles[index][1])


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
    ports = HardwarePorts(driver, observe, cycles, journal, session)
    ports.record(dict(event="session", profile_sha256=hashlib.sha256(raw_profile).hexdigest(),
                      profile=json.loads(raw_profile), session=session))
    cycle = LoadingCycle(ports, len(cycles))
    driver.stage_callback = cycle.stage_changed
    driver._console_hardware = {"available": False}
    driver._console_event = {}

    def _on_hardware(msg):
        try:
            driver._console_hardware = json.loads(msg.data)
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
                profile = json.loads(raw_profile)
                self.reply(200, json.dumps(merge_console_status(
                    snapshot,
                    loading_box_prompt(profile, snapshot),
                    vacuum_view(driver),
                    getattr(driver, "_console_hardware", {"available": False}),
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

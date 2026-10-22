#!/usr/bin/env python3
"""Persistent local commissioning console. Startup never authorizes motion."""
import argparse
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
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
from luggage_planning.hardware_loading_cycle import LoadingCycle, parse_profile
from luggage_planning.pose import MotionSegment


PAGE = """<!doctype html><meta charset="utf-8"><title>Hardware loading</title>
<style>body{font:22px sans-serif;max-width:850px;margin:50px auto}button{font-size:24px;padding:18px}pre{white-space:pre-wrap}</style>
<h1>真机逐箱操作台 · 现场调试模式</h1>
<p>初始化归到配置的观察位；每次 Pick 自动抓取、放置、松吸、退舱、归位。</p>
<p>尚无自动扫舱、视觉放置验收或地图提交。占位仅供避障，并非实测货舱地图。</p>
<label><input id="ack" type="checkbox">已检查场景、上一箱放置和上料区域，人员已离开运动范围</label>
<p><button id="start" disabled>初始化</button> <button id="pick" disabled>开始 Pick</button></p>
<pre id="status"></pre><p id="error"></p>
<p>故障后锁定，不自动松吸或重试。此网页不提供急停，请使用现场实体急停。</p>
<script>
let current=null, pending=false;
const ack=document.getElementById('ack'), a=document.getElementById('start'), b=document.getElementById('pick');
function enable(){a.disabled=pending||!ack.checked||current?.state!=='WAIT_START';b.disabled=pending||!ack.checked||current?.state!=='WAIT_PICKUP_READY';}
async function refresh(){try{current=await(await fetch('/status')).json();document.getElementById('status').textContent=JSON.stringify(current,null,2);enable();}catch(e){a.disabled=b.disabled=true;}}
async function send(command){pending=true;enable();try{const r=await fetch('/command',{method:'POST',headers:{'Content-Type':'application/json','X-Console-Token':'TOKEN'},body:JSON.stringify({command,request_id:current.request_id,schema_version:1,scene_confirmed:ack.checked})});if(!r.ok)throw Error(await r.text());ack.checked=false;document.getElementById('error').textContent='';}catch(e){document.getElementById('error').textContent=e.message;}finally{pending=false;await refresh();}}
a.onclick=()=>send('initialize');b.onclick=()=>send('pick');ack.onchange=enable;setInterval(refresh,700);refresh();
</script>"""


class CommissioningDriver(HardwarePickDriver):
    """Keep nominal placed-box collision proxies separate from cargo mapping."""
    release_touch_links = ("suction_panel", "suction_contact_frame")
    exclusive_trace = True

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
        self.journal.write(json.dumps(dict(time=time.time(), **event)) + "\n")
        self.journal.flush()
        os.fsync(self.journal.fileno())

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
        if (received is None or time.monotonic() - received > 2.0
                or ports.di0() != 0
                or self.driver._vacuum_state.get("vacuum_on", True)):
            raise RuntimeError("fresh empty tool / vacuum off not confirmed")

    def prepare(self):
        ready, reason = self.driver.wait_graph()
        if not ready:
            raise RuntimeError(reason)
        scene = self.driver._scene_client()
        if not scene.wait_ready(timeout_sec=5.0):
            raise RuntimeError("planning scene unavailable")
        self.execute([MotionSegment(name="initialize_observe", type="pose_target",
                                    target_pose=self.observe)])

    def pick_place(self, index):
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
    cli = parser.parse_args(argv)
    # The console runs on the Linux Jazzy laptop. Hold this lock until exit;
    # a second console must never submit independent hardware goals.
    import fcntl
    cell_lock = open("/tmp/roboticarm-hardware-loading.lock", "a")
    fcntl.flock(cell_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    raw_profile = Path(cli.site_profile).read_bytes()
    observe, cycles = parse_profile(json.loads(raw_profile))
    # Fixed execution contract: commissioning cannot disable scene/vacuum or
    # accidentally invoke a preview/pick-only path through forwarded CLI flags.
    args = build_argument_parser().parse_args([])
    args.skip_observe = True  # We return to the explicit TCP pose via MoveIt.
    args.place_target_data = None
    token, session = secrets.token_hex(32), secrets.token_hex(8)
    journal = open(cli.journal, "x", encoding="utf-8")
    rclpy.init()
    driver = CommissioningDriver(args)
    ports = HardwarePorts(driver, observe, cycles, journal, session)
    ports.record(dict(event="session", profile_sha256=hashlib.sha256(raw_profile).hexdigest(),
                      profile=json.loads(raw_profile), session=session))
    cycle = LoadingCycle(ports, len(cycles))
    driver.stage_callback = cycle.stage_changed
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
                self.reply(200, json.dumps(dict(cycle.snapshot(), profile_assumptions=
                    json.loads(raw_profile).get("assumptions", {}))))
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

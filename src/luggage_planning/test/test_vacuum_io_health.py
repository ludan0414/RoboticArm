"""IO fault injection without a robot or ROS."""
from concurrent.futures import Future
from types import SimpleNamespace
import unittest
from luggage_planning.vacuum_io_health import FreshDigitalInput, VacuumIoTransport
from luggage_planning.vacuum_backend import HardwareVacuumBackend

class Client:
    def __init__(self, ready=(True,), complete=True):
        self.ready = iter(ready)
        self.complete = complete
        self.calls = []
        self.future = Future()
    def wait_for_service(self, **kw):
        return next(self.ready)
    def call_async(self, request):
        self.calls.append(request)
        if self.complete:
            self.future.set_result(SimpleNamespace(success=True, message="ok"))
        return self.future

class IoTests(unittest.TestCase):
    def test_missing_stale_recovered_samples(self):
        clock = [0.0]
        value = FreshDigitalInput(lambda: clock[0], 2)
        self.assertIsNone(value.value())
        value.update(1)
        self.assertEqual(value.value(), 1)
        clock[0] = 3
        self.assertIsNone(value.value())
        value.update(0)
        self.assertEqual(value.value(), 0)

    def transport(self, client):
        return VacuumIoTransport({0:client}, bool, lambda: 0, lambda _:None,
                                 lambda _:None, call_timeout=.001)
    def test_service_rediscovery_writes_once(self):
        client = Client((False, True))
        self.assertTrue(self.transport(client).set_do(0, 1)[0])
        self.assertEqual(client.calls, [True])
    def test_unavailable_service_never_writes(self):
        client = Client((False, False))
        ok, message = self.transport(client).set_do(0, 1)
        self.assertFalse(ok)
        self.assertIn("UNAVAILABLE", message)
        self.assertEqual(client.calls, [])
    def test_discovery_exception_is_reported_without_write(self):
        client = Client()
        def fail(**kwargs): raise RuntimeError("disconnected")
        client.wait_for_service = fail
        ok, message = self.transport(client).set_do(0, 1)
        self.assertFalse(ok)
        self.assertIn("DISCOVERY_EXCEPTION", message)
        self.assertEqual(client.calls, [])

    def test_rpc_timeout_is_not_replayed(self):
        client = Client(complete=False)
        ok, message = self.transport(client).set_do(0, 1)
        self.assertFalse(ok)
        self.assertIn("outcome unknown", message)
        self.assertEqual(client.calls, [True])
        self.assertTrue(client.future.cancelled())
    def test_stale_di_does_not_erase_attachment_or_authorize_pump(self):
        commands = []
        io = SimpleNamespace(di0=lambda: None, set_do=lambda *v:commands.append(v))
        backend = HardwareVacuumBackend(io)
        backend._attached = True
        self.assertFalse(backend.follow_step(None,None)[0])
        self.assertTrue(backend.is_attached())
        self.assertFalse(backend.attach({})[0])
        self.assertEqual(commands, [])
    def test_stuck_high_release_fails_and_preserves_attachment(self):
        clock = [0.0]
        commands = []
        def sleep(dt): clock[0] += dt
        def write(*v): commands.append(v); return True, "ok"
        io = SimpleNamespace(di0=lambda: 1, set_do=write, now=lambda:clock[0], sleep=sleep)
        backend = HardwareVacuumBackend(io, release_timeout_sec=.1)
        backend._attached = True
        ok, message = backend.detach({})
        self.assertFalse(ok)
        self.assertIn("RELEASE_UNCONFIRMED", message)
        self.assertTrue(backend.is_attached())
        self.assertEqual(commands[-1], (1, 0))

class ControllerTests(unittest.TestCase):
    """Execute unchanged ROS handler methods with injected transport/backend."""
    def setUp(self):
        import ast, threading, time
        from pathlib import Path
        tree = ast.parse((Path(__file__).resolve().parents[1] / "scripts/vacuum_controller_node.py").read_text())
        original = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "VacuumControllerNode")
        original.bases = []
        original.body = [n for n in original.body if isinstance(n, ast.FunctionDef) and n.name in
                         ("_handle_command", "_perform_command", "_follow_tick")]
        namespace = {"time":time, "HardwareVacuumBackend":HardwareVacuumBackend}
        exec(compile(ast.fix_missing_locations(ast.Module(body=[original], type_ignores=[])), "controller_methods", "exec"), namespace)
        self.node = namespace["VacuumControllerNode"]()
        self.node._command_lock = threading.Lock()
        self.node._publish_state = lambda: None
        self.node._publish_event = lambda *args: None
        self.node.get_logger = lambda: SimpleNamespace(info=lambda *a:None, warning=lambda *a:None)
        self.node._vacuum_on = True
        self.node._last_fail_reason = ""
        self.node._backend = HardwareVacuumBackend(SimpleNamespace(di0=lambda:0))
    def command(self, enable):
        return self.node._handle_command(SimpleNamespace(enable=enable), SimpleNamespace(success=False,message=""))
    def test_busy_command_never_reaches_backend(self):
        self.node._command_lock.acquire()
        response = self.command(False)
        self.assertFalse(response.success)
        self.assertIn("BUSY", response.message)
    def test_failed_release_does_not_advertise_pump_off(self):
        self.node._backend.detach = lambda context:(False, "transport failed")
        self.assertFalse(self.command(False).success)
        self.assertTrue(self.node._vacuum_on)
        self.assertFalse(self.node._command_lock.locked())
    def test_successful_release_clears_pump_state(self):
        self.node._backend.detach = lambda context:(True, "ok")
        self.assertTrue(self.command(False).success)
        self.assertFalse(self.node._vacuum_on)
    def test_stale_idle_io_recovers_without_motion(self):
        self.node._vacuum_on = False
        self.node._backend._io.di0 = lambda:None
        self.node._follow_tick()
        self.assertIn("IO_STALE", self.node._last_fail_reason)
        self.node._backend._io.di0 = lambda:0
        self.node._follow_tick()
        self.assertEqual(self.node._last_fail_reason, "")

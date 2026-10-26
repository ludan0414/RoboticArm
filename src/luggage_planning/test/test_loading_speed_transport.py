"""Execute console ROS service adapter and planner callback against fake transport."""
import ast
from pathlib import Path
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

from test_hardware_loading_ports import load_console
from luggage_planning.trajectory_smoothing import validate_speed


class SpeedTransportTests(unittest.TestCase):
    def test_guard_failure_still_captures_complete_continuous_request(self):
        from luggage_planning.pose import MotionSegment, Pose, Point
        path = Path(__file__).resolve().parents[1] / 'scripts/hardware_pick_driver.py'
        tree = ast.parse(path.read_text(encoding='utf-8'))
        owner = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'RosPickSessionPorts')
        method = next(n for n in owner.body if isinstance(n, ast.FunctionDef) and n.name == 'execute_segment')
        scope = dict(SegmentOutcome=SimpleNamespace)
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), scope)
        events = []
        ports = SimpleNamespace(log_event=events.append,
            _driver=SimpleNamespace(segment_guard=lambda _: (False, 'blocked via')))
        segment = MotionSegment(name='pre_place', type='cartesian',
            waypoints=[Pose(Point(1,0,1)), Pose(Point(2,0,1))], target_pose=Pose(Point(3,0,1)))
        self.assertFalse(scope[method.name](ports, segment, 'place').ok)
        self.assertEqual([p['position']['x'] for p in events[0]['poses']], [1,2,3])

    def test_apply_uses_atomic_parameter_write_then_typed_readback(self):
        module = load_console()
        driver = module.CommissioningDriver()
        driver._args = SimpleNamespace(plan_timeout=90.)
        sent, removed = [], []
        responses = [SimpleNamespace(result=SimpleNamespace(successful=True)),
                     SimpleNamespace(values=[SimpleNamespace(type=3, double_value=.35)])]

        def create(service, name):
            def call(request):
                sent.append((name, request))
                response = responses.pop(0)
                return SimpleNamespace(add_done_callback=lambda cb: cb(None), result=lambda: response)
            return SimpleNamespace(wait_for_service=lambda **kwargs: True, call_async=call)

        driver.create_client, driver.destroy_client = create, removed.append
        services, messages = ModuleType('rcl_interfaces.srv'), ModuleType('rcl_interfaces.msg')
        services.SetParametersAtomically = services.GetParameters = SimpleNamespace(Request=SimpleNamespace)
        messages.Parameter = messages.ParameterValue = SimpleNamespace
        messages.ParameterType = SimpleNamespace(PARAMETER_DOUBLE=3)
        with patch.dict('sys.modules', {'rcl_interfaces.srv': services, 'rcl_interfaces.msg': messages}):
            self.assertEqual(driver.set_motion_speed(.35), .35)
        self.assertEqual([name for name, _ in sent], ['/motion_planner/set_parameters_atomically', '/motion_planner/get_parameters'])
        self.assertEqual(sent[0][1].parameters[0].name, 'speed_scale')
        self.assertEqual(sent[0][1].parameters[0].value.double_value, .35)
        self.assertEqual(sent[1][1].names, ['speed_scale'])
        self.assertEqual(len(removed), 2)
        self.assertAlmostEqual(driver._args.plan_timeout, 90/.35)

    def test_real_parameter_callback_rejects_nonfinite_and_out_of_range(self):
        path = Path(__file__).resolve().parents[1] / 'scripts/motion_planner_node.py'
        tree = ast.parse(path.read_text(encoding='utf-8'))
        owner = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'MotionPlannerNode')
        method = next(n for n in owner.body if isinstance(n, ast.FunctionDef) and n.name == '_validate_speed_parameters')
        scope = dict(validate_speed=validate_speed, SetParametersResult=SimpleNamespace)
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), scope)
        callback = scope[method.name]
        for value in (.09, 1.01, float('nan'), 1, True, '0.5'):
            self.assertFalse(callback(None, [SimpleNamespace(name='speed_scale', value=value)]).successful)
        self.assertTrue(callback(None, [SimpleNamespace(name='speed_scale', value=.35)]).successful)


if __name__ == '__main__':
    unittest.main()

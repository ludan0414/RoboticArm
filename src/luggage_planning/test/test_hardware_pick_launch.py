"""Static checks for the site hardware_pick launch (no ROS graph)."""

import unittest
from pathlib import Path

LAUNCH = Path(__file__).resolve().parents[1] / "launch" / "hardware_pick.launch.py"


class HardwarePickLaunchTest(unittest.TestCase):
    def test_profile_b_is_site_yaml_not_replay(self):
        src = LAUNCH.read_text(encoding="utf-8")
        self.assertIn("preprocessor_d555_site.yaml", src)
        self.assertIn("preprocessor_d555_live.yaml", src)
        self.assertIn(
            'site_pp = os.path.join(perc_share, "config", "preprocessor_d555_site.yaml")',
            src,
        )
        self.assertIn("Do not use ", src)
        self.assertIn("preprocessor_d555_replay.yaml", src)
        self.assertIn("default_value=site_pp", src)
        self.assertIn('"use_compressed"', src)
        self.assertIn('"max_rate_hz"', src)
        self.assertIn('default_value="0.0"', src)
        self.assertNotIn('"max_rate_hz": 2.0', src)
        self.assertIn('default_value="false"', src)
        self.assertIn("input.use_compressed", src)
        self.assertIn('"/compressed"', src)
        self.assertIn('"cloud_max_age_sec": 30.0', src)
        self.assertNotIn(
            'os.path.join(exec_share, "config", "preprocessor_d555_replay.yaml")',
            src,
        )

    def test_site_launch_enables_semantic_overlay(self):
        src = LAUNCH.read_text(encoding="utf-8")
        self.assertIn('DeclareLaunchArgument(\n                "publish_overlay"', src)
        self.assertIn('default_value="true"', src)
        self.assertIn(
            '"publish_overlay": ParameterValue(',
            src,
        )
        self.assertIn(
            'LaunchConfiguration("publish_overlay")',
            src,
        )

    def test_role_switches_gate_perception_and_planning_nodes(self):
        src = LAUNCH.read_text(encoding="utf-8")
        self.assertIn('"start_perception"', src)
        self.assertIn('"start_planning"', src)
        self.assertIn('start_perception = IfCondition(', src)
        self.assertIn('start_planning = IfCondition(', src)
        for name in (
            "semantic_segmenter",
            "semantic_point_filter",
            "luggage_detector",
        ):
            self.assertRegex(
                src,
                r'(?s)name="%s".*?condition=start_perception' % name,
            )
        self.assertIn('name="sensor_preprocessor"', src)
        self.assertIn('LaunchConfiguration("start_perception").perform(context)', src)
        self.assertIn("def _preprocessor_node", src)
        for name in (
            "scene_manager",
            "waypoint_generator",
            "motion_planner",
            "vacuum_controller",
        ):
            self.assertRegex(
                src,
                r'(?s)name="%s".*?condition=start_planning' % name,
            )
        self.assertIn(
            'LaunchConfiguration("start_planning").perform(context)', src)
        self.assertIn("if not _bool_text", src)

    def test_low_level_owners_remain_independent_switches(self):
        src = LAUNCH.read_text(encoding="utf-8")
        for arg in ("start_d555", "start_executor", "start_scene", "use_moveit"):
            self.assertIn('"%s"' % arg, src)
        self.assertIn("condition=start_d555", src)
        self.assertIn("condition=start_executor", src)
        self.assertIn("condition=start_scene", src)

    def test_driver_polls_readiness_and_defaults_observe_current(self):
        driver = (
            Path(__file__).resolve().parents[1]
            / "scripts" / "hardware_pick_driver.py"
        )
        src = driver.read_text(encoding="utf-8")
        self.assertIn("PickSession", src)
        self.assertIn(
            'parser.add_argument(\n        "--observe-pose", default="current"',
            src,
        )
        self.assertIn('"--detect-timeout", type=float, default=40.0', src)
        self.assertIn("service_is_ready", src)
        self.assertIn("server_is_ready", src)
        self.assertIn("waiting for %s", src)
        self.assertIn("def _wait_ready", src)
        self.assertNotIn("wait_for_service(timeout_sec=remain)", src)
        self.assertNotIn("wait_for_server(timeout_sec=remain)", src)

    def test_site_launch_uses_site_poses_and_current_observe(self):
        src = LAUNCH.read_text(encoding="utf-8")
        self.assertIn("robot_poses.site.yaml", src)
        self.assertIn('default_value="current"', src)
        self.assertIn('"observe_pose_name"', src)
        self.assertIn('"default_observe_pose"', src)
        self.assertIn(
            'poses = os.path.join(desc_share, "config", "robot_poses.site.yaml")',
            src,
        )
        self.assertIn("sim_poses =", src)

    def test_execution_backend_defaults_servo_j_and_forwards_settings(self):
        src = LAUNCH.read_text(encoding="utf-8")
        self.assertIn('"execution_backend"', src)
        self.assertIn('default_value="servo_j"', src)
        self.assertIn('"servo_j_servo_time"', src)
        self.assertIn('"servo_j_lookahead_time"', src)
        self.assertIn(
            '"execution_backend": LaunchConfiguration("execution_backend")',
            src,
        )
        self.assertIn(
            '"servo_j_servo_time": LaunchConfiguration("servo_j_servo_time")',
            src,
        )


class MotionExecutorIkFallbackTest(unittest.TestCase):
    def test_pose_target_fails_closed_when_ik_misses(self):
        src = (
            Path(__file__).resolve().parents[1]
            / "luggage_planning" / "motion_executor.py"
        ).read_text(encoding="utf-8")
        self.assertIn("no pose-constraint fallback", src)
        self.assertNotIn('note = "pose constraint fallback"', src)

    def test_cartesian_and_joint_goals_use_rest_to_rest_not_totg(self):
        src = (
            Path(__file__).resolve().parents[1]
            / "luggage_planning" / "motion_executor.py"
        ).read_text(encoding="utf-8")
        self.assertIn("apply_rest_to_rest_to_joint_trajectory", src)
        self.assertIn("time_parameterize_cartesian", src)
        self.assertIn("goal.planning_options.plan_only = True", src)
        self.assertNotIn("goal.planning_options.plan_only = False", src)
        self.assertNotIn("request.max_velocity_scaling_factor", src)
        self.assertNotIn("request.max_acceleration_scaling_factor", src)
        self.assertIn("point.velocities = list(vel)", src)
        self.assertIn("point.accelerations = list(acc)", src)
        self.assertNotIn("first and last steps double as a", src)
        self.assertIn("_execute_robot_trajectory", src)


if __name__ == "__main__":
    unittest.main()

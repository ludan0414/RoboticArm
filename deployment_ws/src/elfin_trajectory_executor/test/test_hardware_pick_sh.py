"""实机代码 (site / real-cell). Not part of the Gazebo simulation stack."""

import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "hardware_pick.sh"
ENV_SH = ROOT / "scripts" / "env_jazzy_real.sh"


class HardwarePickShTest(unittest.TestCase):
    def test_script_exists(self):
        self.assertTrue(SCRIPT.is_file(), SCRIPT)

    def test_no_gpu_defaults_semantic_device_cpu(self):
        src = SCRIPT.read_text(encoding="utf-8")
        self.assertIn("_has_nvidia_gpu", src)
        self.assertIn('launch_args+=("semantic_device:=cpu")', src)
        self.assertIn("nvidia-smi", src)

    def test_comments_point_at_site_profile_b_not_live_replay(self):
        src = SCRIPT.read_text(encoding="utf-8")
        self.assertIn("preprocessor_d555_site.yaml", src)
        self.assertIn("Do not pass preprocessor_d555_replay.yaml", src)
        self.assertIn("Profile B (default)", src)
        self.assertIn("use_compressed:=false", src)
        self.assertIn("use_compressed:=true", src)

    def test_driver_flags_are_not_forwarded_to_launch(self):
        src = SCRIPT.read_text(encoding="utf-8")
        self.assertIn("hardware_pick_driver.py flag", src)
        self.assertIn("skip-observe:=*", src)
        self.assertIn("detect-only:=*", src)
        self.assertIn("observe-pose", src)

    def test_waypoint_default_is_unchanged(self):
        src = SCRIPT.read_text(encoding="utf-8")
        supervisor = (ROOT / "scripts" / "hardware_pick_supervisor.py").read_text(
            encoding="utf-8")
        self.assertIn("hardware_pick.launch.py", supervisor)
        self.assertNotIn('launch_args+=("execution_backend:=servo_j")', src)
        self.assertNotIn("exec ros2 launch luggage_perception", src)

    def test_starts_via_supervisor(self):
        src = SCRIPT.read_text(encoding="utf-8")
        self.assertIn("hardware_pick_supervisor.py", src)
        self.assertIn("exec python3", src)
        self.assertNotIn("stop_hardware_pick.sh", src)


class HardwarePickServoJShTest(unittest.TestCase):
    SCRIPT = ROOT / "scripts" / "hardware_pick_servo_j.sh"

    def test_injects_servo_j_and_leaves_sensors_to_orin(self):
        src = self.SCRIPT.read_text(encoding="utf-8")
        self.assertTrue(self.SCRIPT.is_file(), self.SCRIPT)
        self.assertIn("execution_backend:=servo_j", src)
        self.assertIn("start_d555:=false", src)
        self.assertIn("start_perception:=false", src)
        self.assertIn("servo_esj is rejected", src)
        self.assertIn("hardware_pick.sh", src)
        self.assertIn("/luggage/semantic/yolo_detections", src)
        self.assertIn("/luggage_detector/detect_luggage", src)
        self.assertIn("/livox/lidar", src)
        self.assertIn("/camera/d555/", src)

    def test_does_not_replace_hardware_pick_sh(self):
        src = self.SCRIPT.read_text(encoding="utf-8")
        self.assertIn('PICK="$DEPLOY/scripts/hardware_pick.sh"', src)
        self.assertNotIn("execution_backend:=waypoint", src)


class PerceptionSiteShWrapperTest(unittest.TestCase):
    def test_wrapper_execs_luggage_perception_script(self):
        script = ROOT / "scripts" / "perception_site.sh"
        src = script.read_text(encoding="utf-8")
        self.assertTrue(script.is_file(), script)
        self.assertIn("src/luggage_perception/scripts/perception_site.sh", src)
        self.assertIn('exec "$SRC"', src)


class RunFloorBoxPickShTest(unittest.TestCase):
    SCRIPT = ROOT / "scripts" / "run_floor_box_pick.sh"

    def test_defaults_observe_pose_current(self):
        src = self.SCRIPT.read_text(encoding="utf-8")
        self.assertIn('OBSERVE_POSE="${OBSERVE_POSE:-current}"', src)
        self.assertIn('--observe-pose "${OBSERVE_POSE}"', src)
        self.assertNotIn("--skip-observe --plan-only", src)

    def test_tells_operator_pick_graph_is_required(self):
        src = self.SCRIPT.read_text(encoding="utf-8")
        self.assertTrue(self.SCRIPT.is_file(), self.SCRIPT)
        self.assertIn("hardware_pick.sh or hardware_pick_servo_j.sh", src)
        self.assertIn("record_site.sh is sensors+executor+bag only", src)


class RecordFloorBoxPickShTest(unittest.TestCase):
    def test_records_bag_and_snapshots_startup_pose(self):
        script = ROOT / "scripts" / "record_floor_box_pick.sh"
        src = script.read_text(encoding="utf-8")
        self.assertIn('OBSERVE_POSE="${OBSERVE_POSE:-current}"', src)
        self.assertIn("record_all_topics.sh", src)
        self.assertIn("dump_observe_pose.sh", src)
        self.assertIn("dump_observe_pose.sh", src)
        self.assertNotIn("--observe-snapshot", src)
        self.assertIn("ROS_DOMAIN_ID", src)


class DumpObservePoseShTest(unittest.TestCase):
    def test_script_uses_named_robot_poses(self):
        script = ROOT / "scripts" / "dump_observe_pose.sh"
        src = script.read_text(encoding="utf-8")
        self.assertIn("format_pose_yaml", src)
        self.assertIn("pickup_observe", src)


class EnvJazzyRealShTest(unittest.TestCase):
    def test_overlays_repo_then_livox_then_deploy(self):
        src = ENV_SH.read_text(encoding="utf-8")
        self.assertIn("install/local_setup.bash", src)
        self.assertIn("_HUMBLE_WS", src)
        self.assertIn("livox_ws/install/local_setup.bash", src)
        self.assertIn("${_DEPLOY}/install/local_setup.bash", src)
        self.assertIn("COLCON_", src)
        self.assertIn("_ENV_HAD_NOUNSET", src)
        self.assertIn('ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-7}"', src)
        humble = src.find("_HUMBLE_WS")
        livox = src.find("livox_ws/install/local_setup.bash")
        deploy = src.find("${_DEPLOY}/install/local_setup.bash")
        self.assertLess(humble, livox)
        self.assertLess(livox, deploy)


class RecordPerceptionCaseShTest(unittest.TestCase):
    SCRIPT = ROOT / "scripts" / "record_perception_case.sh"

    def test_starts_raw_perception_graph_and_records_raw_images(self):
        src = self.SCRIPT.read_text(encoding="utf-8")
        self.assertTrue(self.SCRIPT.is_file(), self.SCRIPT)
        self.assertIn("compress:=false", src)
        self.assertIn("publish_raw:=true", src)
        self.assertIn("use_compressed:=false", src)
        self.assertIn("publish_overlay:=false", src)
        self.assertIn("max_rate_hz:=0.0", src)
        self.assertIn("start_d555:=false", src)
        self.assertIn("/camera/d555/color/image_raw$", src)
        self.assertNotIn("/camera/d555/color/image_raw/compressed$", src)
        self.assertIn("hardware_pick.sh", src)
        self.assertNotIn("hardware_pick_driver.py", src)


class SyncOrinClockShTest(unittest.TestCase):
    SCRIPT = ROOT / "scripts" / "sync_orin_clock.sh"

    def test_defaults_to_documented_orin_and_check_mode(self):
        src = self.SCRIPT.read_text(encoding="utf-8")
        self.assertTrue(self.SCRIPT.is_file(), self.SCRIPT)
        self.assertTrue(self.SCRIPT.stat().st_mode & 0o111)
        self.assertIn("hku_reconova@192.168.0.100", src)
        self.assertIn("--check", src)
        self.assertIn("date -s @", src)
        self.assertIn("BatchMode=yes", src)
        self.assertIn("ORIN_SSH_PASSWORD", src)
        self.assertIn("PreferredAuthentications=password", src)
        self.assertNotIn("pkill", src)
        self.assertNotRegex(src, r'ORIN_SSH_PASSWORD="1"')


class RecordSiteShTest(unittest.TestCase):
    SCRIPT = ROOT / "scripts" / "record_site.sh"

    def test_forwards_execution_backend(self):
        src = self.SCRIPT.read_text(encoding="utf-8")
        self.assertTrue(self.SCRIPT.is_file(), self.SCRIPT)
        self.assertIn("execution_backend:=servo_j", src)
        self.assertIn('if [[ "$1" == *:=* ]]; then', src)
        self.assertIn("LAUNCH_EXTRA+=", src)
        self.assertIn("servo_esj is rejected", src)


if __name__ == "__main__":
    unittest.main()

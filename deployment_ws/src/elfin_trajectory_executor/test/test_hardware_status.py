"""实机代码 (site / real-cell). Not part of the Gazebo simulation stack."""

import unittest

from elfin_trajectory_executor.hardware_status import (
    HARDWARE_STATUS_TOPIC,
    SCHEMA,
    build_snapshot,
    fsm_label,
    fsm_severity,
    hrif_hint,
    make_hrif_event,
    should_keep_event,
)
from elfin_trajectory_executor.huayan_interface import (
    ConnectionState,
    HuayanInterface,
)


class HardwareStatusTest(unittest.TestCase):
    def test_topic_and_fsm_labels(self):
        self.assertEqual(HARDWARE_STATUS_TOPIC, "/elfin/hardware_status")
        self.assertEqual(fsm_label(33), "待机 StandBy")
        self.assertEqual(fsm_severity(33), "ok")
        self.assertEqual(fsm_severity(21), "fault")
        self.assertEqual(fsm_severity(25), "moving")
        self.assertEqual(fsm_severity(34), "warn")
        self.assertIn("ScriptRunning", fsm_label(34))
        self.assertIn("未初始化", hrif_hint(40071))
        self.assertIn("StandBy", hrif_hint(40025))

    def test_snapshot_marks_standby_ready_for_servo(self):
        snap = build_snapshot(
            connection="READY",
            fsm_id=33,
            fsm_desc="RobotStandBy",
            last_hrif=make_hrif_event("InitServoEsJ", 0),
            events=[],
        )
        self.assertEqual(snap["schema"], SCHEMA)
        self.assertTrue(snap["ready_for_servo"])
        self.assertEqual(snap["fsm_severity"], "ok")

    def test_should_keep_init_and_faults_not_every_push(self):
        self.assertTrue(should_keep_event("InitServoEsJ", 0))
        self.assertTrue(should_keep_event("PushServoJ", 40071))
        self.assertFalse(should_keep_event("PushServoJ", 0))

    def test_historical_fault_does_not_mark_current_standby_faulted(self):
        snap = build_snapshot(connection="READY", fsm_id=33,
            last_fault=make_hrif_event("InitServoEsJ", 40025))
        self.assertFalse(snap["current_fault"])
        self.assertEqual(snap["last_fault"]["code"], 40025)

    def test_faulted_application_still_reads_fresh_controller_state(self):
        iface = HuayanInterface.__new__(HuayanInterface)
        iface._ensure_hw()
        iface._last_fsm_id, iface._last_fsm_desc = 33, "cached"
        iface._state = ConnectionState.ERROR
        iface._state_lock = __import__("threading").Lock()
        iface._read_fsm = lambda: (25, "Moving")
        self.assertEqual(iface.hardware_status(refresh_fsm=True)["fsm_id"], 25)
        iface._read_fsm = lambda: (None, None)
        self.assertIsNone(iface.hardware_status(refresh_fsm=True)["fsm_id"])

    def test_interface_records_40071_for_the_console(self):
        iface = HuayanInterface.__new__(HuayanInterface)
        iface.last_error_string = ""
        iface._state = ConnectionState.EXECUTING
        iface._state_lock = __import__("threading").Lock()
        iface._current_positions_deg = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        iface._note_hrif("PushServoJ", 40071, "ServoEsJ not initialized", extra={"index": 0})
        snap = iface.hardware_status()
        self.assertTrue(snap["available"])
        self.assertEqual(snap["last_hrif"]["code"], 40071)
        self.assertEqual(snap["last_fault"]["code"], 40071)
        self.assertEqual(snap["last_hrif"]["name"], "PushServoJ")
        self.assertIn("40071", snap["last_error_string"])
        self.assertEqual(snap["events"][-1]["code"], 40071)
        self.assertEqual(snap["joints_deg"][0], 1.0)


if __name__ == "__main__":
    unittest.main()

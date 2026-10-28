import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch

from luggage_planning.telemetry_journal import TelemetryJournal


class TelemetryJournalTests(unittest.TestCase):
    def test_parallel_samples_are_complete_and_sessions_never_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'loading_088.jsonl'
            writer = TelemetryJournal(path, 'first')
            workers = [threading.Thread(target=lambda: [writer.write('/joint_states',
                dict(position=[.1]*6, velocity=[.2]*6, effort=[.3]*6)) for _ in range(25)])
                       for _ in range(4)]
            for worker in workers: worker.start()
            for worker in workers: worker.join()
            writer.close()
            rows = [json.loads(line) for line in writer.path.read_text().splitlines()]
            samples = [row for row in rows if row['topic']=='/joint_states']
            self.assertEqual(len(samples), 100)
            self.assertTrue(all(row['monotonic_sec'] > 0 and row['data']['effort']==[.3]*6 for row in samples))
            summary = json.loads(writer.path.with_suffix('.summary.json').read_text())
            self.assertEqual(summary['topics']['/joint_states']['count'], 100)
            self.assertIn('/elfin/cps_telemetry', summary['missing'])
            original = writer.path.read_bytes()
            second = TelemetryJournal(path, 'second')
            second.close()
            self.assertNotEqual(writer.path, second.path)
            self.assertEqual(writer.path.read_bytes(), original)

    def test_fault_is_flushed_immediately_and_disk_failure_is_visible(self):
        with tempfile.TemporaryDirectory() as directory:
            writer = TelemetryJournal(Path(directory)/'case', 'session')
            writer.write('/joint_states', dict(position=[1.]*6))
            self.assertTrue(writer.write('/rosout', dict(level=40, msg='RobotCollisionStop')))
            self.assertIn('RobotCollisionStop', writer.path.read_text())
            with patch.object(writer.file, 'write', side_effect=OSError('disk full')):
                self.assertFalse(writer.write('/joint_states', {}))
            self.assertIn('disk full', writer.status()['error'])
            self.assertFalse(writer.status()['active'])
            writer.close()

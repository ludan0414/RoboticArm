"""Ensure the eight-case intent draft cannot be loaded as a live profile."""
import json
from pathlib import Path
import unittest
from luggage_planning.hardware_loading_cycle import parse_profile

class ReviewOnlyTest(unittest.TestCase):
    def test_draft_is_not_accepted_by_console(self):
        root=Path(__file__).resolve().parents[3]
        draft=json.loads((root/'deployment_ws/config/loading_cycle.measured8.review.json').read_text(encoding='utf-8'))
        self.assertEqual(draft['kind'],'review_only_not_a_hardware_profile')
        self.assertIs(draft['deployable'],False)
        self.assertEqual(len(draft['cases']),8)
        self.assertEqual(len({c['box_id'] for c in draft['cases']}),8)
        with self.assertRaises(ValueError):parse_profile(draft)

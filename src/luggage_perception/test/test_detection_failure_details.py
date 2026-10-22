import unittest
from luggage_perception.detection_failure_details import format_detection_failure


class FailureDetailTests(unittest.TestCase):
    def test_coverage_retains_actual_measurement(self):
        details = dict(reason='DETECT_COMPONENT_COVERAGE_LOW', coverage=.12,
                       valid_pixels=120, region_pixels=1000, minimum_coverage=.35)
        text = format_detection_failure(details['reason'], component=details)
        self.assertIn('"coverage": 0.12', text)
        self.assertIn('"minimum_coverage": 0.35', text)
        self.assertIn('"valid_pixels": 120', text)

    def test_old_component_does_not_label_another_failure(self):
        self.assertEqual(format_detection_failure('DETECT_TF_FAILED',
                         component=dict(reason='DETECT_COMPONENT_COVERAGE_LOW')),
                         'DETECT_TF_FAILED')

    def test_stale_keeps_age(self):
        self.assertEqual(format_detection_failure('DETECT_STALE_CLOUD',
                         cloud_age=12.3, cloud_max_age=2),
                         'DETECT_STALE_CLOUD (age=12.30s max=2.0s)')

    def test_plain_reason_and_plane_detail(self):
        self.assertEqual(format_detection_failure('DETECT_FAILED'), 'DETECT_FAILED')
        self.assertEqual(format_detection_failure('NO_PLANE', plane_detail='tilt=60'),
                         'NO_PLANE: tilt=60')

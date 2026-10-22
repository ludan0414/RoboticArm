"""Bounded, ROS-free failure text; keep structured reason codes unchanged."""
import json


def format_detection_failure(reason, component=None, plane_detail='',
                             cloud_age=None, cloud_max_age=None):
    reason = str(reason)
    if reason == 'DETECT_STALE_CLOUD' and cloud_age is not None:
        return '%s (age=%.2fs max=%.1fs)' % (reason, cloud_age, cloud_max_age)
    if component and component.get('reason') == reason:
        return reason + ': ' + json.dumps(component, sort_keys=True)
    if plane_detail:
        return reason + ': ' + str(plane_detail)
    return reason

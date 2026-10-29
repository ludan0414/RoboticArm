"""Integrity link between offline validation and frozen placement requests."""
import hashlib
import json

PROFILE_TYPE='site_offline_frozen_v1'
FROZEN_CARTESIAN='offline_frozen_cartesian'


def cycles_digest(cycles):
    return hashlib.sha256(json.dumps(cycles,sort_keys=True,separators=(',',':'),
                                    allow_nan=False).encode()).hexdigest()


def request_digest(profile):
    return cycles_digest({key:profile.get(key) for key in (
        'profile','cycles','frame_id','tool_frame','motion_safety','doorway_clearance','carry_entry')})


def validation_stamp(profile,scope):
    from .manual_carry_transfer import TRANSFER_JOINTS_RAD
    return dict(success=True,cycles_sha256=cycles_digest(profile['cycles']),
        request_sha256=request_digest(profile),scope=scope,hardware_motion_validated=False,
        transfer_joint_q=[list(q) for q in TRANSFER_JOINTS_RAD])


def require_validation(profile):
    if not any(row.get('place_target',{}).get('route_source')=='offline_frozen_v1'
               for row in profile.get('cycles',())):
        return
    if profile.get('profile')!=PROFILE_TYPE:
        raise ValueError('frozen routes require the versioned offline profile format')
    result=profile.get('offline_validation',{})
    from .manual_carry_transfer import TRANSFER_JOINTS_RAD
    if (result.get('success') is not True or
            result.get('cycles_sha256')!=cycles_digest(profile['cycles']) or
            result.get('request_sha256')!=request_digest(profile) or
            result.get('transfer_joint_q')!=[list(q) for q in TRANSFER_JOINTS_RAD]):
        raise ValueError('frozen profile is unvalidated or changed after validation')

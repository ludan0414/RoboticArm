"""ROS-free controller wrist-branch contract, shared with offline planning."""
import math


def measure_wrist(points,margin_rad=.25,joint_index=4,joint_names=()):
    rows=[list(getattr(p,'positions',p)) for p in (points or [])]
    track=[float(row[joint_index]) for row in rows if len(row)>joint_index]
    if len(track)<2:return None
    margin=min(abs(v-round(v/math.pi)*math.pi) for v in track)
    crosses=any(math.floor(a/math.pi)!=math.floor(b/math.pi) for a,b in zip(track,track[1:]))
    return dict(joint=joint_names[joint_index] if joint_index<len(joint_names) else 'joint%d'%joint_index,
        margin_rad=round(margin,4),margin_limit_rad=float(margin_rad),
        delta_rad=round(abs(track[-1]-track[0]),4),crosses_branch=crosses,
        ok=not crosses and margin>=float(margin_rad))

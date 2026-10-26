"""Load the shared ROS-free description contract in standalone Newton scripts."""
from dataclasses import replace
from pathlib import Path
import sys

description_source=str(Path(__file__).resolve().parents[1]/'src/luggage_description')
if description_source not in sys.path:
    sys.path.insert(0,description_source)
from luggage_description.doorway_clearance import DoorwayGeometry,ALIGNMENT_SLACK_M


def clearance_hull(hull, scene):
    if float(scene['container'].get('opening',{}).get('clearance_m',0.0))==0:
        return hull
    doorway=DoorwayGeometry.from_scene(scene)
    if doorway.clearance==0:
        return hull
    # Target slack is larger than Newton's1mm penetration tolerance; the
    # collision guard itself is inflated by that tolerance as well.
    margin=doorway.clearance+ALIGNMENT_SLACK_M+.001
    return replace(hull,door_y0=hull.door_y0+margin,door_y1=hull.door_y1-margin,
                   door_z0=hull.door_z0+margin,door_z1=hull.door_z1-margin)

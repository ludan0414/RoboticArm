"""Operator's photographed home, in controller J1..J6 order.

Hardcoded by explicit site instruction. Keep the signed angles (including
J1 > 180 degrees); do not convert this target through TCP/IK or wrap it to
a different turn. MotionExecutor still plans and checks the whole path.
"""
import math

HOME_JOINTS_DEG = (188.299, -135.510, -95.754, -36.132, 91.924, 12.500)
HOME_JOINTS_RAD = tuple(math.radians(angle) for angle in HOME_JOINTS_DEG)
HOME_SEGMENT_TYPE = 'commissioning_home'

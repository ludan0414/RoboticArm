# Continue after reach boundary

Base 3e0548e. Evidence is loading_114 box006 and telemetry session 7ef315b484e4c1a7. The last attempt projected waypoint 27/29 about 5.4 cm short, then waypoint 28/29 returned IK -31; its target requested a 2.3 cm descent at the still-unreachable XY. The final target repeated that XY and height. P1/P2 and grasp had completed. This is evidence of the failed replan sequence, not proof of the proposed route on hardware.

After one successful intermediate projection, subsequent exact IK failures on intermediate waypoints do not repeatedly project toward the edge. For a lower intermediate waypoint, the executor tries a fully planned exact pose at the achieved XY and requested height; otherwise it skips only numerical -31 waypoints. It then tries later targets from the new endpoint. Execution, cancellation and other errors stop. The final endpoint remains exact and the return trace records only achieved poses, so release is not authorized by a shortened final target.

Focused behavioural tests cover continuing after a projection, lowering before retrying the exact final target, failures at the final target and other stop paths. No robot movement was commanded and no live IK replay was performed.

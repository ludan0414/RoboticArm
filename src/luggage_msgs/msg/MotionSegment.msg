# One leg of a pick/place motion sequence (input to motion_planner).
string name
string type
geometry_msgs/Pose target_pose
geometry_msgs/Pose[] waypoints
# Enforce tool-Z-axis-down OrientationConstraint during planning (heavy payload transit).
bool keep_tool_down
# Enforce camera optical +Z toward world -Z (keep camera looking straight down).
bool keep_camera_down
# Lock J4/J5/J6 near current angles via JointConstraints to prevent wrist spin.
bool lock_wrist
# Legacy hint for alternate planning after explicit geometry/wrist rejection.
# Incomplete Cartesian coverage automatically replans through all route poses
# regardless of this flag; no partial prefix is executed.
bool allow_ompl_fallback
# Legacy coverage hint retained for wire compatibility and probe diagnostics.
# Runtime incomplete paths now trigger complete pose-goal recovery instead
# of fraction-based rejection. Release still requires final target success.
float64 required_cartesian_fraction

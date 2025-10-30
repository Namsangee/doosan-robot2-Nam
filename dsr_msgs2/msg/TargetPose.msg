# TargetPose.msg
# -------------------------------------------------
# Position + Orientation target for MoveIt pose goal
# Used by: /target_pose topic

float64 x               # Target X position [m]
float64 y               # Target Y position [m]
float64 z               # Target Z position [m]

# Option 1: Use orientation_id to choose from predefined quaternion table
int32 orientation_id    # 1~10, defined in ORIENTATION_TABLE

# Option 2 (optional): Direct quaternion (w,x,y,z)
# If orientation_id == 0, these values are used instead.
float64 qx
float64 qy
float64 qz
float64 qw

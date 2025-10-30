# TargetJoint.msg
# -------------------------------------------------
# Target joint command (in degrees)
# Used by: /target_joint topic

float64[6] joint_deg     # 6 joint angles [degrees]
string group_name        # Optional MoveIt group (default: manipulator)

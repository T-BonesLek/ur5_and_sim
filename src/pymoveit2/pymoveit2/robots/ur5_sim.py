from typing import List

MOVE_GROUP_ARM: str = "arm"

def joint_names(prefix: str = "") -> List[str]:
    return [        
        prefix + "shoulder_lift_joint",
        prefix + "elbow_joint",
        prefix + "wrist_1_joint",
        prefix + "wrist_2_joint",
        prefix + "wrist_3_joint",
        prefix + "shoulder_pan_joint",

    ]


def base_link_name(prefix: str = "") -> str:
    return prefix + "base_link"


def end_effector_name(prefix: str = "") -> str:
    return prefix + "wrist_3_link" #Change to gripper or tool0 if needed I think if this is added, the gripper will included to the IK. This project works as it is now, but is something to think about.
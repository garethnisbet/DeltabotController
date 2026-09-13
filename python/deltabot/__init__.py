"""Python control for the 3d printed DeltaBot.

    from deltabot import DeltaBot

    with DeltaBot() as bot:       # or DeltaBot("sim")
        bot.zero()
        bot.move_xy(0.5, 0.2)
        print(bot.pose())
"""

from .config import Connection, Geometry
from .controller import DeltaBot, DeltaBotError, find_port, list_ports
from .kinematics import (Pose, common_mode, estimate_z, leg_directions,
                         pose_to_screws, reachable, screws_to_pose)

__all__ = [
    "Connection", "DeltaBot", "DeltaBotError", "Geometry", "Pose",
    "common_mode", "estimate_z", "find_port", "leg_directions", "list_ports",
    "pose_to_screws", "reachable", "screws_to_pose",
]
__version__ = "1.0"

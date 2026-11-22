"""`lerobot_teleoperator_lucen`: teleoperators for Lucen robots (LeRobot's plugin convention).

- `lucen_leader`: a Lucen leader arm, watched through its own runtime, its
  joints the follower's targets; or, with `arm_config`, a Lucen Arm pair's leader, held
  by torque and moved by hand through the SDK's LeadFollow (`bench_leader`);
- `lucen_quest`: a Meta Quest's hand pose, through IK on the arm's model.

`quest` (the mapping and the IK) needs MuJoCo, not LeRobot; the LeRobot classes
load only where LeRobot is installed.
"""

__all__: list[str] = []

try:
    from lerobot_teleoperator_lucen.config_lucen_leader import LucenLeaderConfig, LucenQuestConfig
    from lerobot_teleoperator_lucen.lucen_leader import LucenLeader
    from lerobot_teleoperator_lucen.lucen_quest import LucenQuest
except ModuleNotFoundError as exc:  # LeRobot is not installed here
    if not (exc.name or "").startswith("lerobot"):
        raise
else:
    __all__ += ["LucenLeader", "LucenLeaderConfig", "LucenQuest", "LucenQuestConfig"]

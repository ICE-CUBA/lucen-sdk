# lerobot_robot_lucen

Lucen robots for [LeRobot](https://github.com/huggingface/lerobot), through LeRobot's
third-party plugin convention: install it beside LeRobot and the robot types are there.
It is a thin client of the Lucen SDK's bridge, the process that owns the MC02 board and
the motors; the plugin never opens the motor bus.

| Type | What it is |
|---|---|
| `lucen_arm` | one Lucen Arm (6+1 DoF): features `joint1.pos` … `joint6.pos`, `gripper_joint.pos`, in radians |
| `lucen_arm --robot.prefix=follower_` | the follower of a Lucen Arm pair (`Arm/Arm_pair.yaml`, one bridge for a follower and a leader) |
| `lucen_arm_pair` | both arms of a pair as one bimanual robot: `left_*` = the `leader_` arm, `right_*` = the `follower_` arm |
| `lucen_humanoid_arm` | Lucen Humanoid Arm: two arms of eight joints, `l_…` then `r_…` |

With `--robot.arm_config` naming the arm's yaml in the Lucen SDK, the plugin starts the
SDK's bridge when nothing listens on its port and stops it when the command ends;
`lerobot-setup-motors` checks that every motor answers and reads inside its range, and
`lerobot-calibrate` zeroes the arm at its stowed pose. Every arm of a pair is parked
before the session ends: the arm has no brake.

```bash
pip install -e lerobot/lerobot_robot_lucen -e lerobot/lerobot_teleoperator_lucen   # from the SDK's root
lerobot-teleoperate --robot.type=lucen_arm --robot.prefix=follower_ \
  --robot.arm_config=$HOME/lucen-sdk/Arm/Arm_pair.yaml \
  --teleop.type=lucen_leader --teleop.arm_config=$HOME/lucen-sdk/Arm/Arm_pair.yaml
```

A leader on a runtime of its own (watched limp, no `arm_config`) is reached with
`--teleop.port=127.0.0.1:9881`, its default; the humanoid arm's bridge is on 9879.

Name the cameras as the robot card does, so a dataset's `observation.images.<name>` keys
match: `wrist` (and an optional `top`) on Lucen Arm; `left_wrist`, `right_wrist` and an
optional `top` on Lucen Humanoid Arm.

The whole flow, from plugging in to running a trained policy:
[docs/lerobot.md](https://github.com/ICE-CUBA/lucen-sdk/blob/main/docs/lerobot.md).

# lerobot_teleoperator_lucen

Teleoperators for Lucen robots in [LeRobot](https://github.com/huggingface/lerobot):

- `lucen_leader --teleop.arm_config=…/Arm_pair.yaml`: the leader of a Lucen Arm pair. It
  holds itself by torque; push it and it follows your hand, let go and it stays where it
  is (the Lucen SDK's LeadFollow, `tools/arm_lead_follow.py`). The follower's target is
  the action. `--teleop.gravity_calibration=true` makes both arms visit two poses at the
  start (about 20 s; clear the bench) and prints how much to add to the bridge's
  `--grav-scale`.
- `lucen_leader` without `arm_config`: a leader arm watched limp on its own runtime
  (`--teleop.port=127.0.0.1:9881` by default); its joints are the follower's targets.
- `lucen_quest`: a Meta Quest's hand pose over WebXR, solved to joint targets on the
  arm's model. Hold the grip to move the arm; the trigger closes the gripper. The pose
  endpoint listens on 127.0.0.1: connect the headset by USB, run
  `adb reverse tcp:8777 tcp:8777`, then open the link it prints, which carries the
  session's secret. Only the first headset that presents it is taken.

```bash
pip install -e lerobot/lerobot_robot_lucen -e lerobot/lerobot_teleoperator_lucen   # from the SDK's root
lerobot-teleoperate --robot.type=lucen_arm --robot.prefix=follower_ \
  --robot.arm_config=$HOME/lucen-sdk/Arm/Arm_pair.yaml \
  --teleop.type=lucen_leader --teleop.arm_config=$HOME/lucen-sdk/Arm/Arm_pair.yaml
```

See [docs/lerobot.md](https://github.com/ICE-CUBA/lucen-sdk/blob/main/docs/lerobot.md).

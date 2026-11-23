# lerobot_policy_lucen

Run a policy on your robot through [LeRobot](https://github.com/huggingface/lerobot) when
your computer has no GPU. The policy runs on the MouseMouse hub; your computer runs
LeRobot's own `lerobot-rollout` with your robot's own plugin, and sends the hub one
observation each time it needs the next chunk of actions.

A person approves every run on the hub first. The approval names this computer, the
policy's exact files and the hub's server, and lasts the minutes you chose.

## Set up once

```bash
pip install -e lerobot/lerobot_policy_lucen        # from the Lucen SDK's root, into LeRobot's env
lucen auth login                                     # paste a key from the hub
lucen-device register --robot lucen/laika-arm --name my-arm-pc --adapter lerobot-rollout --trust approved
```

## Run

```bash
lerobot-rollout --strategy.type=base \
  --policy.type=lucen_endpoint --policy.endpoint=ep_... \
  --robot.type=lucen_arm --robot.prefix=follower_ --robot.arm_config=$HOME/lucen-sdk/Arm/Arm_pair.yaml \
  --robot.cameras="{wrist: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}}" \
  --task="pick up the cube" --duration=60
```

The first time, it asks the hub for the run and prints the page where a person approves
it. It waits there. Once approved, the arm starts.

## What happens when the network is slow or down

- A chunk that arrives late is dropped. The arm holds where it is for that step.
- If the connection drops, the arm holds where it is and the computer reconnects.
- When the approval ends, or someone presses Stop on the hub, the rollout stops and
  LeRobot returns the arm to where it started.

Name the cameras as the policy was trained (`wrist`, and `top` when it has one).

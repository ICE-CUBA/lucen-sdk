# Lucen Arm with LeRobot

This guide takes a Lucen Arm pair from the box to a trained policy with
[LeRobot](https://github.com/huggingface/lerobot)'s own commands: `lerobot-setup-motors`, `lerobot-calibrate`,
`lerobot-teleoperate`, `lerobot-record`, `lerobot-train`, `lerobot-rollout`. If you know LeRobot's SO-101
guide, the steps are the same, and two of them are shorter (see the table at the end).

Read [`safety.md`](safety.md) first. The arms have no brake: an arm whose motors turn off drops.

## What you need

- **Two identical Lucen Arms**, a follower and a leader. An owner of one arm leads it with a second, identical
  arm. Both are on **one MC02 board**: the follower on CAN1 (ids 1–7), the leader on CAN2 (ids 8–14). The
  configuration is `Arm/Arm_pair.yaml` (bridge port 9881).
- The MC02's USB in **your own computer** (Linux), which drives the arm; with an NVIDIA GPU it also trains and
  runs the policy. 48 V to the motors, with the **emergency stop within reach** whenever the motors are on.
- A wrist camera on the follower (USB); a fixed camera over the table is optional.
- For training: that computer's NVIDIA GPU, or a MouseMouse account (Lucen's hub, cloud GPUs).

How the pieces fit: the plugins are clients of the SDK's bridge (`bridge.py`), which owns the MC02, clamps
every target to the joint limits, limits speed and turns the motors off when its client goes quiet. The
plugins start the bridge for you when nothing listens on its port, and stop it when the command ends; a bridge
you started by hand is used and left running.

```
lerobot-* (your computer) ──TCP──> bridge.py --config Arm/Arm_pair.yaml ──USB──> MC02 ──CAN1 / CAN2──> motors
```

## 1. Install

```bash
git clone https://github.com/ICE-CUBA/lucen-sdk.git ~/lucen-sdk && cd ~/lucen-sdk
bash setup.sh                                   # ~/lucen_env: the bridge's environment; udev rule for the MC02 (sudo)

uv venv --python 3.12 ~/lerobot_env && source ~/lerobot_env/bin/activate     # LeRobot 0.6 needs Python 3.12
uv pip install "lerobot[dataset,core-scripts]==0.6.1" python-can \
    -e lerobot/lerobot_robot_lucen -e lerobot/lerobot_teleoperator_lucen
```

- `python-can` is there because zeroing writes the configuration with `tools/calib_arm.py`, which imports the
  motor driver.
- LeRobot finds the plugins by their package names: after this install every `lerobot-*` command knows
  `--robot.type=lucen_arm`, `lucen_arm_pair` and `--teleop.type=lucen_leader`.

The commands below repeat the same flags. Put them in variables once:

```bash
CFG=~/lucen-sdk/Arm/Arm_pair.yaml
FOLLOWER="--robot.type=lucen_arm --robot.prefix=follower_ --robot.arm_config=$CFG --robot.bridge_python=$HOME/lucen_env/bin/python --robot.id=lucen-follower"
LEADER="--teleop.type=lucen_leader --teleop.arm_config=$CFG --teleop.bridge_python=$HOME/lucen_env/bin/python --teleop.id=lucen-leader"
```

## 2. Plug in and check (motors stay off)

```bash
lerobot-setup-motors $FOLLOWER
lerobot-setup-motors $LEADER
```

The motor ids are set before delivery, so this does not assign them: it starts the bridge, reads every motor
without enabling it, and reports each joint's reading against its range. Expect `7 motors answer` and `Every
motor answers and reads inside its range` for each arm. A whole arm silent, or a joint outside its range: see
Troubleshooting.

## 3. Zero (first use, or after servicing a joint)

Each joint's zero lives in `Arm/Arm_pair.yaml`. The zero pose is the **stowed pose**: upper arm lying on its
lower stop, forearm folded back on top of it, wrist straight in line with the forearm, gripper fully open
(`Arm/README.md`, Poses). For joint1, joint5 and joint6, which gravity does not locate, centre the base and keep
the gripper straight, the same on both arms.

```bash
lerobot-calibrate $FOLLOWER
lerobot-calibrate $LEADER
```

Each run turns that arm's motors off (it goes limp: hold it), asks you to fold it to the stowed pose and press
Enter, then shows every joint's reading with its old and new offset and writes them on `y`. The whole file is
kept beside it as `Arm_pair.yaml.before-zero-<time>`. A bridge the plugin started is restarted to load the new
zero and the arm is read again: every joint must read within 0.05 rad of zero.

To zero one joint only (a refitted gripper, for example), name it: `lerobot-calibrate $LEADER
--teleop.zero_joints='["gripper_joint"]'`. Zeroing a joint whose zero is good adds the error of the pose you
hold it in.

If the file says `calibrated: false`, the bridge will not enable the motors until the directions are checked:
`python tools/calib_arm.py --config Arm/Arm_pair.yaml --signs`, then `--mark-calibrated` (`arm_guide.md`,
section 5).

## 4. Teleoperate

Clear the area around both arms, keep the emergency stop within reach, then:

```bash
lerobot-teleoperate $FOLLOWER $LEADER --teleop.gravity_calibration=true \
    --robot.cameras="{ wrist: {type: opencv, index_or_path: /dev/v4l/by-path/<your camera>-video-index0, width: 640, height: 480, fps: 30, fourcc: MJPG}}"
```

- Both arms are enabled where they stand. The **leader is not limp**: it holds itself with position control and
  the gravity feed-forward. Push it past a small threshold and it follows your hand; let go and it stays where it
  is. The follower copies its joint angles, speed-limited, and skips a step that would make it hit itself in the
  model. The follower's gripper follows the leader's.
- With `--teleop.gravity_calibration=true` **both arms move on their own for about 10 s** at the start: they
  visit two poses and return home, to measure what the gravity feed-forward misses. The log then prints
  `Gravity calibration done …` and, when it matters, a `feed-forward hint`: add it to the bridge's
  `--grav-scale` (`--robot.grav_scale=… --teleop.grav_scale=…`) and the leader gets lighter. On Lucen's own
  pair the yaml's `gravity_ff.scale: 0.53` left a hint of about −0.05: nothing to change.
- **Stop with one Ctrl-C** and wait: the follower, then the leader, fold to the stowed pose (about 20 s each)
  before the motors turn off. A second Ctrl-C during that, or killing the process, turns the motors off where
  the arms are, and they drop.

Cameras: `lerobot-find-cameras opencv` lists them. Name a camera by its `/dev/v4l/by-path/…` or
`/dev/v4l/by-id/…` path (a `/dev/videoN` number can change after a reboot), and ask for `MJPG`: many USB
cameras give 30 fps at 640×480 only in MJPG. Name the wrist camera `wrist` and a fixed one `top`, as the robot
card does, and use the same names, sizes and frame rate everywhere a policy will see them.

## 5. Record

```bash
lerobot-record $FOLLOWER $LEADER \
    --robot.cameras="{ wrist: {...}, top: {...}}" \
    --dataset.repo_id=you/cup --dataset.single_task="Put the cup on the plate" \
    --dataset.num_episodes=30 --dataset.episode_time_s=25 --dataset.reset_time_s=8 \
    --dataset.no_stamp=true --dataset.push_to_hub=false
```

- Keys: `n` ends the episode (or the reset), `r` records it again, `q` stops. They work over SSH from an
  interactive terminal (`ssh -t`).
- The dataset is written to `~/.cache/huggingface/lerobot/you/cup` (LeRobot v3.0). `--dataset.no_stamp=true`
  keeps that name (LeRobot otherwise appends the date and time); `--resume=true` adds episodes to it.
- What is recorded: `observation.state` = the follower's measured joints, `action` = the target the follower was
  given, in radians, named `joint1.pos` … `gripper_joint.pos`; the cameras as `observation.images.<name>`.
- LeRobot's advice holds: at least 50 episodes, the cameras fixed, the task doable from the camera images alone.
- Replay an episode on the follower: `lerobot-replay $FOLLOWER --dataset.repo_id=you/cup --dataset.episode=0`.

## 6. Train

**On your own GPU**, on the computer the arm is plugged into: the dataset is already there.

```bash
lerobot-train --dataset.repo_id=you/cup --policy.type=act --policy.device=cuda \
    --output_dir=outputs/cup-act --job_name=cup-act --policy.push_to_hub=false --wandb.enable=false
```

Any LeRobot policy trains on the dataset (`--policy.type=diffusion`, or `--policy.path=lerobot/smolvla_base` to
fine-tune SmolVLA). ACT's default is 100,000 steps; on an RTX 5070 that is about 3.6 hours at batch 8.

Recorded on another computer? Copy the dataset folder to the same place on the GPU computer first, so the command
above finds it by its name. From the computer that recorded it (send it again after you record more episodes):

```bash
ssh <user>@<GPU computer> "mkdir -p ~/.cache/huggingface/lerobot/you/cup"
tar -C ~/.cache/huggingface/lerobot/you/cup -cf - . | ssh <user>@<GPU computer> "tar -C ~/.cache/huggingface/lerobot/you/cup -xf -"
```

**On MouseMouse** (Lucen's hub, a cloud GPU): push the dataset, queue a run, pull the policy.

```bash
pip install lucen-cli && lucen auth login --url https://api.mousemouse.ai     # a key with the train scope
lucen push you/cup ~/.cache/huggingface/lerobot/you/cup --kind dataset
cat > recipe.yaml <<'EOF'
template: act                 # or diffusion, smolvla_lora
dataset: [lucen://you/cup]
robot: lucen/laika-arm
executor: modal
gpu: l4
max_hours: 4
budget_usd: 10
hyperparams: {steps: 100000, batch_size: 8}
EOF
lucen train -f recipe.yaml
lucen pull you/<the run's model repo> ./cup-act        # its policy/ folder is the checkpoint
```

The hub holds one episode in ten out of training, scores every checkpoint on it, and checks the dataset's
joint names against the robot.

## 7. Run the policy on the follower

On your computer's GPU:

```bash
lerobot-rollout --strategy.type=base --policy.path=outputs/cup-act/checkpoints/last/pretrained_model \
    --policy.device=cuda $FOLLOWER --robot.cameras="{ wrist: {...}, top: {...}}" \
    --task="Put the cup on the plate" --duration=60
```

A policy pulled from MouseMouse: its `policy/` folder is the checkpoint. Run the same command from the folder you
pulled it into, with `--policy.path` on that folder:

```bash
lerobot-rollout --strategy.type=base --policy.path=./cup-act/policy \
    --policy.device=cuda $FOLLOWER --robot.cameras="{ wrist: {...}, top: {...}}" \
    --task="Put the cup on the plate" --duration=60
```

- Only the follower is enabled; the leader stays off. Every target the policy sends is held within 0.35 rad of
  the measured position (0.25 rad on the gripper), inside the joint limits, and the bridge limits its speed.
  At the end, or on one Ctrl-C, the follower folds to its stowed pose before its motors turn off.
- The cameras must be the ones the policy was trained with: same names, same size.

## Both arms as one robot

`lucen_arm_pair` drives both arms of the pair as one bimanual robot: `left_joint1.pos` … `right_gripper_joint.pos`
(left = the `leader_` arm, right = the `follower_` arm), cameras `left_wrist`, `right_wrist` and `top`. Use it with
a bimanual policy or teleoperator:

```bash
lerobot-rollout --strategy.type=base --policy.path=<a bimanual policy> --policy.device=cuda \
    --robot.type=lucen_arm_pair --robot.arm_config=$CFG --robot.bridge_python=$HOME/lucen_env/bin/python \
    --robot.cameras="{ left_wrist: {...}, right_wrist: {...}, top: {...}}" --task="..." --duration=60
```

Both arms fold, one after the other, before the motors turn off. Nothing checks for collisions between the two
arms: watch them.

## Rehearse without hardware

Run every command above against the simulated bridge first. Unplug the MC02's USB (so nothing can start a real
bridge), then:

```bash
python tools/fake_bridge.py --config Arm/Arm_pair.yaml --grav-scale 1.0      # terminal 1, ~/lucen_env active
lerobot-setup-motors $FOLLOWER                                               # terminal 2, ~/lerobot_env active
```

The simulated bridge listens on the same port, so the plugins use it and leave it running. To feel what the
gravity calibration measures, start it with `--grav-scale 0.35`: the simulated arm is the model, so the
calibration should report a hint of about `+0.65`.

## SO-101 and Lucen Arm, step by step

| LeRobot's SO-101 guide | Lucen Arm |
|---|---|
| `lerobot-find-port` | not needed: the MC02 has a fixed path (`/dev/mc02`, from `setup.sh`) named in the yaml |
| `lerobot-setup-motors` sets each motor's id, one motor at a time | ids are set before delivery; `lerobot-setup-motors` checks that every motor answers and reads inside its range |
| `lerobot-calibrate`: every joint to the middle, then through its whole range | one pose, the stowed pose: RobStride encoders are absolute within a turn |
| the leader is a limp arm with geared-down servos | the leader is an identical arm that holds itself by torque and follows your hand |
| `lerobot-teleoperate`, `lerobot-record`, `lerobot-train`, `lerobot-rollout` | the same commands |

## Troubleshooting

- **"Not taking over: … reads more than 0.3 rad outside its range"** — that joint's zero is wrong (a gripper or a
  motor was refitted, or the zero was lost). Zero it: `lerobot-calibrate $LEADER
  --teleop.zero_joints='["gripper_joint"]'` with the gripper fully open (or the joint the message names).
- **A whole arm does not answer** — that arm's 48 V, its CAN plug at the MC02, L/H swapped, the 120 Ω
  terminator at the end of the chain (`mc02.md`).
- **The MC02 is not found, or permission denied** — re-plug it; run `bash setup.sh` for the udev rule; with two
  MC02 boards on one computer, name the board's `/dev/serial/by-id/…` path as `mc02.port` in the yaml.
- **The bridge refuses to enable the motors ("calibrated: false")** — zero the arms, check the directions with
  `python tools/calib_arm.py --config Arm/Arm_pair.yaml --signs`, then `--mark-calibrated`.
- **The leader feels heavy, sags, or rises by itself** — the gravity feed-forward is off: teleoperate with
  `--teleop.gravity_calibration=true` and add the printed hint to `--grav-scale`.
- **The shoulder or elbow motors get hot** — do not hold the arms stretched out or pressed on their stops with
  the motors on; rest them in the ready pose.
- **A camera gives 15 fps, or the wrong camera opens** — ask for `fourcc: MJPG`, and name the camera by its
  `/dev/v4l/by-path/…` or `/dev/v4l/by-id/…` path.
- **"No module named 'accelerate'", or another LeRobot version is imported** — an older LeRobot is on your
  `PYTHONPATH`: `unset PYTHONPATH`.
- **The keys n / r / q do nothing** — run the command in an interactive terminal (`ssh -t` over SSH).
- **The process was killed (kill -9, a crash, a lost SSH session without `nohup`)** — the bridge turns the
  motors off within 1.5 s and the arms drop from where they are. Stop with one Ctrl-C, and keep the emergency
  stop within reach.

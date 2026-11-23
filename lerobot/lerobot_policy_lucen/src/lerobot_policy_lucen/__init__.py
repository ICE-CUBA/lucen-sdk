"""`lerobot_policy_lucen`: a MouseMouse hub endpoint as a LeRobot policy (`lucen_endpoint`).

LeRobot imports every installed distribution whose name starts with
`lerobot_policy_`, which registers the policy type here. Then

    lerobot-rollout --strategy.type=base --policy.type=lucen_endpoint \\
        --policy.endpoint=ep_... --robot.type=lucen_arm ...

runs LeRobot's own control loop on the owner's robot while the policy's actions
come from the hub. A person approves the run on the hub first; the policy waits
for that, and asks for it when nobody has yet.
"""

from lerobot_policy_lucen.configuration_lucen_endpoint import LucenEndpointConfig

__all__ = ["LucenEndpointConfig"]

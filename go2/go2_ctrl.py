import os
import time
from collections import deque
import torch
import carb
import gymnasium as gym
from isaaclab.envs import ManagerBasedEnv
import isaaclab_tasks  # Registers the Gym task IDs.

base_vel_cmd_input = None
selected_robot = 0
ros_commands = {}
linear_speed=1.0

def init_base_vel_cmd(num_envs):
    global base_vel_cmd_input
    base_vel_cmd_input = torch.zeros((num_envs, 3), dtype=torch.float32)

def base_vel_cmd(env: ManagerBasedEnv) -> torch.Tensor:
    global base_vel_cmd_input
    command = base_vel_cmd_input.clone()
    now = time.monotonic()
    for index, (velocity, deadline) in ros_commands.items():
        if now < deadline:
            command[index] = velocity
    return command.to(env.device)

def set_ros_command(index, velocity):
    """Preserve fresh ROS commands independently of GUI event updates."""
    value = torch.as_tensor(velocity, dtype=torch.float32)
    ros_commands[index] = (value, time.monotonic() + 0.5)
    base_vel_cmd_input[index] = value

def expire_ros_commands():
    now = time.monotonic()
    for index, (velocity, deadline) in list(ros_commands.items()):
        if now >= deadline:
            if torch.equal(base_vel_cmd_input[index], velocity):
                base_vel_cmd_input[index].zero_()
            del ros_commands[index]

pressed_keys = set()
keyboard_events = deque(maxlen=10000)

def sub_keyboard_event(event) -> bool:
    global selected_robot
    if base_vel_cmd_input is None:
        return True
    key = (event.input.name if hasattr(event.input, "name") else str(event.input)).upper()
    kind = event.type
    keyboard_events.append({"key":key,"type":str(kind)})
    if kind == carb.input.KeyboardEventType.KEY_PRESS:
        select = {str(i+1):i for i in range(9)}
        select.update({f"KEY_{i+1}":i for i in range(9)})
        select.update({f"F{i+1}":i for i in range(12)})
        if key in select and select[key] < len(base_vel_cmd_input):
            ros_commands.clear()
            pressed_keys.clear()
            base_vel_cmd_input.zero_()
            selected_robot = select[key]
            return True
    movement = {"W":(1,0,0),"S":(-1,0,0),"A":(0,1,0),
                "D":(0,-1,0),"Z":(0,0,1),"C":(0,0,-1)}
    if key in movement:
        if kind not in (carb.input.KeyboardEventType.KEY_PRESS, carb.input.KeyboardEventType.KEY_REPEAT, carb.input.KeyboardEventType.KEY_RELEASE):
            return True
        if kind in (carb.input.KeyboardEventType.KEY_PRESS, carb.input.KeyboardEventType.KEY_REPEAT):
            ros_commands.pop(selected_robot, None)
            pressed_keys.add(key)
        elif kind == carb.input.KeyboardEventType.KEY_RELEASE:
            if key not in pressed_keys:
                return True
            pressed_keys.discard(key)
        command = [sum(movement[k][i] for k in pressed_keys)*(linear_speed if i<2 else 1.0) for i in range(3)]
        base_vel_cmd_input[selected_robot] = torch.tensor(command, dtype=torch.float32)
    if key == "SPACE" and kind == carb.input.KeyboardEventType.KEY_PRESS:
        ros_commands.clear()
        pressed_keys.clear()
        base_vel_cmd_input.zero_()
    return True

def _get_ckpt_path():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    ckpt_dir = os.path.join(script_dir, "..", "ckpts")
    return os.path.abspath(ckpt_dir)

def _get_policy(cfg, kind):
    """Load only the frozen Actor: independent of RSL-RL training API versions."""
    if kind == "flat":
        cfg.observations.policy.height_scan = None
    env = gym.make(f"Isaac-Velocity-{kind.title()}-Unitree-Go2-v0", cfg=cfg)
    checkpoint = os.path.join(_get_ckpt_path(), "unitree_go2",
                              "flat_model_6800.pt" if kind == "flat" else "rough_model_7850.pt")
    state = torch.load(checkpoint, map_location=cfg.sim.device, weights_only=False)["model_state_dict"]
    weights = sorted((k for k in state if k.startswith("actor.") and k.endswith(".weight")),
                     key=lambda k: int(k.split(".")[1]))
    layers = []
    for i, key in enumerate(weights):
        weight = state[key]
        layer = torch.nn.Linear(weight.shape[1], weight.shape[0])
        layer.weight.data.copy_(weight)
        layer.bias.data.copy_(state[key.replace("weight", "bias")])
        layers.append(layer)
        if i < len(weights) - 1:
            layers.append(torch.nn.ELU())
    actor = torch.nn.Sequential(*layers).to(cfg.sim.device).eval()
    actor.requires_grad_(False)
    def policy(obs):
        return actor(obs["policy"] if isinstance(obs, dict) or hasattr(obs, "keys") else obs)
    # Keep Gym's native observation dictionary and five-value step interface.
    return env, policy

def get_rsl_flat_policy(cfg):
    return _get_policy(cfg, "flat")

def get_rsl_rough_policy(cfg):
    return _get_policy(cfg, "rough")

#!/usr/bin/env python3
"""Copy a ros2_control controllers YAML and point every controller at its own params file.

ros2_control 6.10.1 (Lyrical) only hands a controller its parameters when the controller
manager has a `<controller>.params_file` entry (or the spawner gets `--param-file`). The stock
`ur_sim_control.launch.py` spawners set neither, so on Lyrical the stock
`joint_trajectory_controller` fails with "Length of parameter 'joints' is '0'". Writing
`params_file: <this file>` next to each `type:` fixes that without editing the stock launch.

usage: inject_params_file.py <in.yaml> <out.yaml>
"""

import sys

import yaml


def inject(src: str, dst: str) -> None:
    with open(src) as f:
        data = yaml.safe_load(f)
    cm = data["controller_manager"]["ros__parameters"]
    for value in cm.values():
        if isinstance(value, dict) and "type" in value:
            value["params_file"] = dst
    with open(dst, "w") as f:
        yaml.safe_dump(data, f, sort_keys=False)


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    inject(sys.argv[1], sys.argv[2])

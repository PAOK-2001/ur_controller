# UR Control Stack

This repository contains a Python-based control stack for a Universal Robots UR5e arm equipped with an AG95 gripper. It provides a high-level interface for controlling the robot's position and velocity, along with safety features for workspace management.

## Install

Create a Python environment, and install the current package. We recommend using uv for package management.

0. Install [uv](https://docs.astral.sh/uv/#installation)

```sh
curl -LsSf https://astral.sh/uv/install.sh | sh
```

1. Clone repo and cd into dir

2. Create venv and install module
```sh
uv venv --python 3.11.7 control_env
source control_env/bin/activate
uv pip install -e .
```

For configuration of the UR5 arm, see [docs/arm_configuration](docs/arm_configuration).

## Architecture

The control system is composed of several key modules that work together to provide a safe and intuitive way to command the robot.

### Core Controller (`Ur5EController`)

The main entry point for robot control is the [`Ur5EController`](src/ur_control/controller.py) class in [`src/ur_control/controller.py`](src/ur_control/controller.py). This class is responsible for:
-   Establishing a real-time communication link with the UR robot using the `ur-rtde` library.
-   Integrating the gripper control interface.
-   Loading and enforcing workspace safety parameters.

### Gripper Interface

Gripper control is handled by the [`Gripper`](src/ur_control/gripper_interface.py) class in [`src/ur_control/gripper_interface.py`](src/ur_control/gripper_interface.py). It communicates with the AG95 gripper over a TCP/IP to Modbus RTU bridge, which is typically set up as a URCap on the robot's controller. It provides simple methods like `open()`, `close()`, and `set_position()`.

### Workspace and Safety

To ensure safe operation, the control stack defines a valid workspace and prevents the robot from entering a self-collision zone.
-   **Configuration**: The physical limits of the workspace, a home position, and a self-collision barrier are defined in [`src/ur_control/config/arm_parms.yaml`](src/ur_control/config/arm_parms.yaml).
-   **Geometry Utilities**: The [`utils.py`](src/ur_control/utils.py) script contains functions to load the geometry from the YAML file, compute a "safe zone" (the valid workspace minus the collision barrier), and clamp any target point to remain within this safe zone using the [`clamp_point_to_safe_zone`](src/ur_control/utils.py) function. This is automatically applied by the `Ur5EController` for position-based moves.

### Teleoperation Example

The [`scripts/teleop.py`](scripts/teleop.py) script provides a complete example of how to use the control stack. It demonstrates:
-   Reading inputs from a joystick using the `inputs` library.
-   Mapping joystick axes to linear and angular velocity commands.
-   Running a multi-threaded application where one thread reads joystick events and another sends continuous velocity commands to the `Ur5EController`.
-   A monitoring thread that checks if the arm has moved outside the defined workspace and stops the robot if it does.

## How to Run

To run the teleoperation demo, connect a joystick and execute the `teleop.py` script:

```sh
python scripts/teleop.py
```
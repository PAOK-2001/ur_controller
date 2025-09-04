"""
Spawns threads for reading joystick input and sending velocity commands to the robot arm based on the joystick state.
"""
#!/usr/bin/env python3
import time
import threading
from inputs import get_gamepad

from ur_control.controller import Ur5EController
from ur_control.gripper_interface import GripperState

def clip(value, lower, upper):
    """Clip value between lower and upper bounds."""
    return lower if value < lower else upper if value > upper else value

class StrikerJoystickHandler:
    def __init__(self):
        print("Starting joystick handler")


        self.scale = 1.0
        self.w_scale = [0.8, 0.8, 0.8]
        # Maximum speeds
        self.vx_max = 1.0
        self.vy_max = 1.0
        self.vz_max = 0.5

        self.wx_max = 0.5
        self.wy_max = 0.5
        self.wz_max = 0.5

        # Current command velocities
        self.vx = 0
        self.vy = 0
        self.vz = 0

        # Timestamp of last received event
        self.last_command_time = time.time()

        # Command structure (similar to ROS Twist)
        self.command = {
            "linear": {"x": 0, "y": 0, "z": 0},
            "angular": {"x": 0, "y": 0, "z": 0}
        }

        self.gripper = GripperState.CLOSED
        self.record = 0
        # Dictionary to hold latest normalized axis values
        # We assume the controller provides these four axes:
        # - ABS_X: Left stick horizontal (used here for angular velocity)
        # - ABS_RX: Right stick horizontal (used for lateral motion)
        # - ABS_RY: Right stick vertical (used for forward/backward motion)
        # - ABS_Y: Left stick vertical (not used in this mapping)
        self.axes = {
            "ABS_X": 0,
            "ABS_Y": 0,
            "ABS_RX": 0,
            "ABS_RY": 0,
            "ABS_HAT0X": 0,
            "ABS_HAT0Y": 0

        }

        self.buttons = {
            "BTN_SOUTH": 0,
            "BTN_NORTH": 0,
            "BTN_WEST": 0,
            "BTN_EAST": 0,
            "BTN_TL": 0,
            "BTN_TR": 0,
            "BTN_SELECT": 0,
            "BTN_START": 0,
            "BTN_MODE": 0,
            "BTN_THUMBL": 0,
            "BTN_THUMBR": 0
        }

    def update_from_event(self, event):
        """Update axis values from a gamepad event.
        
        For simplicity, we assume that the raw event.state is in a range that can be
        normalized to [-1, 1]. In many cases the raw values might be in the range
        [-32768, 32767] or [0, 255] so you may need to adjust the normalization.
        """
        if event.ev_type == "Absolute" and event.code in self.axes:
            # Normalize the axis value. Here we assume a symmetric range.
            # (Adjust the normalization factor if needed for your controller.)
            if "HAT" not in event.code:
                if event.state >= 0:
                    normalized = event.state / 32767.0
                else:
                    normalized = event.state / 32768.0
                self.axes[event.code] = normalized
            else:
                self.axes[event.code] = event.state
            self.last_command_time = time.time()

        if event.ev_type == "Key" and event.code in self.buttons:
            self.buttons[event.code] = event.state
            self.last_command_time = time.time()

    def process_axes(self):
        """Map the latest axis values to command velocities."""
        # Mapping (similar to your ROS code for Xbox 360):
        # - vx from right stick vertical (ABS_RY)
        # - vy from right stick horizontal (ABS_RX)
        # - linear velocity vz from left stick horizontal (ABS_X)
        self.vx = clip(-self.axes.get("ABS_RY", 0) * self.vx_max, -self.vx_max, self.vx_max)
        self.vy = clip(self.axes.get("ABS_RX", 0) * self.vy_max, -self.vy_max, self.vy_max)
        self.vz = clip(-self.axes.get("ABS_Y", 0) * self.vz_max, -self.vz_max, self.vz_max)

        
        self.wy = self.axes.get("ABS_HAT0Y", 0)
        self.wz = self.axes.get("ABS_HAT0X", 0)
        self.record = self.buttons.get("BTN_WEST", 0)

        # Check if the A button (BTN_SOUTH) is pressed to toggle the gripper
        if self.buttons.get("BTN_TL", 0) == 1:
            self.gripper = GripperState.OPENED
        if self.buttons.get("BTN_TR", 0) == 1:
            self.gripper = GripperState.CLOSED
        if self.buttons.get("BTN_NORTH", 0) == 1:
            self.gripper = GripperState.CLOSED


        self.command["linear"]["x"] = self.vx * self.scale
        self.command["linear"]["y"] = self.vy * self.scale
        self.command["linear"]["z"] = self.vz * self.scale

        self.command["angular"]["x"] = self.wy * self.w_scale[1]
        self.command["angular"]["y"] = 0
        self.command["angular"]["z"] = self.wz * self.w_scale[2]
        
        # print("record: ", self.record)


    def check_pose(self, pose, controller : Ur5EController):
        valid_pose = True
        x, y, z = pose[:3]

        if x < controller.workspace_min['x']:
            valid_pose = False
        elif x > controller.workspace_max['x']:
            valid_pose = False

        if y < controller.workspace_min['y']:
            valid_pose = False
        elif y > controller.workspace_max['y']:
            valid_pose = False

        if z < controller.workspace_min['z']:
            valid_pose = False
        elif z > controller.workspace_max['z']:
            valid_pose = False

        return valid_pose


    def monitor(self, controller : Ur5EController, hz = 100):
        """Main loop to update and publish commands at 35Hz."""
        rate = 1 / float(hz)  # 35 Hz loop rate
        stop_event = threading.Event()
        try:
            while True:
                self.process_axes()
                # If no new events in >1 second, zero the command.
                if time.time() - self.last_command_time > 1:
                    self.vx = self.vy = self.vz = 0
                    self.command["linear"]["x"] = 0
                    self.command["linear"]["y"] = 0
                    self.command["linear"]["z"] = 0

                joint_state, arm_pose, gripper_state = controller.get_arm_states()
                arm_pose[:3] = arm_pose[:3] * 1000
                check = self.check_pose(arm_pose, controller)
                if not check:
                    print("\033[91m[ ERROR ] Warning: Arm pose out of bounds. Stopping all threads.\033[0m")
                    controller.velocity_control(cmd_vel={"linear": {"x": 0, "y": 0, "z": 0}, "angular": {"x": 0, "y": 0, "z": 0}}, duration=-1)
                    break
                # "Publish" the command. Here we simply print it.
                # print("Command:", self.command["linear"])
                time.sleep(rate)
        except KeyboardInterrupt:
            print("Shutting down joystick handler.")
        stop_event.set()
def gamepad_thread(handler : StrikerJoystickHandler):
    """Continuously poll for gamepad events and update the handler."""
    while True:
        try:
            events = get_gamepad()  # Blocks until at least one event is available.
            for event in events:
                handler.update_from_event(event)
        except Exception as e:
            print("Error reading gamepad events:", e)
            time.sleep(0.1)

def robot_control_thread(handler, controller : Ur5EController, hz = 100):
    """
    This thread periodically updates a waypoint based on the joystick's velocity
    commands and sends the new waypoint to the robot via the controller.
    
    For this example, we assume:
      - x and y are controlled by the joystick.
      - z, roll, and pitch remain constant.
      - yaw is updated from the joystick's rotation command.
    """
    # Set an initial waypoint: [x, y, z, roll, pitch, yaw]
    waypoint = list(controller.home)
    dt = 1 / float(hz)  
    print("dt: ", dt)
    while True:
        start_time = time.time()
        handler.process_axes()  # Update latest velocities
        cmd_vel = handler.command
        # print(f"\033[94m linear: {cmd_vel['linear']}\033[0m")
        # print(f"\033[92m angular: {cmd_vel['angular']}\033[0m")
        controller.velocity_control(
            cmd_vel= cmd_vel,
            gripper= handler.gripper,
            is_radian=True,
            duration= 0.01
        )
        time.sleep(dt)



if __name__ == "__main__":    
    handler = StrikerJoystickHandler()

    controller = Ur5EController(robot_ip="192.168.0.15")
    
    while not controller.arm_ready:
        print("Waiting for arm to be ready...")
        time.sleep(1)
        
    # Start a background thread to process gamepad events.
    t1 = threading.Thread(target=gamepad_thread, args=(handler,), daemon=True)
    t1.start()

    # Start a background thread for robot control.
    t2 = threading.Thread(target=robot_control_thread, args=(handler, controller), daemon=True)
    t2.start()

    # # Enter the main control loop.
    handler.monitor(controller=controller, hz=100)

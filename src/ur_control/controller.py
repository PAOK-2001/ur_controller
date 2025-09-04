"""
Simple position control interface for the UR5e robot arm and AG95 gripper using RTDE and Modbus TCP.
"""
import time
import math
import yaml
from ur_control.gripper_interface import Gripper
from ur_control.utils import load_geometry_from_yaml, clamp_point_to_safe_zone
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive

class Ur5EController:
    def __init__(
        self, 
        robot_ip="192.168.0.15", 
        gripper_port=54321, 
        rtde_freq=500.0,
        **kwargs
    ):
        """Initialize the UR5e controller for arm position and gripper control."""
        self.arm_ready = False
        self.robot_ip = robot_ip
        # RTDE controller (with URCap External control mode)
        self.controller = RTDEControl(robot_ip, rtde_freq, RTDEControl.FLAG_USE_EXT_UR_CAP)
        self.receiver   = RTDEReceive(robot_ip, rtde_freq)

        # Gripper wrapper (Modbus TCP bridge via URCap)
        self.gripper = Gripper(robot_ip=robot_ip, tcp_port=gripper_port)
        self.gripper.connect()
        # Read gripper setup parameters from kwargs, set defaults if not provided
        gripper_force = kwargs.get("gripper_force", 100)
        gripper_calibration = kwargs.get("gripper_calibration", True)
            
        self.gripper.setup(
            force=gripper_force, 
            do_calibration=gripper_calibration
        )
        time.sleep(0.2)  # allow some time for setup
        self.arm_ready = True
        
        # Load params
        param_file = kwargs.get("param_file", "/home/paok/Documents/MOTORCORTEX/motor_cortex/benchmarks/real_life/ur_control/src/ur_control/config/arm_parms.yaml")
        with open(param_file, "r") as f:
            params = yaml.safe_load(f)
        self.home = params["home"]
        self.self_collision_barrier = params["self_collision_barrier"]
        self.workspace_min = params["workspace_min"]
        self.workspace_max = params["workspace_max"]
        # load safe zone from YAML
        workspace, barrier, safe_zone, zmin, zmax = load_geometry_from_yaml(param_file)
        self.z_min = zmin
        self.z_max = zmax
        self.safe_zone = safe_zone
        
    def set_gripper(self, position: str | float | int):
        """Set the gripper position. Accepts 'open', 'close', or a numeric value (0.0–1.0)."""
        if isinstance(position, str):
            cmd = position.lower()
            if cmd == "open":
                self.gripper.open()
            elif cmd == "close":
                self.gripper.close()
            else:
                raise ValueError(f"Unknown gripper command: {position}")
            
        # Set position width
        elif isinstance(position, (int, float)):
            # Assume normalized [0.0–1.0], convert to permille
            permille = int(max(0.0, min(1.0, position)) * 1000)
            self.gripper.set_position(permille)
        else:
            raise TypeError("position must be 'open', 'close', or numeric (0.0–1.0)")

    def move_to_state(
        self,
        point,
        gripper=None,
        speed=75,
        wait=True,
        use_safety_offset=True,
        is_radian=False
    ):
        """
        Move to a TCP pose and optionally operate the gripper.

        Args:
            point: [x, y, z, thetax, thetay, thetaz] (m, m, m, rad/deg, rad/deg, rad/deg).
            gripper: 'open', 'close', numeric (0–1) or None.
            speed: TCP speed percentage (0–100).
            wait: Block until motion is complete if True.
            use_safety_offset: Move above target first if True.
            is_radian: Orientation values are in radians if True, degrees if False.
        """
        # Convert orientation if needed
        if not is_radian:
            point = [
                point[0],
                point[1],
                point[2],
                math.radians(point[3]),
                math.radians(point[4]),
                math.radians(point[5])
            ]

        # Main motion
        if use_safety_offset:
            point = self.check_and_clip_self_collision(point)
            
        self.controller.moveL(point, speed / 100.0, 0.2, asynchronous=not wait)

        # Operate gripper if requested
        if gripper is not None:
            time.sleep(0.5)  # allow motion to settle
            self.set_gripper(gripper)

        # Wait for completion
        if wait:
            time.sleep(0.5)

    def check_and_clip_self_collision(self, point):
        """
        Enforce:
        1. (x,y,z) is inside the workspace box  [workspace_min…workspace_max]
        2. (x,y) is outside the self-collision (red) box

        Args:
        point: (x, y, z, roll, pitch, yaw)

        Returns:
        A new (x, y, z, roll, pitch, yaw) satisfying both constraints.
        """
        x, y, z, roll, pitch, yaw = point
        pt = (x, y, z)
        # Clamp to safe zone
        clamped_pt = clamp_point_to_safe_zone(pt, self.safe_zone, self.z_min, self.z_max)
        cx, cy, cz = clamped_pt     
        clamped_pose = (cx, cy, cz, roll, pitch, yaw)
        return clamped_pose

    def velocity_control(
        self, 
        cmd_vel, 
        gripper='stop', 
        is_radian=False,
        duration=0.1,
        acceleration=0.25
    ):
        self.set_gripper(gripper)

        # Convert angular to radians if not already
        if not is_radian:
            ang_factor = 3.141592653589793 / 180.0
        else:
            ang_factor = 1.0

        # ur_rtde expects [vx, vy, vz, rx, ry, rz] in m/s and rad/s
        linear = cmd_vel["linear"] 
        angular = cmd_vel["angular"]
        twist = [
            linear["x"], linear["y"], linear["z"],
            angular["x"] * ang_factor,
            angular["y"] * ang_factor,
            angular["z"] * ang_factor
        ]
        print(f"Velocity command: {twist}")
        # Send speed command
        self.controller.speedL(twist, acceleration, duration)
    
    def get_arm_states(self, use_radians=True):
        # --- UR arm states ---
        joint_state = self.receiver.getActualQ()
        if not use_radians:
            joint_state = [math.degrees(q) for q in joint_state]

        arm_pose = self.receiver.getActualTCPPose()
        gripper_state = self.gripper.binary_state
        
        return joint_state, arm_pose, gripper_state
    
    def reset(self, requiere_user_input = False):
        # Stop all motion
        print("Resetting arm and gripper...")
        self.arm_ready = False
        try:
            self.controller.speedStop()
            self.controller.stopScript()
        except Exception as e:
            print(f"[WARN] Could not stop arm cleanly: {e}")
        time.sleep(0.1)
        # Cycle gripper state
        try:
            self.set_gripper("open")
            time.sleep(0.5)
            self.set_gripper("close")
        except Exception as e:
            print(f"[WARN] Could not operate gripper: {e}")
            
        # Go to home position
        self.move_to_state(
            self.home, 
            gripper="close", 
            speed=5, 
            wait=True, 
            is_radian=False
        )
        print("Arm reset to home position.")
        
        if requiere_user_input:
            input("Reset objects & press Enter to continue...")
        self.arm_ready = True
        time.sleep(0.1)
        return

    def shutdown(self):
        """Close gripper connection."""
        self.gripper.close_conn()
        

if __name__ == "__main__":
    ur5 = Ur5EController(robot_ip="192.168.0.15")
    try:
        # Example: Move to a pose and open gripper
        pose = [0.3, -0.5, 0.12, 180, 0, 0]  # degrees for orientation
        ur5.move_to_state(pose, gripper="open", speed=5, is_radian=False, use_safety_offset=True)
        # Test reading arm states
        joint_state, arm_pose, gripper_state = ur5.get_arm_states(use_radians=False)
        print("Joint State:", joint_state)
        print("Arm Pose:", arm_pose)
        input("Press Enter to continue...")
        ur5.reset(requiere_user_input=True)

    finally:
        ur5.shutdown()

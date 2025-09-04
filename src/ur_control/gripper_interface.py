"""
Interface for controlling the AG95 gripper via Modbus RTU over TCP/IP.
"""

import time
import socket
import struct
from enum import IntEnum

class GripperState(IntEnum):
    CLOSED = 0
    OPENED = 1

class Gripper:
    def __init__(self, robot_ip, tcp_port=54321, slave_id=1, timeout=1.0):
        self.robot_ip = robot_ip
        self.tcp_port = tcp_port
        self.slave_id = slave_id
        self.sock = None
        self.timeout = timeout
        
        # State tracking
        self.is_ready = False
        self.binary_state = GripperState.CLOSED
        self.state = 0

    @staticmethod
    def _crc16(data: bytes) -> bytes:
        crc = 0xFFFF
        for pos in data:
            crc ^= pos
            for _ in range(8):
                if crc & 0x0001:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        # Return low byte first, then high byte
        return struct.pack('<H', crc)

    def connect(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(self.timeout)
        self.sock.connect((self.robot_ip, self.tcp_port))
        time.sleep(1.0)  # small pause for stability
        print(f"Connected to gripper at {self.robot_ip}:{self.tcp_port}")

    def close_conn(self):
        if self.sock:
            self.sock.close()
            self.sock = None


    def _write_register(self, reg_addr: int, value: int):
        # Modbus RTU frame: [slave][func=0x06][reg_hi][reg_lo][val_hi][val_lo][crc_lo][crc_hi]
        frame = bytearray()
        frame.append(self.slave_id)
        frame.append(0x06)  # Write Single Register
        frame += struct.pack('>H', reg_addr)  # big-endian for addr
        frame += struct.pack('>H', value)     # big-endian for value
        frame += self._crc16(frame)

        self.sock.sendall(frame)
        # Expect echo of request
        try:
            resp = self.sock.recv(8)
            return resp
        except socket.timeout:
            raise RuntimeError("No response from gripper")

    # ---------------------------
    # High-level AG95 commands
    # ---------------------------
    def initialize(self, full=False):
        """Initialize gripper. full=True => full init (find min & max)."""
        val = 0xA5 if full else 0x01
        self._write_register(0x0100, val)
        time.sleep(2)  # give it some time to complete init

    def set_force(self, percent: int):
        """Set grip force: 20-100%."""
        pct = max(20, min(100, percent))
        self._write_register(0x0101, pct)

    def set_position(self, permil: int):
        """Set target position: 0-1000‰ (0=closed, 1000=open)."""
        pos = max(0, min(1000, permil))
        self.state = pos
        self._write_register(0x0103, pos)

    def open(self):
        """Fully open gripper."""
        self.set_position(1000)
        self.binary_state = GripperState.OPENED
        self.state = 1000

    def close(self):
        """Fully close gripper."""
        self.set_position(0)
        self.binary_state = GripperState.CLOSED
        self.state = 0

    def grip_width(self, width_permille):
        """Grip to a custom opening."""
        self.set_position(width_permille)

    # ---------------------------
    # Convenience: init & default settings
    # ---------------------------
    def setup(self, force=50, do_calibration=True):
        """Initialize and set defaults."""
        self.initialize(full=do_calibration)
        self.set_force(force)
        time.sleep(1.0)
        self.close()  # ensure gripper is closed after setup
        print("Gripper setup complete.")

if __name__ == "__main__":
    # Connect to URCap RS485 TCP bridge
    gripper = Gripper(robot_ip="192.168.0.15", tcp_port=54321)

    try:
        gripper.connect()
        gripper.setup(force=50)   # full init, force=50%, open gripper
        breakpoint()
        input("Press Enter to test gripper...")
        gripper.open()            # open fully
        time.sleep(1)             # wait for gripper to open
        gripper.close()           # close fully
    finally:
        gripper.close_conn()

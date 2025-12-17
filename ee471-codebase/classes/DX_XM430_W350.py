# (c) 2025 S. Farzan, Electrical Engineering Department, Cal Poly
# Motor Class for Dynamixel (X-Series) XM430-W350
# The OpenManipulator-X arm consists of five of these units (four joints + gripper).
# This class abstracts the low-level commands to the Dynamixel motors.

from dynamixel_sdk import (
    PortHandler, PacketHandler, GroupBulkWrite, GroupBulkRead,
    DXL_LOBYTE, DXL_HIBYTE, DXL_LOWORD, DXL_HIWORD
)

import time

"""
A class for controlling Dynamixel XM430-W350 motors, used in OpenManipulator-X.
Handles low-level operations such as reading and writing to the motor's registers.
"""
class DX_XM430_W350:
    # Constants for motor control
    LIB_NAME = 'dxl_x64_c'  # Adjust for your OS: dxl_x86_c / dxl_x64_c / libdxl_x86_c / libdxl_x64_c / libdxl_mac_c
    BAUDRATE = 1000000      # Communication speed
    PROTOCOL_VERSION = 2.0  # Protocol version used by Dynamixel motors
    COMM_SUCCESS = 0        # Communication success constant
    COMM_TX_FAIL = -1001    # Communication failure constant

    # Control table definitions (refer to the motor's documentation: https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#control-table-of-ram-area)
    DRIVE_MODE = 10         # https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#drive-mode10
    OPR_MODE = 11           # https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#operating-mode11
    TORQUE_ENABLE = 64      # Enable Torque Control
    LED = 65                # Turn LED on/off
    GOAL_CURRENT = 102      # Set/Get goal current
    GOAL_VELOCITY = 104     # Set/Get goal velocity
    PROF_ACC = 108          # set acceleration time for trapezoid profile
    PROF_VEL = 112          # set profile time for trapezoid profile
    GOAL_POSITION = 116     # Set/Get goal position
    CURR_CURRENT = 126      # Get current current
    CURR_VELOCITY = 128     # Get current velocity
    CURR_POSITION = 132     # Get current position

    # Message lengths in bytes for different registers
    DRIVE_MODE_LEN = 1
    VEL_PROF = 0
    TIME_PROF = 4
    OPR_MODE_LEN = 1
    CURR_CNTR_MD = 0
    VEL_CNTR_MD = 1
    POS_CNTR_MD = 3
    EXT_POS_CNTR_MD = 4
    CURR_POS_CNTR_MD = 5
    PWM_CNTR_MD = 16
    TORQUE_ENABLE_LEN = 1
    LED_LEN = 1
    PROF_ACC_LEN = 4
    PROF_VEL_LEN = 4
    CURR_LEN = 2
    VEL_LEN = 4
    POS_LEN = 4

    # Unit conversions based on motor specifications
    MS_PER_S = 1000
    TICKS_PER_ROT = 4096
    TICK_POS_OFFSET = TICKS_PER_ROT/2   # position value for a joint angle of 0 (2048 for this case)
    TICKS_PER_DEG = TICKS_PER_ROT/360
    TICKS_PER_ANGVEL = 1/(0.229 * 6)    # 1 tick = 0.229 rev/min = 0.229*360/60 deg/s
    TICKS_PER_mA = 1/2.69               # 1 tick = 2.69 mA

    """
    Initializes the motor instance with specified communication handlers.
    Parameters:
    port_handler (PortHandler): PortHandler object from Dynamixel SDK.
    packet_handler (PacketHandler): PacketHandler object from Dynamixel SDK.
    port (str): Port name.
    motor_id (int): Unique ID for the motor.
    """
    def __init__(self, port_handler, packet_handler, port, motor_id):
        self.port = port
        self.id = motor_id # Dynamixel ID
        self.port_handler = port_handler
        self.packet_handler = packet_handler

    """
    Retrieves the current joint positions, velocities, and efforts from the motor.
    Returns:
    list of 1x3 float: Contains the joint positions (degrees), velocities (degrees/second), and efforts (mA).
    """
    # --- signed helper ---
    @staticmethod
    def _to_signed(val: int, bits: int) -> int:
        return val - (1 << bits) if val >= (1 << (bits - 1)) else val

    def get_joint_readings(self):
        cur_pos_raw = self.read_data(self.CURR_POSITION, self.POS_LEN)
        cur_vel_raw = self.read_data(self.CURR_VELOCITY, self.VEL_LEN)
        cur_curr_raw = self.read_data(self.CURR_CURRENT, self.CURR_LEN)

        cur_pos = (cur_pos_raw - self.TICK_POS_OFFSET) / self.TICKS_PER_DEG
        cur_vel = self._to_signed(cur_vel_raw, 32) / self.TICKS_PER_ANGVEL
        cur_curr = self._to_signed(cur_curr_raw, 16) / self.TICKS_PER_mA
        return [float(cur_pos), float(cur_vel), float(cur_curr)]

    """
    Commands the motor to move to the specified angle.
    Parameters:
    angle (float): The target angle in degrees for the motor.
    """
    def write_position(self, angle):
        position = round(angle * self.TICKS_PER_DEG + self.TICK_POS_OFFSET) % self.TICKS_PER_ROT
        self.write_data(self.GOAL_POSITION, position, self.POS_LEN)

    def read_position(self):
        pos = self.read_data(self.CURR_POSITION, self.POS_LEN)
        angle = ((pos - self.TICK_POS_OFFSET) % self.TICKS_PER_ROT) / self.TICKS_PER_DEG
        return angle

    """
    Creates a time-based trapezoidal profile for motor movements.
    This will cause writePosition to take the desired number of seconds to reach the setpoint. 
    Parameters:
    time (float): Total profile time in seconds. If set to 0, the profile is disabled.
    acc_time (float, optional): Total acceleration time for the profile. Defaults to one third of the total time.
    """
    # Time-based or Velocity-based
    def set_time_based_profile(self, enable: bool):
        dm = self.read_data(self.DRIVE_MODE, self.DRIVE_MODE_LEN)
        dm = (dm | 0b100) if enable else (dm & ~0b100)
        self.write_data(self.DRIVE_MODE, dm, self.DRIVE_MODE_LEN)

    def write_time(self, time, acc_time=None):
        if acc_time is None:
            acc_time = time / 3.0
        self.set_time_based_profile(True)  # <- ensure time-based profile
        self.write_data(self.PROF_ACC, int(acc_time * self.MS_PER_S), self.PROF_ACC_LEN)
        self.write_data(self.PROF_VEL, int(time * self.MS_PER_S), self.PROF_VEL_LEN)

    """
    Commands the motor to achieve a specified angular velocity.
    Parameters:
    velocity (float): The target angular velocity in degrees per second.
    """
    def write_velocity(self, velocity):
        vel_ticks = int(velocity * self.TICKS_PER_ANGVEL)
        self.write_data(self.GOAL_VELOCITY, vel_ticks, self.VEL_LEN)

    """
    Supplies the motor with a specified current.
    Parameters:
    current (float): The current in milliamperes to be supplied to the motor.
    """
    def write_current(self, current):
        current_in_ticks = int(current * self.TICKS_PER_mA)
        self.write_data(self.GOAL_CURRENT, current_in_ticks, self.CURR_LEN)

    """
    Enables or disables torque to hold the motor's position.
    Parameters:
    enable (bool): True to enable torque, False to disable it.
    """
    def toggle_torque(self, enable):
        state = 1 if enable else 0
        self.write_data(self.TORQUE_ENABLE, state, self.TORQUE_ENABLE_LEN)

    """
    Turns the motor's LED on or off.
    Parameters:
    enable (bool): True to turn the LED on, False to turn it off.
    """
    def toggle_led(self, enable):
        state = 1 if enable else 0
        self.write_data(self.LED, state, self.LED_LEN)

    """
    Changes the motor's operating mode.
    Parameters:
    mode (str): Specifies the new operating mode. Valid options include 'current', 'velocity', 
                'position', 'ext position', 'curr position', and 'pwm voltage'.  
    Raises:
    ValueError: If the specified mode is not supported.
    """
    def set_operating_mode(self, mode: str):
        was_enabled = bool(self.read_data(self.TORQUE_ENABLE, self.TORQUE_ENABLE_LEN))
        self.toggle_torque(False)

        if mode in ('current','c'):
            write_mode = self.CURR_CNTR_MD
        elif mode in ('velocity','v'):
            write_mode = self.VEL_CNTR_MD
        elif mode in ('position','p'):
            write_mode = self.POS_CNTR_MD
        elif mode in ('ext position','ep'):
            write_mode = self.EXT_POS_CNTR_MD
        elif mode in ('curr position','cp'):
            write_mode = self.CURR_POS_CNTR_MD
        elif mode in ('pwm voltage','pwm'):
            write_mode = self.PWM_CNTR_MD
        else:
            raise ValueError(f"Unsupported operating mode: {mode}")

        self.write_data(self.OPR_MODE, write_mode, self.OPR_MODE_LEN)

        if was_enabled:
            self.toggle_torque(True)

    """
    Verifies if the packet was sent/received properly, throws an error if not.
    Parameters:
    addr (int): The address of the control table index of the last read/write.
    msg (int, optional): The message sent with the last write operation.
    Raises:
    Exception: If communication fails or an error packet is received.
    """
    def check_packet(self, addr, msg=None):
        dxl_comm_result = self.packet_handler.getLastTxRxResult(self.port_handler, self.PROTOCOL_VERSION)
        dxl_error = self.packet_handler.getLastRxPacketError(self.port_handler, self.PROTOCOL_VERSION)
        packet_error = (dxl_comm_result != self.COMM_SUCCESS) or (dxl_error != 0)

        if msg is not None and packet_error:
            print(f'[msg] {msg}')

        if dxl_comm_result != self.COMM_SUCCESS:
            print(f'[addr:{addr}] {self.packet_handler.getTxRxResult(self.PROTOCOL_VERSION, dxl_comm_result)}')
            raise Exception("Communication Error: See above.")
        elif dxl_error != 0:
            print(f'[addr:{addr}] {self.packet_handler.getRxPacketError(self.PROTOCOL_VERSION, dxl_error)}')
            raise Exception("Received Error Packet: See above.")

    def _fatal(self, msg: str) -> None:
        raise RuntimeError(msg)

    # Attempts to connect via serial
    def startConnection(self):
        if not self.portHandler.openPort():
            self._fatal(f"Failed to open serial port '{self.portHandler.getPortName()}'.")
        if not self.portHandler.setBaudRate(self.BAUDRATE):
            # Avoid leaving the port open on failure
            self.portHandler.closePort()
            self._fatal(f"Failed to set baud {self.BAUDRATE} on '{self.portHandler.getPortName()}'.")

        self._connected = True

    def enable_torque(self):
        self.packet_handler.write1ByteTxRx(self.port_handler, self.id, self.TORQUE_ENABLE, 1)

    def disable_torque(self):
        self.packet_handler.write1ByteTxRx(self.port_handler, self.id, self.TORQUE_ENABLE, 0)

    def write_data(self, address, data, length):
        if length == 1:
            dxl_comm, dxl_err = self.packet_handler.write1ByteTxRx(self.port_handler, self.id, address, data)
        elif length == 2:
            dxl_comm, dxl_err = self.packet_handler.write2ByteTxRx(self.port_handler, self.id, address, data)
        elif length == 4:
            # Important: write *TxRx* so we consume the status packet and avoid stale reads
            dxl_comm, dxl_err = self.packet_handler.write4ByteTxRx(self.port_handler, self.id, address, data)
        else:
            raise ValueError("Invalid length for write operation")

        if dxl_comm != self.COMM_SUCCESS or dxl_err != 0:
            # raise for visibility
            raise RuntimeError(
                f"DXL write failed (id={self.id}, addr=0x{address:X}, len={length}): "
                f"{self.packet_handler.getTxRxResult(dxl_comm)} / {self.packet_handler.getRxPacketError(dxl_err)}"
            )

    def read_data(self, address, length, retries: int = 3, delay: float = 0.005):
        """
        Robust read with tiny retries to survive occasional missing status packets.
        Returns an int (0 if all retries fail).
        """
        for _ in range(retries):
            if length == 1:
                value, dxl_comm, dxl_err = self.packet_handler.read1ByteTxRx(self.port_handler, self.id, address)
            elif length == 2:
                value, dxl_comm, dxl_err = self.packet_handler.read2ByteTxRx(self.port_handler, self.id, address)
            elif length == 4:
                value, dxl_comm, dxl_err = self.packet_handler.read4ByteTxRx(self.port_handler, self.id, address)
            else:
                return 0
            if dxl_comm == self.COMM_SUCCESS and dxl_err == 0:
                return int(value)
            # No/short reply this try; brief backoff and try again
            time.sleep(delay)
        # Fallback if all retries fail
        return 0

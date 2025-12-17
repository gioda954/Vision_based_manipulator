# (c) 2025 S. Farzan, Electrical Engineering Department, Cal Poly
# OM_X_arm class for OpenManipulator-X Robot for EE 471
# This class/file should not need to be modified in any way for EE 471

import serial.tools.list_ports
from .DX_XM430_W350 import DX_XM430_W350
from dynamixel_sdk import (
    PortHandler, PacketHandler, GroupBulkWrite, GroupBulkRead,
    DXL_LOBYTE, DXL_HIBYTE, DXL_LOWORD, DXL_HIWORD
)

"""
OM_X_arm class for the OpenManipulator-X Robot.
Abstracts the serial connection and read/write methods from the Robot class.
"""
class OM_X_arm:
    """
    Initialize the OM_X_arm class.
    Sets up the serial connection, motor IDs, and initializes the motors and gripper.
    """
    def __init__(self):
        self.motorsNum = 4
        self.motorIDs = [11, 12, 13, 14]
        self.gripperID = 15
        self.deviceName = self.find_device_name()

        print(f"Port #: {self.deviceName}")
        self.port_handler = PortHandler(self.deviceName)
        self.packet_handler = PacketHandler(DX_XM430_W350.PROTOCOL_VERSION)

        # Create array of motors
        self.motors = [DX_XM430_W350(self.port_handler, self.packet_handler, self.deviceName, motor_id) for motor_id in self.motorIDs]
        
        # Create Gripper and set operating mode/torque
        self.gripper = DX_XM430_W350(self.port_handler, self.packet_handler, self.deviceName, self.gripperID)
        
        if not self.port_handler.openPort():
            print("Failed to open the port.")
            self._print_port_hint()
            self._fatal(f"Could not open '{self.port_handler.getPortName()}'")

        if not self.port_handler.setBaudRate(DX_XM430_W350.BAUDRATE):
            print("Failed to change the baudrate; closing port.")
            self.port_handler.closePort()
            self._fatal(f"Could not set {DX_XM430_W350.BAUDRATE} on '{self.port_handler.getPortName()}'")

        print("Port open OK; baudrate set.")

        self.gripper.set_operating_mode('position') # TODO
        self.gripper.toggle_torque(True) # TODO

        # Ensure all servos reply to instructions (SRL = 2)
        for m in self.motors + [self.gripper]:
            m.write_data(68, 2, 1)  # Address 68, length 1, value 2

        self.groupBulkWrite = GroupBulkWrite(self.port_handler, self.packet_handler)
        self.groupBulkRead = GroupBulkRead(self.port_handler, self.packet_handler)

        # Enable motors and set drive mode
        enable = 1
        disable = 0
        self.bulk_read_write(DX_XM430_W350.TORQUE_ENABLE_LEN, DX_XM430_W350.TORQUE_ENABLE, [enable]*self.motorsNum)
        self.bulk_read_write(DX_XM430_W350.DRIVE_MODE_LEN, DX_XM430_W350.DRIVE_MODE, [DX_XM430_W350.TIME_PROF]*self.motorsNum)
        # dxl_mode_result = self.bulk_read_write(DX_XM430_W350.DRIVE_MODE_LEN, DX_XM430_W350.DRIVE_MODE)
        # if dxl_mode_result[0] != DX_XM430_W350.TIME_PROF:
        #     print("Failed to set the DRIVE_MODE to TIME_PROF")
        #     quit()
        self.bulk_read_write(DX_XM430_W350.TORQUE_ENABLE_LEN, DX_XM430_W350.TORQUE_ENABLE, [disable]*self.motorsNum)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()

    def __del__(self):
        # Best-effort cleanup; not guaranteed on interpreter shutdown
        try:
            self.close()
        except Exception:
            pass

    """
    Finds the device name for the serial connection.
    Returns:
    str: The device name (e.g., 'COM3' for Windows, 'ttyUSB0' for Linux).
    Raises:
    Exception: If no serial devices are found.
    """
    def find_device_name(self):
        ports = list(serial.tools.list_ports.comports())
        for port in ports:
            if 'COM' in port.device and port.device != 'COM1':  # Skip COM1 # ttyUSB
                return port.device
        raise Exception("Failed to connect via serial, no devices found.")

    """
    Reads or writes messages of length n from/to the desired address for all joints.
    Parameters:
    n (int): The size in bytes of the message (1 for most settings, 2 for current, 4 for velocity/position).
    addr (int): Address of control table index to read from or write to.
    msgs (list of 1x4 int, optional): The messages (in bytes) to send to each joint, respectively. If not provided, a read operation is performed. If a single integer is provided, the same message will be sent to all four joints.
    Returns:
    list of int: The result of a bulk read, empty if bulk write.
    """
    def bulk_read_write(self, n, addr, msgs=None):
        if msgs is None:  # Bulk read
            results = []
            self.groupBulkRead.clearParam()

            # Add all params
            for motor_id in self.motorIDs:
                ok = self.groupBulkRead.addParam(motor_id, addr, n)
                if ok != True:
                    self.groupBulkRead.clearParam()
                    raise RuntimeError(f"[ID:{motor_id:03d}] GroupBulkRead addParam failed (addr=0x{addr:X}, n={n})")

            # Read
            dxl_comm_result = self.groupBulkRead.txRxPacket()
            if dxl_comm_result != DX_XM430_W350.COMM_SUCCESS:
                self.groupBulkRead.clearParam()
                raise RuntimeError(self.packet_handler.getTxRxResult(dxl_comm_result))

            # Fetch data
            for motor_id in self.motorIDs:
                if not self.groupBulkRead.isAvailable(motor_id, addr, n):
                    self.groupBulkRead.clearParam()
                    raise RuntimeError(f"[ID:{motor_id:03d}] BulkRead data not available at 0x{addr:X} (len={n})")
                result = self.groupBulkRead.getData(motor_id, addr, n)
                results.append(result)

            self.groupBulkRead.clearParam()
            return results

        else:  # Bulk write
            if len(msgs) != len(self.motorIDs):
                raise ValueError(f"'msgs' length {len(msgs)} must match number of motors {len(self.motorIDs)}")

            self.groupBulkWrite.clearParam()

            # Add values to the Bulk write parameter storage
            for i, motor_id in enumerate(self.motorIDs):
                val = int(msgs[i])
                try:
                    param_msg = self._pack_le(val, n)  # masks & packs LE for n=1/2/4
                except ValueError:
                    self.groupBulkWrite.clearParam()
                    raise  # re-raise the ValueError("Unsupported length ...")

                ok = self.groupBulkWrite.addParam(motor_id, addr, n, param_msg)
                if ok != True:
                    self.groupBulkWrite.clearParam()  # avoid stale entries on next call
                    raise RuntimeError(f"[ID:{motor_id:03d}] GroupBulkWrite addParam failed (addr=0x{addr:X}, n={n})")

            # Bulk write
            dxl_comm_result = self.groupBulkWrite.txPacket()
            # Clear bulkwrite parameter storage regardless of success
            self.groupBulkWrite.clearParam()
            if dxl_comm_result != DX_XM430_W350.COMM_SUCCESS:
                raise RuntimeError(self.packet_handler.getTxRxResult(dxl_comm_result))
            # (intentionally no return value, preserving original behavior)

    def close(self):
        """Release SDK buffers and close the serial port (safe to call multiple times)."""
        try:
            self.groupBulkRead.clearParam()
        except Exception:
            pass
        try:
            self.groupBulkWrite.clearParam()
        except Exception:
            pass
        try:
            self.port_handler.closePort()
        except Exception:
            pass

    def _fatal(self, msg: str) -> None:
        """Fail in a catchable way (instead of quit())."""
        raise RuntimeError(msg)

    def _pack_le(self, value: int, n: int):
        """Little-endian packer for 1/2/4 byte values (for GroupBulkWrite)."""
        if n == 4:
            return [DXL_LOBYTE(DXL_LOWORD(value)), DXL_HIBYTE(DXL_LOWORD(value)),
                    DXL_LOBYTE(DXL_HIWORD(value)), DXL_HIBYTE(DXL_HIWORD(value))]
        if n == 2:
            return [DXL_LOBYTE(value), DXL_HIBYTE(value)]
        if n == 1:
            return [value & 0xFF]
        raise ValueError(f"Unsupported length n={n}")

    def _print_port_hint(self):
        try:
            print("Available serial ports:")
            for p in serial.tools.list_ports.comports():
                print("  -", p.device)
        except Exception:
            pass

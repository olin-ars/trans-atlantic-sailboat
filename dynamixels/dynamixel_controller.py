"""
Note: This module uses code adapted from the Robotis DynamixelSDK (https://github.com/ROBOTIS-GIT/DynamixelSDK/).
"""

from threading import Thread

from dynamixels import dynamixel_functions as dynamixel
import time


class DynamixelController:
    current_position = 0

    def __init__(self, device_id, usb_device='/dev/ttyUSB0', baudrate=1000000):

        self.config = DynamixelConfig()
        self.config.baudrate = baudrate

        # Initialize PortHandler
        self.port_num = dynamixel.portHandler(self.config.serial_port)

        # Initialize PacketHandler
        dynamixel.packetHandler()

        self.config.motor_id = device_id

        # Open port
        if not dynamixel.openPort(self.port_num):
            print("Failed to open the port!")
            self.configured_successfully = False
            return

        # Set port baudrate
        if not dynamixel.setBaudRate(self.port_num, baudrate):
            print("Failed to change the baudrate!")
            self.configured_successfully = False
            return

        # Enable torque control
        self.enable_torque()

        while True:
            ipt = int(input("Motor position: "))
            if ipt < 0:
                break
            print('Setting position to {}'.format(ipt))
            self.set_position(ipt)
            time.sleep(3)

            # Read present position
            current_position = dynamixel.read2ByteTxRx(self.port_num, self.config.PROTOCOL_VERSION, self.config.motor_id,
                                                       self.config.ADDR_MX_PRESENT_POSITION)
            dxl_error = dynamixel.getLastRxPacketError(self.port_num, self.config.PROTOCOL_VERSION)
            if dxl_error != 0:
                print(dynamixel.getRxPacketError(self.config.PROTOCOL_VERSION, dxl_error))
            else:
                self.position_read(current_position)

    def set_position(self, theta):
        # Write goal position
        dynamixel.write2ByteTxRx(self.port_num, self.config.PROTOCOL_VERSION, self.config.motor_id,
                                 self.config.ADDR_MX_GOAL_POSITION, theta)
        dxl_comm_result = dynamixel.getLastTxRxResult(self.port_num, self.config.PROTOCOL_VERSION)
        dxl_error = dynamixel.getLastRxPacketError(self.port_num, self.config.PROTOCOL_VERSION)
        if dxl_comm_result != self.config.COMM_SUCCESS:
            print(dynamixel.getTxRxResult(self.config.PROTOCOL_VERSION, dxl_comm_result))
            return False
        elif dxl_error != 0:
            print(dynamixel.getRxPacketError(self.config.PROTOCOL_VERSION, dxl_error))
            return False
        # Position set successfully
        return True

    def enable_torque(self):
        # Enable Dynamixel Torque
        dynamixel.write1ByteTxRx(self.port_num, self.config.PROTOCOL_VERSION, self.config.motor_id,
                                 self.config.ADDR_MX_TORQUE_ENABLE, self.config.TORQUE_ENABLE)
        dxl_comm_result = dynamixel.getLastTxRxResult(self.port_num, self.config.PROTOCOL_VERSION)
        dxl_error = dynamixel.getLastRxPacketError(self.port_num, self.config.PROTOCOL_VERSION)
        if dxl_comm_result != self.config.COMM_SUCCESS:
            print(dynamixel.getTxRxResult(self.config.PROTOCOL_VERSION, dxl_comm_result))
            self.config.configured_successfully = False
        elif dxl_error != 0:
            print(dynamixel.getRxPacketError(self.config.PROTOCOL_VERSION, dxl_error))
            self.configured_successfully = False
        else:
            print("Dynamixel has been successfully connected")
            self.configured_successfully = True

    def disable_torque(self):
        # Disable Dynamixel Torque
        dynamixel.write1ByteTxRx(self.port_num, self.config.PROTOCOL_VERSION, self.config.motor_id,
                                 self.config.ADDR_MX_TORQUE_ENABLE, self.config.TORQUE_DISABLE)
        dxl_comm_result = dynamixel.getLastTxRxResult(self.port_num, self.config.PROTOCOL_VERSION)
        dxl_error = dynamixel.getLastRxPacketError(self.port_num, self.config.PROTOCOL_VERSION)
        if dxl_comm_result != self.config.COMM_SUCCESS:
            print(dynamixel.getTxRxResult(self.config.PROTOCOL_VERSION, dxl_comm_result))
        elif dxl_error != 0:
            print(dynamixel.getRxPacketError(self.config.PROTOCOL_VERSION, dxl_error))

        # Close port
        dynamixel.closePort(self.port_num)

    """
    Updates the position reported by the Dynamixel (called by a thread).
    """

    def position_read(self, theta):
        self.current_position = theta
        print('Motor at position {}'.format(theta))


class DynamixelConfig:
    # ---------- Configuration ----------
    motor_id = 0
    serial_port = "/dev/ttyUSB1".encode('utf-8')  # Check which port is being used on your controller
    baudrate = 1000000

    # ---------- Constants ----------
    # Protocol version
    PROTOCOL_VERSION = 1  # See which protocol version is used in the Dynamixel
    DXL_MOVING_STATUS_THRESHOLD = 10  # Dynamixel moving status threshold

    # (You probably don't want to change anything in this class below here)
    # Control table address
    ADDR_MX_TORQUE_ENABLE = 24  # Control table address (varies model to model, so check this)
    ADDR_MX_GOAL_POSITION = 30
    ADDR_MX_PRESENT_POSITION = 36

    TORQUE_ENABLE = 1  # Value for enabling the torque
    TORQUE_DISABLE = 0  # Value for disabling the torque

    COMM_SUCCESS = 0  # Communication Success result value
    COMM_TX_FAIL = -1001  # Communication Tx Failed


class PositionMonitor(Thread):
    def __init__(self, port, config, position_read_callback):
        Thread.__init__(self)
        self.port = port
        self.config = config
        self.callback = position_read_callback

    def run(self):
        # Read present position
        current_position = dynamixel.read2ByteTxRx(self.port, self.config.PROTOCOL_VERSION, self.config.dxl_id,
                                                   self.config.ADDR_MX_PRESENT_POSITION)
        dxl_error = dynamixel.getLastRxPacketError(self.config.port_num, self.config.PROTOCOL_VERSION)
        if dxl_error != 0:
            print(dynamixel.getRxPacketError(self.config.PROTOCOL_VERSION, dxl_error))
        else:
            self.callback(current_position)


if __name__ == '__main__':
    DynamixelController(1)

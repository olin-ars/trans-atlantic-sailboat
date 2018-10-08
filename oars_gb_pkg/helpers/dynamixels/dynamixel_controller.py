"""
Note: This module uses code adapted from the Robotis DynamixelSDK (https://github.com/ROBOTIS-GIT/DynamixelSDK/).
"""

from threading import Thread
import time

from oars_gb_pkg.helpers.dynamixels import dynamixel_functions as dynamixel


class DynamixelController:

    def __init__(self, device_id, usb_device='/dev/ttyUSB0', baudrate=1000000, torque=False):

        self.config = DynamixelConfig()
        self.config.baudrate = baudrate
        self.config.motor_id = device_id
        self.current_position = None

        # Initialize PortHandler
        self.port_num = dynamixel.portHandler(self.config.serial_port)

        # Initialize PacketHandler
        dynamixel.packetHandler()

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
        if torque:
            self.enable_torque()

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
        """
        Enables torque
        """
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
        """
        Disables torque
        """
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
    serial_port = "/dev/ttyUSB0".encode('utf-8')  # Check which port is being used on your controller
    baudrate = 1000000

    # ---------- Constants ----------
    # Protocol version
    PROTOCOL_VERSION = 1  # See which protocol version is used in the Dynamixel
    DXL_MOVING_STATUS_THRESHOLD = 10  # Dynamixel moving status threshold

    # Control table address
    ADDR_MX_TORQUE_ENABLE = 24  # Control table address (varies model to model, so check this)
    ADDR_MX_GOAL_POSITION = 30
    ADDR_MX_PRESENT_POSITION = 36
    ADDR_MX_SHUTDOWN_CONTROL = 63

    TORQUE_ENABLE = 1  # Value for enabling the torque
    TORQUE_DISABLE = 0  # Value for disabling the torque

    SHUTDOWN_OPTIONS = 0b00100100  # Shut down if overloaded or overheated

    COMM_SUCCESS = 0  # Communication Success result value
    COMM_TX_FAIL = -1001  # Communication Tx Failed


class PositionMonitor(Thread):
    def __init__(self, port, config, position_read_callback, update_frequency=2):
        """
        The PositionMonitor thread reads the motor position repeatedly and reports the reading via a callback.
        :param port: the serial port to use to connect to the motor
        :param config: the Dynamixel configuration of the motor to watch
        :param position_read_callback: called with the motor angle (in radians) as an argument to report a reading
        :param update_frequency: the frequency at which to read the motors (in hertz)
        """
        Thread.__init__(self)
        self.port = port
        self.config = config
        self.callback = position_read_callback
        self.sleep_period = 1/update_frequency
        self.running = True

    def run(self):
        while self.running:
            # Read present position
            current_position = dynamixel.read2ByteTxRx(self.port, self.config.PROTOCOL_VERSION, self.config.dxl_id,
                                                       self.config.ADDR_MX_PRESENT_POSITION)
            dxl_error = dynamixel.getLastRxPacketError(self.config.port_num, self.config.PROTOCOL_VERSION)
            if dxl_error != 0:
                print(dynamixel.getRxPacketError(self.config.PROTOCOL_VERSION, dxl_error))
            else:
                self.callback(current_position)

            time.sleep(self.sleep_period)


if __name__ == '__main__':
    DynamixelController(DynamixelController.JIB)

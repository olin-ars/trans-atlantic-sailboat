from serial.tools.list_ports import comports
from serial import SerialException


def resolve_device_port(serial_number):
    """
    Finds the serial port name associated with a given device's serial number.
    :param serial_number: the serial number of the device to look for.
    :return: the full path to the device (e.g. /dev/ttyUSB0), or None if the device could not be found.
    """
    try:
        for port in comports():
            if port.serial_number == serial_number:
                return port.device
    except SerialException:
        pass

    return None

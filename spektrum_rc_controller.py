from evdev import InputDevice

from dynamixels.dynamixel_motor import DynamixelMotor


class SpektrumRCController:

    MAIN_PROFILE = {
        'id': 1,
        'min': 0,
        'max': 13200
    }
    JIB_PROFILE = {
        'id': 2,
        'min': 0,
        'max': -13200
    }
    RUDDER_PROFILE = {
        'id': 0,
        'min': 300,
        'max': 710
    }

    def __init__(self, device):
        """
        Creates a new ControllerListener for listening to messages from an RC controller.
        """
        if not device:
            raise ValueError('Parameter "device" not specified')

        # Connect to the controller
        self.gamepad = InputDevice(device)

        # Initialize the motors
        self.main = DynamixelMotor(self.MAIN_PROFILE)
        self.jib = DynamixelMotor(self.JIB_PROFILE)
        self.rudder = DynamixelMotor(self.RUDDER_PROFILE)

    @staticmethod
    def normalize(pos, max_pos, min_pos):
        """
        Normalizes a position.
        :param pos: the position
        :param max_pos: the maximum of its range
        :param min_pos: the minimum of its range
        :return: the normalized value (between 0 and 1, inclusive)
        """
        return (pos - min_pos) / (max_pos - min_pos)

    def run(self, debug=False):

        for event in self.gamepad.read_loop():
            if not event.type == 0:
                if event.code == 1:
                    position = self.normalize(event.value, 1700, 0)
                    if debug:
                        print('Setting sails to position {:0.4f}'.format(position))
                    self.main.set_position(position, debug)
                    self.jib.set_position(position, debug)
                elif event.code == 2:
                    position = self.normalize(event.value, 1700, 170)
                    if debug:
                        print('Setting rudder to position {:0.4f}'.format(position))
                    self.rudder.set_position(position, debug)
                # elif event.code == TL_TOGG:  # Use top-left toggle switch for torque control
                #     if event.value == TOGG_DOWN:
                #         main.enable_torque()
                #         rudder.enable_torque()
                #     else:
                #         main.disable_torque()
                #         rudder.disable_torque()


if __name__ == '__main__':
    SpektrumRCController('/dev/input/by-id/usb-Horizon_Hobby_SPEKTRUM_RECEIVER_00000000001A-event-joystick').run(True)
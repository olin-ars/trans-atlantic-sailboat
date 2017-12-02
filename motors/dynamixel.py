from dynamixels.dynamixel_controller import DynamixelController


class DynamixelMotor:

    def __init__(self, profile):
        """
        Creates a new instance of a Dynamixel motor.
        :param profile: a motor profile with ID of the Dynamixel and its max and min positions
        """
        self.id = profile['id']
        self.motor = DynamixelController(self.id)
        self.min = profile['min']
        self.max = profile['max']
        self.range = self.max - self.min
        print('Range of Dynamixel {} is {}'.format(self.id, self.range))

    def set_position(self, pos, debug=False):
        """
        Moves the motor to the specified position.
        :param pos: a normalized (between 0 and 1, inclusive) position
        """
        posrange = pos * self.range
        pos = posrange + self.min
        if debug:
            print('Setting Dynamixel {} with posrange {} to position {}'.format(self.id, posrange, pos))
        self.motor.set_position(int(pos))

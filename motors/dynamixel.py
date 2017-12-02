from dynamixels.dynamixel_controller import DynamixelController


class DynamixelMotor:

    def __init__(self, profile):
        """
        Creates a new instance of a Dynamixel motor.
        :param profile: a motor profile with ID of the Dynamixel and its max and min positions
        """
        self.motor = DynamixelController(profile['id'])
        self.min = profile['min']
        self.max = profile['max']
        self.range = self.max - self.min

    def set_position(self, pos):
        """
        Moves the motor to the specified position.
        :param pos: a normalized (between 0 and 1, inclusive) position
        """
        pos = pos * self.range + self.min
        self.motor.set_position(int(pos))

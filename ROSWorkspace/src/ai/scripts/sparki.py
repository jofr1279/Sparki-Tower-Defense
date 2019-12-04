from enum import Enum


class Vector2(object):

    def __init__(self, x=0, y=0):
        """ Constructor for Vector2.

        @type x: int
        @type y: int
        """

        self.x = x
        self.y = y


class Direction(Enum):
    NORTH = 0
    EAST = 1
    SOUTH = 2
    WEST = 3


class Sparki(object):

    def __init__(self):
        self.position = Vector2(0, 0)
        self.direction = Direction.NORTH

        self.laser_range = 0
        """ The range (in grid units) in which Sparki's laser can fire. """

        self.servo_range = 0
        """ The range (in degrees) in which the servo can move. If this number is 45, then Sparki can move his servo 45
            degrees in each direction, or a total of 90 degrees. """

    def update(self, data):
        """ Updates the instance data using the data given by the Sparki node.

        @type data: dict
        """
        try:
            self.position.x = data['position']['x']
            self.position.y = data['position']['y']
            self.direction = data['direction']
            self.laser_range = data['laser_range']
            self.servo_range = data['servo_range']

        except KeyError:
            print 'Invalid data received from sparki node.'

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
        """ Constructor for Sparki.
        """

        self.position = Vector2(0, 0)
        self.direction = Direction.NORTH
        self.laser_range = 0

        self.servo_range = 0
        """ The range (degrees) in which the servo can move. If this number is 45, then Sparki can move his servo 45
            degrees in each direction, or a total of 90 degrees.
        """

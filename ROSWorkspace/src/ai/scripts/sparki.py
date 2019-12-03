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

    def __init__(self, position=None, direction=Direction.NORTH):
        """ Constructor for Sparki.

        @type position: Vector2
        @type direction: Direction
        """

        self.position = Vector2(0, 0) if position is None else position
        self.direction = direction

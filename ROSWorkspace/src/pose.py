from enum import Enum


class Vector2(object):
    """ Simple wrapper for x, y coordinate pairs.
    """

    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def __repr__(self):
        return '({}, {})'.format(self.x, self.y)


class Direction(Enum):
    """ Simple wrapper for cardinal directions.
    """

    NORTH = 0
    EAST = 1
    SOUTH = 2
    WEST = 3

    @staticmethod
    def from_num(num):
        return [Direction.NORTH, Direction.EAST, Direction.SOUTH, Direction.WEST][num]

    @staticmethod
    def to_motor(direction):
        return {
            Direction.NORTH: (1, 1),
            Direction.EAST: (1, -1),
            Direction.SOUTH: (-1, -1),
            Direction.WEST: (-1, 1)
        }[direction]

from enum import Enum


class Vector2(object):
    """ Simple wrapper for x, y coordinate pairs.
    """

    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def __repr__(self):
        return '({}, {})'.format(self.x, self.y)

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __sub__(self, other):
        return Vector2(
            self.x - other.x,
            self.y - other.y,
        )


class Direction(Enum):
    """ Simple wrapper for cardinal directions.
    """

    NORTH = 0
    EAST = 1
    SOUTH = 2
    WEST = 3

    @staticmethod
    def from_num(num):
        """ Converts a number ranging from 0 to 3 into a corresponding cardinal direction clockwise.

        @type num: int
        """

        return [Direction.NORTH, Direction.EAST, Direction.SOUTH, Direction.WEST][num]

    @staticmethod
    def to_motor(direction):
        """ Converts a direction into its corresponding motor power values.

        @type direction: Direction
        """

        return {
            Direction.NORTH: (1, 1),
            Direction.EAST: (1, -1),
            Direction.SOUTH: (-1, -1),
            Direction.WEST: (-1, 1)
        }[direction]

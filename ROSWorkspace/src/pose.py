class Vector2(object):
    """ Simple wrapper for x, y coordinate pairs.
    """

    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def __repr__(self):
        return 'Vector2({}, {})'.format(self.x, self.y)

    def __eq__(self, other):
        return (self.x == other.x and self.y == other.y) if isinstance(other, Vector2) else False

    def __add__(self, other):
        return Vector2(
            self.x + other.x,
            self.y + other.y,
        )

    def __sub__(self, other):
        return Vector2(
            self.x - other.x,
            self.y - other.y,
        )


class Direction(object):
    """ Simple wrapper for cardinal directions.
    """

    def __init__(self, num):
        self.num = num

    @staticmethod
    def from_num(num):
        """ Converts a number ranging from 0 to 3 into a corresponding cardinal direction clockwise.

        @type num: int
        """

        return [NORTH, EAST, SOUTH, WEST][num]

    @staticmethod
    def to_motor(direction):
        """ Converts a direction into its corresponding motor power values.

        @type direction: Direction
        """

        return {
            NORTH: (1, 1),
            EAST: (1, -1),
            SOUTH: (-1, -1),
            WEST: (-1, 1)
        }[direction]


NORTH = Direction(0)
EAST = Direction(1)
SOUTH = Direction(2)
WEST = Direction(3)

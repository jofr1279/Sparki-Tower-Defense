import math

from pose import Vector2, Direction


def pose_to_vector(pose):
    """ Converts pose (real world) to vector (grid coordinates).

    @type pose: Pose2D
    @rtype: Vector2
    """

    # TODO: The division factor will need to be adjusted.
    return Vector2(
        pose.x // 1,
        pose.y // 1,
    )


def pose_to_direction(pose):
    """ Converts pose (real world) to direction (grid coordinates)

    @type pose: Pose2D
    @rtype: Direction
    """

    return Direction.from_num(int(pose.theta // (math.pi / 2)))

import rospy

from pose import Vector2, Direction
from helper import pose_to_vector, pose_to_direction

from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray


class Sparki(object):
    """ Describes Sparki's pose and abilities.
    """

    def __init__(self, laser_range, servo_range):
        """
        @type laser_range: int
        @param laser_range The range (in grid units) in which Sparki's laser can fire.

        @type servo_range: int
        @param servo_range The range (in degrees) in which the servo can move. If this number is 45, then Sparki can
                           move his servo 45 degrees in each direction, or a total of 90 degrees.
        """

        self.laser_range = laser_range
        self.servo_range = servo_range

        self.position = Vector2(0, 0)
        self.direction = Direction.NORTH

        self.motor_publisher = rospy.Publisher('/sparki/motor_command', Float32MultiArray, queue_size=10)

    def _init_topics(self):
        rospy.Subscriber('/sparki/odometry', Pose2D, self._odometry_update)

    def _odometry_update(self, pose):
        """ Updates the position and direction data using the data given by the Sparki node.

        @type pose: Pose2D
        """
        self.odometry_update(pose_to_vector(pose), pose_to_direction(pose))

    def odometry_update(self, position, direction):
        """ Updates the position and direction.

        @type position: Vector2
        @type direction: Direction
        """

        self.position = position
        self.direction = direction

    def move(self, direction):
        """ Sets Sparki to move in the specified direction. If None is passed, Sparki will stop.

        @type direction: Direction | None
        """

        array = Float32MultiArray()
        array.data = 0, 0 if direction is None else Direction.to_motor(direction)

        self.motor_publisher.publish(array)

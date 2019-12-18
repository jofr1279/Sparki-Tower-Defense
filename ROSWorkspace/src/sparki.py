import time
import rospy

from pose import Vector2, NORTH, Direction
from config import CM_PER_GRID_CELL, SPARKI_RETRY_INTERVAL

from geometry_msgs.msg import Pose2D
from std_msgs.msg import Int16, Float32


class Sparki(object):
    """ Describes Sparki's pose and abilities.
    """

    def __init__(self, laser_range, servo_range, ready_function):
        """
        @type laser_range: int
        @param laser_range The range (in grid units) in which Sparki's laser can fire.

        @type servo_range: int
        @param servo_range The range (in degrees) in which the servo can move. If this number is 45, then Sparki can
                           move his servo 45 degrees in each direction, or a total of 90 degrees.
        """

        self.laser_range = laser_range
        self.servo_range = servo_range
        self.ready_function = ready_function

        self.last_ready_call = time.time()

        self.position = Vector2(5, 4)
        self.direction = NORTH

        self._init_topics()

    def _init_topics(self):
        rospy.Subscriber('/sparki/odometry', Pose2D, self.sparki_ready)

        self.move_publisher = rospy.Publisher('/sparki/forward_command', Float32, queue_size=10)
        self.turn_publisher = rospy.Publisher('/sparki/turn_command', Float32, queue_size=10)
        self.servo_publisher = rospy.Publisher('/sparki/set_servo', Int16, queue_size=10)

    def sparki_ready(self, _):
        """ Called when Sparki's odometry is online, indicating he is ready for another command. """

        if time.time() - self.last_ready_call > SPARKI_RETRY_INTERVAL:
            self.ready_function()
            self.last_ready_call = time.time()

    def move(self):
        """ Tells Sparki to move forward. """

        self.position += Direction.to_vector(self.direction)
        self.move_publisher.publish(Float32(CM_PER_GRID_CELL))

    def turn(self, angle):
        """ Tells Sparki to turn. """

        if angle < 0:
            self.direction = self.direction.left()
        else:
            self.direction = self.direction.right()

        self.turn_publisher.publish(Float32(angle))

    def look(self, angle):
        """ Sets Sparki's servo to look in the desired angle.

        @type angle: int
        @param angle: The angle (in degrees) the servo should look at.
        """

        self.servo_publisher.publish(angle)

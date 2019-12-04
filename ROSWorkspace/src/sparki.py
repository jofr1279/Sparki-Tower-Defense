import rospy

from pose import Vector2, Direction

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

    def update(self, position, direction):
        """ Updates the instance data using the data given by the Sparki node.

        @type position: Vector2
        @type direction: Direction
        """

        self.position = position
        self.direction = direction

    def move(self, direction):
        """ Sets Sparki to move in the specified direction. If None is passed, Sparki will stop.

        @type direction: Direction | None
        """

        arr = Float32MultiArray()

        if direction is None:
            arr.data = 0, 0
        else:
            arr.data = Direction.to_motor(direction)

        self.motor_publisher.publish(arr)

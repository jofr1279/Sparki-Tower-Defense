import rospy

from std_msgs.msg import String, Float32MultiArray

from config import WORLD_SIZE, GOAL_POSITION, SPARKI_LASER_RANGE, SPARKI_SERVO_RANGE
from world import World
from sparki import Sparki
from helper import to_float_array


class Game(object):

    def __init__(self):
        self._sparki = Sparki(SPARKI_LASER_RANGE, SPARKI_SERVO_RANGE)
        self._world = World(WORLD_SIZE, self._sparki, GOAL_POSITION)

        self._init_topics()

    def _init_topics(self):
        self._best_target_publisher = rospy.Publisher('/game/best_target', Float32MultiArray, queue_size=10)

    def update(self):
        best_direction = self._world.best_direction()
        best_target = self._world.best_target()
        best_target_angle = self._world.best_target_angle()

        self._sparki.move(best_direction)
        self._sparki.look(best_target_angle)
        self._best_target_publisher.publish(to_float_array(best_target))

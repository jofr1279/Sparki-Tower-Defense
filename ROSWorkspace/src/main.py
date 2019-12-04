#!/usr/bin/env python

import json
import math
import rospy

from geometry_msgs.msg import Pose2D
from std_msgs.msg import String, Float32MultiArray

from helper import pose_to_vector, pose_to_direction
from pose import Direction
from world import World
from sparki import Sparki
from config import NODE_NAME, WORLD_SIZE, GOAL_POSITION, SPARKI_LASER_RANGE, SPARKI_SERVO_RANGE


class GameNode(object):

    def __init__(self):
        rospy.init_node(NODE_NAME, anonymous=True)
        self._init_topics()

        self.sparki = Sparki(SPARKI_LASER_RANGE, SPARKI_SERVO_RANGE)
        self.world = World(WORLD_SIZE, self.sparki, GOAL_POSITION)

        print 'Node "{}" node initialized.'.format(NODE_NAME)

        rospy.spin()

    def _init_topics(self):
        rospy.Subscriber('/sparki/odometry', Pose2D, self._sparki_update)

        self.ai_publisher = rospy.Publisher('/game/ai', String, queue_size=10)

    def _sparki_update(self, pose):
        """ The handler function when Sparki's data is updated. Upon receiving the data, it updates the Sparki instance,
            gets the best direction and best target for Sparki, and publishes it.

        @type pose: Pose2D
        """

        self.sparki.move(Direction.NORTH)

        self.sparki.update(pose_to_vector(pose), pose_to_direction(pose))

        best_direction = self.world.best_direction()
        best_target = self.world.best_target()

        self.ai_publisher.publish(json.dumps({
            'direction': best_direction,
            'target': best_target,
        }))


if __name__ == '__main__':
    GameNode()

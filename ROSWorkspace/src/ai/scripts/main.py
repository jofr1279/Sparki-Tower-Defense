#!/usr/bin/env python

import json
import rospy

from std_msgs.msg import String

from world import World
from sparki import Vector2, Sparki

NODE_NAME = 'ai'
WORLD_SIZE = Vector2(20, 20)
GOAL_POSITION = Vector2(0, 0)


class AINode(object):

    def __init__(self):
        rospy.init_node(NODE_NAME, anonymous=True)

        self.sparki = Sparki()
        self.world = World(WORLD_SIZE, self.sparki, GOAL_POSITION)

        print 'Node "{}" node initialized'.format(NODE_NAME)

        rospy.spin()

    def _init_topics(self):
        rospy.Subscriber('sparki', String, self._sparki_update)

        self.ai_publisher = rospy.Publisher('ai', String)

    def _sparki_update(self, message):
        """ The handler function when Sparki's data is updated. Upon receiving the data, it updates the Sparki instance,
            gets the best direction and best target for Sparki, and publishes it.

        @type message: String
        """
        self.sparki.update(json.loads(message.data))

        best_direction = self.world.best_direction()
        best_target = self.world.best_target()

        self.ai_publisher.publish(json.dumps({
            'direction': best_direction,
            'target': best_target,
        }))


if __name__ == '__main__':
    AINode()

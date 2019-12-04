#!/usr/bin/env python

import rospy

from world import World
from sparki import Sparki
from config import NODE_NAME, WORLD_SIZE, GOAL_POSITION, SPARKI_LASER_RANGE, SPARKI_SERVO_RANGE


class GameNode(object):

    def __init__(self):
        rospy.init_node(NODE_NAME)

        self.sparki = Sparki(SPARKI_LASER_RANGE, SPARKI_SERVO_RANGE)
        self.world = World(WORLD_SIZE, self.sparki, GOAL_POSITION)

        print 'Node "{}" node initialized.'.format(NODE_NAME)

        rospy.spin()


if __name__ == '__main__':
    GameNode()

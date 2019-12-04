#!/usr/bin/env python
import rospy

from std_msgs.msg import String

from world import World
from sparki import Vector2, Sparki

NODE_NAME = 'ai'
WORLD_SIZE = Vector2(20, 20)
GOAL_POSITION = Vector2(0, 0)


if __name__ == '__main__':
    rospy.init_node(NODE_NAME, anonymous=True)

    sparki = Sparki()
    world = World(WORLD_SIZE, sparki, GOAL_POSITION)

    print 'Node "{}" node initialized'.format(NODE_NAME)

    rospy.spin()

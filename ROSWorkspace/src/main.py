#!/usr/bin/env python

import rospy

from game import Game
from config import NODE_NAME


if __name__ == '__main__':
    rospy.init_node(NODE_NAME)

    game = Game()

    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        game.update()

        rate.sleep()

    print 'Node "{}" node initialized.'.format(NODE_NAME)

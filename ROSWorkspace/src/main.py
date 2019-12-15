#!/usr/bin/env python

import rospy

from game import Game
from config import NODE_NAME


if __name__ == '__main__':
    rospy.init_node(NODE_NAME)

    game = Game()

    print 'Node "{}" node initialized.'.format(NODE_NAME)

    rospy.spin()

from pose import NORTH, EAST, SOUTH, WEST
from config import WORLD_SIZE, GOAL_POSITION, SPARKI_LASER_RANGE, SPARKI_SERVO_RANGE
from world import World
from sparki import Sparki


class Game(object):

    def __init__(self):
        self._sparki = Sparki(SPARKI_LASER_RANGE, SPARKI_SERVO_RANGE, self.sparki_ready)
        self._world = World(WORLD_SIZE, self._sparki, GOAL_POSITION)

    def sparki_ready(self):
        print 'Updating sparki...'
        print self._world

        best_direction = self._world.best_direction()
        best_target_angle = self._world.best_target_angle()
        print 'Best direction:', best_direction
        print 'Best target angle:', best_target_angle

        # TODO: This logic should be moved to sparki.py and rewritten with Direction.right/left.
        if self._sparki.direction == best_direction:
            self._sparki.move()
        elif ((self._sparki.direction == NORTH and best_direction == EAST) or
              (self._sparki.direction == EAST and best_direction == SOUTH) or
              (self._sparki.direction == SOUTH and best_direction == WEST) or
              (self._sparki.direction == WEST and best_direction == NORTH)):
            self._sparki.turn(90)
        else:
            self._sparki.turn(-90)

        if best_target_angle:
            self._sparki.look(best_target_angle)

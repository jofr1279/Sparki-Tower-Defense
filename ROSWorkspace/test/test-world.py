import unittest

from ..src.world import World
from ..src.pose import Vector2, Direction
from ..src.sparki import Sparki


class TestWorld(unittest.TestCase):

    def setUp(self):
        self.sparki = Sparki(2, 30)
        self.world = World(Vector2(5, 5), self.sparki, Vector2(4, 4))

    def test_init(self):
        self.assertFalse(self.world.obstacles[0][0])
        self.assertFalse(self.world.targets[0][0])

    def test_add_object(self):
        self.world.add_object(Vector2(1, 1), False)
        self.world.add_object(Vector2(0, 1), False)
        self.world.add_object(Vector2(2, 2), True)
        self.assertTrue(self.world.obstacles[1][1])
        self.assertFalse(self.world.targets[1][1])
        self.assertTrue(self.world.targets[2][2])

    def test_best_direction(self):
        self.sparki.position = Vector2(4, 0)
        print self.world
        self.assertEqual(self.world.best_direction(), Direction.EAST)

        self.sparki.position = Vector2(0, 4)
        print self.world
        self.assertEqual(self.world.best_direction(), Direction.SOUTH)

        self.world.add_object(Vector2(1, 1), False)
        self.world.add_object(Vector2(0, 1), False)
        self.sparki.position = Vector2(0, 0)
        print self.world
        self.assertEqual(self.world.best_direction(), Direction.SOUTH)

        self.world.add_object(Vector2(1, 0), False)
        print self.world
        self.assertEqual(self.world.best_direction(), None)

        self.world.remove_object(Vector2(0, 1))
        print self.world
        self.assertEqual(self.world.best_direction(), Direction.EAST)


if __name__ == '__main__':
    unittest.main()

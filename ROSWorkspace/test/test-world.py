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
        self.assertTrue(self.world.obstacles[1][1])
        self.assertFalse(self.world.targets[1][1])

    def test_best_direction(self):
        self.assertEqual(self.world.best_direction(), Direction.SOUTH)


if __name__ == '__main__':
    unittest.main()

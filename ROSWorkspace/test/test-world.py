import unittest

from ..src.pose import Vector2, EAST, SOUTH, WEST, NORTH
from ..src.sparki import Sparki
from ..src.world import World


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
        self.assertEqual(self.world.best_direction(), EAST)

        self.sparki.position = Vector2(0, 4)
        self.assertEqual(self.world.best_direction(), SOUTH)

        self.world.add_object(Vector2(1, 1), False)
        self.world.add_object(Vector2(0, 1), False)
        self.sparki.position = Vector2(0, 0)
        self.assertEqual(self.world.best_direction(), SOUTH)

        self.world.add_object(Vector2(1, 0), False)
        self.assertIsNone(self.world.best_direction())

        self.world.remove_object(Vector2(0, 1))
        self.assertEqual(self.world.best_direction(), EAST)

    def test_best_target(self):
        self.assertIsNone(self.world.best_target())

        # Sparki facing east with an obstacle to the east
        self.sparki.direction = EAST
        self.world.add_object(Vector2(0, 1), False)
        self.assertIsNone(self.world.best_target())
        self.world.remove_object(Vector2(0, 1))

        # Sparki facing east with a target to the east
        self.world.add_object(Vector2(0, 1), True)
        self.assertEqual(self.world.best_target(), Vector2(0, 1))

        # Sparki facing west with a target to the east
        self.sparki.direction = WEST
        self.assertIsNone(self.world.best_target())

        # Sparki facing south with a target to the east with large servo range
        self.sparki.direction = SOUTH
        self.sparki.servo_range = 100
        self.assertEqual(self.world.best_target(), Vector2(0, 1))

        # Sparki facing east with two targets to the east
        self.sparki.direction = EAST
	self.world.add_object(Vector2(0, 2), True)
        self.assertEqual(self.world.best_target(), Vector2(0, 1))


    def test_best_angle(self):
        self.assertIsNone(self.world.best_target_angle())

        # Sparki facing east with an obstacle to the east
        self.sparki.direction = EAST
        self.world.add_object(Vector2(0, 1), False)
        self.assertIsNone(self.world.best_target_angle())
        self.world.remove_object(Vector2(0, 1))

        # Sparki facing east with a target to the east
        self.world.add_object(Vector2(0, 1), True)
        self.assertEqual(self.world.best_target_angle(), 0)

        # Sparki facing west with a target to the east
        self.sparki.direction = WEST
        self.assertIsNone(self.world.best_target_angle())

        # Sparki facing south with a target to the east with large servo range
        self.sparki.direction = SOUTH
        self.sparki.servo_range = 100
        self.assertEqual(self.world.best_target_angle(), -90)
	
	# Sparki facing north with a target to the east
        self.sparki.direction = NORTH
        self.assertEqual(self.world.best_target_angle(), 90)

	# Sparki facing west with a target to the north
	self.sparki.position = Vector2(1, 1)
        self.sparki.direction = WEST
        self.assertEqual(self.world.best_target_angle(), 90)
	
	# Sparki facing east with a target to the north
        self.sparki.direction = EAST
        self.assertEqual(self.world.best_target_angle(), -90)
	
	# Sparki facing north with a target to the northwest
	self.sparki.position = Vector2(1, 2)
        self.sparki.direction = NORTH
        self.assertEqual(self.world.best_target_angle(), -45)

	# Sparki facing north with a target to the northeast
	self.sparki.position = Vector2(1, 0)
        self.assertEqual(self.world.best_target_angle(), 45)

	# Sparki facing south with a target to the southeast
	self.world.add_object(Vector2(2, 1), True)
	self.sparki.direction = SOUTH
        self.assertEqual(self.world.best_target_angle(), -45)


if __name__ == '__main__':
    unittest.main()

from sparki import Vector2, Direction, Sparki


class World(object):

    def __init__(self, size, sparki, goal_position):
        """ Constructor for World.

        @type size: Vector2
        @type sparki: Sparki
        @type goal_position: Vector2
        """

        self.size = size
        self.sparki = sparki
        self.goal_position = goal_position

        self.obstacles = [[False] * size.x for _ in range(size.y)]
        self.targets = [[False] * size.x for _ in range(size.y)]

    def best_direction(self):
        """ Calculates the best direction Sparki should face to get to the goal position. For simplicity, assume Sparki
            can only face in one of the four cardinal directions. Returns None if there is no possible path to the goal.

        @rtype: Direction | None
        """

        # TODO (Tiffany and Elizabeth): Implement this function.

        # Hint: Use values like...
        #        - self.sparki.position.x
        #        - self.sparki.position.y
        #        - self.sparki.direction
        #        - self.obstacles
        #        - self.goal_position.x
        #        - self.goal_position.y

        return None

    def best_target(self):
        """ Calculates the best target Sparki should shoot at. If there is a target in the path towards the goal, it
            should return that. If there is no target in the path, it should return the closest target. If there are no
            targets in the laser range, it should return None.

        @rtype: Vector2 | None
        """

        # TODO (Tiffany and Elizabeth): Implement this function.

        # Hint: Use values like...
        #        - self.sparki.position.x
        #        - self.sparki.position.y
        #        - self.sparki.direction
        #        - self.targets
        #        - self.goal_position.x
        #        - self.goal_position.y

        return None


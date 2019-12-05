import json

import rospy
from std_msgs.msg import String

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
        self.valid_direction_x = [0, 0, -1, 1]
        self.valid_direction_y = [-1, 1, 0, 0]

        self.obstacles = [[False] * size.x for _ in range(size.y)]
        self.targets = [[False] * size.x for _ in range(size.y)]

        self._init_topics()

    def __repr__(self):
        string = 'World:\n'
        for row in range(self.size.x):
            for col in range(self.size.y):
                if self.sparki.position == Vector2(row, col):
                    string += 'S'
                elif self.goal_position == Vector2(row, col):
                    string += 'G'
                elif self.targets[row][col]:
                    string += 'T'
                elif self.obstacles[row][col]:
                    string += 'O'
                else:
                    string += '_'

            string += '\n'

        return string

    def _init_topics(self):
        rospy.Subscriber('/unity/add_object', String, self.add_object)
        rospy.Subscriber('/unity/remove_object', String, self.remove_object)

    def _add_object(self, string):
        """

        @type string: String
        """
        data = json.loads(string.data)
        self.add_object(Vector2(data['x'], data['y']), data['target'])

    def _remove_object(self, string):
        """

        @type string: String
        """
        data = json.loads(string.data)
        self.remove_object(Vector2(data['x'], data['y']))

    def add_object(self, position, target=False):
        """

        @type position: Vector2
        @type target: bool
        """

        (self.targets if target else self.obstacles)[position.x][position.y] = True

    def remove_object(self, position):
        """

        @type position: Vector2
        """
        self.targets[position.x][position.y] = False
        self.obstacles[position.x][position.y] = False

    def in_bounds(self, coordinate):
        """

        @type coordinate: Vector2
        @rtype: bool
        """

        return 0 <= coordinate.x < self.size.x and 0 <= coordinate.y < self.size.y

    def get_travel_cost(self, source, dest):
        """

        @type source: Vector 2
        @type dest: Vector 2
        """

        x_diff = abs(source.x - dest.x)
        y_diff = abs(source.y - dest.y)
        if self.in_bounds(source) and self.in_bounds(dest):  # if they're in bounds
            if (x_diff or y_diff) and not (x_diff and y_diff):  # if one is non-zero but not both
                if not (self.obstacles[source.x][source.y] or self.obstacles[dest.x][dest.y]):  # if they are unoccupied
                    return 1

        return 1000

    def run_dijkstra(self):

        dist = [[0] * self.size.x for _ in range(self.size.y)]
        prev = [[-1] * self.size.x for _ in range(self.size.y)]
        Q_cost = []

        for x in range(self.size.x):
            for y in range(self.size.y):
                if not (self.sparki.position.x == x and self.sparki.position.y == y):
                    dist[x][y] = 999999

                coord = Vector2(x, y)
                Q_cost.append([coord, dist[x][y]])

        while Q_cost:
            element = min(Q_cost, key=lambda x: x[1])
            Q_cost.remove(element)
            u = element[0]

            for i in range(len(self.valid_direction_x)):
                next_direction = Vector2(u.x + self.valid_direction_x[i], u.y + self.valid_direction_y[i])
                if (not self.in_bounds(next_direction)) or (next_direction not in [x[0] for x in Q_cost]):
                    continue

                alt = dist[u.x][u.y] + self.get_travel_cost(u, next_direction)

                if alt < dist[next_direction.x][next_direction.y]:
                    dist[next_direction.x][next_direction.y] = alt
                    prev[next_direction.x][next_direction.y] = u

                    for index, element in enumerate(Q_cost):
                        if element[0] == next_direction:
                            Q_cost[index] = (element[0], alt)
                            break

        return prev

    def reconstruct_path(self, prev):
        current = self.goal_position
        final_path = []

        while current != -1:
            final_path.append(current)
            current = prev[current.x][current.y]

        return final_path

    def best_direction(self):
        """ Calculates the best direction Sparki should face to get to the goal position. For simplicity, assume Sparki
            can only face in one of the four cardinal directions. Returns None if there is no possible path to the goal.

        @rtype: Direction | None
        """

        prev = self.run_dijkstra()

        final_path = self.reconstruct_path(prev)
        diff = final_path[-2] - self.sparki.position
        print(self.sparki.position) 

        isObstacle = []
        for i in range(len(self.valid_direction_x)):

            next_direction = Vector2(self.sparki.position.x + self.valid_direction_x[i], self.sparki.position.y + self.valid_direction_y[i])
            if self.in_bounds(next_direction):
                isObstacle.append(self.obstacles[next_direction.x][next_direction.y])

        if all(isObstacle): 
            return None


        if diff.x == -1:
            return Direction.NORTH
        if diff.y == 1:
            return Direction.EAST
        if diff.x == 1:
            return Direction.SOUTH
        if diff.y == -1:
            return Direction.WEST

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

        x_min = 999999
        y_min = 999999
        return_target = None
        '''
        for i in range(self.size.y):
            for j in range(self.size.x):
                if self.targets[i][j]:
                    # TODO Use direction to check that it is in the path
                    if i == self.goal.position.y or j == self.goal.position.x:
                        return self.targets[i][j]
                    # Find closest target
                    x_diff = abs(self.sparki.position.x - j)
                    y_diff = abs(self.sparki.position.y - i)
                    if y_diff < y_min and x_diff < x_min:
                        x_min = x_diff
                        y_min = y_diff
                        return_target = self.targets[i][j]

        '''

        return return_target

        return None

    def best_target_angle(self):
        """ Calculates the angle (in degrees) Sparki's servo should face so that is faces the best target. If there are
            no targets in the laser range, it should return None.

        @rtype: int | None
        """

        best_target = self.best_target()

        if best_target is None:
            return None

        # TODO (Tiffany and Elizabeth): Implement this function.

        return 0

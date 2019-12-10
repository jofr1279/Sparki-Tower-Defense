import json
import math

import rospy
from std_msgs.msg import String

from pose import NORTH, EAST, SOUTH, WEST, Vector2, Direction
from sparki import Sparki


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

        @type source: Vector2
        @type dest: Vector2
        """

        x_diff = abs(source.x - dest.x)
        y_diff = abs(source.y - dest.y)
        if self.in_bounds(source) and self.in_bounds(dest):  # if they're in bounds
            if (x_diff or y_diff) and not (x_diff and y_diff):  # if one is non-zero but not both
                if not (self.obstacles[source.x][source.y] or self.obstacles[dest.x][dest.y]):  # if they are unoccupied
                    return 1

        return 1000

    def run_dijkstra(self):
        """ Calculates the best path that Sparki should take to get to the end goal.

        @rtype: 2D list [[]]
        """

        dist = [[0] * self.size.x for _ in range(self.size.y)]
        prev = [[-1] * self.size.x for _ in range(self.size.y)]
        q_cost = []

        for y in range(self.size.y):
            for x in range(self.size.x):
                if not (self.sparki.position.x == x and self.sparki.position.y == y):
                    dist[x][y] = 999999

                coord = Vector2(x, y)
                q_cost.append([coord, dist[x][y]])

        while q_cost:
            element = min(q_cost, key=lambda x: x[1])
            q_cost.remove(element)
            u = element[0]

            for i in range(len(self.valid_direction_x)):
                next_direction = Vector2(u.x + self.valid_direction_x[i], u.y + self.valid_direction_y[i])
                if (not self.in_bounds(next_direction)) or (next_direction not in [x[0] for x in q_cost]):
                    continue

                alt = dist[u.x][u.y] + self.get_travel_cost(u, next_direction)

                if alt < dist[next_direction.x][next_direction.y]:
                    dist[next_direction.x][next_direction.y] = alt
                    prev[next_direction.x][next_direction.y] = u

                    for index, element in enumerate(q_cost):
                        if element[0] == next_direction:
                            q_cost[index] = (element[0], alt)
                            break

        return prev

    def reconstruct_path(self, prev):
        """

        @type prev: 2D list
        """

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

        is_obstacle = []
        for i in range(len(self.valid_direction_x)):

            next_direction = Vector2(
                self.sparki.position.x + self.valid_direction_x[i],
                self.sparki.position.y + self.valid_direction_y[i]
            )
            if self.in_bounds(next_direction):
                is_obstacle.append(self.obstacles[next_direction.x][next_direction.y])

        if all(is_obstacle):
            return None

        if diff.x == -1:
            return NORTH
        if diff.y == 1:
            return EAST
        if diff.x == 1:
            return SOUTH
        if diff.y == -1:
            return WEST

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
        
        min_dist = 999999
        # Store variables for readibility
        x_offset = self.sparki.position.x
        y_offset = self.sparki.position.y
        radius = self.sparki.laser_range
        
        # Set angle offset
        if self.sparki.direction == NORTH: angle_offset = 90 
        if self.sparki.direction == WEST: angle_offset = 180 
        if self.sparki.direction == SOUTH: angle_offset = 270
        if self.sparki.direction == EAST: angle_offset = 360
        return_target = None
        # self.sparki.laser_range,servo_range
        

        for i in range(self.size.x):
            for j in range(self.size.y):
                if self.targets[i][j]:
                    # Check within laser range
                    rel_x = i - x_offset
                    rel_y = j - y_offset
                    print rel_x,rel_y
                    if (pow(rel_x,2) + pow(rel_y,2)) < pow(radius,2):
                        # Check within servo range, x is col and y is row
                        left_x = radius*math.cos(math.radians((self.sparki.servo_range)+angle_offset))
                        left_y = radius*math.sin(math.radians((self.sparki.servo_range)+angle_offset))
                        right_x = radius*math.cos(math.radians(angle_offset - (self.sparki.servo_range)))
                        right_y = radius*math.sin(math.radians(angle_offset - (self.sparki.servo_range)))
                        print left_x,left_y,right_x,right_y
                        if (left_x < rel_x and left_y > rel_y and right_x > rel_x and right_y > rel_y):
                            # TODO Use direction to check that it is in the path to goal
                            '''
                            if i == self.goal_position.y or j == self.goal_position.x:
                                return Vector2(i,j)
                            '''
                            # Find closest target
                            if (pow(rel_x,2) + pow(rel_y,2)) < pow(min_dist,2):
                                min_dist = pow(rel_x,2) + pow(rel_y,2)
                                return_target = Vector2(i,j)
                            
        return return_target


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

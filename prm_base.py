import pygame
import random
import logging

logging.basicConfig(level=logging.INFO)


class PRMMap:
    def __init__(self, start, goal, map_dimensions, obs_dim, obs_num):
        self.start = start
        self.goal = goal
        self.map_dimensions = map_dimensions
        self.obs_dim = obs_dim
        self.obs_num = obs_num
        self.map_h, self.map_w = self.map_dimensions

        # window settings
        self.map_window_name = "PRM path planning"
        pygame.display.set_caption(self.map_window_name)
        self.map = pygame.display.set_mode((self.map_w, self.map_h))
        self.map.fill((255, 255, 255))
        self.node_rad = 2
        self.node_thickness = 1
        self.edge_thickness = 1

        self.obstacles = []

        # colors
        self.grey_color = (70, 70, 70)
        self.blue_color = (0, 0, 255)
        self.green_color = (0, 255, 0)
        self.red_color = (255, 0, 0)
        self.white_color = (255, 255, 255)

    def draw_map(self, obstacles, nodes):
        """draws the map to show RRT interaction

        Args:
            obstacles (list(pygame.Rect)): list of rectangles made by pygame
        """
        pygame.draw.circle(self.map, self.green_color, self.start, self.node_rad + 5, 0)
        pygame.draw.circle(self.map, self.red_color, self.goal, self.node_rad + 20, 1)

        self.draw_obs(obstacles)
        self.draw_nodes(nodes)

    def draw_obs(self, obstacles):
        obstacles_list = obstacles.copy()
        while len(obstacles_list) > 0:
            obstacle = obstacles_list.pop(0)
            pygame.draw.rect(self.map, self.grey_color, obstacle)

    def draw_nodes(self, nodes):
        for node in nodes:
            if node.free:
                pygame.draw.circle(
                    self.map, self.blue_color, (node.x, node.y), self.node_rad + 5, 0
                )
            else:
                pygame.draw.circle(
                    self.map, self.red_color, (node.x, node.y), self.node_rad + 5, 0
                )
        return True

    def draw_connections(self, edges_connections, mod_nodes):
        for edge_connection in edges_connections:
            n1, n2 = edge_connection
            pygame.draw.line(
                self.map,
                self.blue_color,
                (mod_nodes[n1].x, mod_nodes[n1].y),
                (mod_nodes[n2].x, mod_nodes[n2].y),
                self.edge_thickness,
            )

    def draw_path(self, path, nodes):

        pygame.draw.line(
            self.map,
            self.red_color,
            (self.start[0], self.start[1]),
            (nodes[path[0]].x, nodes[path[0]].y),
            self.edge_thickness + 4,
        )

        for i in range(0, len(path) - 1):
            pygame.draw.line(
                self.map,
                self.red_color,
                (nodes[path[i]].x, nodes[path[i]].y),
                (nodes[path[i + 1]].x, nodes[path[i + 1]].y),
                self.edge_thickness + 4,
            )

        pygame.draw.line(
            self.map,
            self.red_color,
            (nodes[path[-1]].x, nodes[path[-1]].y),
            (self.goal[0], self.goal[1]),
            self.edge_thickness + 4,
        )


class PRMGraph:
    def __init__(self, start, goal, map_dimensions, obs_dim, obs_num, nodes_number):
        (_x, _y) = start
        self.start = start
        self.goal = goal
        self.goal_success = False
        self.map_h, self.map_w = map_dimensions
        self.x = []
        self.y = []
        self.parent = []

        self.obstacles = []
        self.obs_dim = obs_dim
        self.obs_num = obs_num
        self.edges = []

        self.nodes_number = nodes_number
        self.nodes = []

        self.connections = []

        self.neighbor = 100

        self.path = []

    def connect_nodes(self):
        for i in range(0, len(self.nodes)):
            n1 = self.nodes[i]
            for j in range(0, len(self.nodes)):
                n2 = self.nodes[j]
                if n1.id != n2.id:
                    if self.distance(n1, n2) < self.neighbor:
                        if (
                            self.connections.count((n1.id, n2.id)) == 0
                            and self.connections.count((n2.id, n1.id)) == 0
                            and not self.cross_obstacle(n1.x, n2.x, n1.y, n2.y)
                        ):
                            self.connections.append((n1.id, n2.id))
                        n1.add_connection(n2.id)
            self.nodes[i] = n1
        return self.connections, self.nodes

    def cross_obstacle(self, _x1, _x2, _y1, _y2):
        obs = self.obstacles.copy()
        while len(obs) > 0:
            rectang = obs.pop(0)
            for i in range(0, 101):
                _u = i / 100
                _x = _x1 * _u + _x2 * (1 - _u)
                _y = _y1 * _u + _y2 * (1 - _u)
                if rectang.collidepoint(_x, _y):
                    return True
        return False

    def make_obs(self):
        obs = []
        for i in range(0, self.obs_num):
            rectang = None
            startgoalcol = True
            while startgoalcol:
                upper = self.make_random_rect()
                rectang = pygame.Rect(upper, (self.obs_dim, self.obs_dim))
                if rectang.collidepoint(self.start) or rectang.collidepoint(self.goal):
                    startgoalcol = True
                else:
                    startgoalcol = False
            obs.append(rectang)
        self.obstacles = obs.copy()
        return obs

    def make_random_rect(self):
        uppercornerx = int(random.uniform(0, self.map_w - self.obs_dim))
        uppercornery = int(random.uniform(0, self.map_h - self.obs_dim))

        return (uppercornerx, uppercornery)

    def sample_env(self):
        _x = int(random.uniform(0, self.map_w))
        _y = int(random.uniform(0, self.map_h))
        return _x, _y

    def generate_nodes(self, obstacles):
        for i in range(0, self.nodes_number):
            flag = 0
            obs = obstacles.copy()
            _x, _y = self.sample_env()
            while len(obs) > 0:
                rectang = obs.pop(0)
                if rectang.collidepoint(_x, _y):
                    flag = 1
                    self.nodes.append(Node(i, _x, _y, False))
                    break
            if not flag:
                self.nodes.append(Node(i, _x, _y, True))
            flag = 0

        return self.nodes

    def add_node(self, n, x, y):
        """register a node position and number

        Args:
            n (int): node number
            x (int): node at x position
            y (int): node at y position
        """
        self.x.insert(n, x)
        self.y.append(y)

    def number_of_nodes(self):
        """get the number of registered nodes

        Returns:
            int: amount of registered nodes
        """
        return len(self.x)

    def distance(self, n1, n2):
        """get distance between two nodes"""
        (_x1, _y1) = (n1.x, n1.y)
        (_x2, _y2) = (n2.x, n2.y)

        p_x = (float(_x1) - float(_x2)) ** 2
        p_y = (float(_y1) - float(_y2)) ** 2
        return (p_x + p_y) ** (0.5)

    def is_free(self):
        _n = self.number_of_nodes() - 1
        (_x, _y) = (self.x[_n], self.y[_n])
        obs = self.obstacles.copy()
        while len(obs) > 0:
            rectang = obs.pop(0)
            if rectang.collidepoint(_x, _y):
                self.remove_node(_n)
                return False
        return True

    def nearest(self, n):
        d_min = self.distance(0, n)
        nnear = 0
        for i in range(0, n):
            if self.distance(i, n) < d_min:
                d_min = self.distance(i, n)
                nnear = i
        return nnear

    def raw_distance(self, _x1, _y1, _x2, _y2):
        """get distance between two nodes"""
        p_x = (float(_x1) - float(_x2)) ** 2
        p_y = (float(_y1) - float(_y2)) ** 2
        return (p_x + p_y) ** (0.5)

    def start_node(self):
        distance = 1000000000000000000
        nearest_possible_node = None
        for node in self.nodes:
            start_to_node = self.raw_distance(
                self.start[0], self.start[1], node.x, node.y
            )
            if start_to_node < distance and not self.cross_obstacle(
                self.start[0], node.x, self.start[1], node.y
            ):
                distance = start_to_node
                nearest_possible_node = node.id
        if nearest_possible_node == None:
            return False
        return nearest_possible_node

    def get_path_coords(self):
        return self.path

    def path_to_goal(self):
        start_node = self.start_node()
        if isinstance(start_node, bool):
            return False
        self.path.append(start_node)

        n1 = self.nodes[self.path[-1]]

        # initial_cost = self.raw_distance(n1.x, n1.y, self.goal[0], self.goal[1])

        initial_cost = 10000000

        search_flag = None

        following_node = None

        while search_flag == None:
            n1 = self.nodes[self.path[-1]]
            for connection in self.connections:
                if self.path[-1] in connection:
                    if self.path[-1] == connection[0]:
                        n2 = self.nodes[connection[1]]
                    else:
                        n2 = self.nodes[connection[0]]
                    current_cost = self.raw_distance(
                        n1.x, n1.y, n2.x, n2.y
                    ) + self.raw_distance(n2.x, n2.y, self.goal[0], self.goal[1])
                    if current_cost < initial_cost:
                        initial_cost = current_cost
                        following_node = n2.id
            if following_node == self.path[-1]:
                search_flag = False
            elif following_node != None:
                self.path.append(following_node)
        last_node = self.nodes[self.path[-1]]
        if not self.cross_obstacle(
            self.goal[0], last_node.x, self.goal[1], last_node.y
        ):
            return True
        logging.info("No solution found.")
        return False


class Node:
    def __init__(self, id, x, y, free_point):
        self.id = id
        self.x = x
        self.y = y
        self.connections = set()
        self.free = free_point

    def set_free(self, val):
        self.free = val

    def add_connection(self, val):
        self.connections.add(val)

import pygame
import random


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
            print(len(nodes))
            print(node.free)
            if node.free:
                pygame.draw.circle(
                    self.map, self.blue_color, (node.x, node.y), self.node_rad + 5, 0
                )
            else:
                pygame.draw.circle(
                    self.map, self.red_color, (node.x, node.y), self.node_rad + 5, 0
                )
        return True


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

        self.neighbor = 30

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

        # print(len(self.nodes))
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
        (_x1, _y1) = (self.x[n1], self.y[n1])
        (_x2, _y2) = (self.x[n2], self.y[n2])

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


class Node:
    def __init__(self, id, x, y, free_point):
        self.id = id
        self.x = x
        self.y = y
        self.connections = set()
        self.free = free_point

    def set_free(self, val):
        self.free = val

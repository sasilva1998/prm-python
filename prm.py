import pygame
from prm_base import PRMGraph, PRMMap
import time


def main():
    try:
        dimensions = (600, 1000)
        goal = (50, 50)
        start = (510, 510)
        obs_dim = 50
        obs_num = 50

        pygame.init()
        map = PRMMap(start, goal, dimensions, obs_dim, obs_num)
        graph = PRMGraph(start, goal, dimensions, obs_dim, obs_num, 1000)

        obstacles = graph.make_obs()
        nodes = graph.generate_nodes(obstacles)
        map.draw_map(obstacles, nodes)

        pygame.display.update()

        edges_connections, mod_nodes = graph.connect_nodes()

        map.draw_connections(edges_connections, mod_nodes)

        pygame.display.update()

        if graph.path_to_goal():
            map.draw_path(graph.get_path_coords(), mod_nodes)

        pygame.display.update()

        pygame.event.clear()
        pygame.event.wait(0)

    except Exception as e:
        print(e)


if __name__ == "__main__":
    result = False
    while not result:
        try:
            main()
            result = True
        except:
            result = False

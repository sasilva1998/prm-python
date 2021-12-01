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

        iteration = 0
        t1 = time.time()

        pygame.init()
        map = PRMMap(start, goal, dimensions, obs_dim, obs_num)
        graph = PRMGraph(start, goal, dimensions, obs_dim, obs_num, 500)

        obstacles = graph.make_obs()
        nodes = graph.generate_nodes(obstacles)
        # map.map.fill((map.white_color))

        # elapsed = time.time() - t1
        # t1 = time.time()
        # if elapsed > 10:
        #     raise ("timed up")
        map.draw_map(obstacles, nodes)

        pygame.display.update()

        time.sleep(3)

        edges_connections, mod_nodes = graph.connect_nodes()

        map.draw_connections(edges_connections, mod_nodes)

        pygame.display.update()

        time.sleep(3)

        if graph.path_to_goal():
            map.draw_path(graph.get_path_coords())

        pygame.event.clear()
        pygame.event.wait(0)

        # pygame.display.update()

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

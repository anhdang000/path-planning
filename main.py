import sys
import os
import os.path as osp
import time
from datetime import datetime

from RRTbase import RRTGraph


def main():
    map_size = (400, 400)
    start = (50, 50)
    goal = (350, 350)
    obs_dim = 50
    num_obs = 5
    
    # pygame.init()
    graph = RRTGraph(start, goal, map_size, obs_dim, num_obs)
    obs = graph.make_obs()
    graph.draw_map(obs)

    iteration = 0
    while not graph.path_to_goal():
        if iteration % 5 == 0:
            X, Y, Parent = graph.bias(goal)
            graph.draw_edge((X[-1], Y[-1]), (X[Parent[-1]], Y[Parent[-1]]))
            graph.draw_node((X[-1], Y[-1]))
        else:
            X, Y, Parent = graph.expand()
            graph.draw_edge((X[-1], Y[-1]), (X[Parent[-1]], Y[Parent[-1]]))
            graph.draw_node((X[-1], Y[-1]))
        
        if iteration % 5 == 0:
            graph.pause()

        iteration += 1

    path = graph.get_path_coords()
    path = graph.optimize_path(path)
    graph.draw_path(path)
    graph.show()
if __name__ == "__main__":
    main()
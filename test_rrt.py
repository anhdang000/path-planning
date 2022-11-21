import sys
import os
import os.path as osp
import time
from datetime import datetime

import math
import numpy as np
import random
import matplotlib.pyplot as plt

from RRTbase import RRTGraph
from purepursuit import PurePursuit


def main():
    start = (50, 50)
    goal = (250, 250)
    obs_dim = 50
    num_obs = 7

    graph = RRTGraph(start, goal, obs_dim, num_obs)
    obs = graph.make_random_obs()
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

    pure_pursuit = PurePursuit(path[::-1], ax=graph.ax, followerSpeed=35, lookaheadDistance=12)
    pure_pursuit.add_follower(path[-1][0], path[-1][1])

    for i in range(1000):
        pure_pursuit.draw()
        graph.set_xylim()
        graph.draw_startgoal()
        graph.draw_map(graph.obstacles)

        graph.pause()

        if not plt.fignum_exists(1):
            f = open("transfer_data.txt", "a")
            f.write("0.00 0.00\n")
            f.close()
            break

    graph.show()
if __name__ == "__main__":
    main()
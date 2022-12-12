import sys
import os
import os.path as osp
import time
from datetime import datetime

import math
import numpy as np
import random
import matplotlib.pyplot as plt
import scipy.interpolate

from modules import RRTGraph, PurePursuit


def main():
    start = (50, 50)
    good = (100, 250)
    goal = (350, 350)
    obs_dim = 20
    num_obs = 2
    obs_pts = [(80, 150), (250, 300)]
    graph = RRTGraph(start, goal, good, obs_dim, num_obs)
    # obs = graph.make_random_obs()
    obs = graph.make_obs(obs_pts)
    graph.draw_map(obs)

    # Stage 1: Start -> Good
    graph.set_new_plan(start, good)
    graph.draw_startgoal()

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

    theta = math.atan2(path[-2][1] - path[-1][1], path[-2][0] - path[-1][0])
    print(f'Theta: {math.degrees(theta):.2f}')

    f = open("transfer_data/send.txt", "a")
    f.write(f"r {math.degrees(theta):.2f}\n")
    f.close()
    time.sleep(1)

    pure_pursuit = PurePursuit(path[::-1], ax=graph.ax, followerSpeed=40, lookaheadDistance=20)
    pure_pursuit.set_follower(path[-1][0], path[-1][1])
    pure_pursuit.follower.theta = theta

    for i in range(1000):
        pure_pursuit.draw()

        if pure_pursuit.follower.is_dead:
            print(f'Good reached')
            break

        graph.set_xylim()
        graph.draw_startgoal()
        graph.draw_map(graph.obstacles)

        graph.pause()

        if not plt.fignum_exists(1):
            f = open("transfer_data/send.txt", "a")
            f.write("v 0 0\n")
            f.close()
            break

    # Stage 2: Good -> Goal
    graph.set_new_plan(good, goal)
    graph.draw_startgoal()
    graph.set_xylim()
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

    theta = math.atan2(path[-2][1] - path[-1][1], path[-2][0] - path[-1][0])
    print(f'Theta: {math.degrees(theta):.2f}')

    f = open("transfer_data/send.txt", "a")
    for _ in range(10):
        f.write(f"r {math.degrees(theta):.2f}\n")
    f.close()
    time.sleep(1)

    pure_pursuit = PurePursuit(path[::-1], ax=graph.ax, followerSpeed=40, lookaheadDistance=20)
    pure_pursuit.set_follower(path[-1][0], path[-1][1])
    pure_pursuit.follower.theta = theta

    for i in range(1000):
        pure_pursuit.draw()
        graph.set_xylim()
        graph.draw_startgoal()
        graph.draw_map(graph.obstacles)

        if pure_pursuit.follower.is_dead:
            print(f'Goal reached')
            break

        graph.pause()

        if not plt.fignum_exists(1):
            f = open("transfer_data/send.txt", "a")
            f.write("v 0 0\n")
            f.close()
            break

    graph.show()



if __name__ == "__main__":
    main()
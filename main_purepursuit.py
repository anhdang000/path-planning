import sys
import os
import os.path as osp

import threading

import cv2
from cv2 import aruco
import numpy as np
import math
from scipy.spatial import distance
import matplotlib.pyplot as plt

from modules import RRTGraph, PurePursuit, Camera


class Camera2RRT(Camera):
    def __init__(self):
        # Planner
        self.graph = None
        self.path = None

        # Obs
        self.obs_size = 10
        self.obs_points = []
        self.num_obs = 1
        

    def core_planning(self, start, goal):
        """Conduct planning given start point and goal point

        Args:
            start (Tuple(float, float)): Start point
            goal (Tuple(float, float)): Goal point
        """
        self.graph.set_new_plan(start, goal)
        self.graph.draw_startgoal()

        iteration = 0
        while not self.graph.path_to_goal():
            if iteration % 5 == 0:
                X, Y, Parent = self.graph.bias(self.goal)
                self.graph.draw_edge((X[-1], Y[-1]), (X[Parent[-1]], Y[Parent[-1]]))
                self.graph.draw_node((X[-1], Y[-1]))
            else:
                X, Y, Parent = self.graph.expand()
                self.graph.draw_edge((X[-1], Y[-1]), (X[Parent[-1]], Y[Parent[-1]]))
                self.graph.draw_node((X[-1], Y[-1]))
            
            if iteration % 5 == 0:
                self.graph.pause()

            iteration += 1

        self.path = self.graph.get_path_coords()
        self.path = self.graph.optimize_path(self.path)
        self.graph.draw_path(self.path)

    
    def plan_a_track(self, start, goal):
        # Plan
        self.core_planning(start, goal)

        # Plot current robot position
        theta = np.pi/2
        while True:
            pure_pursuit = PurePursuit(
                self.path[::-1], 
                ax=self.graph.ax, 
                followerSpeed=40, 
                lookaheadDistance=30
                )
            pure_pursuit.set_follower(
                self.components['robot'][0], 
                self.components['robot'][1],
                theta
                )

            for _ in range(2):
                pure_pursuit.draw()
                self.graph.set_xylim()
                self.graph.draw_startgoal()
                self.graph.draw_map(self.graph.obstacles)

                self.graph.pause()

            theta = pure_pursuit.follower.theta
            if not plt.fignum_exists(1):
                f = open("transfer_data/send.txt", "a")
                f.write("v 0 0\n")
                f.close()
                break


    def plan_and_plot(self):
        while not all(self.components.values()):
            pass # Wait until all components on the map are detected

        start = self.components['robot']
        goal = self.components['good']

        self.graph = RRTGraph(start, goal, self.obs_size, self.num_obs)
        obs = self.graph.make_obs(self.components['obs'])
        self.graph.draw_map(obs)

        tracks = [['robot', 'good'], ['good', 'goal']]
        for track in tracks:
            self.plan_a_track(
                self.components[track[0]], 
                self.components[track[1]]
                )


    def process_frame(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break

            self.detect_color_obj(frame)
            self.detect_markers(frame)

            cv2.imshow("Camera", frame)
            key = cv2.waitKey(1)
            if key == ord("q"):
                break


    def run(self):
        t1 = threading.Thread(target=self.process_frame, args=())
        t1.start()

        self.plan_and_plot()

        
if __name__ == "__main__":
    planner = Camera2RRT()
    planner.run()



    #     pure_pursuit = PurePursuit(path[::-1], ax=graph.ax, followerSpeed=12, lookaheadDistance=50)
    #     pure_pursuit.add_follower(path[-1][0], path[-1][1])

    #     for _ in range(100):
    #         pure_pursuit.draw()
    #         graph.pause()


    #     if not plt.fignum_exists(1):
    #         break
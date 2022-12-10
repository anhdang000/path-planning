import random
import math

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.collections import PatchCollection
from scipy.spatial import distance

from plotting import Viz

class RRTGraph(Viz):
    def __init__(self, start, goal, good, obs_dim, num_obs):
        super().__init__(start, goal)
        (x, y) = start
        self.goal_flag = False
        self.good = good

        # Init the tree
        self.X = []
        self.Y = []
        self.parent = []
        self.X.append(x)
        self.Y.append(y)
        self.parent.append(0)
        
        # Obstacles
        self.obstacles = []
        self.obs_dim = obs_dim
        self.num_obs = num_obs

        # Path
        self.goal_state = None
        self.path = []


    def set_new_plan(self, start, goal):
        self.start = start
        self.goal = goal
        self.X = [start[0]]
        self.Y = [start[1]]
        self.parent = [0]
        self.goal_state = None
        self.goal_flag = False
        self.path = []

        
    def random_point(self):
        x = int(random.uniform(0, self.map_w))
        y = int(random.uniform(0, self.map_h))
        return (x, y)
        
    def make_random_obs(self):
        obs = []
        for i in range(self.num_obs):
            start_goal_col = True
            while start_goal_col:
                center = self.random_point()
                ob = mpatches.Circle(center, self.obs_dim, facecolor=self.colors["obstacle"], edgecolor='g')
                if distance.euclidean(center, self.start) <= self.obs_dim or \
                    distance.euclidean(center, self.goal) <= self.obs_dim or \
                    distance.euclidean(center, self.good) <= self.obs_dim:
                    start_goal_col = True
                else:
                    start_goal_col = False
            obs.append(ob)
        self.obstacles = obs.copy()
        return obs

    def make_obs(self, pts):
        obs = []
        for c in pts:
            ob = mpatches.Circle(c, self.obs_dim, facecolor=self.colors["obstacle"], edgecolor='g')
            obs.append(ob)
        self.obstacles = obs.copy()
        return obs

    def add_node(self, n, x, y):
        self.X.insert(n, x)
        self.Y.insert(n, y)

    def remove_node(self, n):
        self.X.pop(n)
        self.Y.pop(n)

    def add_edge(self, parent, child):
        self.parent.insert(child, parent)

    def remove_edge(self, n):
        self.parent.pop(n)

    def number_of_nodes(self):
        return len(self.X)

    def distance(self, n1, n2):
        (x1, y1) = (self.X[n1], self.Y[n1])
        (x2, y2) = (self.X[n2], self.Y[n2])
        px = (float(x1) - float(x2))**2
        py = (float(y1) - float(y2))**2
        return (px + py)**0.5
    
    def sample_env(self):
        x = int(random.uniform(0, self.map_w))
        y = int(random.uniform(0, self.map_h))
        return x, y
        
    def nearest(self, n):
        d_min = self.distance(0, n)
        n_near = 0
        for i in range(n):
            if self.distance(i, n) < d_min:
                d_min = self.distance(i, n)
                n_near = i
        return n_near
                
    def is_free(self):
        n = self.number_of_nodes() - 1
        (x, y) = (self.X[n], self.Y[n])
        obs = self.obstacles.copy()
        while len(obs) > 0:
            ob = obs.pop(0)
            if distance.euclidean(ob.center, (x, y)) <= self.obs_dim:
                self.remove_node(n)
                return False
        return True
        
    def cross_obstacle(self, x1, y1, x2, y2):
        obs = self.obstacles.copy()
        while len(obs) > 0:
            ob = obs.pop(0)
            r = 100
            mid_pts = []
            for i in range(r):
                u = i/r
                x = x1*u + x2*(1-u)
                y = y1*u + y2*(1-u)
                mid_pts.append((x, y))

                if distance.euclidean(ob.center, (x, y)) <= self.obs_dim:
                    return True
        return False

        
    def connect(self, n1, n2):
        (x1, y1) = (self.X[n1], self.Y[n1])
        (x2, y2) = (self.X[n2], self.Y[n2])
        if self.cross_obstacle(x1, y1, x2, y2):
            self.remove_node(n2)
            return False
        else:
            # self.ax.plot([x1,x2],[y1,y2])
            self.add_edge(n1, n2)
            return True


    def step(self, n_near, n_rand, d_max=35):
        d = self.distance(n_near, n_rand)
        if d > d_max:
            u = d_max/d
            (x_near, y_near) = (self.X[n_near], self.Y[n_near])
            (x_rand, y_rand) = (self.X[n_rand], self.Y[n_rand])
            (px, py) = (x_rand - x_near, y_rand - y_near)
            theta = math.atan2(py, px)
            x = int(x_near + d_max * math.cos(theta))
            y = int(y_near + d_max * math.sin(theta))
            self.remove_node(n_rand)
            if abs(x-self.goal[0]) < d_max and abs(y-self.goal[1]) < d_max:
                self.add_node(n_rand, self.goal[0], self.goal[1])
                self.goal_state = n_rand
                self.goal_flag = True
            else:
                self.add_node(n_rand, x, y)

        
    def path_to_goal(self):
        if self.goal_flag:
            self.path = []
            self.path.append(self.goal_state)
            try:
                new_pos = self.parent[self.goal_state]
                while new_pos != 0:
                    self.path.append(new_pos)
                    new_pos = self.parent[new_pos]
                self.path.append(0)
            except:
                self.goal_flag = False
        return self.goal_flag

        
    def get_path_coords(self):
        path_coords = []
        for node in self.path:
            x, y = (self.X[node], self.Y[node])
            path_coords.append((x, y))
        return path_coords
        
    def optimize_path(self, path):
        path = np.array(path)
        num_points = len(path)
        first_idx = 0
        optimal_path = [path[first_idx]]

        while True:
            second_idx = first_idx + 1
            while second_idx < num_points:
                p1 = path[first_idx, :]
                p2 = path[second_idx, :]
                if self.cross_obstacle(p1[0], p1[1], p2[0], p2[1]):
                    second_idx = second_idx - 1
                    first_idx = second_idx
                    optimal_path.append(path[second_idx, :])
                    break
                elif second_idx == num_points - 1:
                    optimal_path.append(path[second_idx, :])
                    break
                else:
                    second_idx = second_idx + 1
            if second_idx == num_points - 1:
                break
        return np.array(optimal_path).tolist()


        
    def bias(self, n_goal):
        n = self.number_of_nodes()
        self.add_node(n, n_goal[0], n_goal[1])
        n_near = self.nearest(n)
        self.step(n_near, n)
        res = self.connect(n_near, n)
        return self.X, self.Y, self.parent
        
    def expand(self):
        n = self.number_of_nodes()
        x, y = self.sample_env()
        self.add_node(n, x, y)
        if self.is_free():
            x_nearest = self.nearest(n)
            self.step(x_nearest, n)
            self.connect(x_nearest, n)
        return self.X, self.Y, self.parent
        
    def cost(self):
        pass

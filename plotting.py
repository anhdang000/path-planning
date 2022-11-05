import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.collections import PatchCollection
import seaborn as sns

sns.set_theme()


class Viz:
    def __init__(self, start, goal, dims=(6, 6)) -> None:
        self.dims = dims
        self.start = start
        self.goal = goal
        self.margin = 50

        self.fig = plt.figure(figsize=dims)
        self.ax = self.fig.add_subplot(111)
        self.ax.grid(True)
        self.ax.set_xlim(self.start[0] - self.margin, self.goal[0] + self.margin)
        self.ax.set_ylim(self.start[1] - self.margin, self.goal[1] + self.margin)
        self.ax.set_aspect('equal', 'box')

        # Color
        self.colors = {}
        self.colors["start"] = tuple(map(lambda x: x/255, (22, 35, 135)))
        self.colors["goal"] = tuple(map(lambda x: x/255, (59, 148, 3)))
        self.colors["robot"] = tuple(map(lambda x: x/255, (179, 2, 2)))
        self.colors["obstacle"] = tuple(map(lambda x: x/255, (250, 231, 60)))
        self.colors["node_path"] = tuple(map(lambda x: x/255, (60, 174, 250)))
        self.colors["path"] = tuple(map(lambda x: x/255, (0, 113, 188)))
        self.colors["node_sampling"] = tuple(map(lambda x: x/255, (93, 94, 94, 150)))
        self.colors["edge_sampling"] = tuple(map(lambda x: x/255, (170, 170, 170, 150)))

        # Formatting
        self.node_rad = 2

        # Draw intial map
        self.start_obj = mpatches.Rectangle((self.start[0] - 5, self.start[1] - 5), 10, 10, color=self.colors["start"])
        self.ax.add_patch(self.start_obj)
        self.goal_obj = mpatches.Circle(self.goal, 5, color=self.colors["goal"])
        self.ax.add_patch(self.goal_obj)

        self.path_col = None

    
    def draw_map(self, obs):
        for ob in obs:
            self.ax.add_patch(ob)
    

    def draw_path(self, path):
        path = np.array(path)
        self.ax.plot(path[:, 0], path[:, 1], '-s', color=self.colors["path"], markerfacecolor=self.colors["node_path"], markeredgecolor=self.colors["node_path"])
        
            

    def draw_node(self, pt):
        self.ax.add_patch(mpatches.Circle(pt, self.node_rad, color=self.colors["node_sampling"]))


    def draw_edge(self, pt1, pt2):
        self.ax.add_patch(mpatches.Polygon([pt1, pt2], closed=False, linewidth=2, edgecolor=self.colors["edge_sampling"]))


    def show(self):
        plt.show()


    def pause(self):
        plt.pause(0.05)


if __name__ == "__main__":
    viz = Viz((40, 40), (350, 350))
    viz.show()

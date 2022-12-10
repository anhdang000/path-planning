import sys
import os
import os.path as osp

import threading

import cv2
from cv2 import aruco
import numpy as np
import math
from scipy.spatial import distance

from RRTbase import RRTGraph


class Camera2RRT:
    def __init__(self):
        # Planner
        self.graph = None
        self.path = None
        self.target_waypoint_idx = 0
        self.dist_thresh = 5
        self.is_planned = False

        # Environment
        self.start = None
        self.goal = None

        # Camera configurations
        self.calib_data_path = "calib_data/MultiMatrix.npz"
        calib_data = np.load(self.calib_data_path)
        self.cam_mat = calib_data["camMatrix"]
        self.dist_coef = calib_data["distCoef"]

        # Aruco
        self.cap = cv2.VideoCapture(1)
        self.marker_size = 20
        self.marker_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.param_markers = aruco.DetectorParameters_create()

        # Robot & Goal
        self.rbt_info = {}
        self.goal_info = {}

        # Obs
        self.obs_size = 10
        self.obs_points = []
        self.num_obs = 1

        # Errors for sending
        self.dist_err = 0
        self.target_pose = 0
        
        # Boolean states
        self.is_goal_read = False
        self.is_obs_read = False
        

    def plan_and_plot(self):
        while not (self.is_goal_read and self.is_obs_read and self.start and self.goal):
            pass # Wait

        self.graph = RRTGraph(self.start, self.goal, self.obs_size, self.num_obs)
        obs = self.graph.make_obs(self.obs_points)
        # obs = self.graph.make_random_obs()
        self.graph.draw_map(obs)

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
        self.is_planned = True      # Mark done planning
        # self.graph.show()

        # Plot current robot position
        curr_rbt = self.rbt_info.copy()
        while True:
            change_dist = distance.euclidean((curr_rbt["x"], curr_rbt["y"]), (self.rbt_info["x"], self.rbt_info["y"]))
            if change_dist > self.dist_thresh:
                self.graph.ax.plot(self.rbt_info["x"], self.rbt_info["y"], 'o', color='r')
                self.graph.pause()
                curr_rbt = self.rbt_info.copy()


    def process_frame(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break
            obs_pts = []
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            marker_corners, marker_IDs, _ = aruco.detectMarkers(
                gray_frame, self.marker_dict, parameters=self.param_markers
            )
            if marker_corners:
                rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
                    marker_corners, self.marker_size, self.cam_mat, self.dist_coef
                )
                total_markers = range(0, marker_IDs.size)
                
                for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
                    cv2.polylines(
                        frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv2.LINE_AA
                    )
                    corners = corners.reshape(4, 2)
                    corners = corners.astype(int)
                    top_right = corners[0].ravel()
                    bottom_right = corners[2].ravel()

                    # Draw the pose of the marker
                    cv2.drawFrameAxes(frame, self.cam_mat, self.dist_coef, rVec[i], tVec[i], self.marker_size, 4)
                    cv2.putText(
                        frame,
                        f"x:{round(tVec[i][0][0],1)} y: {round(tVec[i][0][1],1)} ",
                        bottom_right,
                        cv2.FONT_HERSHEY_PLAIN,
                        1.0,
                        (0, 0, 255),
                        2,
                        cv2.LINE_AA,
                    )

                    if ids[0] == 33:
                        cv2.putText(
                        frame,
                        f"Robot",
                        top_right,
                        cv2.FONT_HERSHEY_PLAIN,
                        1.3,
                        (0, 0, 255),
                        2,
                        cv2.LINE_AA,
                        )
                        self.rbt_info["x"] = tVec[i][0][0]
                        self.rbt_info["y"] = -tVec[i][0][1]
                        self.rbt_info["ang"] = rVec[i][0][0]

                        self.start = (self.rbt_info["x"], self.rbt_info["y"])

                    elif ids[0] == 50:
                        cv2.putText(
                        frame,
                        f"Goal",
                        top_right,
                        cv2.FONT_HERSHEY_PLAIN,
                        1.3,
                        (0, 0, 255),
                        2,
                        cv2.LINE_AA,
                        )
                        if not self.is_goal_read:
                            self.goal_info["x"] = tVec[i][0][0]
                            self.goal_info["y"] = -tVec[i][0][1]
                        
                            self.is_goal_read = True

                            self.goal = (self.goal_info["x"], self.goal_info["y"])

                    elif ids[0] == 56 and not self.is_obs_read:
                        cv2.putText(
                        frame,
                        f"Obstacle",
                        top_right,
                        cv2.FONT_HERSHEY_PLAIN,
                        1.3,
                        (0, 0, 255),
                        2,
                        cv2.LINE_AA,
                        )
                        obs_pts.append((tVec[i][0][0], -tVec[i][0][1]))
                        if len(obs_pts) == self.num_obs:
                            self.is_obs_read = True
                            self.obs_points = obs_pts

                if self.is_planned:
                    self.dist_err = distance.euclidean((self.rbt_info["x"], self.rbt_info["y"]), self.path[self.target_waypoint_idx])
                    self.target_pose = math.degrees(math.atan2(
                        self.goal_info["y"] - self.rbt_info["y"], 
                        self.goal_info["x"] - self.rbt_info["x"]
                        ))
                    # print(f'dist_err: {self.dist_err}\ttarget_pose: {self.target_pose}')

                    dist2goal = distance.euclidean((self.rbt_info["x"], self.rbt_info["y"]), (self.goal_info["x"], self.goal_info["y"]))
                    if dist2goal > self.dist_thresh:
                        f = open("transfer_data/send.txt", "a")
                        f.write(f"{self.dist_err:.2f} {self.target_pose:.2f}\n")
                        f.close()
                    else:
                        self.dist_err, self.target_pose = 0, 0
                        self.reach_goal = True
                        f = open("transfer_data/send.txt", "a")
                        f.write(f"{self.dist_err:.2f} {self.target_pose:.2f}\n")
                        f.close()

                    if self.dist_err < self.dist_thresh:
                        self.target_waypoint_idx = self.target_waypoint_idx + 1

            cv2.imshow("Camera", frame)
            key = cv2.waitKey(1)
            if key == ord("q"):
                break

    
    def run(self):
        # t1 = threading.Thread(target=self.plan_and_plot, args=())
        t2 = threading.Thread(target=self.process_frame, args=())

        # t1.start()
        t2.start()

        # t1.join()
        self.plan_and_plot()
        t2.join()


        
if __name__ == "__main__":
    planner = Camera2RRT()
    planner.run()
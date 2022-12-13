import sys
import os
import os.path as osp
import time
import threading

import cv2
from cv2 import aruco
import numpy as np
import math
from scipy.spatial import distance
import matplotlib.pyplot as plt

from easyEEZYbotARM.kinematic_model import EEZYbotARM_Mk2
from easyEEZYbotARM.serial_communication import arduinoController

from modules import RRTGraph, PurePursuit, Camera
from utils import log_generator


class Camera2RRT(Camera):
    def __init__(self, with_rotate=False):
        super().__init__()
        # Planner
        self.graph = None
        self.path = None

        # Robot arm
        self.robot_arm = EEZYbotARM_Mk2(initial_q1=0, initial_q2=90, initial_q3=-130)
        self.arduino = arduinoController()
        self.servoAngle_EE_closed = 10
        self.servoAngle_EE_open = 90

        # Obs
        self.obs_size = 10
        self.obs_points = []
        self.num_obs = 1

        # Line tracking
        self.with_rotate = with_rotate
        

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
                X, Y, Parent = self.graph.bias(goal)
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

    
    def plan_a_track(self, start, goal, previous_theta=None):
        # Plan
        self.core_planning(start, goal)

        # Plot and conduct pure-pursuit
        if self.with_rotate:
            theta = math.atan2(
                self.path[-2][1] - self.path[-1][1], 
                self.path[-2][0] - self.path[-1][0]
                )
            f = open("transfer_data/send.txt", "a")
            f.write(f"r {math.degrees(theta):.2f}\n")
            f.close()
            print(f'Send target pose: {math.degrees(theta):.2f}')
            time.sleep(1)
        else:
            theta = previous_theta

        while True:
            rbt2goal = distance.euclidean(self.components['robot'], self.components['goal'])
            print(f'dist: {rbt2goal:.2f}')
            if rbt2goal <= 25:
                print('Destination reached')
                f = open("transfer_data/send.txt", "a")
                f.write("v 0 0\n")
                f.close()
                break
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

            for _ in range(20):
                pure_pursuit.draw()
                self.graph.set_xylim()
                self.graph.draw_startgoal()
                self.graph.draw_map(self.graph.obstacles)

                self.graph.pause()

                if pure_pursuit.follower.is_dead:
                    break
                
            if pure_pursuit.follower.is_dead:
                print(f'Destination reached')
                break

            theta = pure_pursuit.follower.theta

            if not plt.fignum_exists(1):
                f = open("transfer_data/send.txt", "a")
                f.write("v 0 0\n")
                f.close()
                break
        return theta

    
    def get_rbt_pose(self):
        log_follow = log_generator("transfer_data/receive.txt")
        for lines in log_follow:
            if len(lines) > 0:
                return float(lines[-1]) 


    def compute_to_pick(self):
        """
            1. Get current robot and good position 
            2. (Optional) Calculate target pose and send to client for robot to straightly head to good
            3. Read current robot pose
            4. Compute (x, y) of good in arm coordinate
            5. Compute inverse kinematic and send data to client
        """
        rbt2good = distance.euclidean(
            self.components['robot'], 
            self.components['good']
            )
        rbt2good_ang = math.atan2(
            self.components['good'][1] - self.components['robot'][1],
            self.components['good'][0] - self.components['robot'][0]
            )

        # delta_ang = rbt2good_ang - pure_pursuit.follower.theta
        rbt_pose = self.get_rbt_pose()
        delta_ang = rbt2good_ang - rbt_pose
        arm_x = rbt2good * math.cos(delta_ang) * 10
        arm_y = rbt2good * math.sin(delta_ang) * 10
        arm_z = 85
        a1, a2, a3 = self.robot_arm.inverseKinematics(arm_x, arm_y, arm_z)
        self.robot_arm.updateJointAngles(q1=a1, q2=a2, q3=a3)
        servo_q1, servo_q2, servo_q3 = self.robot_arm.map_kinematicsToServoAngles()
        msg = self.arduino.composeMessage(
            servoAngle_q1=servo_q1, 
            servoAngle_q2=servo_q2, 
            servoAngle_q3=servo_q3, 
            servoAngle_EE=self.servoAngle_EE_open
            )

        f = open("transfer_data/send.txt", "a")
        f.write(msg + "\n")
        f.close()


    def plan_and_plot(self):
        while not all(self.components.values()):
            pass # Wait until all components on the map are detected
        self.graph = RRTGraph(
            self.components['robot'], 
            self.components['goal'], 
            self.obs_size, 
            self.num_obs
            )
        obs = self.graph.make_obs(self.components['obs'])
        self.graph.draw_map(obs)

        tracks = [['robot', 'good'], ['good', 'goal']]
        theta = np.pi/2
        for track in tracks:
            print(self.components[track[0]])
            print(self.components[track[1]])
            theta = self.plan_a_track(
                self.components[track[0]], 
                self.components[track[1]],
                previous_theta=theta
                )
            if track[1] == 'good':
                self.compute_to_pick()
            

    def process_frame(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break

            # self.detect_color_obj(frame)
            self.detect_markers(frame)

            cv2.imshow("Camera", frame)
            key = cv2.waitKey(1)
            if key == ord("q"):
                break


    def run(self):
        t1 = threading.Thread(target=self.process_frame, args=())
        t1.start()

        self.plan_and_plot()
        t1.join()

        
if __name__ == "__main__":
    planner = Camera2RRT(with_rotate=False)
    planner.run()

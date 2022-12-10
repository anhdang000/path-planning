import cv2
from cv2 import aruco
import math
from scipy.spatial import distance
import numpy as np


class Camera:
    def __init__(self, calib_data_path='calib_data/MultiMatrix.npz'):
        # Configurations
        calib_data = np.load(calib_data_path)
        self.cam_mat = calib_data["camMatrix"]
        self.dist_coef = calib_data["distCoef"]
        self.cam_height = 200

        # Aruco
        self.cap = cv2.VideoCapture(1)
        self.marker_size = 8.8
        self.marker_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.param_markers = aruco.DetectorParameters_create()
        self.id_to_role = {33: 'robot', 50: 'goal', 56: 'obs'}

        # Good detection (by color)
        self.red_lower = np.array([5, 50, 50], dtype=np.uint8)
        self.red_upper = np.array([15, 255, 255], dtype=np.uint8)

        # Track
        self.components = {'robot': None, 'goal': None, 'good': None, 'obs': None}


    def detect_color_obj(self, img):
        """Detect red-colored good object. 
            Return good's location is detected and None otherwise

        Args:
            img (np.ndarray): Input image

        Returns:
            (x_world, y_world): position of good in world coordinate
        """
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # define mask
        red_mask = cv2.inRange(hsv_img, self.red_lower, self.red_upper)

        kernel = np.ones((5, 5), "uint8")
        red_mask = cv2.dilate(red_mask, kernel)
        
        # Creating contour to track red color
        contours, _ = cv2.findContours(red_mask,
                                            cv2.RETR_TREE,
                                            cv2.CHAIN_APPROX_SIMPLE)

        x_world, y_world = None, None
        for _, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if (area > 100):
                x, y, w, h = cv2.boundingRect(contour)
                img = cv2.rectangle(img, (x, y), 
                                        (x + w, y + h), 
                                        (0, 0, 255), 2)

                fx = self.cam_mat[0, 0]
                fy = self.cam_mat[1, 1]
                cx = self.cam_mat[0, 2]
                cy = self.cam_mat[1, 2]

                x_world = (x - cx) * self.cam_height / fx
                y_world = (y - cy) * self.cam_height / fy

                cv2.putText(img, f"Object: ({x_world:.1f}, {y_world:.1f})", (x-5, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 0, 0))

        if x_world and y_world:
            self.components['good'] = (x_world, y_world)


    def detect_markers(self, img):
        gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        marker_corners, marker_IDs, _ = aruco.detectMarkers(
            gray_img, self.marker_dict, parameters=self.param_markers
        )
        # Check if all objects are already on the map
        if set(self.id_to_role).issubset(set(marker_IDs.flatten().tolist())):
            # Reset storage
            self.components['robot'] = None
            self.components['goal'] = None
            self.components['obs'] = []

            rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
                marker_corners, self.marker_size, self.cam_mat, self.dist_coef
            )
            total_markers = range(0, marker_IDs.size)
            for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
                cv2.polylines(
                    img, [corners.astype(np.int32)], True, (0, 255, 255), 2, cv2.LINE_AA
                )
                corners = corners.reshape(4, 2)
                corners = corners.astype(int)
                top_right = corners[0].ravel()
                bottom_right = corners[2].ravel()

                # Store positions
                x = tVec[i][0][0]
                y = -tVec[i][0][1]
                if self.id_to_role[ids[0]] == 'obs':
                    self.components['obs'].append((x, y))
                else:
                    self.components[self.id_to_role[ids[0]]] = (x, y)

                # Draw the pose of the marker
                cv2.drawFrameAxes(img, self.cam_mat, self.dist_coef, rVec[i], tVec[i], 4, 2)
                cv2.putText(
                    img,
                    self.id_to_role[ids[0]],
                    top_right,
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 0, 0),
                    1,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    img,
                    f"({tVec[i][0][0]:.1f}, {tVec[i][0][1]:.1f})",
                    bottom_right,
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 0, 0),
                    1,
                    cv2.LINE_AA,
                )


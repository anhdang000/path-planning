import sys
import os
import os.path as osp

import math

import cv2 as cv
from cv2 import aruco
import numpy as np

calib_data_path = "calib_data/MultiMatrix.npz"
calib_data = np.load(calib_data_path)
cam_mat = calib_data["camMatrix"]
dist_coef = calib_data["distCoef"]
r_vectors = calib_data["rVector"]
t_vectors = calib_data["tVector"]

MARKER_SIZE = 9.7  # centimeters
marker_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
param_markers = aruco.DetectorParameters_create()

cap = cv.VideoCapture(1)
while True:
    ret, frame = cap.read()
    if not ret:
        break
    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    marker_corners, marker_IDs, reject = aruco.detectMarkers(
        gray_frame, marker_dict, parameters=param_markers
    )
    if marker_corners:
        rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
            marker_corners, MARKER_SIZE, cam_mat, dist_coef
        )
        total_markers = range(0, marker_IDs.size)

        rbt_info = {}
        goal_info = {}
        
        for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
            if ids[0] in [33, 50]:
                cv.polylines(
                    frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA
                )
                corners = corners.reshape(4, 2)
                corners = corners.astype(int)
                top_right = corners[0].ravel()
                top_left = corners[1].ravel()
                bottom_right = corners[2].ravel()
                bottom_left = corners[3].ravel()

                # Draw the pose of the marker
                point = cv.drawFrameAxes(frame, cam_mat, dist_coef, rVec[i], tVec[i], MARKER_SIZE, 4)
                cv.putText(
                    frame,
                    f"x:{round(tVec[i][0][0],1)} y: {round(tVec[i][0][1],1)} ",
                    bottom_right,
                    cv.FONT_HERSHEY_PLAIN,
                    1.0,
                    (0, 0, 255),
                    2,
                    cv.LINE_AA,
                )

                if ids[0] == 33:
                    cv.putText(
                    frame,
                    f"Robot",
                    top_right,
                    cv.FONT_HERSHEY_PLAIN,
                    1.3,
                    (0, 0, 255),
                    2,
                    cv.LINE_AA,
                    )
                    rbt_info["x"] = tVec[i][0][0]
                    rbt_info["y"] = -tVec[i][0][1]
                    rbt_info["ang"] = rVec[i][0][0]
                else:
                    cv.putText(
                    frame,
                    f"Goal",
                    top_right,
                    cv.FONT_HERSHEY_PLAIN,
                    1.3,
                    (0, 0, 255),
                    2,
                    cv.LINE_AA,
                    )
                    goal_info["x"] = tVec[i][0][0]
                    goal_info["y"] = -tVec[i][0][1]

        if len(rbt_info.items()) > 0 and len(goal_info.items()) > 0:
            # Compute errors
            # target_ang = math.degrees(math.atan2(
            #     rbt_info["y"] - goal_info["y"], 
            #     rbt_info["x"] - goal_info["x"]
            #     ))
            target_ang = math.degrees(math.atan2(
                goal_info["y"] - rbt_info["y"], 
                goal_info["x"] - rbt_info["x"]
                ))
            print(f'rbt_ang: {math.degrees(rbt_info["ang"]):.2f}\ttarget_ang: {target_ang:.2f}')
            dist = math.sqrt(((rbt_info["x"] - goal_info["x"])**2 + (rbt_info["y"] - goal_info["y"])**2))
            f = open("transfer_data/send.txt", "a")
            f.write(f"{dist:.2f} {target_ang:.2f}\n")
            # print(f"Dist: {dist:.2f}\tTarget ang: {target_ang:.2f}")
            f.close()
        else:
            f = open("transfer_data/send.txt", "a")
            f.write(f"0 0\n")
            print(f"Objects not found!")
            f.close()
    cv.imshow("frame", frame)
    key = cv.waitKey(1)
    if key == ord("q"):
        break
cap.release()
cv.destroyAllWindows()

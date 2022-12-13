import sys
import os
import os.path as osp
from glob import glob
import argparse

import cv2
from cv2 import aruco
import numpy as np


calib_data_path = "calib_data/MultiMatrix.npz"
calib_data = np.load(calib_data_path)
cam_mat = calib_data["camMatrix"]
dist_coef = calib_data["distCoef"]
r_vectors = calib_data["rVector"]
t_vectors = calib_data["tVector"]
cam_height = 200

marker_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
param_markers = aruco.DetectorParameters_create()
MARKER_SIZE = 9.8   # cm



def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input-type', help='Image or Video')
    parser.add_argument('--img-dir', '-i', default='sample/images', help='Directory of sample images')
    parser.add_argument('--vid-source', default=0)
    return parser.parse_args()


def detect_markers(img):
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    marker_corners, marker_IDs, _ = aruco.detectMarkers(
        gray_img, marker_dict, parameters=param_markers
    )
    if marker_corners:
        rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
            marker_corners, MARKER_SIZE, cam_mat, dist_coef
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

            # Draw the pose of the marker
            point = cv2.drawFrameAxes(img, cam_mat, dist_coef, rVec[i], tVec[i], 4, 2)
            cv2.putText(
                img,
                f"ID: {ids[0]}",
                top_right,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 0),
                1,
                cv2.LINE_AA,
            )
            cv2.putText(
                img,
                f"x:{round(tVec[i][0][0],1)} y: {round(tVec[i][0][1],1)} ",
                bottom_right,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 0),
                1,
                cv2.LINE_AA,
            )
    return img


def detect_color_obj(img):
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # define mask
    red_lower = np.array([5, 50, 50], np.uint8)
    red_upper = np.array([15, 255, 255], np.uint8)
    red_mask = cv2.inRange(hsv_img, red_lower, red_upper)

    kernel = np.ones((5, 5), "uint8")
    red_mask = cv2.dilate(red_mask, kernel)
    res_red = cv2.bitwise_and(img, img, mask=red_mask)

    # Creating contour to track red color
    contours, _ = cv2.findContours(red_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)

    for _, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if(area > 100):
            x, y, w, h = cv2.boundingRect(contour)
            img = cv2.rectangle(img, (x, y), 
                                       (x + w, y + h), 
                                       (0, 0, 255), 2)

            fx = cam_mat[0, 0]
            fy = cam_mat[1, 1]
            cx = cam_mat[0, 2]
            cy = cam_mat[1, 2]

            x_world = (x - cx) * cam_height / fx
            y_world = (y - cy) * cam_height / fy

            cv2.putText(img, f"Object: ({x_world:.1f}, {y_world:.1f})", (x-5, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 0, 0))


def detect_frame(img, save_path=None):
    detect_markers(img)
    # detect_color_obj(img)
    if save_path:
        cv2.imwrite(save_path, img)


def detect_sequence(vid_source):
    cap = cv2.VideoCapture(1)
    num_saved = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        detect_frame(frame)

        cv2.imshow("frame", frame)
        key = cv2.waitKey(1)
        if key == ord("x"):
            num_saved += 1
            frame_path = osp.join('frames', f"{num_saved}.jpg")
            cv2.imwrite(frame_path, frame)
            print(f"Captured frame at: {frame_path}")
        if key == ord("q"):
            break
         

if __name__ == "__main__":
    args = get_args()
    if args.input_type == 'image':
        img_paths = glob(osp.join(args.img_dir, '*.jpg'))
        for img_path in img_paths:
            print(f'Image path: {img_path}')
            img = cv2.imread(img_path)
            detect_frame(img)
    elif args.input_type == 'video':
        detect_sequence(args.vid_source)
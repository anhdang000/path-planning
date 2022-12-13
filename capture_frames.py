import sys
import os
import os.path as osp

import cv2

dst_dir = "sample"
os.makedirs(dst_dir, exist_ok=True)

cap = cv2.VideoCapture(1)

num_saved = 0
while True:
    ret, frame = cap.read()
    if not ret:
        break
    cv2.imshow("frame", frame)
    key = cv2.waitKey(1)
    if key == ord("x"):
        num_saved += 1
        frame_path = osp.join(dst_dir, f"{num_saved}.jpg")
        cv2.imwrite(frame_path, frame)
        print(f"Captured frame at: {frame_path}")
    if key == ord("q"):
        break

from json import dumps
import cv2
import numpy as np

#objp = np.zeros((4*11, 3), np.float32)
#objp[:,:2] = np.mgrid[0:4,0:11].T.reshape(-1,2)
objp = np.zeros((9*6, 3), np.float32)
objp[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2)
print(objp)
objpoints = []
imgpoints = []

cam = cv2.VideoCapture()
cam.open(0)
while not cam.isOpened():
    pass

j = 0
while True:
    ret_cam, _frame = cam.read()
    if not ret_cam:
        continue
    gray = cv2.cvtColor(_frame, cv2.COLOR_BGR2GRAY)
    #ret, corners = cv2.findCirclesGrid(gray, (4, 11), flags=cv2.CALIB_CB_ASYMMETRIC_GRID)
    ret, corners = cv2.findChessboardCorners(gray, (9, 6))
    if ret:
        objpoints.append(objp)
        imgpoints.append(corners)
        gray_corner = gray.copy()
        cv2.drawChessboardCorners(gray_corner, (9, 6), corners, ret)
        cv2.imshow("test", gray_corner)
        char = cv2.waitKey()
        print(j)
        j += 1
        if char == 27:
            break

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

undist = cv2.undistort(_frame, mtx, dist, None, mtx)
cv2.imshow("test2", undist)
cv2.imshow("test", _frame)
cv2.waitKey()
with open("CalibData.json", "w") as f:
    f.write(dumps({
        "cameraMat": mtx.tolist(),
        "distCoef": dist.tolist(),
    }))
